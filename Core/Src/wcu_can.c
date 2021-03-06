/**
 * @author Adrian Cinal
 * @file wcu_can.c
 * @brief Source file implementing the CAN service
 */

#include <wcu_sdio.h>
#include "wcu_can.h"
#include "wcu_events.h"
#include "wcu_logger.h"
#include "wcu_sdio.h"
#include "wcu_xbee.h"
#include "wcu_diagnostics.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;
extern osMessageQId canMessagesQueueHandle;

static EWcuRet WcuCanLoadSubscription(void);
static void WcuCanSetDefaultSubscription(void);

/* Default telemetry subscription */
#define WCU_DEFAULT_TELEMETRY_SUBSCRIPTION  { 0x400, 0x401, 0x402, 0x403, \
                                              0x404, 0x405, 0x406, 0x407, \
                                              0x408, 0x640, 0x648, 0x649 }

/**
 * @brief CAN service startup
 * @retval None
 */
void WcuCanStartup(void) {

	if (g_WcuSdioReady) {

		/* Try fetching the subscription from the SD card */
		if (EWcuRet_Ok != WcuCanLoadSubscription()) {

			WcuLogError(
					"%s(): Failed to load telemetry subscription. Setting default filter config...", __FUNCTION__);
			WcuCanSetDefaultSubscription();
		}

	} else {

		WcuLogInfo("%s(): SDIO not ready. Setting default filter config...", __FUNCTION__);
		WcuCanSetDefaultSubscription();
	}

	/* Enable interrupts on message pending in RX FIFOs and on error */
	(void) HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
					| CAN_IT_ERROR | CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE
					| CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

	/* Start the CAN peripheral */
	(void) HAL_CAN_Start(&hcan1);
}

/**
 * @brief Forward pending CAN message to a software queue
 * @param hwFifo Hardware queue identifier
 * @retval None
 */
void WcuCanForwardMessageFromIsrToSwQueue(uint32_t hwFifo) {

	SCanMessage message;
	if (HAL_OK
			== HAL_CAN_GetRxMessage(&hcan1, hwFifo, &message.RxHeader,
					message.PayloadTbl)) {

		if (pdPASS == xQueueSendFromISR(canMessagesQueueHandle, &message, NULL)) {

			WcuEventSend(EWcuEventType_CanRxMessagePending, NULL, 0);

		} else {

			/* A data race may occur here as this incrementing is done from outside the dispatchers' context, but since
			 * this is the only producer, we ignore it */
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(CanReceiveQueueStarvations);
		}
	}
}

/**
 * @brief Handle CAN error
 * @retval None
 */
void WcuCanHandleBusError(void) {

	/* Get the error flags */
	uint32_t errflags = HAL_CAN_GetError(&hcan1);

	/* Log the error */
	WcuLogError("%s(): error code 0x%08X", __FUNCTION__, errflags);

	/* Increment the statistics counter */
	WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(CanErrors);

	/* Reset the error flags */
	(void) HAL_CAN_ResetError(&hcan1);
}

/**
 * @brief Handle pending CAN message
 * @retval None
 */
void WcuCanHandlePendingMessage(void) {

	SCanMessage message;

	/* Receive the message */
	if (pdPASS == xQueueReceive(canMessagesQueueHandle, &message, 0)) {

		/* Increment the statistics counter */
		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(CanMessagesReceived);

		/* Send the telemetry data via XBEE-Pro */
		(void) WcuXbeeSendTelemetryData(&message);
	}
}

/**
 * @brief Send the message to the CAN bus
 * @param message CAN frame
 * @retval EWcuRet Status
 */
EWcuRet WcuCanMessageSend(SCanMessage *message) {

	EWcuRet status = EWcuRet_Ok;

	uint32_t dummy;
	/* Send the message */
	if (HAL_OK
			== HAL_CAN_AddTxMessage(&hcan1, &message->TxHeader,
					message->PayloadTbl, &dummy)) {

		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(CanMessagesSent);

	} else {

		WcuLogError("%s(): Send failed", __FUNCTION__);
		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief Set the new telemetry subscription
 * @param messageIds Array of message IDs
 * @param count Number of IDs in the messageIds array
 * @retval EWcuRet Status
 */
void WcuCanSetNewSubscription(uint32_t *messageIds, uint32_t count) {

	SetCanFilterList(&hcan1, messageIds, count);
}

/**
 * @brief Try fetching the telemetry subscription from the SD card
 * @retval EWcuRet Status
 */
static EWcuRet WcuCanLoadSubscription(void) {

	EWcuRet status = EWcuRet_Ok;

	FIL subscriptionFd;
	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM];
	uint32_t numOfFrames;

	/* Open the subscription file */
	if (EWcuRet_Ok
			!= WcuSdioFileOpen(&subscriptionFd, WCU_TELEMETRY_SUBSCRIPTION_PATH,
			FA_READ | FA_OPEN_EXISTING)) {

		WcuLogError("%s(): Open failed", __FUNCTION__);
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Read the number of frames */
		if (EWcuRet_Ok
				!= WcuSdioFileRead(&subscriptionFd, (uint8_t*) &numOfFrames,
						sizeof(numOfFrames))) {

			WcuLogError("%s(): Read failed", __FUNCTION__);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid number of frames */
		if (numOfFrames > R3TP_VER1_MAX_FRAME_NUM) {

			WcuLogError("%s(): Invalid frame number", __FUNCTION__);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the subscription */
		if (EWcuRet_Ok
				!= WcuSdioFileRead(&subscriptionFd, (uint8_t*) subscription,
						numOfFrames * sizeof(uint32_t))) {

			WcuLogError("%s(): Read failed", __FUNCTION__);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		WcuCanSetNewSubscription(subscription, numOfFrames);
	}

	/* Cleanup */
	(void) WcuSdioFileClose(&subscriptionFd);

	return status;
}

/**
 * @brief Set default telemetry subscription
 * @retval None
 */
void WcuCanSetDefaultSubscription(void) {

	uint32_t defaultSubscription[] = WCU_DEFAULT_TELEMETRY_SUBSCRIPTION;
	WcuCanSetNewSubscription(defaultSubscription,
			sizeof(defaultSubscription) / sizeof(uint32_t));
}
