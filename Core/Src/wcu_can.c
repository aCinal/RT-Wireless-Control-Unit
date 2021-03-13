/**
 * @author Adrian Cinal
 * @file wcu_can.c
 * @brief Source file implementing the CAN service
 */

#include <wcu_sdio.h>
#include "wcu_can.h"
#include "wcu_logger.h"
#include "wcu_sdio.h"
#include "wcu_xbee.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "stm32f4xx_hal.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

static EWcuRet WcuCanLoadSubscription(void);

/**
 * @brief CAN service startup
 * @retval None
 */
void WcuCanStartup(void) {

	if (g_WcuSdioReady) {

		/* Try fetching the subscription from the SD card */
		(void) WcuCanLoadSubscription();
	}
	/* Start the CAN peripheral */
	(void) HAL_CAN_Start(&hcan1);
}

/**
 * @brief Handle pending CAN message
 * @param fifo CAN FIFO identifier
 * @retval None
 */
void WcuCanHandlePendingMessage(uint32_t fifo) {

	SCanFrame message;
	/* Receive the message */
	if (HAL_OK
			== HAL_CAN_GetRxMessage(&hcan1, fifo, &message.RxHeader,
					message.PayloadTbl)) {

		/* Turn on the LED */
		SET_PIN(CAN_LED);

		/* Send the telemetry data via XBEE-Pro */
		(void) WcuXbeeSendTelemetryData(&message);

		/* Turn off the LED */
		RESET_PIN(CAN_LED);
	}
}

/**
 * @brief Send the message to the CAN bus
 * @param message CAN frame
 * @retval EWcuRet Status
 */
EWcuRet WcuCanMessageSend(SCanFrame *message) {

	EWcuRet status = EWcuRet_Ok;

	uint32_t dummy;
	/* Send the message */
	if (HAL_OK
			!= HAL_CAN_AddTxMessage(&hcan1, &message->TxHeader,
					message->PayloadTbl, &dummy)) {

		WcuLogError("WcuCanMessageSend: Send failed");
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

		WcuLogError("WcuCanLoadSubscription: Open failed");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Read the number of frames */
		if (EWcuRet_Ok
				!= WcuSdioFileRead(&subscriptionFd, (uint8_t*) &numOfFrames,
						sizeof(numOfFrames))) {

			WcuLogError("WcuCanLoadSubscription: Read failed");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid number of frames */
		if (numOfFrames > R3TP_VER1_MAX_FRAME_NUM) {

			WcuLogError("WcuCanLoadSubscription: Invalid frame number");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the subscription */
		if (EWcuRet_Ok
				!= WcuSdioFileRead(&subscriptionFd, (uint8_t*) subscription,
						numOfFrames * sizeof(uint32_t))) {

			WcuLogError("WcuCanLoadSubscription: Read failed");
			status = EWcuRet_Error;
		}
	}

	/* Cleanup */
	(void) WcuSdioFileClose(&subscriptionFd);

	return status;

}
