/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"

#include "wcu_common.h"
#include "s2lp_api.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CAN_ID_TPMS_1       ( (uint32_t) 0x411 )       /* CAN ID: _411_WDTM_TPMS_1 */
#define FREQ_XO             ( (uint32_t) 16 )          /* S2-LP crystal oscillator frequency in MHz */
#define FREQ_TPMS           ( (uint32_t) 433 )         /* TPMS radio frequency in MHz */
#define RFRX_READ_BUF_SIZE  ( (uint32_t) 8 )           /* RX buffer size */
#define TPMS_FL_ID          ( (uint32_t) 0x00000000 )  /* Front-left TPMS sensor ID */
#define TPMS_FR_ID          ( (uint32_t) 0x00000001 )  /* Front-right TPMS sensor ID */
#define TPMS_RL_ID          ( (uint32_t) 0x00000002 )  /* Rear-left TPMS sensor ID */
#define TPMS_RR_ID          ( (uint32_t) 0x00000003 )  /* Rear-right TPMS sensor ID */

extern osThreadId rfRxHandle;
extern osMessageQId rfRxInternalMailQueueHandle;

/**
 * @brief TPMS data structure
 */
typedef struct STpmsData {
	uint8_t tireAirPressure;
	uint8_t tireAirTemperature;
	uint32_t sensorId;
} STpmsData;

static void RfRxParsePayload(STpmsData *tpmsDataPtr, uint8_t *payloadPtr);
static void RfRxAddDataToCanFrame(SCanFrame *canFramePtr,
		STpmsData *tpmsDataPtr);

/**
 * @brief Configure the S2-LP device
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxDeviceConfig(void) {

	ERfRxRet status = ERfRxRet_Ok;

	/* Go to standby state */
	if (ES2lpApiRet_Ok != S2lpApi_GoToStandbyState()) {

		status = ERfRxRet_Error;

	}

	if (ERfRxRet_Ok == status) {

		SS2lpApiBaseFrequencyConfig config;
		config.BANDSELECT = 1;
		config.REFDIV = 0;
		config.SYNT = 0x00400000 * FREQ_TPMS / FREQ_XO;
		/* Set base frequency */
		if (ES2lpApiRet_Ok != S2lpApi_SetBaseFrequency(config)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Select channel */
		if (ES2lpApiRet_Ok != S2lpApi_SetCenterFrequency(0, 0)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Enable Manchester encoding */
		if (ES2lpApiRet_Ok != S2lpApi_EnableManchester(true)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Configure CRC calculation */
		if (ES2lpApiRet_Ok != S2lpApi_ConfigCrc(ES2lpApiCrcMode_Poly0x07)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Configure GPIO0 */
		if (ES2lpApiRet_Ok
				!= S2lpApi_ConfigGpio(ES2lpApiGpioPin_0,
						ES2lpApiGpioMode_DigitalOutputLowPower,
						ES2lpApiGpioSignal_o_Irq)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Enable interrupt on RX data ready event */
		if (ES2lpApiRet_Ok
				!= S2lpApi_EnableInterrupt(ES2lpApiInterruptEvent_RxDataReady,
				true)) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Go to READY state */
		if (ES2lpApiRet_Ok != S2lpApi_GoToReadyState()) {

			status = ERfRxRet_Error;

		}

	}

	if (ERfRxRet_Ok == status) {

		/* Go to RX state */
		if (ES2lpApiRet_Ok != S2lpApi_GoToRxState()) {

			status = ERfRxRet_Error;

		}

	}

	return status;

}

/**
 * @brief Handle internal messages
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxHandleInternalMail(void) {

	ERfRxRet status = ERfRxRet_Ok;

	ERfRxInternalMail mail;
	/* Wait for messages */
	if (pdPASS
			== xQueueReceive(rfRxInternalMailQueueHandle, &mail,
					WCU_COMMON_TIMEOUT)) {

		static SCanFrame canFrame;
		uint8_t rxBufTbl[RFRX_READ_BUF_SIZE];
		bool dataReady;
		STpmsData tpmsData;

		switch (mail) {

		case ERfRxInternalMail_Exti:

			/* Assert data ready */
			if (ES2lpApiRet_Ok
					!= S2lpApi_TestInterrupt(ES2lpApiInterruptEvent_RxDataReady,
							&dataReady)) {

				LogPrint("RfRxHandleInternalMail: Test interrupt failed");
				status = ERfRxRet_Error;

			}

			if ((ERfRxRet_Ok == status) && dataReady) {

				/* Read data from the device's FIFO */
				if (ES2lpApiRet_Ok
						!= S2lpApi_ReadRxPayload(rxBufTbl,
								RFRX_READ_BUF_SIZE)) {

					LogPrint("RfRxHandleInternalMail: Read payload failed");
					status = ERfRxRet_Error;

				}

			}

			if ((ERfRxRet_Ok == status) && dataReady) {

				/* Parse the data */
				RfRxParsePayload(&tpmsData, rxBufTbl);
				/* Add the data to the CAN frame */
				RfRxAddDataToCanFrame(&canFrame, &tpmsData);

			}
			break;

		case ERfRxInternalMail_PeriodElapsed:

			/* Configure the CAN Tx header */
			canFrame.TxHeader.DLC = 8;
			canFrame.TxHeader.IDE = CAN_ID_STD;
			canFrame.TxHeader.RTR = CAN_RTR_DATA;
			canFrame.TxHeader.StdId = CAN_ID_TPMS_1;
			canFrame.TxHeader.TransmitGlobalTime = DISABLE;
			/* Transmit the frame */
			SendToCan(&canFrame);
			/* Clear the frame */
			(void) memset(&canFrame, 0, sizeof(canFrame));
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Callback on external interrupt
 * @retval None
 */
void RfRxExtiCallback(void) {

	ERfRxInternalMail mail = ERfRxInternalMail_Exti;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxInternalMailQueueHandle, &mail, NULL);

}

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void RfRxPeriodElapsedCallback(void) {

	ERfRxInternalMail mail = ERfRxInternalMail_PeriodElapsed;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxInternalMailQueueHandle, &mail, NULL);

}

/**
 * @brief Parse the TPMS sensor payload
 * @param tpmsDataPtr Buffer for the parsed TPMS data
 * @param payloadPtr Payload buffer
 * @retval None
 */
static void RfRxParsePayload(STpmsData *tpmsDataPtr, uint8_t *payloadPtr) {

	/* TODO: Parse the data */

}

/**
 * @brief Add the TPMS data to the CAN frame
 * @param canFramePtr CAN frame structure
 * @param tpmsDataPtr TPMS data structure
 * @retval None
 */
static void RfRxAddDataToCanFrame(SCanFrame *canFramePtr,
		STpmsData *tpmsDataPtr) {

	/* Identify the sensor */
	switch (tpmsDataPtr->sensorId) {

	case TPMS_FL_ID:

		canFramePtr->PayloadTbl[0] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[4] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_FR_ID:

		canFramePtr->PayloadTbl[1] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[5] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_RL_ID:

		canFramePtr->PayloadTbl[2] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[6] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_RR_ID:

		canFramePtr->PayloadTbl[3] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[7] = tpmsDataPtr->tireAirTemperature;
		break;

	default:

		LogPrint("RfRxAddDataToCanFrame: Unknown ID");
		break;

	}

}
