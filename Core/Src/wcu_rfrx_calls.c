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
#include <stdbool.h>
#include <string.h>

#define CAN_ID_TPMS_1       ((uint32_t) 0x411)  /* CAN ID: _411_WDTM_TPMS_1 */
#define FREQ_XO             ((uint32_t) 16)     /* S2-LP crystal oscillator frequency in MHz */
#define FREQ_TPMS           ((uint32_t) 433)    /* TPMS radio frequency in MHz */
#define RFRX_READ_BUF_SIZE  ((uint32_t) 8)      /* RX buffer size */

extern osThreadId rfRxHandle;
extern osMessageQId rfRxInternalMailQueueHandle;

/**
 * @brief Configure the S2-LP device
 * @retval ERfRxRet Status
 */
ERfRxRet rfRx_DeviceConfig(void) {

	ERfRxRet status = ERfRxRet_Ok;

	/* Go to standby state */
	if (ES2lpApiRet_Ok != S2lpApi_GoToStandbyState()) {

		status = ERfRxRet_Error;

	}

	if (ERfRxRet_Ok == status) {

		SS2lpApiBaseFrequencyConfig config;
		config.BANDSELECT = 0;
		config.REFDIV = 0;
		config.SYNT = 0x00200000 * FREQ_TPMS / FREQ_XO;
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
ERfRxRet rfRx_HandleInternalMail(void) {

	ERfRxRet status = ERfRxRet_Ok;

	ERfRxInternalMail mail;
	/* Wait for messages */
	if (pdPASS == xQueueReceive(rfRxInternalMailQueueHandle, &mail, WCU_COMMON_TIMEOUT)) {

		static SCanFrame canFrame;
		bool dataReady;

		switch(mail) {

		case ERfRxInternalMail_Exti:

			/* Assert data ready */
			if(ES2lpApiRet_Ok != S2lpApi_TestInterrupt(ES2lpApiInterruptEvent_RxDataReady, &dataReady)) {

				status = ERfRxRet_Error;

			}

			if( (ERfRxRet_Ok == status) && dataReady ) {

				/* TODO: Read data from the device's FIFO */

				/* TODO: Validate the message (unless validated in hardware by S2-LP) */

			}

			if( (ERfRxRet_Ok == status) && dataReady ) {

				/* TODO: Identify the sensor, add data to CAN frame */

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
			AddToCanTxQueue(&canFrame, "rfRx failed to send to canTxQueue");
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
void rfRx_ExtiCallback(void) {

	ERfRxInternalMail mail = ERfRxInternalMail_Exti;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxInternalMailQueueHandle, &mail, NULL);

}

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void rfRx_PeriodElapsedCallback(void) {

	ERfRxInternalMail mail = ERfRxInternalMail_PeriodElapsed;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxInternalMailQueueHandle, &mail, NULL);

}
