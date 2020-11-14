/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.c
 * @brief Source file defining functions called by the xbeeTxRx task
 */

#include "wcu_xbeetxrx_calls.h"

#include "wcu_common.h"
#include "xbeepro_api.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"
#include "rt12e_libs_uartringbuffer.h"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

#define XBEETXRX_RING_BUF_SIZE  ( (uint32_t) (2 * R3TP_MAX_FRAME_SIZE) )  /* UART ring buffer size */
#define CAN_ID_TELEMETRY_DIAG   ( (uint32_t) 0x733 )                      /* CAN ID: _733_TELEMETRY_DIAG */
#define TELEMETRY_STATE_BIT     ( (uint8_t) 0x80 )                        /* Telemetry_State bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_WARNING_BIT   ( (uint8_t) 0x40 )                        /* Telemetry_Warning bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_PIT_BIT       ( (uint8_t) 0x20 )                        /* Telemetry_Pit bit of the TELEMETRY_DIAG CAN frame */
#define XBEE_GUARD_TIMES        ( (uint16_t) 0x000A )                     /* Desired value of the Guard Times parameter */
#define XBEE_UART_HANDLE        (huart4)                                  /* UART handle alias */
#define XBEE_UART_INSTANCE      (UART4)                                   /* UART instance alias */

extern UART_HandleTypeDef XBEE_UART_HANDLE;
extern osThreadId canGtkpHandle;
extern osThreadId sdioGtkpHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId xbeeTxRxInternalMailQueueHandle;
SUartRingBuf gXbeeTxRxRingBuffer;

/**
 * @brief Telemetry diagnostics structure
 */
typedef struct STelemetryDiagnostics {
	uint8_t greenWarningDuration;
	uint8_t redWarningDuration;
	uint8_t rssi;
} STelemetryDiagnostics;

static void xbeeTxRx_RingBufferIdleCallback(void);
static EXbeeTxRxRet xbeeTxRx_HandleNewSubscription(uint8_t rxBufTbl[]);
static EXbeeTxRxRet xbeeTxRx_HandleDriverWarning(uint8_t rxBufTbl[],
		STelemetryDiagnostics *diagPtr);
static void xbeeTxRx_SendDiagnostics(STelemetryDiagnostics *diagPtr);
static void xbeeTxRx_UpdateWarnings(STelemetryDiagnostics *diagPtr);
static EXbeeTxRxRet xbeeTxRx_ReadRssi(uint8_t *rssiPtr);
static EXbeeTxRxRet xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[],
		size_t count);
static EXbeeTxRxRet xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[],
		size_t count);

/**
 * @brief Configure the XBEE-Pro device
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet xbeeTxRx_DeviceConfig(void) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Set the RESET pin high */
	HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_SET);

	/* Allow the device the time to boot */
	vTaskDelay(pdMS_TO_TICKS(50));

	/* Set the guard time */
	if (EXbeeProApiRet_Ok != XbeeProApi_SetGuardTimes(XBEE_GUARD_TIMES)) {

		status = EXbeeTxRxRet_Error;

	}

	return status;

}

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet xbeeTxRx_StartRingBufferIdleDetectionRx(void) {

	static uint8_t ringBufTbl[XBEETXRX_RING_BUF_SIZE];

	/* Configure the ring buffer structure */
	gXbeeTxRxRingBuffer.BufferPtr = ringBufTbl;
	gXbeeTxRxRingBuffer.BufferSize = XBEETXRX_RING_BUF_SIZE;
	gXbeeTxRxRingBuffer.Callback = &xbeeTxRx_RingBufferIdleCallback;
	gXbeeTxRxRingBuffer.PeriphHandlePtr = &XBEE_UART_HANDLE;

	/* Start listening */
	return UartRingBuf_Start(&gXbeeTxRxRingBuffer);

}

/**
 * @brief Handle internal messages
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet xbeeTxRx_HandleInternalMail(void) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	EXbeeTxRxInternalMail mail;
	/* Check for messages */
	if (pdPASS == xQueueReceive(xbeeTxRxInternalMailQueueHandle, &mail, 0)) {

		static uint8_t rxBufTbl[R3TP_MAX_FRAME_SIZE];
		static STelemetryDiagnostics diagnostics;

		switch (mail) {

		case EXbeeTxRxInternalMail_MessageReceived:

			/* Read the data from the ring buffer */
			(void) UartRingBuf_Read(&gXbeeTxRxRingBuffer, rxBufTbl,
			R3TP_MAX_FRAME_SIZE);

			/* Identify the protocol version */
			switch (rxBufTbl[0]) {

			case R3TP_VER1_VER_BYTE:

				status = xbeeTxRx_HandleNewSubscription(rxBufTbl);
				break;

			case R3TP_VER2_VER_BYTE:

				status = xbeeTxRx_HandleDriverWarning(rxBufTbl, &diagnostics);
				break;

			default:

				LogPrint(
						"xbeeTxRx_HandleInternalMail: Unknown protocol version");
				status = EXbeeTxRxRet_Error;
				break;

			}

			if (EXbeeTxRxRet_Ok == status) {

				/* Poll the device for the RSSI value */
				status = xbeeTxRx_ReadRssi(&diagnostics.rssi);

			}
			break;

		case EXbeeTxRxInternalMail_PeriodElapsed:

			/* Transmit the diagnostic frame */
			xbeeTxRx_SendDiagnostics(&diagnostics);
			/* Update the diagnostics structure */
			xbeeTxRx_UpdateWarnings(&diagnostics);
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Handle transmitting telemetry data
 * @retval None
 */
void xbeeTxRx_HandleOutgoingR3tpCom(void) {

	SCanFrame frBuf; /* Buffer for the CAN frame */

	/* Listen on the canRxQueue for messages to send */
	if (pdPASS == xQueueReceive(canRxQueueHandle, &frBuf, WCU_COMMON_TIMEOUT)) {

		static uint8_t txBufTbl[R3TP_VER0_FRAME_SIZE]; /* UART Tx buffer */
		/* Clear the buffer */
		(void) memset(txBufTbl, 0, R3TP_VER0_FRAME_SIZE);

		/* Set VER and RES/SEQ field */
		txBufTbl[0] = R3TP_VER0_VER_BYTE;

		static uint8_t seqNum = 0; /* Sequence number */
		/* Set the SEQ NUM field */
		txBufTbl[1] = seqNum;
		/* Increment the sequence number */
		seqNum = (seqNum < 255U) ? seqNum + 1U : 0;

		/* Set the END SEQ field */
		txBufTbl[R3TP_VER0_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
		txBufTbl[R3TP_VER0_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

		/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
		txBufTbl[4] = _bits0_7(frBuf.RxHeader.StdId);
		txBufTbl[5] = _bits8_15(frBuf.RxHeader.StdId);

		/* Set the DLC field */
		txBufTbl[8] = (uint8_t) frBuf.RxHeader.DLC;

		/* Set the DATA field */
		for (uint8_t i = 0; i < frBuf.RxHeader.DLC; i += 1U) {

			txBufTbl[9U + i] = frBuf.PayloadTbl[i];

		}

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(txBufTbl, R3TP_VER0_FRAME_SIZE);

		/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
		txBufTbl[2] = _bits0_7(calculatedCrc);
		txBufTbl[3] = _bits8_15(calculatedCrc);

		/* Transmit the frame */
		XbeeProApi_SendPayload(txBufTbl, R3TP_VER0_FRAME_SIZE);

	}

}

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void xbeeTxRx_PeriodElapsedCallback(void) {

	/* Notify the task */
	EXbeeTxRxInternalMail xbeeTxRxInternalMail =
			EXbeeTxRxInternalMail_PeriodElapsed;
	xQueueSendFromISR(xbeeTxRxInternalMailQueueHandle, &xbeeTxRxInternalMail,
			NULL);

}

/**
 * @brief Callback on idle line detection in the ring buffer implementation
 * @retval None
 */
static void xbeeTxRx_RingBufferIdleCallback(void) {

	EXbeeTxRxInternalMail mail = EXbeeTxRxInternalMail_MessageReceived;
	/* Notify the task */
	(void) xQueueSendFromISR(xbeeTxRxInternalMailQueueHandle, &mail, NULL);

}

/**
 * @brief Handle the new telemetry subscription
 * @param rxBufTbl UART Rx Buffer
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet xbeeTxRx_HandleNewSubscription(uint8_t *rxBufTbl) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Read the number of frames in the payload */
	uint32_t frNum = _reinterpret32bits(rxBufTbl[7], rxBufTbl[6], rxBufTbl[5],
			rxBufTbl[4]);

	/* Assert the payload won't overflow the buffer */
	if (frNum > R3TP_VER1_MAX_FRAME_NUM) {

		LogPrint("xbeeTxRx_HandleNewSubscription: Invalid frame number");
		status = EXbeeTxRxRet_Error;

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Validate the END SEQ field */
		if ((R3TP_END_SEQ_LOW_BYTE
				!= rxBufTbl[R3TP_VER1_MESSAGE_LENGTH(frNum) - 2U])
				|| (R3TP_END_SEQ_HIGH_BYTE
						!= rxBufTbl[R3TP_VER1_MESSAGE_LENGTH(frNum) - 1U])) {

			LogPrint("xbeeTxRx_HandleNewSubscription: Invalid end sequence");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
		uint16_t readCrc = _reinterpret16bits(rxBufTbl[3], rxBufTbl[2]);

		/* Clear the CHECKSUM field */
		rxBufTbl[2] = 0x00U;
		rxBufTbl[3] = 0x00U;

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(rxBufTbl,
				R3TP_VER1_MESSAGE_LENGTH(frNum));

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			/* Log the error */
			LogPrint("xbeeTxRx_HandleNewSubscription: Invalid CRC");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
		/* Read the payload */
		uint8_t *payload = R3TP_VER1_PAYLOAD(rxBufTbl);
		for (uint32_t i = 0; i < frNum; i += 1UL) {

			subscription[i] = _reinterpret32bits(payload[3UL + 4UL * i],
					payload[2UL + 4UL * i], payload[1UL + 4UL * i],
					payload[4UL * i]);

		}

		/* Forward the subscription to the canGtkp task */
		status = xbeeTxRx_SendSubscriptionToCanGtkp(subscription, frNum);

		/* Forward the subscription to the sdioGtkp task */
		(void) xbeeTxRx_SendSubscriptionToSdioGtkp(subscription, frNum);

	}

	return status;

}

/**
 * @brief Handle the driver warning
 * @param rxBufTbl UART Rx Buffer
 * @param diagPtr Pointer to the diagnostics structure
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet xbeeTxRx_HandleDriverWarning(uint8_t *rxBufTbl,
		STelemetryDiagnostics *diagPtr) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Validate the END SEQ */
	if ((R3TP_END_SEQ_LOW_BYTE != rxBufTbl[R3TP_VER2_FRAME_SIZE - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE != rxBufTbl[R3TP_VER2_FRAME_SIZE - 1U])) {

		LogPrint("xbeeTxRx_HandleDriverWarning: Invalid end sequence");
		status = EXbeeTxRxRet_Error;

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
		uint16_t readCrc = _reinterpret16bits(rxBufTbl[3], rxBufTbl[2]);

		/* Clear the CHECKSUM field */
		rxBufTbl[2] = 0x00;
		rxBufTbl[3] = 0x00;

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(rxBufTbl, R3TP_VER2_FRAME_SIZE);

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			LogPrint("xbeeTxRx_HandleDriverWarning: Invalid CRC");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the payload */
		switch (rxBufTbl[4]) {

		case R3TP_GREEN_WARNING_BYTE:

			/* Set the green warning duration */
			diagPtr->greenWarningDuration = rxBufTbl[5];
			break;

		case R3TP_RED_WARNING_BYTE:

			/* Set the red warning duration */
			diagPtr->redWarningDuration = rxBufTbl[5];
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Poll the XBEE-Pro device for the RSSI value of the last transmission received
 * @param rssiPtr Pointer to pass the received value out of the function
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet xbeeTxRx_ReadRssi(uint8_t *rssiPtr) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Stop data transfer to the ring buffer */
	if (EUartRingBufRet_Ok != UartRingBuf_Stop(&gXbeeTxRxRingBuffer)) {

		LogPrint("xbeeTxRx_ReadRssi: Ring buffer stop failed");
		status = EXbeeTxRxRet_Error;

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read RSSI */
		if (EXbeeProApiRet_Ok != XbeeProApi_ReadRssi(rssiPtr)) {

			LogPrint("xbeeTxRx_ReadRssi: Read RSSI failed");
			status = EXbeeTxRxRet_Error;

		}

	}

	/* Resume data transfer to the ring buffer */
	if (EUartRingBufRet_Ok != UartRingBuf_Start(&gXbeeTxRxRingBuffer)) {

		LogPrint("xbeeTxRx_ReadRssi: Ring buffer resume failed");
		status = EXbeeTxRxRet_Error;

	}

	return status;

}

/**
 * @brief Send the telemetry diagnostic frame to the CAN bus
 * @param diagPtr Pointer to the diagnostics structure
 * @retval None
 */
static void xbeeTxRx_SendDiagnostics(STelemetryDiagnostics *diagPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 2;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_TELEMETRY_DIAG;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the RSSI to the frame payload */
	canFrame.PayloadTbl[0] = diagPtr->rssi;

	/* Clear the status flags */
	canFrame.PayloadTbl[1] = 0x00;

	/* Poll the XBEE_STATUS */
	if (GPIO_PIN_SET
			== HAL_GPIO_ReadPin(XBEE_STATUS_GPIO_Port, XBEE_STATUS_Pin)) {

		/* Set the Telemetry_State flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_STATE_BIT);

	}

	/* Test if the green warning is active */
	if (0U < diagPtr->greenWarningDuration) {

		/* Set the Telemetry_Pit flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_PIT_BIT);

	}

	/* Test if the red warning is active */
	if (0U < diagPtr->redWarningDuration) {

		/* Set the Telemetry_Warning flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_WARNING_BIT);

	}

	/* Transmit the frame */
	SendToCan(&canFrame);

}

/**
 * @brief Decrement the warning duration counters and updates the warning active flags
 * @param diagPtr Pointer to the diagnostics structure
 * @retval None
 */
static void xbeeTxRx_UpdateWarnings(STelemetryDiagnostics *diagPtr) {

	/* Test if the green warning is active */
	if (0U < diagPtr->greenWarningDuration) {

		/* Decrement the warning duration counter */
		diagPtr->greenWarningDuration -= 1U;

	}

	/* Test if the red warning is active */
	if (0U < diagPtr->redWarningDuration) {

		/* Decrement the warning duration counter */
		diagPtr->redWarningDuration -= 1U;

	}

}

/**
 * @brief Forward the telemetry subscription to the CAN gatekeeper for the appropriate filters to be set
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[],
		size_t count) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		/* Send the frame to the queue */
		if (pdPASS != xQueueSend(canSubQueueHandle, ids + i, 0)) {

			/* Cleanup */
			(void) xQueueReset(canSubQueueHandle);

			LogPrint("xbeeTxRx_SendSubscriptionToCanGtkp: Queue is full");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Notify CAN gatekeeper */
		(void) xTaskNotify(canGtkpHandle, count, eSetValueWithOverwrite);

	}

	return status;

}

/**
 * @brief Forward the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[],
		size_t count) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		/* Send the frame to the queue */
		if (pdPASS != xQueueSend(sdioSubQueueHandle, ids + i, 0)) {

			/* Cleanup */
			(void) xQueueReset(sdioSubQueueHandle);

			LogPrint("xbeeTxRx_SendSubscriptionToSdioGtkp: Queue is full");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Notify SDIO gatekeeper */
		(void) xTaskNotify(sdioGtkpHandle, count, eSetValueWithOverwrite);

	}

	return status;

}
