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
#define XBEE_UART_HANDLE        (huart4)                                  /* UART handle alias */
#define XBEE_UART_INSTANCE      (UART4)                                   /* UART instance alias */

extern UART_HandleTypeDef XBEE_UART_HANDLE;
extern osThreadId canGtkpHandle;
extern osThreadId sdioGtkpHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId xbeeTxRxEventQueueHandle;
SUartRingBuf gXbeeTxRxRingBuffer;

/**
 * @brief Driver warnings structure
 */
typedef struct SDriverWarnings {
	uint8_t greenWarningDuration;
	uint8_t redWarningDuration;
} SDriverWarnings;

static void XbeeTxRxRingBufferIdleCallback(void);
static EXbeeTxRxRet XbeeTxRxHandleMessageReceived(
		SDriverWarnings *warningsStatePtr);
static EXbeeTxRxRet XbeeTxRxHandleNewSubscription(uint8_t rxBufTbl[]);
static EXbeeTxRxRet XbeeTxRxHandleDriverWarning(uint8_t rxBufTbl[],
		SDriverWarnings *warningsStatePtr);
static void XbeeTxRxSendDiagnostics(SDriverWarnings *warningsStatePtr);
static void XbeeTxRxUpdateWarnings(SDriverWarnings *warningsStatePtr);
static EXbeeTxRxRet XbeeTxRxSendSubscriptionToCanGtkp(uint32_t ids[],
		size_t count);
static EXbeeTxRxRet XbeeTxRxSendSubscriptionToSdioGtkp(uint32_t ids[],
		size_t count);
static EXbeeTxRxRet XbeeTxRxSendAcknowledge(uint8_t msgId);

/**
 * @brief Configure the XBEE-Pro device
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet XbeeTxRxDeviceConfig(void) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Set the RESET pin high */
	SET_PIN(XBEE_RESET);

	/**
	 * XBee-Pro modules should be configured using XCTU software provided by Digi International.
	 * The telemetry receiver must be paired with the module on the car - destination address
	 * of one must correspond to the serial number of the other
	 */

	return status;

}

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet XbeeTxRxStartRingBufferIdleDetectionRx(void) {

	static uint8_t ringBufTbl[XBEETXRX_RING_BUF_SIZE];

	/* Configure the ring buffer structure */
	gXbeeTxRxRingBuffer.BufferPtr = ringBufTbl;
	gXbeeTxRxRingBuffer.BufferSize = XBEETXRX_RING_BUF_SIZE;
	gXbeeTxRxRingBuffer.Callback = &XbeeTxRxRingBufferIdleCallback;
	gXbeeTxRxRingBuffer.PeriphHandlePtr = &XBEE_UART_HANDLE;

	/* Start listening */
	return UartRingBufStart(&gXbeeTxRxRingBuffer);

}

/**
 * @brief Handle event messages
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet XbeeTxRxHandleEvents(void) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	EXbeeTxRxEvent mail;
	/* Check for messages */
	if (pdPASS == xQueueReceive(xbeeTxRxEventQueueHandle, &mail, 0)) {

		static SDriverWarnings warningsState;

		switch (mail) {

		case EXbeeTxRxEvent_MessageReceived:

			status = XbeeTxRxHandleMessageReceived(&warningsState);
			break;

		case EXbeeTxRxEvent_PeriodElapsed:

			/* Transmit the diagnostic frame */
			XbeeTxRxSendDiagnostics(&warningsState);
			/* Update the warnings structure */
			XbeeTxRxUpdateWarnings(&warningsState);
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Handle transmitting telemetry data
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet XbeeTxRxHandleOutgoingR3tpCom(void) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	SCanFrame frBuf;
	/* Listen on the canRxQueue for messages to send */
	if (pdPASS == xQueueReceive(canRxQueueHandle, &frBuf, WCU_COMMON_TIMEOUT)) {

		static uint8_t txBufTbl[R3TP_VER0_FRAME_SIZE]; /* UART Tx buffer */
		/* Clear the buffer */
		(void) memset(txBufTbl, 0, R3TP_VER0_FRAME_SIZE);

		/* Set VER and RES/SEQ field */
		txBufTbl[0] = R3TP_VER0_VER_BYTE;

		static uint8_t seqNum = 0;
		/* Set the SEQ NUM field */
		txBufTbl[1] = seqNum;
		/* Increment the sequence number */
		seqNum = (seqNum < 255U) ? seqNum + 1U : 0;

		/* Set the END SEQ field */
		txBufTbl[R3TP_VER0_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
		txBufTbl[R3TP_VER0_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

		/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
		txBufTbl[4] = _getbyte(frBuf.RxHeader.StdId, 0);
		txBufTbl[5] = _getbyte(frBuf.RxHeader.StdId, 1);

		/* Set the DLC field */
		txBufTbl[8] = (uint8_t) frBuf.RxHeader.DLC;

		/* Set the DATA field */
		for (uint8_t i = 0; i < frBuf.RxHeader.DLC; i += 1U) {

			txBufTbl[9U + i] = frBuf.PayloadTbl[i];

		}

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(txBufTbl, R3TP_VER0_FRAME_SIZE);

		/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
		txBufTbl[2] = _getbyte(calculatedCrc, 0);
		txBufTbl[3] = _getbyte(calculatedCrc, 1);

		/* Transmit the frame */
		if (EXbeeProApiRet_Ok
				!= XbeeProApiSendPayload(txBufTbl, R3TP_VER0_FRAME_SIZE)) {

			LogError("XbeeTxRxHandleOutgoingR3tpCom: Send failed");
			status = EXbeeTxRxRet_Error;

		}

	}

	return status;

}

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void XbeeTxRxPeriodElapsedCallback(void) {

	/* Notify the task */
	EXbeeTxRxEvent event = EXbeeTxRxEvent_PeriodElapsed;
	xQueueSendFromISR(xbeeTxRxEventQueueHandle, &event, NULL);

}

/**
 * @brief Callback on idle line detection in the ring buffer implementation
 * @retval None
 */
static void XbeeTxRxRingBufferIdleCallback(void) {

	EXbeeTxRxEvent event = EXbeeTxRxEvent_MessageReceived;
	/* Notify the task */
	(void) xQueueSendFromISR(xbeeTxRxEventQueueHandle, &event, NULL);

}

/**
 * @brief Handle the received message
 * @param warningsStatePtr Pointer to the warnings structure
 * @retval None
 */
static EXbeeTxRxRet XbeeTxRxHandleMessageReceived(
		SDriverWarnings *warningsState) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;
	uint8_t rxBufTbl[R3TP_MAX_FRAME_SIZE];

	/* Read the data from the ring buffer */
	(void) UartRingBufRead(&gXbeeTxRxRingBuffer, rxBufTbl,
	R3TP_MAX_FRAME_SIZE);

	/* Identify the protocol version */
	switch (rxBufTbl[0]) {

	case R3TP_VER1_VER_BYTE:

		status = XbeeTxRxHandleNewSubscription(rxBufTbl);
		break;

	case R3TP_VER2_VER_BYTE:

		status = XbeeTxRxHandleDriverWarning(rxBufTbl, warningsState);
		break;

	default:

		status = EXbeeTxRxRet_Error;
		LogError("XbeeTxRxHandleMessageReceived: Unexpected VER byte received");
		break;

	}

	return status;
}

/**
 * @brief Handle the new telemetry subscription
 * @param rxBufTbl UART Rx Buffer
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet XbeeTxRxHandleNewSubscription(uint8_t *rxBufTbl) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Read the number of frames in the payload */
	uint32_t frNum = _reinterpret32bits(rxBufTbl[7], rxBufTbl[6], rxBufTbl[5],
			rxBufTbl[4]);

	/* Assert the payload won't overflow the buffer */
	if (frNum > R3TP_VER1_MAX_FRAME_NUM) {

		LogError("XbeeTxRxHandleNewSubscription: Invalid frame number");
		status = EXbeeTxRxRet_Error;

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Validate the END SEQ field */
		if (!R3TP_VALID_END_SEQ(rxBufTbl, R3TP_VER1_MESSAGE_LENGTH(frNum))) {

			LogError("XbeeTxRxHandleNewSubscription: Invalid end sequence");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the CHECKSUM field */
		uint16_t readCrc = R3TP_READ_CRC(rxBufTbl);

		/* Clear the CHECKSUM field */
		rxBufTbl[2] = 0x00;
		rxBufTbl[3] = 0x00;

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(rxBufTbl,
				R3TP_VER1_MESSAGE_LENGTH(frNum));

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			/* Log the error */
			LogError("XbeeTxRxHandleNewSubscription: Invalid CRC");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		uint32_t subscriptionTbl[R3TP_VER1_MAX_FRAME_NUM];
		/* Read the payload */
		uint8_t *payload = R3TP_VER1_PAYLOAD(rxBufTbl);
		for (uint32_t i = 0; i < frNum; i += 1UL) {

			subscriptionTbl[i] = _reinterpret32bits(payload[3UL + 4UL * i],
					payload[2UL + 4UL * i], payload[1UL + 4UL * i],
					payload[4UL * i]);

		}

		/* Forward the subscription to the canGtkp task */
		status = XbeeTxRxSendSubscriptionToCanGtkp(subscriptionTbl, frNum);

		/* Forward the subscription to the sdioGtkp task */
		(void) XbeeTxRxSendSubscriptionToSdioGtkp(subscriptionTbl, frNum);

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Send acknowledge message */
		status = XbeeTxRxSendAcknowledge(R3TP_VER1_VER_BYTE);

	}

	return status;

}

/**
 * @brief Handle the driver warning
 * @param rxBufTbl UART Rx Buffer
 * @param warningsStatePtr Pointer to the diagnostics structure
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet XbeeTxRxHandleDriverWarning(uint8_t *rxBufTbl,
		SDriverWarnings *warningsStatePtr) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Validate the END SEQ */
	if ((R3TP_END_SEQ_LOW_BYTE != rxBufTbl[R3TP_VER2_FRAME_SIZE - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE != rxBufTbl[R3TP_VER2_FRAME_SIZE - 1U])) {

		LogError("XbeeTxRxHandleDriverWarning: Invalid end sequence");
		status = EXbeeTxRxRet_Error;

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the CHECKSUM field */
		uint16_t readCrc = R3TP_READ_CRC(rxBufTbl);

		/* Clear the CHECKSUM field */
		rxBufTbl[2] = 0x00;
		rxBufTbl[3] = 0x00;

		/* Calculate the CRC */
		uint16_t calculatedCrc = GetR3tpCrc(rxBufTbl, R3TP_VER2_FRAME_SIZE);

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			LogError("XbeeTxRxHandleDriverWarning: Invalid CRC");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Read the payload */
		switch (rxBufTbl[4]) {

		case R3TP_GREEN_WARNING_BYTE:

			/* Set the green warning duration */
			warningsStatePtr->greenWarningDuration = rxBufTbl[5];
			LogInfo("XbeeTxRxHandleDriverWarning: Green warning set");
			break;

		case R3TP_RED_WARNING_BYTE:

			/* Set the red warning duration */
			warningsStatePtr->redWarningDuration = rxBufTbl[5];
			LogInfo("XbeeTxRxHandleDriverWarning: Red warning set");
			break;

		default:

			break;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Send acknowledge message */
		status = XbeeTxRxSendAcknowledge(R3TP_VER2_VER_BYTE);

	}

	return status;

}

/**
 * @brief Send the telemetry diagnostic frame to the CAN bus
 * @param warningsStatePtr Pointer to the diagnostics structure
 * @retval None
 */
static void XbeeTxRxSendDiagnostics(SDriverWarnings *warningsStatePtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 2;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_TELEMETRY_DIAG;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the RSSI to the frame payload - *UNUSED* */
	canFrame.PayloadTbl[0] = 0x00;

	/* Clear the status flags */
	canFrame.PayloadTbl[1] = 0x00;

	/* Poll the XBEE_STATUS pin */
	if (GPIO_PIN_SET
			== HAL_GPIO_ReadPin(XBEE_STATUS_GPIO_Port, XBEE_STATUS_Pin)) {

		/* Set the Telemetry_State flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_STATE_BIT);

	}

	/* Test if the green warning is active */
	if (0U < warningsStatePtr->greenWarningDuration) {

		/* Set the Telemetry_Pit flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_PIT_BIT);

	}

	/* Test if the red warning is active */
	if (0U < warningsStatePtr->redWarningDuration) {

		/* Set the Telemetry_Warning flag */
		SET_BIT(canFrame.PayloadTbl[1], TELEMETRY_WARNING_BIT);

	}

	/* Transmit the frame */
	SendToCan(&canFrame);

}

/**
 * @brief Decrement the warning duration counters and updates the warning active flags
 * @param warningsStatePtr Pointer to the diagnostics structure
 * @retval None
 */
static void XbeeTxRxUpdateWarnings(SDriverWarnings *warningsStatePtr) {

	/* Test if the green warning is active */
	if (0U < warningsStatePtr->greenWarningDuration) {

		/* Decrement the warning duration counter */
		warningsStatePtr->greenWarningDuration -= 1U;

		if (0U == warningsStatePtr->greenWarningDuration) {

			LogInfo("XbeeTxRxUpdateWarnings: Green warning expired");

		}

	}

	/* Test if the red warning is active */
	if (0U < warningsStatePtr->redWarningDuration) {

		/* Decrement the warning duration counter */
		warningsStatePtr->redWarningDuration -= 1U;

		if (0U == warningsStatePtr->redWarningDuration) {

			LogInfo("XbeeTxRxUpdateWarnings: Red warning expired");

		}

	}

}

/**
 * @brief Forward the telemetry subscription to the CAN gatekeeper for the appropriate filters to be set
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet XbeeTxRxSendSubscriptionToCanGtkp(uint32_t ids[],
		size_t count) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		/* Send the frame to the queue */
		if (pdPASS != xQueueSend(canSubQueueHandle, ids + i, 0)) {

			/* Cleanup */
			(void) xQueueReset(canSubQueueHandle);

			LogError("XbeeTxRxSendSubscriptionToCanGtkp: Queue is full");
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
static EXbeeTxRxRet XbeeTxRxSendSubscriptionToSdioGtkp(uint32_t ids[],
		size_t count) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		/* Send the frame to the queue */
		if (pdPASS != xQueueSend(sdioSubQueueHandle, ids + i, 0)) {

			/* Cleanup */
			(void) xQueueReset(sdioSubQueueHandle);

			LogError("XbeeTxRxSendSubscriptionToSdioGtkp: Queue is full");
			status = EXbeeTxRxRet_Error;

		}

	}

	if (EXbeeTxRxRet_Ok == status) {

		/* Notify SDIO gatekeeper */
		(void) xTaskNotify(sdioGtkpHandle, count, eSetValueWithOverwrite);

	}

	return status;

}

/**
 * @brief Send an acknowledge message confirming reception of an R3TP message
 * @param msgId VER byte of the message being acknowledged
 * @retval EXbeeTxRxRet Status
 */
static EXbeeTxRxRet XbeeTxRxSendAcknowledge(uint8_t msgId) {

	EXbeeTxRxRet status = EXbeeTxRxRet_Ok;

	uint8_t txBufTbl[R3TP_VER3_FRAME_SIZE];
	/* Clear the buffer */
	(void) memset(txBufTbl, 0, R3TP_VER3_FRAME_SIZE);

	/* Set VER and RES/SEQ field */
	txBufTbl[0] = R3TP_VER3_VER_BYTE;

	static uint8_t seqNum = 0;
	/* Set the SEQ NUM field */
	txBufTbl[1] = seqNum;
	/* Increment the sequence number */
	seqNum = (seqNum < 255U) ? seqNum + 1U : 0;

	/* Set the END SEQ field */
	txBufTbl[R3TP_VER3_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
	txBufTbl[R3TP_VER3_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

	/* Set the MSG ID field */
	txBufTbl[4] = msgId;

	/* Calculate the CRC */
	uint16_t calculatedCrc = GetR3tpCrc(txBufTbl, R3TP_VER3_FRAME_SIZE);

	/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
	txBufTbl[2] = _getbyte(calculatedCrc, 0);
	txBufTbl[3] = _getbyte(calculatedCrc, 1);

	/* Transmit the frame */
	if (EXbeeProApiRet_Ok
			!= XbeeProApiSendPayload(txBufTbl, R3TP_VER3_FRAME_SIZE)) {

		LogError("XbeeTxRxSendAcknowledge: Send failed");
		status = EXbeeTxRxRet_Error;

	}

	return status;

}
