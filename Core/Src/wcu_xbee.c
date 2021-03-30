/**
 * @author Adrian Cinal
 * @file wcu_xbee.c
 * @brief Source file implementing the XBEE service
 */

#include "wcu_xbee.h"

#include "wcu_defs.h"
#include "wcu_logger.h"
#include "wcu_events.h"
#include "wcu_wrappers.h"
#include "wcu_can.h"
#include "wcu_sdio.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_uartringbuffer_tx.h"
#include "rt12e_libs_uartringbuffer_rx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>
#include "main.h"

#define WCU_XBEE_TX_RING_BUFFER_SIZE   ( (uint32_t) (10 * R3TP_VER0_FRAME_SIZE) )  /* UART TX ring buffer size */
#define WCU_XBEE_RX_RING_BUFFER_SIZE   ( (uint32_t) (2 * R3TP_MAX_FRAME_SIZE) )    /* UART RX ring buffer size */
#define WCU_CAN_ID_TELEMETRY_DIAG      ( (uint32_t) 0x733 )                        /* CAN ID: _733_TELEMETRY_DIAG */
#define WCU_TELEMETRY_STATE_BIT        ( (uint8_t) 0x80 )                          /* Telemetry_State bit of the TELEMETRY_DIAG CAN frame */
#define WCU_TELEMETRY_WARNING_BIT      ( (uint8_t) 0x40 )                          /* Telemetry_Warning bit of the TELEMETRY_DIAG CAN frame */
#define WCU_TELEMETRY_PIT_BIT          ( (uint8_t) 0x20 )                          /* Telemetry_Pit bit of the TELEMETRY_DIAG CAN frame */

extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim7;

static uint8_t g_GreenWarningDuration = 0;
static uint8_t g_RedWarningDuration = 0;
static uint8_t g_SeqNum = 0;
SUartTxRb g_WcuXbeeTxRingBuffer;
SUartRxRb g_WcuXbeeRxRingBuffer;

static EWcuRet WcuXbeeTxRingBufferInit(void);
static EWcuRet WcuXbeeRxRingBufferInit(void);
static EWcuRet WcuXbeeDeviceConfig(void);
static EWcuRet WcuXbeeSendDiagnostics(void);
static void WcuXbeeWarningsTick(void);
static EWcuRet WcuXbeeHandleR3tpMessage(void);
static EWcuRet WcuXbeeSendAcknowledge(uint8_t msgId);
static EWcuRet WcuXbeeHandleNewSubscription(uint8_t *r3tpMessage);
static EWcuRet WcuXbeeHandleDriverWarning(uint8_t *r3tpMessage);
static EWcuRet WcuXbeeStoreNewSubscription(uint32_t *ids, uint32_t numOfFrames);
static void WcuXbeeSendData(uint8_t *data, uint32_t len);
static void WcuXbeeRxCallback(void);

/**
 * @brief XBEE service startup
 * @retval None
 */
void WcuXbeeStartup(void) {

	/* Initialize the ring buffers */
	if (EWcuRet_Ok == WcuXbeeTxRingBufferInit()) {

		WcuLogInfo("WcuXbeeStartup: XBEE TX ring buffer initialized");

	} else {

		WcuLogError(
				"WcuXbeeStartup: XBEE TX ring buffer initialization failed");
	}

	if (EWcuRet_Ok == WcuXbeeRxRingBufferInit()) {

		WcuLogInfo("WcuXbeeStartup: XBEE RX ring buffer initialized");

	} else {

		WcuLogError(
				"WcuXbeeStartup: XBEE RX ring buffer initialization failed");
	}

	/* Start the timer */
	(void) HAL_TIM_Base_Start_IT(&htim7);

	/* Device config */
	(void) WcuXbeeDeviceConfig();
}

/**
 * @brief Handle the pending RX message
 * @retval None
 */
void WcuXbeeHandlePendingRxMessage(void) {

	(void) WcuXbeeHandleR3tpMessage();
}

/**
 * @brief Handle the timer expired event
 * @retval None
 */
void WcuXbeeHandleTimerExpired(void) {

	(void) WcuXbeeSendDiagnostics();
	WcuXbeeWarningsTick();
}

/**
 * @brief Transmit CAN frame via R3TP to the XBEE receiver
 * @param canMessage Pointer to the CAN frame
 * @retval EWcuRet Status
 */
EWcuRet WcuXbeeSendTelemetryData(SCanMessage *canMessage) {

	EWcuRet status = EWcuRet_Ok;

	uint8_t buffer[R3TP_VER0_FRAME_SIZE];

	/* Clear the buffer */
	memset(buffer, 0, sizeof(buffer));

	buffer[0] = R3TP_VER0_VER_BYTE;
	/* Set the SEQ NUM field */
	buffer[1] = g_SeqNum;
	/* Increment the sequence number */
	g_SeqNum = (g_SeqNum < 255U) ? g_SeqNum + 1U : 0;

	/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
	buffer[4] = _getbyte(canMessage->RxHeader.StdId, 0);
	buffer[5] = _getbyte(canMessage->RxHeader.StdId, 1);
	buffer[6] = _getbyte(canMessage->RxHeader.StdId, 2);
	buffer[7] = _getbyte(canMessage->RxHeader.StdId, 3);

	/* Set the DLC field */
	buffer[8] = (uint8_t) canMessage->RxHeader.DLC;

	/* Set the DATA field */
	for (uint8_t i = 0; i < canMessage->RxHeader.DLC; i += 1U) {

		buffer[9U + i] = canMessage->PayloadTbl[i];
	}

	/* Set the END SEQ field */
	buffer[R3TP_VER0_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
	buffer[R3TP_VER0_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

	/* Calculate the CRC */
	uint16_t calculatedCrc = WcuGetR3tpCrc(buffer, R3TP_VER0_FRAME_SIZE);

	/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
	buffer[2] = _getbyte(calculatedCrc, 0);
	buffer[3] = _getbyte(calculatedCrc, 1);

	/* Transmit the frame */
	WcuXbeeSendData(buffer, R3TP_VER0_FRAME_SIZE);

	return status;
}

/**
 * @brief Initialize the XBEE TX ring buffer
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeTxRingBufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_XBEE_TX_RING_BUFFER_SIZE];

	/* Configure the ring buffer structure */
	(void) UartTxRbInit(&g_WcuXbeeTxRingBuffer, &huart4, ringbuffer,
			sizeof(ringbuffer), NULL);

	return status;
}

/**
 * @brief Initialize the XBEE RX ring buffer
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeRxRingBufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_XBEE_RX_RING_BUFFER_SIZE];

	/* Configure the ring buffer structure */
	(void) UartRxRbInit(&g_WcuXbeeRxRingBuffer, &huart4, ringbuffer,
			sizeof(ringbuffer), &WcuXbeeRxCallback);

	/* Start listening */
	if (EUartRxRbRet_Ok != UartRxRbRecv(&g_WcuXbeeRxRingBuffer)) {

		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief Configure the XBee-Pro device
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeDeviceConfig(void) {

	EWcuRet status = EWcuRet_Ok;

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
 * @brief Send XBEE diagnostics CAN frame
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeSendDiagnostics(void) {

	EWcuRet status = EWcuRet_Ok;
	SCanMessage canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = 2;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = WCU_CAN_ID_TELEMETRY_DIAG;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the RSSI to the frame payload - *UNUSED* */
	canMessage.PayloadTbl[0] = 0x00;

	/* Clear the status flags */
	canMessage.PayloadTbl[1] = 0x00;

	/* Poll the XBEE_STATUS pin */
	if (GPIO_PIN_SET
			== HAL_GPIO_ReadPin(XBEE_STATUS_GPIO_Port, XBEE_STATUS_Pin)) {

		/* Set the Telemetry_State flag */
		SET_BIT(canMessage.PayloadTbl[1], WCU_TELEMETRY_STATE_BIT);
	}

	/* Test if the green warning is active */
	if (0U < g_GreenWarningDuration) {

		/* Set the Telemetry_Pit flag */
		SET_BIT(canMessage.PayloadTbl[1], WCU_TELEMETRY_PIT_BIT);
	}

	/* Test if the red warning is active */
	if (0U < g_RedWarningDuration) {

		/* Set the Telemetry_Warning flag */
		SET_BIT(canMessage.PayloadTbl[1], WCU_TELEMETRY_WARNING_BIT);
	}

	/* Transmit the frame */
	status = WcuCanMessageSend(&canMessage);
	return status;
}

/**
 * @brief Decrement the warnings counters
 * @retval None
 */
static void WcuXbeeWarningsTick(void) {

	/* Test if the green warning is active */
	if (0U < g_GreenWarningDuration) {

		/* Decrement the warning duration counter */
		g_GreenWarningDuration -= 1U;

		if (0U == g_GreenWarningDuration) {

			WcuLogInfo("WcuXbeeWarningsTick: Green warning expired");
		}
	}

	/* Test if the red warning is active */
	if (0U < g_RedWarningDuration) {

		/* Decrement the warning duration counter */
		g_RedWarningDuration -= 1U;

		if (0U == g_RedWarningDuration) {

			WcuLogInfo("WcuXbeeWarningsTick: Red warning expired");
		}
	}
}

static EWcuRet WcuXbeeHandleR3tpMessage(void) {

	EWcuRet status = EWcuRet_Ok;

	uint8_t buffer[R3TP_MAX_FRAME_SIZE];
	size_t bytesRead = 0;
	/* Read the message from the buffer */
	if (EUartRxRbRet_Ok
			!= UartRxRbRead(&g_WcuXbeeRxRingBuffer, buffer, sizeof(buffer),
					&bytesRead)) {

		WcuLogError("WcuXbeeHandleR3tpMessage: Ring buffer read failed");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Identify the protocol version */
		switch (R3TP_PROTOCOL_VERSION(buffer)) {

		case R3TP_VER1_VER_BYTE:

			WcuXbeeHandleNewSubscription(buffer);
			break;

		case R3TP_VER2_VER_BYTE:

			WcuXbeeHandleDriverWarning(buffer);
			break;

		default:

			WcuLogError("WcuXbeeHandleR3tpMessage: Unknown protocol version");
			status = EWcuRet_Error;
			break;
		}
	}

	return status;
}

/**
 * @brief Send the R3TP acknowledge frame
 * @param msgId ID of the message being acknowledged (R3TP VER byte)
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeSendAcknowledge(uint8_t msgId) {

	EWcuRet status = EWcuRet_Ok;

	uint8_t buffer[R3TP_VER3_FRAME_SIZE];

	/* Clear the buffer */
	memset(buffer, 0, sizeof(buffer));

	/* Set the VER field */
	buffer[0] = R3TP_VER3_VER_BYTE;

	/* Set the SEQ NUM field */
	buffer[1] = g_SeqNum;
	/* Increment the sequence number */
	g_SeqNum = (g_SeqNum < 255U) ? g_SeqNum + 1U : 0;

	/* Set the MSG ID field */
	buffer[4] = msgId;

	/* Set the END SEQ field */
	buffer[R3TP_VER3_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
	buffer[R3TP_VER3_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

	/* Calculate the CRC */
	uint16_t calculatedCrc = WcuGetR3tpCrc(buffer, R3TP_VER3_FRAME_SIZE);

	/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
	buffer[2] = _getbyte(calculatedCrc, 0);
	buffer[3] = _getbyte(calculatedCrc, 1);

	/* Transmit the frame */
	WcuXbeeSendData(buffer, R3TP_VER3_FRAME_SIZE);

	return status;
}

/**
 * @brief Handle the new subscription message
 * @param r3tpMessage R3TP VER1 message
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeHandleNewSubscription(uint8_t *r3tpMessage) {

	EWcuRet status = EWcuRet_Ok;

	/* Read the number of frames in the payload */
	uint32_t numOfFrames = _reinterpret32bits(r3tpMessage[7], r3tpMessage[6],
			r3tpMessage[5], r3tpMessage[4]);

	/* Assert the payload won't overflow the buffer */
	if (numOfFrames > R3TP_VER1_MAX_FRAME_NUM) {

		WcuLogError("WcuXbeeHandleNewSubscription: Invalid frame number");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Validate the END SEQ field */
		if (!R3TP_VALID_END_SEQ(r3tpMessage,
				R3TP_VER1_MESSAGE_LENGTH(numOfFrames))) {

			WcuLogError("WcuXbeeHandleNewSubscription: Invalid end sequence");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the CHECKSUM field */
		uint16_t readCrc = R3TP_READ_CRC(r3tpMessage);

		/* Clear the CHECKSUM field */
		r3tpMessage[2] = 0x00;
		r3tpMessage[3] = 0x00;

		/* Calculate the CRC */
		uint16_t calculatedCrc = WcuGetR3tpCrc(r3tpMessage,
				R3TP_VER1_MESSAGE_LENGTH(numOfFrames));

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			/* Log the error */
			WcuLogError("WcuXbeeHandleNewSubscription: Invalid CRC");
			status = EWcuRet_Error;
		}

	}

	if (EWcuRet_Ok == status) {

		uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM];
		/* Read the payload */
		uint8_t *payload = R3TP_VER1_PAYLOAD(r3tpMessage);
		for (uint32_t i = 0; i < numOfFrames; i += 1UL) {

			subscription[i] = _reinterpret32bits(payload[3UL + 4UL * i],
					payload[2UL + 4UL * i], payload[1UL + 4UL * i],
					payload[4UL * i]);

		}

		/* Set the new subscription */
		WcuCanSetNewSubscription(subscription, numOfFrames);

		/* Commit the new subscription to disk */
		status = WcuXbeeStoreNewSubscription(subscription, numOfFrames);

		/* Send the acknowledge message */
		(void) WcuXbeeSendAcknowledge(R3TP_VER1_VER_BYTE);
	}

	return status;
}

/**
 * @brief Handle the driver warning message
 * @param r3tpMessage R3TP VER2 message
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeHandleDriverWarning(uint8_t *r3tpMessage) {

	EWcuRet status = EWcuRet_Ok;

	/* Validate the END SEQ */
	if (!R3TP_VALID_END_SEQ(r3tpMessage, R3TP_VER2_FRAME_SIZE)) {

		WcuLogError("WcuXbeeHandleDriverWarning: Invalid end sequence");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Read the CHECKSUM field */
		uint16_t readCrc = R3TP_READ_CRC(r3tpMessage);

		/* Clear the CHECKSUM field */
		r3tpMessage[2] = 0x00;
		r3tpMessage[3] = 0x00;

		/* Calculate the CRC */
		uint16_t calculatedCrc = WcuGetR3tpCrc(r3tpMessage,
		R3TP_VER2_FRAME_SIZE);

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			WcuLogError("WcuXbeeHandleDriverWarning: Invalid CRC");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the payload */
		switch (r3tpMessage[4]) {

		case R3TP_GREEN_WARNING_BYTE:

			/* Set the green warning duration */
			g_GreenWarningDuration = r3tpMessage[5];
			WcuLogInfo("WcuXbeeHandleDriverWarning: Green warning set");
			break;

		case R3TP_RED_WARNING_BYTE:

			/* Set the red warning duration */
			g_RedWarningDuration = r3tpMessage[5];
			WcuLogInfo("WcuXbeeHandleDriverWarning: Red warning set");
			break;

		default:

			WcuLogInfo("WcuXbeeHandleDriverWarning: Unrecognized warning byte");
			status = EWcuRet_Error;
			break;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Send the acknowledge message */
		status = WcuXbeeSendAcknowledge(R3TP_VER2_VER_BYTE);
	}

	return status;
}

/**
 * @brief Commit the new telemetry subscription to the SD card
 * @retval EWcuRet Status
 */
static EWcuRet WcuXbeeStoreNewSubscription(uint32_t *ids, uint32_t numOfFrames) {

	EWcuRet status = EWcuRet_Ok;

	FIL subscriptionFd;

	/* Open the subscription file */
	if (EWcuRet_Ok
			!= WcuSdioFileOpen(&subscriptionFd, WCU_TELEMETRY_SUBSCRIPTION_PATH,
			FA_WRITE | FA_CREATE_ALWAYS)) {

		WcuLogError("WcuXbeeStoreNewSubscription: Open failed");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Print the number of frames to the SD card */
		if (EWcuRet_Ok
				!= WcuSdioFileWrite(&subscriptionFd, (uint8_t*) numOfFrames,
						sizeof(numOfFrames))) {

			WcuLogError("WcuXbeeStoreNewSubscription: Write failed");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Print the frames themselves to the file */
		if (EWcuRet_Ok
				!= WcuSdioFileWrite(&subscriptionFd, (uint8_t*) ids,
						sizeof(ids[0]) * numOfFrames)) {

			WcuLogError("WcuXbeeStoreNewSubscription: Write failed");
			status = EWcuRet_Error;
		}
	}

	/* Cleanup */
	(void) WcuSdioFileClose(&subscriptionFd);

	return status;
}

/**
 * @brief Send the data via UART
 * @param data Data buffer
 * @param len Length of the data buffer
 * @retval None
 */
static void WcuXbeeSendData(uint8_t *data, uint32_t len) {

	/* Write the data into the ring buffer */
	(void) UartTxRbWrite(&g_WcuXbeeTxRingBuffer, data, len);

	/* Tell the dispatcher to initiate transmission */
	(void) WcuEventSend(EWcuEventSignal_UartTxMessagePending,
			&g_WcuXbeeTxRingBuffer);
}

/**
 * @brief Message received callback
 * @retval None
 */
static void WcuXbeeRxCallback(void) {

	(void) WcuEventSend(EWcuEventSignal_XbeeRxMessagePending, NULL);
}