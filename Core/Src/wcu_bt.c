/**
 * @author Adrian Cinal
 * @file wcu_bt.h
 * @brief Source file implementing the BT service
 */

#include "wcu_bt.h"
#include "wcu_defs.h"
#include "wcu_wrappers.h"
#include "wcu_events.h"
#include "wcu_logger.h"
#include "wcu_can.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_uartringbuffer.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "stm32f4xx_hal.h"

#define WCU_BT_RING_BUFFER_SIZE    ( (uint32_t) 50 )     /* UART ring buffer size */

extern UART_HandleTypeDef huart1;
SUartRb g_WcuBtRingBuffer;

static EWcuRet WcuBtRingBufferInit(void);
static EWcuRet WcuBtForwardWdtsMessageToCan(void);
static void WcuBtRxCallback(void);

/**
 * @brief BT service startup
 * @retval None
 */
void WcuBtStartup(void) {

	/* Initialize the ring buffer */
	if (EWcuRet_Ok == WcuBtRingBufferInit()) {

		WcuLogInfo("WcuBtStartup: BT ring buffer initialized");

	} else {

		WcuLogError("WcuBtStartup: BT ring buffer initialization failed");
	}

}

/**
 * @brief Handle the pending message
 * @retval None
 */
void WcuBtHandlePendingMessage(void) {

	(void) WcuBtForwardWdtsMessageToCan();
}

/**
 * @brief Initialize the BT ring buffer
 * @retval EWcuRet Status
 */
static EWcuRet WcuBtRingBufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_BT_RING_BUFFER_SIZE];

	/* Configure the ring buffer structure */
	(void) UartRbInit(&g_WcuBtRingBuffer, &huart1, ringbuffer, sizeof(ringbuffer), &WcuBtRxCallback);

	/* Start listening */
	if (EUartRbRet_Ok != UartRbStart(&g_WcuBtRingBuffer)) {

		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief Forward the WDTS message to the CAN bus
 * @retval EWcuRet Status
 */
static EWcuRet WcuBtForwardWdtsMessageToCan(void) {

	EWcuRet status = EWcuRet_Ok;
	uint8_t buffer[R3TP_VER0_FRAME_SIZE];
	size_t bytesRead = 0;
	/* Read the message from the ring buffer */
	if (EUartRbRet_Ok != UartRbRead(&g_WcuBtRingBuffer, buffer, sizeof(buffer), &bytesRead)) {

		WcuLogError("WcuBtForwardWdtsMessageToCan: Ring buffer read failed");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid number of bytes received */
		if (R3TP_VER0_FRAME_SIZE != bytesRead) {

			WcuLogError("WcuBtForwardWdtsMessageToCan: Invalid number of bytes received");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid protocol version used */
		if (R3TP_VER0_VER_BYTE == R3TP_PROTOCOL_VERSION(buffer)) {

			WcuLogError("WcuBtForwardWdtsMessageToCan: Unknown protocol version");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Validate end sequence */
		if (!R3TP_VALID_END_SEQ(buffer, R3TP_VER0_FRAME_SIZE)) {

			WcuLogError("WcuBtForwardWdtsMessageToCan: Invalid end sequence");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the checksum */
		uint16_t readCrc = R3TP_READ_CRC(buffer);

		/* Clear the checksum field */
		buffer[2] = 0;
		buffer[3] = 0;

		/* Calculate the CRC */
		uint16_t calculatedCrc = WcuGetR3tpCrc(buffer, R3TP_VER0_FRAME_SIZE);

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			WcuLogError("WcuBtForwardWdtsMessageToCan: Invalid CRC");
			status = EWcuRet_Error;
		}
	}

	SCanFrame canMessage;

	if (EWcuRet_Ok == status) {

		/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
		canMessage.TxHeader.StdId = _reinterpret32bits(buffer[7],
				buffer[6], buffer[5], buffer[4]);
		/* Read the DLC */
		canMessage.TxHeader.DLC = (uint32_t) buffer[8];
		/* Assert valid DLC */
		if (CAN_PAYLOAD_SIZE < canMessage.TxHeader.DLC) {

			WcuLogError("WcuBtForwardWdtsMessageToCan: Invalid DLC");
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the payload */
		for (uint8_t i = 0; i < canMessage.TxHeader.DLC; i += 1U) {

			canMessage.PayloadTbl[i] = buffer[9U + i];
		}

		/* Configure the remaining CAN Tx header fields */
		canMessage.TxHeader.RTR = CAN_RTR_DATA;
		canMessage.TxHeader.TransmitGlobalTime = DISABLE;

		/* Transmit the frame */
		status = WcuCanMessageSend(&canMessage);
	}

	return status;
}

/**
 * @brief Message received callback
 * @retval None
 */
static void WcuBtRxCallback(void) {

	(void) WcuEventSend(EWcuEventSignal_BtPendingMessage, NULL);
}
