/**
 * @author Adrian Cinal
 * @file wcu_bt.h
 * @brief Source file implementing the BT service
 */

#include <wcu_utils.h>
#include "wcu_bt.h"
#include "wcu_defs.h"
#include "wcu_events.h"
#include "wcu_logger.h"
#include "wcu_can.h"
#include "wcu_diagnostics.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_uartringbuffer_rx.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "stm32f4xx_hal.h"

#define WCU_BT_RING_BUFFER_SIZE    ( (uint32_t) 256 )  /* UART ring buffer size */

extern UART_HandleTypeDef huart1;
SUartRxRb g_WcuBtRxRingbuffer;

static inline EWcuRet WcuBtRingbufferInit(void);
static inline EWcuRet WcuBtForwardWdtsMessageToCan(void);
static void WcuBtRxCallback(void);

/**
 * @brief BT service startup
 * @retval None
 */
void WcuBtStartup(void) {

	/* Initialize the ring buffer */
	if (EWcuRet_Ok == WcuBtRingbufferInit()) {

		WcuLogInfo("%s(): BT ring buffer initialized", __FUNCTION__);

	} else {

		WcuLogError("%s(): BT ring buffer initialization failed", __FUNCTION__);
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
static inline EWcuRet WcuBtRingbufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_BT_RING_BUFFER_SIZE];

	/* Configure the ring buffer structure */
	(void) UartRxRbInit(&g_WcuBtRxRingbuffer, &huart1, ringbuffer,
			sizeof(ringbuffer), &WcuBtRxCallback);

	/* Start listening */
	if (EUartRxRbRet_Ok != UartRxRbRecv(&g_WcuBtRxRingbuffer)) {

		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief Forward the WDTS message to the CAN bus
 * @retval EWcuRet Status
 */
static inline EWcuRet WcuBtForwardWdtsMessageToCan(void) {

	EWcuRet status = EWcuRet_Ok;
	SCanMessage canMessage;

	uint8_t buffer[R3TP_VER0_FRAME_SIZE];
	size_t bytesRead = 0;
	/* Read the message from the ring buffer */
	if (EUartRxRbRet_Ok
			!= UartRxRbRead(&g_WcuBtRxRingbuffer, buffer, sizeof(buffer),
					&bytesRead)) {

		WcuLogError("%s(): Ring buffer read failed", __FUNCTION__);
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid number of bytes received */
		if (R3TP_VER0_FRAME_SIZE != bytesRead) {

			WcuLogError(
					"%s(): Invalid number of bytes received: %d (expected: %d)",
					__FUNCTION__, bytesRead, R3TP_VER0_FRAME_SIZE);
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtInvalidMessagesReceived);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Assert valid protocol version used */
		if (R3TP_VER0_VER_BYTE != R3TP_PROTOCOL_VERSION(buffer)) {

			WcuLogError("%s(): Unknown protocol version: 0x%02X", __FUNCTION__,
					R3TP_PROTOCOL_VERSION(buffer));
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtInvalidMessagesReceived);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Validate end sequence */
		if (!R3TP_VALID_END_SEQ(buffer, R3TP_VER0_FRAME_SIZE)) {

			WcuLogError("%s(): Invalid end sequence", __FUNCTION__);
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtInvalidMessagesReceived);
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

			WcuLogError("%s(): Invalid CRC (read: 0x%04X, calculated: 0x%04X)",
					__FUNCTION__, readCrc, calculatedCrc);
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtInvalidMessagesReceived);
			status = EWcuRet_Error;
		}
	}

	if (EWcuRet_Ok == status) {

		/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
		canMessage.TxHeader.StdId = _reinterpret32bitsle(buffer[4], buffer[5],
				buffer[6], buffer[7]);
		/* Read the DLC */
		canMessage.TxHeader.DLC = (uint32_t) buffer[8];
		/* Assert valid DLC */
		if (CAN_MAX_PAYLOAD_SIZE < canMessage.TxHeader.DLC) {

			WcuLogError("%s(): Invalid DLC: %d", __FUNCTION__, canMessage.TxHeader.DLC);
			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtInvalidMessagesReceived);
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

		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(BtMessagesForwarded);
	}

	return status;
}

/**
 * @brief Message received callback
 * @retval None
 */
static void WcuBtRxCallback(void) {

	(void) WcuEventSend(EWcuEventType_BtRxMessagePending, NULL, 0);
}
