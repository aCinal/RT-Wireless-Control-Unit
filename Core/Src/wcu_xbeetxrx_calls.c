/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.c
 * @brief Source file defining functions called by the xbeeTxRx task
 */

#include "wcu_xbeetxrx_calls.h"
#include "wcu_base.h"
#include "main.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"
#include <string.h>

/**
 * @brief Configures the XBEE Pro device
 * @retval None
 */
void xbeeTxRx_DeviceConfig(void) {

	/*
	 * TODO
	 */

}

/**
 * @brief Handles internal messages
 * @param rxBuffTable UART Rx Buffer
 * @retval None
 */
void xbeeTxRx_HandleInternalMail(uint8_t rxBuffTable[]) {

	EXbeeInternalMail mail;
	/* Wait for messages */
	if (pdPASS == xQueueReceive(xbeeInternalMailQueueHandle, &mail, 0)) {

		static SXbeeDiagnostics diagnostics;

		uint32_t ver1FrameNum; /* Buffer for the number of frames in an incoming subscription */

		switch (mail) {

		case XBEE_INTERNAL_R3TP_VER1_HEADER_RECEIVED:

			/* Read the number of frames in the payload */
			ver1FrameNum = _join32bits(rxBuffTable[7], rxBuffTable[6],
					rxBuffTable[5], rxBuffTable[4]);

			/* Assert the payload won't overflow the buffer */
			if (ver1FrameNum > R3TP_VER1_MAX_FRAME_NUM) {

				/* Log the error */
				LOGERROR("Invalid FRAME NUM in xbeeTxRx\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				IGNORE_REST_OF_THE_MESSAGE();
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, rxBuffTable,
				R3TP_HEADER_SIZE);
				break;

			}

			/* Listen for the rest of the message */
			(void) HAL_UART_Receive_DMA(&XBEE_UART_HANDLE,
					R3TP_VER1_PAYLOAD_BEGIN(rxBuffTable),
					(R3TP_VER1_MESSAGE_LENGTH(
							ver1FrameNum) - 8));
			break;

		case XBEE_INTERNAL_R3TP_VER1_MESSAGE_RECEIVED:

			/* Handle the new subscription */
			xbeeTxRx_HandleNewSubscription(rxBuffTable);

			/* Poll the device for the RSSI value */
			xbeeTxRx_PollForRssi(&diagnostics.rssi);

			/* Listen for the next message */
			(void) HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, rxBuffTable,
			R3TP_HEADER_SIZE);
			break;

		case XBEE_INTERNAL_R3TP_VER2_MESSAGE_RECEIVED:

			/* Handle the driver warning */
			xbeeTxRx_HandleDriverWarning(rxBuffTable, &diagnostics);

			/* Poll the device for the RSSI value */
			xbeeTxRx_PollForRssi(&diagnostics.rssi);

			/* Listen for the next message */
			(void) HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, rxBuffTable,
			R3TP_HEADER_SIZE);
			break;

		case XBEE_INTERNAL_PERIOD_ELAPSED:

			/* Transmit the diagnostic frame */
			xbeeTxRx_SendDiagnostics(&diagnostics);
			/* Update the diagnostics structure */
			xbeeTxRx_UpdateWarnings(&diagnostics);
			break;

		case XBEE_INTERNAL_UNKNOWN_PROTOCOL:

			/* Log the error */
			LOGERROR("Invalid VER byte in xbeeTxRx\r\n");
			/* Assert the invalid message won't raise any more interrupts */
			IGNORE_REST_OF_THE_MESSAGE();
			break;

		}

	}

}

/**
 * @brief Handles transmitting telemetry data
 * @retval None
 */
void xbeeTxRx_HandleOutgoingR3tpComms(void) {

	SCanFrame frameBuff; /* Buffer for the CAN frame */

	/* Listen on the canRxQueue for messages to send */
	if (pdPASS == xQueueReceive(canRxQueueHandle, &frameBuff, 0)) {

		/* Assert valid data direction */
		if (RX != frameBuff.EDataDirection) {

			/* Log the error */
			LOGERROR("Invalid DataDirection in xbeeTxRx\r\n");
			return;

		}

		static uint8_t txBuff[R3TP_VER0_FRAME_SIZE ]; /* UART Tx buffer */
		/* Clear the buffer */
		(void) memset(txBuff, 0x00U, R3TP_VER0_FRAME_SIZE);

		/* Set VER and RES/SEQ field */
		txBuff[0] = R3TP_VER0_VER_BYTE;

		static uint8_t seqNum = 0; /* Sequence number */
		/* Set the SEQ NUM field */
		txBuff[1] = seqNum;
		/* Increment the sequence number */
		seqNum = (seqNum < 255U) ? seqNum + 1U : 0;

		/* Set the END SEQ field */
		txBuff[R3TP_VER0_FRAME_SIZE - 2U] = R3TP_END_SEQ_LOW_BYTE;
		txBuff[R3TP_VER0_FRAME_SIZE - 1U] = R3TP_END_SEQ_HIGH_BYTE;

		/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
		txBuff[4] = _bits0_7(frameBuff.UHeader.Rx.StdId);
		txBuff[5] = _bits8_15(frameBuff.UHeader.Rx.StdId);

		/* Set the DLC field */
		txBuff[8] = (uint8_t) frameBuff.UHeader.Rx.DLC;

		/* Set the DATA field */
		for (uint8_t i = 0; i < frameBuff.UHeader.Rx.DLC; i += 1U) {

			txBuff[9U + i] = frameBuff.PayloadTable[i];

		}

		/* Calculate the CRC */
		uint16_t calculatedCrc;
		/* Acquire crcMutex */
		if (osOK == osMutexWait(crcMutexHandle, WCU_DEFAULT_TIMEOUT)) {

			/* Calculate the CRC */
			calculatedCrc =
					_bits0_15(
							HAL_CRC_Calculate(&hcrc, (uint32_t*) txBuff, R3TP_VER0_FRAME_SIZE / 4U));
			/* Release crcMutex */
			(void) osMutexRelease(crcMutexHandle);

		} else {

			/* If failed to acquire crcMutex */
			/* Log the error */
			LOGERROR("crcMutex timeout in xbeeTxRx\r\n");
			return;

		}

		/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
		txBuff[2] = _bits0_7(calculatedCrc);
		txBuff[3] = _bits8_15(calculatedCrc);

		/* Transmit the frame */
		(void) HAL_UART_Transmit(&XBEE_UART_HANDLE, txBuff,
		R3TP_VER0_FRAME_SIZE, WCU_DEFAULT_TIMEOUT);

	}

}

/**
 * @brief Handles the new telemetry subscription
 * @param rxBuffTable UART Rx Buffer
 * @retval None
 */
void xbeeTxRx_HandleNewSubscription(uint8_t rxBuffTable[]) {

	/* Read the number of frames in the payload */
	uint32_t frameNum = _join32bits(rxBuffTable[7], rxBuffTable[6],
			rxBuffTable[5], rxBuffTable[4]);

	/* Validate the END SEQ field */
	if ((R3TP_END_SEQ_LOW_BYTE
			!= rxBuffTable[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE
					!= rxBuffTable[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 1U])) {

		/* Log the error */
		LOGERROR("Invalid END SEQ in xbeeTxRx\r\n");
		return;

	}

	/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
	uint16_t readCrc = _join16bits(rxBuffTable[3], rxBuffTable[2]);

	/* Clear the CHECKSUM field */
	rxBuffTable[2] = 0x00U;
	rxBuffTable[3] = 0x00U;

	/* Calculate the CRC */
	uint16_t calculatedCrc;
	/* Acquire crcMutex */
	if (osOK == osMutexWait(crcMutexHandle, portMAX_DELAY)) {

		/* Calculate the CRC */
		calculatedCrc =
				_bits0_15(
						HAL_CRC_Calculate(&hcrc, (uint32_t*) rxBuffTable, R3TP_VER1_MESSAGE_LENGTH(frameNum) / 4U));

		/* Release crcMutex */
		(void) osMutexRelease(crcMutexHandle);

	} else {

		/* If failed to acquire crcMutex */
		/* Log the error */
		LOGERROR("crcMutex timeout in xbeeTxRx\r\n");
		return;

	}

	/* Validate the CRC */
	if (readCrc != calculatedCrc) {

		/* Log the error */
		LOGERROR("Invalid CRC in xbeeTxRx\r\n");
		return;

	}

	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM ]; /* Buffer for telemetry subscription CAN IDs */
	/* Read the payload */
	for (uint32_t i = 0; i < frameNum; i += 1UL) {

		subscription[i] = _join32bits(
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(rxBuffTable, 3U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(rxBuffTable, 2U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(rxBuffTable, 1U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(rxBuffTable, 4U * i)));

	}

	/* Forward the subscription to the canGtkp task */
	(void) xbeeTxRx_SendSubscriptionToCanGtkp(subscription, frameNum);

	/* Forward the subscription to the sdioGtkp task */
	(void) xbeeTxRx_SendSubscriptionToSdioGtkp(subscription, frameNum);

}

/**
 * @brief Handles the driver warning
 * @param[in] rxBuffTable UART Rx Buffer
 * @param[out] diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
void xbeeTxRx_HandleDriverWarning(uint8_t rxBuffTable[],
		SXbeeDiagnostics *diagnosticsPtr) {

	/* Validate the END SEQ */
	if ((R3TP_END_SEQ_LOW_BYTE != rxBuffTable[R3TP_VER2_FRAME_SIZE - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE != rxBuffTable[R3TP_VER2_FRAME_SIZE - 1U])) {

		/* Log the error and return */
		LOGERROR("Invalid END SEQ in xbeeTxRx\r\n");
		return;

	}

	/* Read the CRC */
	uint16_t readCrc = _join16bits(rxBuffTable[3], rxBuffTable[2]);

	/* Calculate the CRC */
	uint16_t calculatedCrc;
	/* Acquire crcMutex */
	if (osOK == osMutexWait(crcMutexHandle, portMAX_DELAY)) {

		/* Calculate the CRC */
		calculatedCrc =
				_bits0_15(
						HAL_CRC_Calculate(&hcrc, (uint32_t*) rxBuffTable, R3TP_VER2_FRAME_SIZE / 4U));

		/* Release crcMutex */
		(void) osMutexRelease(crcMutexHandle);

	} else {

		/* If failed to acquire crcMutex */
		/* Log the error and return */
		LOGERROR("crcMutex timeout in xbeeTxRx\r\n");
		return;

	}

	/* Validate the CRC */
	if (readCrc != calculatedCrc) {

		/* Log the error and return */
		LOGERROR("Invalid CRC in xbeeTxRx\r\n");
		return;

	}

	/* Read the payload */
	switch(rxBuffTable[4]) {

	case R3TP_GREEN_WARNING_BYTE:

		/* Set the green warning */
		diagnosticsPtr->greenWarningActive = true;
		diagnosticsPtr->greenWarningDuration = rxBuffTable[5];
		break;

	case R3TP_RED_WARNING_BYTE:

		/* Set the red warning */
		diagnosticsPtr->redWarningActive = true;
		diagnosticsPtr->redWarningDuration = rxBuffTable[5];
		break;

	}

}

/**
 * @brief Polls the XBEE Pro device for the RSSI value of the last transmission received
 * @param[out] rssiPtr Address where the received RSSI value will be stored
 * @retval None
 */
void xbeeTxRx_PollForRssi(uint8_t *rssiPtr) {

	/*
	 * TODO
	 */

}

/**
 * @brief Sends the telemetry diagnostic frame to the CAN bus
 * @param diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
void xbeeTxRx_SendDiagnostics(SXbeeDiagnostics *diagnosticsPtr) {

	SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.UHeader.Tx.DLC = 2;
	canFrame.UHeader.Tx.IDE = CAN_ID_STD;
	canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
	canFrame.UHeader.Tx.StdId = WCU_CAN_ID_TELEMETRY_DIAG;
	canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

	/* Write the RSSI to the frame payload */
	canFrame.PayloadTable[0] = diagnosticsPtr->rssi;

	/* Clear the status flags */
	canFrame.PayloadTable[1] = 0x00;

	/* Poll the XBEE_STATUS */
	if (GPIO_PIN_SET
			== HAL_GPIO_ReadPin(XBEE_STATUS_GPIO_Port, XBEE_STATUS_Pin)) {

		/* Set the Telemetry_State flag */
		SET_BIT(canFrame.PayloadTable[1], TELEMETRY_STATE_BIT);

	}

	/* Test if the green warning flag is set */
	if (diagnosticsPtr->greenWarningActive) {

		/* Set the Telemetry_Pit flag */
		SET_BIT(canFrame.PayloadTable[1], TELEMETRY_PIT_BIT);

	}

	/* Test if the red warning flag is set */
	if (diagnosticsPtr->redWarningActive) {

		/* Set the Telemetry_Warning flag */
		SET_BIT(canFrame.PayloadTable[1], TELEMETRY_WARNING_BIT);

	}

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame, "xbeeTxRx failed to send to canTxQueue\r\n");

}

/**
 * @brief Decrements the warning duration counters and updates the warning active flags
 * @param diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
void xbeeTxRx_UpdateWarnings(SXbeeDiagnostics *diagnosticsPtr) {

	/* Test if the green warning is active */
	if (diagnosticsPtr->greenWarningActive) {

		/* Decrement the warning duration counter */
		diagnosticsPtr->greenWarningDuration -= 1U;

		/* Test if the remaining duration is zero */
		if (0U == diagnosticsPtr->greenWarningDuration) {

			/* Deactivate the green warning */
			diagnosticsPtr->greenWarningActive = false;

		}

	}

	/* Test if the red warning is active */
	if (diagnosticsPtr->redWarningActive) {

		/* Decrement the warning duration counter */
		diagnosticsPtr->redWarningDuration -= 1U;

		/* Test if the remaining duration is zero */
		if (0U == diagnosticsPtr->redWarningDuration) {

			/* Deactivate the green warning */
			diagnosticsPtr->redWarningActive = false;

		}

	}

}

/**
 * @brief Forwards the telemetry subscription to the CAN gatekeeper for the appropriate filters to be set
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[], size_t count) {

	BaseType_t ret = pdPASS; /* Buffer for xQueueSend return value */

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		ret = xQueueSend(sdioSubQueueHandle, ids + i, 0);
		/* Send the frame to the queue */
		if (pdPASS != ret) {

			/* Log the error */
			LOGERROR("xbeeTxRx failed to send to canSubQueue\r\n");
			/* Cleanup */
			(void) xQueueReset(canSubQueueHandle);
			return ret;

		}

	}

	/* Notify sdioGatekeeper */
	(void) xTaskNotify(canGtkpHandle, count, eSetValueWithOverwrite);

	return ret;

}

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count) {

	BaseType_t ret = pdPASS; /* Buffer for xQueueSend return value */

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		ret = xQueueSend(sdioSubQueueHandle, ids + i, 0);
		/* Send the frame to the queue */
		if (pdPASS != ret) {

			/* Log the error */
			LOGERROR("xbeeTxRx failed to send to sdioSubQueue\r\n");
			/* Cleanup */
			(void) xQueueReset(sdioSubQueueHandle);
			return ret;

		}

	}

	/* Notify sdioGatekeeper */
	(void) xTaskNotify(sdioGtkpHandle, count, eSetValueWithOverwrite);

	return ret;

}
