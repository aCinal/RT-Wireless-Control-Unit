/**
 * @author Adrian Cinal
 * @file wcu_xbeerx_calls.c
 * @brief Source file defining functions called by the xbeeRx task
 */

#include "wcu_xbeerx_calls.h"
#include "wcu_basic.h"

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeRx_WaitSubscriptionFromSdioGtkp(void) {

	uint32_t frameNum; /* Number of frames in a subscription */
	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	uint32_t notificationValue; /* Buffer to pass the notification value out of the xTaskNotifyWait function */

	/* Wait for sdioGtkp to notify the task if there is a valid subscription stored on the SD card */
	if (pdTRUE
			== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &notificationValue,
			WCU_XBEERX_XTASKNOTIFYWAIT_TIMEOUT)) {

		/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue, else - an error occured */
		if (notificationValue <= 28UL) {

			frameNum = notificationValue;

			for (uint32_t i = 0; i < frameNum; i += 1UL) {

				if (pdPASS
						!= xQueueReceive(sdioSubQueueHandle, subscription + i,
						WCU_SDIOSUBQUEUE_XQUEUERECEIVE_TIMEOUT)) {

					/* Log the error and return */
					LOGERROR("xbeeRx failed to receive from sdioSubQueue\r\n");
					return;

				}

			}

			/* Set the CAN filters */
			setCanFilterList(&hcan1, subscription, frameNum);

		}

	}

}

/**
 * @brief Receives and handles the telemetry subscription via UART
 * @param buff UART Rx buffer of size R3TP_VER1_MAX_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveSubscription(uint8_t buff[]) {
	/* Receive SEQ NUM, CHECKSUM and FRAME NUM */
	if (HAL_OK != HAL_UART_Receive(&XBEE_UART_HANDLE, buff + 1U, 7,
	WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive SEQ NUM, CHECKSUM and FRAME NUM in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Read the FRAME NUM field */
	uint32_t frameNum = _join32bits(buff[7], buff[6], buff[5], buff[4]);

	/* Assert the payload won't overflow the buffer */
	if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Log the error */
		LOGERROR("Invalid FRAME NUM in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1U,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {

			__NOP();

		}
		return;

	}

	/* Receive the payload */
	if (HAL_OK
			!= HAL_UART_Receive(&XBEE_UART_HANDLE,
					R3TP_VER1_PAYLOAD_BEGIN(buff), frameNum * 4U,
					WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive the payload in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Receive the frame align bytes (two) and END SEQ (also two bytes) */
	if (HAL_OK
			!= HAL_UART_Receive(&XBEE_UART_HANDLE,
					R3TP_VER1_EPILOGUE_BEGIN(buff, frameNum), 4U,
					WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive END SEQ in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Validate the END SEQ field */
	if ((R3TP_END_SEQ_LOW_BYTE != buff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE
					!= buff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 1U])) {

		/* Log the error */
		LOGERROR("Invalid END SEQ in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1U,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {
			__NOP();
		}
		return;

	}

	/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
	uint16_t readCrc = _join16bits(buff[3], buff[2]);

	/* Clear the CHECKSUM field */
	buff[2] = 0x00U;
	buff[3] = 0x00U;

	/* Calculate the CRC */
	uint16_t calculatedCrc;
	/* Acquire crcMutex */
	if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {

		/* Calculate the CRC */
		calculatedCrc =
				_bits0_15(
						HAL_CRC_Calculate(&hcrc, (uint32_t*)buff, R3TP_VER1_MESSAGE_LENGTH(frameNum) / 4U));

		/* Release crcMutex */
		(void) osMutexRelease(crcMutexHandle);

	} else {

		/* If failed to acquire crcMutex */
		/* Log the error */
		LOGERROR("crcMutex timeout in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Validate the CRC */
	if (readCrc != calculatedCrc) {

		/* Log the error */
		LOGERROR("Invalid CRC in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {

			__NOP();

		}
		return;

	}

	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	/* Read the payload */
	for (uint32_t i = 0; i < frameNum; i += 1UL) {

		subscription[i] = _join32bits(
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 3U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 2U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 1U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 4U * i)));

	}

	/* Forward the subscription to the sdioGtkp */
	(void) xbeeRx_SendSubscriptionToSdioGtkp(subscription, frameNum);

	/* Set the CAN filters */
	setCanFilterList(&hcan1, subscription, frameNum);

}

/**
 * @brief Receives the warning for the driver via UART
 * @param buff UART Rx buffer of size R3TP_VER2_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveWarning(uint8_t buff[]) {

	/* Validate the END SEQ */
	if ((R3TP_END_SEQ_LOW_BYTE != buff[R3TP_VER2_FRAME_SIZE - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE != buff[R3TP_VER2_FRAME_SIZE - 1U])) {

		/* Log the error and return */
		LOGERROR("Invalid END SEQ in xbeeRx_UartReceiveWarning\r\n");
		return;

	}

	/* Read the CRC */
	uint16_t readCrc = _join16bits(buff[3], buff[2]);

	/* Calculate the CRC */
	uint16_t calculatedCrc;
	/* Acquire crcMutex */
	if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {

		/* Calculate the CRC */
		calculatedCrc =
				_bits0_15(
						HAL_CRC_Calculate(&hcrc, (uint32_t* )buff, R3TP_VER2_FRAME_SIZE / 4U));

		/* Release crcMutex */
		(void) osMutexRelease(crcMutexHandle);

	} else {

		/* If failed to acquire crcMutex */
		/* Log the error and return */
		LOGERROR("crcMutex timeout in xbeeRx_UartReceiveWarning\r\n");
		return;

	}

	/* Validate the CRC */
	if (readCrc != calculatedCrc) {

		/* Log the error and return */
		LOGERROR("Invalid CRC in xbeeRx_UartReceiveWarning\r\n");
		return;

	}

	/* Notify xbeeDiag */
	(void) xTaskNotify(xbeeDiagHandle,
			_join32bits(0x00, 0x00, buff[5], buff[4]), eSetValueWithOverwrite);
	return;

}

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count) {

	BaseType_t ret = pdPASS; /* Buffer for xQueueSend return value */

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		ret = xQueueSend(sdioSubQueueHandle, ids + i,
				WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT);
		/* Send the frame to the queue */
		if (pdPASS != ret) {

			/* Log the error */
			LOGERROR("xbeeRx failed to send to sdioSubQueue\r\n");
			/* Cleanup */
			(void) xQueueReset(sdioSubQueueHandle);
			return ret;

		}

	}

	/* Notify sdioGatekeeper */
	(void) xTaskNotify(sdioGtkpHandle, count, eSetValueWithOverwrite);

	return ret;
}
