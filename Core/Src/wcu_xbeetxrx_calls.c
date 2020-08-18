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
#include "rt12e_libs_uartcircularbuffer.h"
#include <string.h>

/**
 * @brief Circular buffer structure
 */
SUartCircularBuffer gXbeeTxRxCircularBuffer;

/**
 * @brief Telemetry diagnostics structure
 */
typedef struct STelemetryDiagnostics {

	bool greenWarningActive; /* Fleg set when the green warning is active */

	uint8_t greenWarningDuration; /* Buffer for the green warning's duration */

	bool redWarningActive; /* Flag set when the red warning is active */

	uint8_t redWarningDuration; /* Buffer for the red warning's duration */

	uint8_t rssi; /* Buffer for the RSSI value */

} STelemetryDiagnostics;

#define CIRCULAR_BUFFER_SIZE			2 * R3TP_MAX_FRAME_SIZE	/* UART circular buffer size */
#define CAN_ID_TELEMETRY_DIAG			(uint32_t)(0x733UL)		/* CAN ID: _733_TELEMETRY_DIAG */
#define TELEMETRY_STATE_BIT				(uint8_t)(0x80U)		/* Telemetry_State bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_WARNING_BIT			(uint8_t)(0x40U)		/* Telemetry_Warning bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_PIT_BIT				(uint8_t)(0x20U)		/* Telemetry_Pit bit of the TELEMETRY_DIAG CAN frame */
#define XBEE_GT_DEFAULT					(uint16_t)(0x0CE4U)		/* XBEE Pro Guard Times default value */
#define XBEE_GT_DESIRED					(uint16_t)(0x000AU)		/* XBEE Pro Guard Times desired value */

/**
 * @brief Returns the length of the string not counting the NULL character
 */
#define StaticStrlen(str) (uint32_t)(sizeof(str) - 1U)

/**
 * @brief Guard Times parameter of XBEE Pro
 */
static uint16_t gGT = XBEE_GT_DEFAULT;

static void xbeeTxRx_CircularBufferIdleCallback(void);
static void xbeeTxRx_HandleNewSubscription(uint8_t rxBuffTable[]);
static void xbeeTxRx_HandleDriverWarning(uint8_t rxBuffTable[], STelemetryDiagnostics *diagnosticsPtr);
static void xbeeTxRx_SendDiagnostics(STelemetryDiagnostics *diagnosticsPtr);
static void xbeeTxRx_UpdateWarnings(STelemetryDiagnostics *diagnosticsPtr);
static void xbeeTxRx_PollForRssi(uint8_t *rssiPtr);
static BaseType_t xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[], size_t count);
static BaseType_t xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count);
void Error_Handler(void);

/**
 * @brief Configures the XBEE Pro device
 * @retval None
 */
void xbeeTxRx_DeviceConfig(void) {

	/* Wait the default guard time */
	vTaskDelay(pdMS_TO_TICKS(XBEE_GT_DEFAULT));

	/* Enter command mode */
	const uint8_t ENTER_COMMAND_MODE[] = "+++";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE,
					(uint8_t*) ENTER_COMMAND_MODE,
					StaticStrlen(ENTER_COMMAND_MODE),
					WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();
		return;

	}

	/* Wait the default guard time */
	vTaskDelay(pdMS_TO_TICKS(XBEE_GT_DEFAULT));

	/* Set required period of silence before and after the Command Sequence Characters */
	const uint8_t SET_GT[] = "ATGT000A\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE, (uint8_t*) SET_GT,
					StaticStrlen(SET_GT), WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();

	}

	/* Exit command mode */
	const uint8_t EXIT_COMMAND_MODE[] = "ATCN\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE,
					(uint8_t*) EXIT_COMMAND_MODE,
					(sizeof(EXIT_COMMAND_MODE) - 1U),
					WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();

	}

	/* Set the Guard Times global variable */
	/* The desired value is hard-coded because it is also contained in the SET_GL command string */
	gGT = 0x000AU;

}

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus xbeeTxRx_StartCircularBufferIdleDetectionRx(void) {

	static uint8_t buff[CIRCULAR_BUFFER_SIZE]; /* Circular buffer */

	/* Configure the circular buffer structure */
	gXbeeTxRxCircularBuffer.BufferPtr = buff;
	gXbeeTxRxCircularBuffer.BufferSize = CIRCULAR_BUFFER_SIZE;
	gXbeeTxRxCircularBuffer.Callback = &xbeeTxRx_CircularBufferIdleCallback;
	gXbeeTxRxCircularBuffer.PeriphHandlePtr = &XBEE_UART_HANDLE;

	/* Start listening */
	return uartCircularBuffer_start(&gXbeeTxRxCircularBuffer);

}

/**
 * @brief Handles internal messages
 * @retval None
 */
void xbeeTxRx_HandleInternalMail(void) {

	static uint8_t rxBuffTable[R3TP_MAX_FRAME_SIZE]; /* UART Rx buffer */
	static EXbeeInternalMail mail; /* Internal mail buffer */

	/* Wait for messages */
	if (pdPASS == xQueueReceive(xbeeInternalMailQueueHandle, &mail, 0)) {

		static STelemetryDiagnostics diagnostics;

		switch (mail) {

		case EXbeeInternalMail_MessageReceived:

			/* Read the data from the circular buffer */
			(void) uartCircularBuffer_read(&gXbeeTxRxCircularBuffer, rxBuffTable, R3TP_MAX_FRAME_SIZE);

			/* Identify the protocol version */
			switch(rxBuffTable[0]) {

			case R3TP_VER1_VER_BYTE:

				xbeeTxRx_HandleNewSubscription(rxBuffTable);
				break;

			case R3TP_VER2_VER_BYTE:

				xbeeTxRx_HandleDriverWarning(rxBuffTable, &diagnostics);
				break;

			default:

				LOGERROR("Invalid protocol version in xbeeTxRx\r\n");
				break;

			}

			/* Poll the device for the RSSI value */
			xbeeTxRx_PollForRssi(&diagnostics.rssi);

		case EXbeeInternalMail_PeriodElapsed:

			/* Transmit the diagnostic frame */
			xbeeTxRx_SendDiagnostics(&diagnostics);
			/* Update the diagnostics structure */
			xbeeTxRx_UpdateWarnings(&diagnostics);
			break;

		default:

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

		static uint8_t txBuff[R3TP_VER0_FRAME_SIZE ]; /* UART Tx buffer */
		/* Clear the buffer */
		(void) memset(txBuff, 0x00, R3TP_VER0_FRAME_SIZE);

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
 * @brief Function registered as callback for idle line callback in the circular buffer implementation
 * @retval None
 */
static void xbeeTxRx_CircularBufferIdleCallback(void) {

	/* Notify the xbeeTxRx task */
	EXbeeInternalMail mail = EXbeeInternalMail_MessageReceived;
	(void) xQueueSendFromISR(xbeeInternalMailQueueHandle, &mail, NULL);

}

/**
 * @brief Handles the new telemetry subscription
 * @param rxBuffTable UART Rx Buffer
 * @retval None
 */
static void xbeeTxRx_HandleNewSubscription(uint8_t rxBuffTable[]) {

	/* Read the number of frames in the payload */
	uint32_t frameNum = _join32bits(rxBuffTable[7], rxBuffTable[6],
			rxBuffTable[5], rxBuffTable[4]);

	/* Assert the payload won't overflow the buffer */
	if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Log the error */
		LOGERROR("Invalid FRAME NUM in xbeeTxRx\r\n");
		return;

	}

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
static void xbeeTxRx_HandleDriverWarning(uint8_t rxBuffTable[],
		STelemetryDiagnostics *diagnosticsPtr) {

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
	switch (rxBuffTable[4]) {

	case R3TP_GREEN_WARNING_BYTE :

		/* Set the green warning */
		diagnosticsPtr->greenWarningActive = true;
		diagnosticsPtr->greenWarningDuration = rxBuffTable[5];
		break;

	case R3TP_RED_WARNING_BYTE :

		/* Set the red warning */
		diagnosticsPtr->redWarningActive = true;
		diagnosticsPtr->redWarningDuration = rxBuffTable[5];
		break;

	default:

		break;

	}

}

/**
 * @brief Polls the XBEE Pro device for the RSSI value of the last transmission received
 * @param[out] rssiPtr Address where the received RSSI value will be stored
 * @retval None
 */
static void xbeeTxRx_PollForRssi(uint8_t *rssiPtr) {

	/* Wait the guard time */
	vTaskDelay(pdMS_TO_TICKS(gGT));

	/* Stop the data to the circular buffer */
	uartCircularBuffer_stop(&gXbeeTxRxCircularBuffer);

	/* Enter command mode */
	const uint8_t ENTER_COMMAND_MODE[] = "+++";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE,
					(uint8_t*) ENTER_COMMAND_MODE,
					StaticStrlen(ENTER_COMMAND_MODE),
					WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();
		return;

	}

	/* Wait the guard time */
	vTaskDelay(pdMS_TO_TICKS(gGT));

	/* Request RSSI */
	const uint8_t REQUEST_RSSI[] = "ATDB\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE, (uint8_t*) REQUEST_RSSI,
					StaticStrlen(REQUEST_RSSI), WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();
		return;

	}

	/* Receive RSSI */
	if (HAL_OK != HAL_UART_Receive(&XBEE_UART_HANDLE, rssiPtr, 1,
	WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();

	}

	/* Exit command mode */
	const uint8_t EXIT_COMMAND_MODE[] = "ATCN\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEE_UART_HANDLE,
					(uint8_t*) EXIT_COMMAND_MODE,
					StaticStrlen(EXIT_COMMAND_MODE),
					WCU_DEFAULT_TIMEOUT)) {

		Error_Handler();

	}

	/* Resume the data to the circular buffer */
	uartCircularBuffer_start(&gXbeeTxRxCircularBuffer);

}

/**
 * @brief Sends the telemetry diagnostic frame to the CAN bus
 * @param diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
static void xbeeTxRx_SendDiagnostics(STelemetryDiagnostics *diagnosticsPtr) {

	SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.UHeader.Tx.DLC = 2;
	canFrame.UHeader.Tx.IDE = CAN_ID_STD;
	canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
	canFrame.UHeader.Tx.StdId = CAN_ID_TELEMETRY_DIAG;
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
static void xbeeTxRx_UpdateWarnings(STelemetryDiagnostics *diagnosticsPtr) {

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

			/* Deactivate the red warning */
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
static BaseType_t xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[], size_t count) {

	BaseType_t ret = pdPASS; /* Buffer for xQueueSend return value */

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		ret = xQueueSend(canSubQueueHandle, ids + i, 0);
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
static BaseType_t xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count) {

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
