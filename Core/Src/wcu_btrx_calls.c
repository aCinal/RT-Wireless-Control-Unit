/**
 * @author Adrian Cinal
 * @file wcu_btrx_calls.c
 * @brief Source file defining functions called by the btRx task
 */

#include "wcu_btrx_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"
#include "rt12e_libs_uartcircularbuffer.h"

#include "cmsis_os.h"

#define BTRX_CIRCULAR_BUFSIZE  ((uint32_t) 50)  /* UART circular buffer size */

/**
 * @brief Circular buffer structure
 */
SUartCirBuf gBtRxCircularBuffer;

extern osThreadId btRxHandle;
extern osMutexId crcMutexHandle;
extern CRC_HandleTypeDef hcrc;

static void btRx_CircularBufferIdleCallback(void);

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartCirBufRet Status
 */
EUartCirBufRet btRx_StartCircularBufferIdleDetectionRx(void) {

	static uint8_t cirBufTbl[BTRX_CIRCULAR_BUFSIZE];
	/* Configure the circular buffer structure */
	gBtRxCircularBuffer.BufferPtr = cirBufTbl;
	gBtRxCircularBuffer.BufferSize = BTRX_CIRCULAR_BUFSIZE;
	gBtRxCircularBuffer.Callback = &btRx_CircularBufferIdleCallback;
	gBtRxCircularBuffer.PeriphHandlePtr = &BT_UART_HANDLE;

	/* Start listening */
	return uartCirBuf_start(&gBtRxCircularBuffer);

}


/**
 * @brief Handles the BT message
 * @retval None
 */
void btRx_HandleMessage(void) {

	/* Wait for notification from idle line detection callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, 0)) {

		static uint8_t rxBufTbl[R3TP_VER0_FRAME_SIZE];
		/* Read the data from the circular buffer */
		uartCirBuf_read(&gBtRxCircularBuffer, rxBufTbl, R3TP_VER0_FRAME_SIZE);

		/* Validate protocol version */
		if (R3TP_VER0_VER_BYTE != rxBufTbl[0]) {

			LogPrint("Invalid VER/RES/SEQ in btRx\r\n");
			return;

		}

		/* Validate end sequence */
		if ((R3TP_END_SEQ_LOW_BYTE != rxBufTbl[R3TP_VER0_FRAME_SIZE - 2U])
				|| (R3TP_END_SEQ_HIGH_BYTE
						!= rxBufTbl[R3TP_VER0_FRAME_SIZE - 1U])) {

			LogPrint("Invalid END SEQ in btRx\r\n");
			return;

		}

		uint16_t readCrc;
		/* Read the checksum - note that the checksum is transmitted as little endian */
		readCrc = _reinterpret16bits(rxBufTbl[3], rxBufTbl[2]);

		/* Clear the checksum field */
		rxBufTbl[2] = 0x00U;
		rxBufTbl[3] = 0x00U;

		uint16_t calculatedCrc;
		/* Acquire crcMutex */
		if (osOK == osMutexWait(crcMutexHandle, portMAX_DELAY)) {

			/* Calculate the CRC */
			calculatedCrc =
					_bits0_15(
							HAL_CRC_Calculate(&hcrc, (uint32_t*)rxBufTbl, R3TP_VER0_FRAME_SIZE / 4U));

			/* Release crcMutex */
			(void) osMutexRelease(crcMutexHandle);

		} else {

			/* If failed to acquire crcMutex */
			LogPrint("crcMutex timeout in btRx\r\n");
			return;

		}

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			LogPrint("Invalid CRC in btRx\r\n");
			return;

		}

		static SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */
		/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
		canFrame.UHeader.Tx.StdId = _reinterpret32bits(rxBufTbl[7], rxBufTbl[6],
				rxBufTbl[5], rxBufTbl[4]);
		/* Read the DLC */
		canFrame.UHeader.Tx.DLC = (uint32_t) rxBufTbl[8];
		/* Assert valid DLC */
		if (CAN_PAYLOAD_SIZE < canFrame.UHeader.Tx.DLC) {

			LogPrint("Invalid DLC in btRx\r\n");
			return;

		}

		/* Read the payload */
		for (uint8_t i = 0; i < canFrame.UHeader.Tx.DLC; i += 1U) {

			canFrame.PayloadTbl[i] = rxBufTbl[9U + i];

		}

		/* Configure the remaining CAN Tx header fields */
		canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
		canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

		/* Transmit the frame */
		AddToCanTxQueue(&canFrame, "btRx failed to send to canTxQueue\r\n");

	}

}

/**
 * @brief Function registered as callback for idle line callback in the circular buffer implementation
 * @retval None
 */
static void btRx_CircularBufferIdleCallback(void) {

	/* Notify the btRx task */
	vTaskNotifyGiveFromISR(btRxHandle, NULL);

}
