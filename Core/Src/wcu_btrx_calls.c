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
 * @retval EBtRxRet Status
 */
EBtRxRet btRx_HandleCom(void) {

	EBtRxRet status = EBtRxRet_Ok;

	/* Wait for notification from idle line detection callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, 0)) {

		static uint8_t rxBufTbl[R3TP_VER0_FRAME_SIZE];
		/* Read the data from the circular buffer */
		uartCirBuf_read(&gBtRxCircularBuffer, rxBufTbl, R3TP_VER0_FRAME_SIZE);

		/* Validate protocol version */
		if (R3TP_VER0_VER_BYTE != rxBufTbl[0]) {

			LogPrint("Invalid VER/RES/SEQ in btRx");
			status = EBtRxRet_Error;

		}

		if(EBtRxRet_Ok == status) {

			/* Validate end sequence */
			if ((R3TP_END_SEQ_LOW_BYTE != rxBufTbl[R3TP_VER0_FRAME_SIZE - 2U])
					|| (R3TP_END_SEQ_HIGH_BYTE
							!= rxBufTbl[R3TP_VER0_FRAME_SIZE - 1U])) {

				LogPrint("Invalid END SEQ in btRx");
				status = EBtRxRet_Error;

			}

		}


		if(EBtRxRet_Ok == status) {

			/* Read the checksum - note that the checksum is transmitted as little endian */
			uint16_t readCrc = _reinterpret16bits(rxBufTbl[3], rxBufTbl[2]);

			/* Clear the checksum field */
			rxBufTbl[2] = 0x00U;
			rxBufTbl[3] = 0x00U;

			/* Calculate the CRC */
			uint16_t calculatedCrc = GetR3tpCrc(rxBufTbl, R3TP_VER0_FRAME_SIZE);

			/* Validate the CRC */
			if (readCrc != calculatedCrc) {

				LogPrint("Invalid CRC in btRx");
				status = EBtRxRet_Error;

			}

		}

		SCanFrame canFrame;

		if(EBtRxRet_Ok == status) {

			/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
			canFrame.TxHeader.StdId = _reinterpret32bits(rxBufTbl[7], rxBufTbl[6],
					rxBufTbl[5], rxBufTbl[4]);
			/* Read the DLC */
			canFrame.TxHeader.DLC = (uint32_t) rxBufTbl[8];
			/* Assert valid DLC */
			if (CAN_PAYLOAD_SIZE < canFrame.TxHeader.DLC) {

				LogPrint("Invalid DLC in btRx");
				status = EBtRxRet_Error;

			}

		}

		if(EBtRxRet_Ok == status) {

			/* Read the payload */
			for (uint8_t i = 0; i < canFrame.TxHeader.DLC; i += 1U) {

				canFrame.PayloadTbl[i] = rxBufTbl[9U + i];

			}

			/* Configure the remaining CAN Tx header fields */
			canFrame.TxHeader.RTR = CAN_RTR_DATA;
			canFrame.TxHeader.TransmitGlobalTime = DISABLE;

			/* Transmit the frame */
			AddToCanTxQueue(&canFrame, "btRx failed to send to canTxQueue");

		}

	}

	return status;

}

/**
 * @brief Function registered as callback for idle line callback in the circular buffer implementation
 * @retval None
 */
static void btRx_CircularBufferIdleCallback(void) {

	/* Notify the btRx task */
	vTaskNotifyGiveFromISR(btRxHandle, NULL);

}
