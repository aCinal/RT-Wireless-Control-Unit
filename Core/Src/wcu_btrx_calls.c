/**
 * @author Adrian Cinal
 * @file wcu_btrx_calls.c
 * @brief Source file defining functions called by the btRx task
 */

#include "wcu_btrx_calls.h"
#include "wcu_base.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"


/**
 * @brief Listens for and handles the BT message
 * @retval None
 */
void btRx_HandleMessage(void) {

	static SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	static uint8_t uartRxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Rx buffer */
	/* Listen for the message */
	(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
	R3TP_VER0_FRAME_SIZE);

	/* Wait for notification from ISR/message received callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, 0)) {

		/* Validate the VER */
		if (R3TP_VER0_VER_BYTE != uartRxBuff[0]) {

			/* Log the error */
			LOGERROR("Invalid VER/RES/SEQ in btRx\r\n");
			return;

		}

		/* Validate the END SEQ */
		if ((R3TP_END_SEQ_LOW_BYTE != uartRxBuff[R3TP_VER0_FRAME_SIZE - 2U])
				|| (R3TP_END_SEQ_HIGH_BYTE
						!= uartRxBuff[R3TP_VER0_FRAME_SIZE - 1U])) {

			/* Log the error */
			LOGERROR("Invalid END SEQ in btRx\r\n");
			return;

		}

		static uint16_t readCrc; /* Buffer for the transmitted CRC */
		/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
		readCrc = _join16bits(uartRxBuff[3], uartRxBuff[2]);

		/* Clear the CHECKSUM field */
		uartRxBuff[2] = 0x00U;
		uartRxBuff[3] = 0x00U;

		/* Calculate the CRC */
		uint16_t calculatedCrc;
		/* Acquire crcMutex */
		if (osOK == osMutexWait(crcMutexHandle, portMAX_DELAY)) {

			/* Calculate the CRC */
			calculatedCrc =
					_bits0_15(
							HAL_CRC_Calculate(&hcrc, (uint32_t* )uartRxBuff, R3TP_VER0_FRAME_SIZE / 4U));

			/* Release crcMutex */
			(void) osMutexRelease(crcMutexHandle);

		} else {

			/* If failed to acquire crcMutex */
			/* Log the error */
			LOGERROR("crcMutex timeout in btRx\r\n");
			return;

		}

		/* Validate the CRC */
		if (readCrc != calculatedCrc) {

			/* Log the error */
			LOGERROR("Invalid CRC in btRx\r\n");
			return;

		}

		/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
		canFrame.UHeader.Tx.StdId = _join32bits(uartRxBuff[7], uartRxBuff[6],
				uartRxBuff[5], uartRxBuff[4]);
		/* Read the Data Length Code */
		canFrame.UHeader.Tx.DLC = (uint32_t) uartRxBuff[8];
		if (CAN_PAYLOAD_SIZE < canFrame.UHeader.Tx.DLC) {

			/* Log the error */
			LOGERROR("Invalid DLC in btRx\r\n");
			return;

		}

		/* Read the payload */
		for (uint8_t i = 0; i < canFrame.UHeader.Tx.DLC; i += 1U) {

			canFrame.PayloadTable[i] = uartRxBuff[9U + i];

		}

		/* Configure the remaining CAN Tx header fields */
		canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
		canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

		/* Transmit the frame */
		ADDTOCANTXQUEUE(&canFrame, "btRx failed to send to canTxQueue\r\n");

	}

}
