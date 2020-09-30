/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer.c
 * @brief Source file implementing the UART ring buffer
 */

#include "rt12e_libs_uartringbuffer.h"

#include <stdlib.h>

/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartUartCircularBufferStatus Status
 */
EUartRingBufRet UartRingBuf_Start(SUartRingBuf *ringBufPtr) {

	EUartRingBufRet status = EUartRingBufRet_Ok;

	/* Assert valid parameters */
	if (( NULL == ringBufPtr) || (NULL == ringBufPtr->PeriphHandlePtr)
			|| (NULL == ringBufPtr->BufferPtr)
			|| (0 == ringBufPtr->BufferSize)) {

		status = EUartRingBufRet_InvalidParams;

	}

	if (EUartRingBufRet_Ok == status) {

		/* Reset the head and tail */
		ringBufPtr->Head = 0;
		ringBufPtr->Tail = 0;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if (HAL_OK
				!= HAL_UART_Receive_DMA(ringBufPtr->PeriphHandlePtr,
						ringBufPtr->BufferPtr, ringBufPtr->BufferSize)) {

			status = EUartRingBufRet_HalError;

		}

	}

	return status;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_Stop(SUartRingBuf *ringBufPtr) {

	EUartRingBufRet status = EUartRingBufRet_Ok;

	/* Assert valid parameters */
	if (NULL == ringBufPtr) {

		status = EUartRingBufRet_InvalidParams;

	}

	if (EUartRingBufRet_Ok == status) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);
		/* Abort the data transfer */
		if (HAL_OK != HAL_UART_Abort(ringBufPtr->PeriphHandlePtr)) {

			status = EUartRingBufRet_HalError;

		}

	}

	return status;

}

/**
 * @brief The ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_IrqHandlerCallback(SUartRingBuf *ringBufPtr) {

	EUartRingBufRet status = EUartRingBufRet_Ok;

	/* Assert valid parameters */
	if (NULL == ringBufPtr) {

		status = EUartRingBufRet_InvalidParams;

	}

	if (EUartRingBufRet_Ok == status) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(ringBufPtr->PeriphHandlePtr,
				UART_IT_IDLE) && __HAL_UART_GET_FLAG(ringBufPtr->PeriphHandlePtr,
						UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(ringBufPtr->PeriphHandlePtr);

			/* Update the head */
			ringBufPtr->Head = ringBufPtr->BufferSize
					- ringBufPtr->PeriphHandlePtr->hdmarx->Instance->NDTR;

			/* Callback */
			if (NULL != ringBufPtr->Callback) {

				ringBufPtr->Callback();

			}

		}

	}

	return status;

}

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param ringBufPtr Pointer to the ring buffer structure
 * @param dstBufPtr Destination address
 * @param dstBufSize Size of the destination buffer
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_Read(SUartRingBuf *ringBufPtr, uint8_t *dstBufPtr,
		size_t dstBufSize) {

	EUartRingBufRet status = EUartRingBufRet_Ok;

	/* Disable interrupts */
	__HAL_UART_DISABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

	/* Assert valid parameters */
	if (( NULL == ringBufPtr) || ( NULL == dstBufPtr) || (0 == dstBufSize)) {

		status = EUartRingBufRet_InvalidParams;

	}

	if (EUartRingBufRet_Ok == status) {

		/* Test if the ring buffer is empty */
		if (ringBufPtr->Head == ringBufPtr->Tail) {

			status = EUartRingBufRet_BufferEmpty;

		}

	}

	if (EUartRingBufRet_Ok == status) {

		/* Test if the ring buffer has overflown */
		if (ringBufPtr->Head > ringBufPtr->Tail) {

			/* Assert no overflow in the destination buffer */
			size_t len =
					((ringBufPtr->Head - ringBufPtr->Tail) < dstBufSize) ?
							(ringBufPtr->Head - ringBufPtr->Tail) : dstBufSize;

			/* Transfer the data */
			for (size_t i = 0; i < len; i += 1UL) {

				dstBufPtr[i] = ringBufPtr->BufferPtr[ringBufPtr->Tail + i];

			}

			/* Move the tail forward in the buffer */
			ringBufPtr->Tail += len;

		} else {

			/* Assert no overflow in the destination buffer from the upper part of the buffer */
			size_t lenUpper =
					((ringBufPtr->BufferSize - ringBufPtr->Tail) < dstBufSize) ?
							(ringBufPtr->BufferSize - ringBufPtr->Tail) :
							dstBufSize;

			/* Transfer the data from the upper part of the ring buffer */
			for (size_t i = 0; i < lenUpper; i += 1UL) {

				dstBufPtr[i] = ringBufPtr->BufferPtr[ringBufPtr->Tail + i];

			}

			/* Assert no overflow in the destination buffer from the lower part of the buffer */
			size_t lenLower =
					(ringBufPtr->Head < (dstBufSize - lenUpper)) ?
							ringBufPtr->Head : (dstBufSize - lenUpper);

			/* Transfer the data from the lower part of the buffer */
			for (size_t i = 0; i < lenLower; i += 1UL) {

				dstBufPtr[lenUpper + i] = ringBufPtr->BufferPtr[i];

			}

			/* Update the tail */
			ringBufPtr->Tail = lenLower;

		}

	}

	/* Enable interrupts */
	__HAL_UART_ENABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

	return status;

}
