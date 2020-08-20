/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartcircularbuffer.c
 * @brief Source file implementing the UART circular buffer
 */

#include "rt12e_libs_uartcircularbuffer.h"
#include <stdlib.h>

/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval EUartUartCircularBufferStatus Error code
 */
EUartCirBufRet uartCirBuf_start(SUartCirBuf *cirBufPtr) {

	EUartCirBufRet ret = EUartCirBufRet_OK; /* Return value */

	if ((NULL != cirBufPtr) && (NULL != cirBufPtr->PeriphHandlePtr)
			&& (NULL != cirBufPtr->BufferPtr) && (0 < cirBufPtr->BufferSize)) {

		/* Reset the head and tail */
		cirBufPtr->Head = 0;
		cirBufPtr->Tail = 0;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(cirBufPtr->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if (HAL_OK
				!= HAL_UART_Receive_DMA(cirBufPtr->PeriphHandlePtr,
						cirBufPtr->BufferPtr, cirBufPtr->BufferSize)) {

			ret = EUartCirBufRet_HalError;

		}

	} else {

		ret = EUartCirBufRet_InvalidParams;

	}

	return ret;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval EUartCirBufRet Error code
 */
EUartCirBufRet uartCirBuf_stop(SUartCirBuf *cirBufPtr) {

	EUartCirBufRet ret = EUartCirBufRet_OK; /* Return value */

	/* Assert valid parameters */
	if (NULL != cirBufPtr) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(cirBufPtr->PeriphHandlePtr, UART_IT_IDLE);
		/* Abort the data transfer */
		if (HAL_OK != HAL_UART_Abort(cirBufPtr->PeriphHandlePtr)) {

			ret = EUartCirBufRet_HalError;

		}

	} else {

		ret = EUartCirBufRet_InvalidParams;

	}

	return ret;

}

/**
 * @brief The ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCirBuf_irqHandlerCallback(SUartCirBuf *cirBufPtr) {

	/* Assert valid parameters */
	if (NULL != cirBufPtr) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(cirBufPtr->PeriphHandlePtr,
				UART_IT_IDLE) && __HAL_UART_GET_FLAG(cirBufPtr->PeriphHandlePtr,
						UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(cirBufPtr->PeriphHandlePtr);

			/* Update the head */
			cirBufPtr->Head = cirBufPtr->BufferSize
					- cirBufPtr->PeriphHandlePtr->hdmarx->Instance->NDTR;

			/* Callback */
			if (NULL != cirBufPtr->Callback) {

				cirBufPtr->Callback();

			}

		}

	}

}

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param cirBufPtr Pointer to the circular buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartCirBufRet Error code
 */
EUartCirBufRet uartCirBuf_read(SUartCirBuf *cirBufPtr, uint8_t *dstBuffPtr,
		size_t dstBuffSize) {

	EUartCirBufRet ret = EUartCirBufRet_OK; /* Return value */

	/* Assert valid parameters */
	if ((NULL != cirBufPtr) && (NULL != dstBuffPtr) && (dstBuffSize > 0)) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(cirBufPtr->PeriphHandlePtr, UART_IT_IDLE);

		if (cirBufPtr->Head != cirBufPtr->Tail) {

			/* Test if the circular buffer has overflown */
			if (cirBufPtr->Head > cirBufPtr->Tail) {

				/* Assert no overflow in the destination buffer */
				size_t len =
						((cirBufPtr->Head - cirBufPtr->Tail) < dstBuffSize) ?
								(cirBufPtr->Head - cirBufPtr->Tail) :
								dstBuffSize;

				/* Transfer the data */
				for (size_t i = 0; i < len; i += 1UL) {

					dstBuffPtr[i] = cirBufPtr->BufferPtr[cirBufPtr->Tail + i];

				}

				/* Move the tail forward in the buffer */
				cirBufPtr->Tail += len;

			} else {

				/* Assert no overflow in the destination buffer from the upper part of the circular buffer */
				size_t lenUpperBuffer =
						((cirBufPtr->BufferSize - cirBufPtr->Tail) < dstBuffSize) ?
								(cirBufPtr->BufferSize - cirBufPtr->Tail) :
								dstBuffSize;

				/* Transfer the data from the upper part of the ring buffer */
				for (size_t i = 0; i < lenUpperBuffer; i += 1UL) {

					dstBuffPtr[i] = cirBufPtr->BufferPtr[cirBufPtr->Tail + i];

				}

				/* Assert no overflow in the destination buffer from the lower part of the circular buffer */
				size_t lenLowerBuffer =
						(cirBufPtr->Head < (dstBuffSize - lenUpperBuffer)) ?
								cirBufPtr->Head :
								(dstBuffSize - lenUpperBuffer);

				/* Transfer the data from the lower part of the circular buffer */
				for (size_t i = 0; i < lenLowerBuffer; i += 1UL) {

					dstBuffPtr[lenUpperBuffer + i] = cirBufPtr->BufferPtr[i];

				}

				/* Update the tail */
				cirBufPtr->Tail = lenLowerBuffer;

			}

		} else {

			ret = EUartCirBufRet_BufferEmpty;

		}

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(cirBufPtr->PeriphHandlePtr, UART_IT_IDLE);

	} else {

		ret = EUartCirBufRet_InvalidParams;

	}

	return ret;

}
