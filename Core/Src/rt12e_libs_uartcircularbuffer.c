/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartcircularbuffer.c
 * @brief Source file implementing the UART circular buffer
 */

#include "rt12e_libs_uartcircularbuffer.h"
#include <stdlib.h>

/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param cirbufPtr Pointer to the circular buffer structure
 * @retval EUartUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_start(SUartCircularBuffer *cirbufPtr) {

	EUartCircularBufferStatus ret = EUartCircularBufferStatus_OK; /* Return value */

	if ((NULL != cirbufPtr) && (NULL != cirbufPtr->PeriphHandlePtr)
			&& (NULL != cirbufPtr->BufferPtr) && (0 < cirbufPtr->BufferSize)) {

		/* Reset the head and tail */
		cirbufPtr->Head = 0;
		cirbufPtr->Tail = 0;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(cirbufPtr->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if(HAL_OK != HAL_UART_Receive_DMA(cirbufPtr->PeriphHandlePtr, cirbufPtr->BufferPtr,
				cirbufPtr->BufferSize)) {

			ret = EUartCircularBufferStatus_HalError;

		}

	} else {

		ret = EUartCircularBufferStatus_InvalidParams;

	}

	return ret;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param cirbufPtr Pointer to the circular buffer structure
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_stop(SUartCircularBuffer *cirbufPtr) {

	EUartCircularBufferStatus ret = EUartCircularBufferStatus_OK; /* Return value */

	/* Assert valid parameters */
	if(NULL != cirbufPtr) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(cirbufPtr->PeriphHandlePtr, UART_IT_IDLE);
		/* Abort the data transfer */
		if(HAL_OK != HAL_UART_Abort(cirbufPtr->PeriphHandlePtr)) {

			ret = EUartCircularBufferStatus_HalError;

		}

	} else {

		ret = EUartCircularBufferStatus_InvalidParams;

	}

	return ret;

}

/**
 * @brief ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param cirbufPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCircularBuffer_irqHandlerCallback(SUartCircularBuffer *cirbufPtr) {

	/* Assert valid parameters */
	if(NULL != cirbufPtr) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(cirbufPtr->PeriphHandlePtr, UART_IT_IDLE) && __HAL_UART_GET_FLAG(cirbufPtr->PeriphHandlePtr,
				UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(cirbufPtr->PeriphHandlePtr);

			/* Update the head */
			cirbufPtr->Head = cirbufPtr->BufferSize - cirbufPtr->PeriphHandlePtr->hdmarx->Instance->NDTR;

			/* Callback */
			if (NULL != cirbufPtr->Callback) {

				cirbufPtr->Callback();

			}

		}

	}

}

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param cirbufPtr Pointer to the circular buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_read(SUartCircularBuffer *cirbufPtr, uint8_t *dstBuffPtr,
		size_t dstBuffSize) {

	EUartCircularBufferStatus ret = EUartCircularBufferStatus_OK; /* Return value */

	/* Assert valid parameters */
	if ((NULL != cirbufPtr) && (NULL != dstBuffPtr) && (dstBuffSize > 0)) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(cirbufPtr->PeriphHandlePtr, UART_IT_IDLE);

		if (cirbufPtr->Head != cirbufPtr->Tail) {

			/* Test if the ring buffer has overflown */
			if (cirbufPtr->Head > cirbufPtr->Tail) {

				/* Assert no overflow in the destination buffer */
				size_t len =
						((cirbufPtr->Head - cirbufPtr->Tail) < dstBuffSize) ?
								(cirbufPtr->Head - cirbufPtr->Tail) : dstBuffSize;

				/* Transfer the data */
				for (size_t i = 0; i < len; i += 1UL) {

					dstBuffPtr[i] = cirbufPtr->BufferPtr[cirbufPtr->Tail + i];

				}

			} else {

				/* Assert no overflow in the destination buffer from the upper part of the ring buffer */
				size_t lenUpperBuffer =
						((cirbufPtr->BufferSize - cirbufPtr->Tail) < dstBuffSize) ?
								(cirbufPtr->BufferSize - cirbufPtr->Tail) : dstBuffSize;

				/* Transfer the data from the upper part of the ring buffer */
				for (size_t i = 0; i < lenUpperBuffer; i += 1UL) {

					dstBuffPtr[i] = cirbufPtr->BufferPtr[cirbufPtr->Tail + i];

				}

				/* Assert no overflow in the destination buffer from the lower part of the ring buffer */
				size_t lenLowerBuffer =
						(cirbufPtr->Head < (dstBuffSize - lenUpperBuffer)) ?
								cirbufPtr->Head : (dstBuffSize - lenUpperBuffer);

				/* Transfer the data from the lower part of the ring buffer */
				for (size_t i = 0; i < lenLowerBuffer; i += 1UL) {

					dstBuffPtr[lenUpperBuffer + i] = cirbufPtr->BufferPtr[i];

				}

			}

			/* Update the tail */
			cirbufPtr->Tail = cirbufPtr->Head;

		} else {

			ret = EUartCircularBufferStatus_BufferEmpty;

		}

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(cirbufPtr->PeriphHandlePtr, UART_IT_IDLE);

	} else {

		ret = EUartCircularBufferStatus_InvalidParams;

	}

	return ret;

}
