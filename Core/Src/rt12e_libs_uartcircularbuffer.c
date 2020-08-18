/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartcircularbuffer.c
 * @brief Source file implementing the UART circular buffer
 */

#include "rt12e_libs_uartcircularbuffer.h"
#include <stdlib.h>

/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param rbPtr Pointer to the circular buffer structure
 * @retval EUartUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_start(SUartCircularBuffer *rbPtr) {

	EUartCircularBufferStatus ret = EUartCircularBufferStatus_OK; /* Return value */

	if ((NULL != rbPtr) && (NULL != rbPtr->PeriphHandlePtr)
			&& (NULL != rbPtr->BufferPtr) && (0 < rbPtr->BufferSize)) {

		/* Set the head and tail */
		rbPtr->Head = 0;
		rbPtr->Tail = 0;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(rbPtr->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if(HAL_OK != HAL_UART_Receive_DMA(rbPtr->PeriphHandlePtr, rbPtr->BufferPtr,
				rbPtr->BufferSize)) {

			ret = EUartCircularBufferStatus_HalError;

		}

	} else {

		ret = EUartCircularBufferStatus_InvalidParams;

	}

	return ret;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param rbPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCircularBuffer_stop(SUartCircularBuffer *rbPtr) {

	HAL_UART_Abort(rbPtr->PeriphHandlePtr);

}

/**
 * @brief ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param rbPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCircularBuffer_irqHandlerCallback(SUartCircularBuffer *rbPtr) {

	/* Assert valid parameters */
	if(NULL != rbPtr) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(rbPtr->PeriphHandlePtr, UART_IT_IDLE) && __HAL_UART_GET_FLAG(rbPtr->PeriphHandlePtr,
				UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(rbPtr->PeriphHandlePtr);

			/* Update the buffer tail */
			rbPtr->Tail = rbPtr->BufferSize - rbPtr->PeriphHandlePtr->hdmarx->Instance->NDTR;

			/* Callback */
			if (NULL != rbPtr->Callback) {

				rbPtr->Callback();

			}

		}

	}

}

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param rbPtr Pointer to the circular buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_read(SUartCircularBuffer *rbPtr, uint8_t *dstBuffPtr,
		size_t dstBuffSize) {

	EUartCircularBufferStatus ret = EUartCircularBufferStatus_OK; /* Return value */

	/* Assert valid parameters */
	if ((NULL != rbPtr) && (NULL != dstBuffPtr) && (dstBuffSize > 0)) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(rbPtr->PeriphHandlePtr, UART_IT_IDLE);

		if (rbPtr->Head != rbPtr->Tail) {

			/* Test if the ring buffer has overflown */
			if (rbPtr->Tail > rbPtr->Head) {

				/* Assert no overflow in the destination buffer */
				size_t len =
						((rbPtr->Tail - rbPtr->Head) < dstBuffSize) ?
								(rbPtr->Tail - rbPtr->Head) : dstBuffSize;

				/* Transfer the data */
				for (size_t i = 0; i < len; i += 1UL) {

					dstBuffPtr[i] = rbPtr->BufferPtr[rbPtr->Head + i];

				}

			} else {

				/* Assert no overflow in the destination buffer from the upper part of the ring buffer */
				size_t lenUpperBuffer =
						((rbPtr->BufferSize - rbPtr->Head) < dstBuffSize) ?
								(rbPtr->BufferSize - rbPtr->Head) : dstBuffSize;

				/* Transfer the data from the upper part of the ring buffer */
				for (size_t i = 0; i < lenUpperBuffer; i += 1UL) {

					dstBuffPtr[i] = rbPtr->BufferPtr[rbPtr->Head + i];

				}

				/* Assert no overflow in the destination buffer from the lower part of the ring buffer */
				size_t lenLowerBuffer =
						(rbPtr->Tail < (dstBuffSize - lenUpperBuffer)) ?
								rbPtr->Tail : (dstBuffSize - lenUpperBuffer);

				/* Transfer the data from the lower part of the ring buffer */
				for (size_t i = 0; i < lenLowerBuffer; i += 1UL) {

					dstBuffPtr[lenUpperBuffer + i] = rbPtr->BufferPtr[i];

				}

			}

			/* Update the head */
			rbPtr->Head = rbPtr->Tail;

		} else {

			ret = EUartCircularBufferStatus_BufferEmpty;

		}

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(rbPtr->PeriphHandlePtr, UART_IT_IDLE);

	} else {

		ret = EUartCircularBufferStatus_InvalidParams;

	}

	return ret;

}
