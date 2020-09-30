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
 * @retval EUartUartCircularBufferStatus Error code
 */
EUartRingBufRet UartRingBuf_Start(SUartRingBuf *ringBufPtr) {

	EUartRingBufRet ret = EUartRingBufRet_Ok; /* Return value */

	if ((NULL != ringBufPtr) && (NULL != ringBufPtr->PeriphHandlePtr)
			&& (NULL != ringBufPtr->BufferPtr) && (0 < ringBufPtr->BufferSize)) {

		/* Reset the head and tail */
		ringBufPtr->Head = 0;
		ringBufPtr->Tail = 0;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if (HAL_OK
				!= HAL_UART_Receive_DMA(ringBufPtr->PeriphHandlePtr,
						ringBufPtr->BufferPtr, ringBufPtr->BufferSize)) {

			ret = EUartRingBufRet_HalError;

		}

	} else {

		ret = EUartRingBufRet_InvalidParams;

	}

	return ret;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Error code
 */
EUartRingBufRet UartRingBuf_Stop(SUartRingBuf *ringBufPtr) {

	EUartRingBufRet ret = EUartRingBufRet_Ok; /* Return value */

	/* Assert valid parameters */
	if (NULL != ringBufPtr) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);
		/* Abort the data transfer */
		if (HAL_OK != HAL_UART_Abort(ringBufPtr->PeriphHandlePtr)) {

			ret = EUartRingBufRet_HalError;

		}

	} else {

		ret = EUartRingBufRet_InvalidParams;

	}

	return ret;

}

/**
 * @brief The ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval None
 */
void UartRingBuf_IrqHandlerCallback(SUartRingBuf *ringBufPtr) {

	/* Assert valid parameters */
	if (NULL != ringBufPtr) {

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

}

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param ringBufPtr Pointer to the ring buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartRingBufRet Error code
 */
EUartRingBufRet UartRingBuf_Read(SUartRingBuf *ringBufPtr, uint8_t *dstBuffPtr,
		size_t dstBuffSize) {

	EUartRingBufRet ret = EUartRingBufRet_Ok; /* Return value */

	/* Assert valid parameters */
	if ((NULL != ringBufPtr) && (NULL != dstBuffPtr) && (dstBuffSize > 0)) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

		if (ringBufPtr->Head != ringBufPtr->Tail) {

			/* Test if the ring buffer has overflown */
			if (ringBufPtr->Head > ringBufPtr->Tail) {

				/* Assert no overflow in the destination buffer */
				size_t len =
						((ringBufPtr->Head - ringBufPtr->Tail) < dstBuffSize) ?
								(ringBufPtr->Head - ringBufPtr->Tail) :
								dstBuffSize;

				/* Transfer the data */
				for (size_t i = 0; i < len; i += 1UL) {

					dstBuffPtr[i] = ringBufPtr->BufferPtr[ringBufPtr->Tail + i];

				}

				/* Move the tail forward in the buffer */
				ringBufPtr->Tail += len;

			} else {

				/* Assert no overflow in the destination buffer from the upper part of the ring buffer */
				size_t lenUpperBuffer =
						((ringBufPtr->BufferSize - ringBufPtr->Tail) < dstBuffSize) ?
								(ringBufPtr->BufferSize - ringBufPtr->Tail) :
								dstBuffSize;

				/* Transfer the data from the upper part of the ring buffer */
				for (size_t i = 0; i < lenUpperBuffer; i += 1UL) {

					dstBuffPtr[i] = ringBufPtr->BufferPtr[ringBufPtr->Tail + i];

				}

				/* Assert no overflow in the destination buffer from the lower part of the ring buffer */
				size_t lenLowerBuffer =
						(ringBufPtr->Head < (dstBuffSize - lenUpperBuffer)) ?
								ringBufPtr->Head :
								(dstBuffSize - lenUpperBuffer);

				/* Transfer the data from the lower part of the ring buffer */
				for (size_t i = 0; i < lenLowerBuffer; i += 1UL) {

					dstBuffPtr[lenUpperBuffer + i] = ringBufPtr->BufferPtr[i];

				}

				/* Update the tail */
				ringBufPtr->Tail = lenLowerBuffer;

			}

		} else {

			ret = EUartRingBufRet_BufferEmpty;

		}

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(ringBufPtr->PeriphHandlePtr, UART_IT_IDLE);

	} else {

		ret = EUartRingBufRet_InvalidParams;

	}

	return ret;

}
