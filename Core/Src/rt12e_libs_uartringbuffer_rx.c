/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer_rx.c
 * @brief Source file implementing the UART RX ring buffer
 */

#include "rt12e_libs_uartringbuffer_rx.h"

#include <stdlib.h>

#define UART_RB_INITIALIZED(rb)  ( ( (rb)->UartHandlePtr != NULL ) && ( (rb)->BufferPtr != NULL ) \
                                  && ( (rb)->BufferSize != 0 ) && ( (rb)->MsgReceivedCallback != NULL ) )

/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param msgReceivedCallback Function to be called on message received event
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbInit(SUartRxRb *rb, UART_HandleTypeDef *uartHandle,
		uint8_t *buffer, size_t bufferSize, void (*msgReceivedCallback)(void)) {

	EUartRxRbRet status = EUartRxRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || (NULL == uartHandle) || (NULL == buffer)
			|| (0 == bufferSize) || (NULL == msgReceivedCallback)) {

		status = EUartRxRbRet_InvalidParams;
	}

	if (EUartRxRbRet_Ok == status) {

		rb->UartHandlePtr = uartHandle;
		rb->BufferPtr = buffer;
		rb->BufferSize = bufferSize;
		rb->MsgReceivedCallback = msgReceivedCallback;

		/* Reset the internal state */
		rb->Head = 0;
		rb->Tail = 0;
		rb->Dirty = false;
	}

	return status;
}

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbIsr(SUartRxRb *rb) {

	EUartRxRbRet status = EUartRxRbRet_Ok;

	/* Assert valid parameters */
	if (NULL == rb || !UART_RB_INITIALIZED(rb)) {

		status = EUartRxRbRet_InvalidParams;
	}

	if (EUartRxRbRet_Ok == status) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(rb->UartHandlePtr,
				UART_IT_IDLE) && __HAL_UART_GET_FLAG(rb->UartHandlePtr,
						UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(rb->UartHandlePtr);

			/* Update the head */
			rb->Head = rb->BufferSize
					- rb->UartHandlePtr->hdmarx->Instance->NDTR;

			/* Set the dirty flag */
			rb->Dirty = true;

			/* Call the function registered by the user */
			rb->MsgReceivedCallback();
		}
	}

	return status;
}

/**
 * @brief Enable interrupts and start the data transfer to the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartUartCircularBufferStatus Status
 */
EUartRxRbRet UartRxRbRecv(SUartRxRb *rb) {

	EUartRxRbRet status = EUartRxRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb)) {

		status = EUartRxRbRet_InvalidParams;
	}

	if (EUartRxRbRet_Ok == status) {

		/* Reset the internal state */
		rb->Head = 0;
		rb->Tail = 0;
		rb->Dirty = false;

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(rb->UartHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if (HAL_OK
				!= HAL_UART_Receive_DMA(rb->UartHandlePtr, rb->BufferPtr,
						rb->BufferSize)) {

			status = EUartRxRbRet_Error;
		}

	}

	return status;

}

/**
 * @brief Disable interrupts and stop the data transfer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbAbort(SUartRxRb *rb) {

	EUartRxRbRet status = EUartRxRbRet_Ok;

	/* Assert valid parameters */
	if (NULL == rb || !UART_RB_INITIALIZED(rb)) {

		status = EUartRxRbRet_InvalidParams;
	}

	if (EUartRxRbRet_Ok == status) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(rb->UartHandlePtr, UART_IT_IDLE);

		/* Abort the data transfer */
		if (HAL_OK != HAL_UART_Abort(rb->UartHandlePtr)) {

			status = EUartRxRbRet_Error;
		}
	}

	return status;
}

/**
 * @brief Move the data from the ring buffer to the destination
 * @param rb Pointer to the ring buffer control block
 * @param buffer Destination buffer
 * @param bufferSize Size of the destination buffer
 * @param bytesRead Number of bytes read from the buffer
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbRead(SUartRxRb *rb, uint8_t *buffer, size_t bufferSize,
		size_t *bytesRead) {

	EUartRxRbRet status = EUartRxRbRet_Ok;

	/* Assert valid parameters */
	if (( NULL == rb) || !UART_RB_INITIALIZED(rb) || (NULL == buffer)
			|| (0 == bufferSize) || (NULL == bytesRead)) {

		status = EUartRxRbRet_InvalidParams;
	}

	if (EUartRxRbRet_InvalidParams != status) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(rb->UartHandlePtr, UART_IT_IDLE);
	}

	if (EUartRxRbRet_Ok == status) {

		/* Assert the ring buffer is dirty */
		if (false == rb->Dirty) {

			status = EUartRxRbRet_BufferEmpty;
		}
	}

	if (EUartRxRbRet_Ok == status) {

		/* Test if the ring buffer has overflowed */
		if (rb->Head > rb->Tail) {

			size_t len;
			/* Determine the size of the data to be copied */
			if ((rb->Head - rb->Tail) <= bufferSize) {

				len = (rb->Head - rb->Tail);
				/* Clear the dirty flag, the entire buffer has been read */
				rb->Dirty = false;

			} else {

				len = bufferSize;
				/* The dirty flag remains set */
				rb->Dirty = true;
			}

			/* Transfer the data */
			for (size_t i = 0; i < len; i += 1UL) {

				buffer[i] = rb->BufferPtr[rb->Tail + i];
			}

			/* Move the tail forward in the buffer */
			rb->Tail += len;

			/* Save the number of bytes read for the caller to inspect */
			*bytesRead = len;

		} else {

			size_t lenUpper;
			/* Determine the size of the data to be copied from the upper part of the buffer */
			if ((rb->BufferSize - rb->Tail) <= bufferSize) {

				lenUpper = (rb->BufferSize - rb->Tail);

			} else {

				/* Assert no overflow in the destination buffer from the upper part of the buffer */
				lenUpper = bufferSize;
			}

			/* Transfer the data from the upper part of the ring buffer */
			for (size_t i = 0; i < lenUpper; i += 1UL) {

				buffer[i] = rb->BufferPtr[rb->Tail + i];
			}

			size_t lenLower;
			/* Determine the size of the data to be copied from the upper part of the buffer */
			if (rb->Head <= (bufferSize - lenUpper)) {

				lenLower = rb->Head;
				/* Clear the dirty flag, the entire buffer has been read */
				rb->Dirty = false;

			} else {

				/* Assert no overflow in the destination buffer from the lower part of the buffer */
				lenLower = (bufferSize - lenUpper);
				/* The dirty flag remains set */
				rb->Dirty = true;
			}

			/* Transfer the data from the lower part of the buffer */
			for (size_t i = 0; i < lenLower; i += 1UL) {

				buffer[lenUpper + i] = rb->BufferPtr[i];
			}

			/* Update the tail */
			rb->Tail = lenLower;

			/* Save the number of bytes read for the caller to inspect */
			*bytesRead = lenUpper + lenLower;
		}
	}

	if (EUartRxRbRet_InvalidParams != status) {

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(rb->UartHandlePtr, UART_IT_IDLE);
	}

	return status;
}

