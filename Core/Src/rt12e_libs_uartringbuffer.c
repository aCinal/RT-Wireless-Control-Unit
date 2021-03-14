/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer.c
 * @brief Source file implementing the UART ring buffer
 */

#include "rt12e_libs_uartringbuffer.h"

#include <stdlib.h>

#define UART_RB_INITIALIZED(rb)  ( ( (rb)->PeriphHandlePtr != NULL ) && ( (rb)->BufferPtr != NULL ) \
                                  && ( (rb)->BufferSize != 0 ) )

/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param callback Function to be called on message received
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbInit(SUartRb *rb, UART_HandleTypeDef *uartHandle,
		uint8_t *buffer, size_t bufferSize, void (*callback)(void)) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || (NULL == uartHandle) || (NULL == buffer)
			|| (0 == bufferSize)) {

		status = EUartRbRet_InvalidParams;
	}


	if (EUartRbRet_Ok == status) {

		rb->PeriphHandlePtr = uartHandle;
		rb->BufferPtr = buffer;
		rb->BufferSize = bufferSize;
		rb->Callback = callback;

		/* Reset the internal state */
		rb->Head = 0;
		rb->Tail = 0;
		rb->Dirty = false;
	}

	return status;
}

/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartUartCircularBufferStatus Status
 */
EUartRbRet UartRbStart(SUartRb *rb) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_Ok == status) {

		/* Enable interrupts on idle line */
		__HAL_UART_ENABLE_IT(rb->PeriphHandlePtr, UART_IT_IDLE);

		/* Start receving */
		if (HAL_OK
				!= HAL_UART_Receive_DMA(rb->PeriphHandlePtr, rb->BufferPtr,
						rb->BufferSize)) {

			status = EUartRbRet_HalError;
		}

	}

	return status;

}

/**
 * @brief Disables interrupts and stops the data transfer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbStop(SUartRb *rb) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if (NULL == rb || !UART_RB_INITIALIZED(rb)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_Ok == status) {

		/* Disable the idle line detection interrupt */
		__HAL_UART_DISABLE_IT(rb->PeriphHandlePtr, UART_IT_IDLE);

		/* Abort the data transfer */
		if (HAL_OK != HAL_UART_Abort(rb->PeriphHandlePtr)) {

			status = EUartRbRet_HalError;
		}
	}

	return status;
}

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbIsr(SUartRb *rb) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if (NULL == rb || !UART_RB_INITIALIZED(rb)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_Ok == status) {

		/* Assert idle line detection interrupt is on and the line is idle */
		if (__HAL_UART_GET_IT_SOURCE(rb->PeriphHandlePtr,
				UART_IT_IDLE) && __HAL_UART_GET_FLAG(rb->PeriphHandlePtr,
						UART_FLAG_IDLE)) {

			/* Clear the idle flag */
			__HAL_UART_CLEAR_IDLEFLAG(rb->PeriphHandlePtr);

			/* Update the head */
			rb->Head = rb->BufferSize
					- rb->PeriphHandlePtr->hdmarx->Instance->NDTR;

			/* Set the dirty flag */
			rb->Dirty = true;

			/* Call the function registered by the user */
			if (NULL != rb->Callback) {

				rb->Callback();
			}
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
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbRead(SUartRb *rb, uint8_t *buffer, size_t bufferSize, size_t* bytesRead) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if (( NULL == rb) || !UART_RB_INITIALIZED(rb) || (NULL == bytesRead)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_InvalidParams != status) {

		/* Disable interrupts */
		__HAL_UART_DISABLE_IT(rb->PeriphHandlePtr, UART_IT_IDLE);
	}

	if (EUartRbRet_Ok == status) {

		/* Assert the ring buffer is dirty */
		if (false == rb->Dirty) {

			status = EUartRbRet_BufferEmpty;
		}
	}

	if (EUartRbRet_Ok == status) {

		/* Test if the ring buffer has overflown */
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

	if (EUartRbRet_InvalidParams != status) {

		/* Enable interrupts */
		__HAL_UART_ENABLE_IT(rb->PeriphHandlePtr, UART_IT_IDLE);
	}

	return status;

}

/**
 * @brief Test if the buffer is dirty
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status: EUartRbRet_Ok if the buffer is dirty, EUartRbRet_BufferEmpty otherwise
 */
EUartRbRet UartRbDirty(SUartRb *rb) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_Ok == status) {

		if (false == rb->Dirty) {

			status = EUartRbRet_BufferEmpty;
		}
	}

	return status;
}

/**
 * @brief Flush the buffer and reset the dirty flag
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbInvalidate(SUartRb *rb) {

	EUartRbRet status = EUartRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb)) {

		status = EUartRbRet_InvalidParams;
	}

	if (EUartRbRet_Ok == status) {

		rb->Tail = rb->Head;
		rb->Dirty = false;
	}

	return status;
}
