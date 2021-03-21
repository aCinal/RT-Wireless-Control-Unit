/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer_tx.c
 * @brief Source file implementing functions for using the ring buffer with UART DMA for transmission
 */

#include "rt12e_libs_uartringbuffer_tx.h"

#include <string.h>

#define UART_RB_INITIALIZED(rb)  ( ( (rb)->UartHandlePtr != NULL ) && ( (rb)->BufferPtr != NULL ) \
                                  && ( (rb)->BufferSize != 0 ) )

#define UART_RB_HEAD(rb)         ( &( (rb)->BufferPtr[(rb)->Head] ) )
#define UART_RB_TAIL(rb)         ( &( (rb)->BufferPtr[(rb)->Tail] ) )

#define UART_RB_FREERTOS_IN_USE 0

#if UART_RB_FREERTOS_IN_USE
#include <cmsis_os.h>
#else /* !UART_RB_FREERTOS_IN_USE */
#include <stdlib.h>
#endif /* !UART_RB_FREERTOS_IN_USE */

/**
 * @brief Memory allocator wrapper
 * @note When using FreeRTOS, it is better to call pvPortMalloc() from here instead of malloc()
 * @param bufferSize Size of the memory block to be allocated
 * @retval Pointer to the allocated block on success or NULL on failure
 */
static void *UartTxRbAllocateBuffer(size_t bufferSize) {

#if UART_RB_FREERTOS_IN_USE
	return pvPortMalloc(bufferSize);
#else /* !UART_RB_FREERTOS_IN_USE */
	return malloc(bufferSize);
#endif /* !UART_RB_FREERTOS_IN_USE */
}

/**
 * @brief Memory allocator wrapper
 * @note When using FreeRTOS, it is better to call vPortFree() from here instead of free()
 * @param buffer Pointer to the previously allocated memory block
 * @retval None
 */
static void UartTxRbFreeBuffer(void *buffer) {

#if UART_RB_FREERTOS_IN_USE
	vPortFree(buffer);
#else /* !UART_RB_FREERTOS_IN_USE */
	free(buffer);
#endif /* !UART_RB_FREERTOS_IN_USE */

}

/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param msgSentCallback Function to be called on message sent event
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbInit(SUartTxRb *rb, UART_HandleTypeDef *uartHandle,
		uint8_t *buffer, size_t bufferSize, void (*msgSentCallback)(void)) {

	EUartTxRbRet status = EUartTxRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || (NULL == uartHandle) || (NULL == buffer)
			|| (0 == bufferSize) || (NULL == msgSentCallback)) {

		status = EUartTxRbRet_InvalidParams;
	}

	if (EUartTxRbRet_Ok == status) {

		rb->BufferPtr = buffer;
		rb->BufferSize = bufferSize;
		rb->UartHandlePtr = uartHandle;
		rb->MsgSentCallback = msgSentCallback;

		/* Reset the internal state */
		rb->Head = 0;
		rb->Tail = 0;
		rb->Dirty = false;
		rb->TransferInProgress = false;
		rb->LinearBuffer = NULL;
	}

	return status;
}

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbIsr(SUartTxRb *rb) {

	EUartTxRbRet status = EUartTxRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb)) {

		status = EUartTxRbRet_InvalidParams;
	}

	if (EUartTxRbRet_Ok == status) {

		/* Assert transfer complete flag is set */
		if (__HAL_UART_GET_IT_SOURCE(rb->UartHandlePtr,
				UART_IT_TC) && __HAL_UART_GET_FLAG(rb->UartHandlePtr,
						UART_FLAG_TC)) {

			/* Reset the transfer in progress flag */
			rb->TransferInProgress = false;

			/* Free the linear buffer if allocated */
			if (NULL != rb->LinearBuffer) {

				UartTxRbFreeBuffer(rb->LinearBuffer);
				rb->LinearBuffer = NULL;
			}

			/* Call the function registered by the user */
			if (NULL != rb->MsgSentCallback) {

				rb->MsgSentCallback();
			}
		}
	}

	return status;
}

/**
 * @brief Write a message into the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @param data Data buffer
 * @param len Size of the data buffer
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbWrite(SUartTxRb *rb, uint8_t *data, size_t len) {

	EUartTxRbRet status = EUartTxRbRet_Ok;

	/* Assert valid parameters */
	if ((NULL == rb) || !UART_RB_INITIALIZED(rb) || (NULL == data) || (0 == len)
			|| (len > rb->BufferSize)) {

		status = EUartTxRbRet_InvalidParams;
	}

	if (EUartTxRbRet_Ok == status) {

		/* Test for overflow */
		if (rb->Head + len > rb->BufferSize) {

			/* Fill the upper part of the ring buffer */
			size_t lenUpper = rb->BufferSize - rb->Head;
			memcpy(UART_RB_HEAD(rb), data, lenUpper);

			/* Fill the lower part of the ring buffer */
			size_t lenLower = len - lenUpper;
			memcpy(rb->BufferPtr, &(data[lenUpper]), lenLower);

		} else {

			/* Copy the data into the buffer */
			memcpy(UART_RB_HEAD(rb), data, len);
		}

		/* Update the head */
		rb->Head = (rb->Head + len) % rb->BufferSize;

		/* Set the dirty flag */
		rb->Dirty = true;
	}

	return status;
}

/**
 * @brief Transmit data sitting in the buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbSend(SUartTxRb *rb) {

	EUartTxRbRet status = EUartTxRbRet_Ok;

	/* Assert valid parameters */
	if (NULL == rb || !UART_RB_INITIALIZED(rb)) {

		status = EUartTxRbRet_InvalidParams;
	}

	if (EUartTxRbRet_Ok == status) {

		if (!rb->Dirty) {

			status = EUartTxRbRet_BufferEmpty;
		}
	}

	if (EUartTxRbRet_Ok == status) {

		if (rb->TransferInProgress) {

			status = EUartTxRbRet_Again;
		}
	}

	if (EUartTxRbRet_Ok == status) {

		/* Test if the ring buffer has overflowed */
		if (rb->Head > rb->Tail) {

			/* No overflow - send data directly from the ring buffer */
			if (HAL_OK
					== HAL_UART_Transmit_DMA(rb->UartHandlePtr,
							UART_RB_TAIL(rb), rb->Head - rb->Tail)) {

				/* Set the transfer in progress flag */
				rb->TransferInProgress = true;

			} else {

				status = EUartTxRbRet_Error;
			}

		} else {

			/* On overflow allocate linear buffer and copy the data into it */
			size_t messageSize = rb->BufferSize - rb->Tail + rb->Head;
			rb->LinearBuffer = UartTxRbAllocateBuffer(messageSize);

			if (NULL != rb->LinearBuffer) {

				/* Copy the upper part of the ring buffer into the linear buffer */
				memcpy(rb->LinearBuffer, UART_RB_TAIL(rb),
						rb->BufferSize - rb->Tail);

				/* Copy the lower part of the ring buffer into the linear buffer */
				memcpy(&(rb->LinearBuffer[rb->BufferSize - rb->Tail]),
						rb->BufferPtr, rb->Head);

				/* Initiate transmission from the allocated buffer */
				if (HAL_OK
						== HAL_UART_Transmit_DMA(rb->UartHandlePtr,
								rb->LinearBuffer, messageSize)) {

					/* Set the transfer in progress flag */
					rb->TransferInProgress = true;

				} else {

					status = EUartTxRbRet_Error;
				}

			} else {

				status = EUartTxRbRet_Error;
			}

			/* On allocation failure still invalidate the buffer to avoid entering a corrupted state. */

		}

		/* Invalidate data in the buffer */
		rb->Tail = rb->Head;
		rb->Dirty = false;
	}

	return status;
}
