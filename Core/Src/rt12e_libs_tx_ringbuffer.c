/**
 * @author Adrian Cinal
 * @file rt12e_libs_tx_ringbuffer.c
 * @brief Source file implementing the generic ring buffer utilities
 */

#include "rt12e_libs_tx_ringbuffer.h"

#include <stddef.h>
#include <string.h>

#define TX_RB_VALID(RB)       ( (NULL != (RB)) && (NULL != (RB)->Buffer) && (0 != (RB)->Length) && (NULL != (RB)->Router) )
#define TX_RB_HEAD(RB)        ( &( (RB)->Buffer[(RB)->Head] ) )
#define TX_RB_TAIL(RB)        ( &( (RB)->Buffer[(RB)->Tail] ) )
#define TX_RB_OVERFLOWED(RB)  ( (RB)->Dirty && ( (RB)->Head <= (RB)->LockedTail ) )

#define TX_RB_FREERTOS_IN_USE 1

#if TX_RB_FREERTOS_IN_USE
#include <cmsis_os.h>

extern QueueHandle_t garbageQueueHandle;

#else /* !TX_RB_FREERTOS_IN_USE */
#include <stdlib.h>
#endif /* !TX_RB_FREERTOS_IN_USE */

static ETxRbRet TxRbCommitDirectlyFromRingbuffer(STxRb *rb);
static ETxRbRet TxRbCommitFromLinearBuffer(STxRb *rb);
static void* TxRbAllocateBuffer(size_t bufferSize);
static void TxRbFreeBuffer(void *buffer);

/**
 * @brief Initialize the ring buffer control block
 * @param rb Uninitialized ring buffer control block
 * @param buffer Buffer to be used as the ring buffer
 * @param len Size of the buffer
 * @param router Router function used for committing/sending the data
 * @param callback User-defined callback on transfer completion
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbInit(STxRb *rb, uint8_t *buffer, size_t len, TTxRbRouter router, TTxRbUserCallback callback) {

	/* Assert valid parameters */
	ETxRbRet status = ETxRbRet_Ok;

	if ((NULL == rb) || (NULL == buffer) || (0 == len) || (NULL == router)) {

		status = ETxRbRet_InvalidParams;
	}

	if (ETxRbRet_Ok == status) {

		/* Save the buffer address and relevant callbacks */
		rb->Buffer = buffer;
		rb->Length = len;
		rb->Router = router;
		rb->UserCallback = callback;

		/* Set initial internal state */
		rb->Head = 0;
		rb->Tail = 0;
		rb->Dirty = false;
		rb->TransferInProgress = false;
		rb->LinearBuffer = NULL;
		rb->LockedTail = 0;
	}

	return status;
}

/**
 * @brief Write data to the ring buffer
 * @param rb Ring buffer control block
 * @param data Data buffer
 * @param len Length of the data
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbWrite(STxRb *rb, uint8_t *data, size_t len) {

	/* Assert valid parameters */
	ETxRbRet status = ETxRbRet_Ok;

	/* Assert valid parameters */
	if(!TX_RB_VALID(rb)|| (NULL == data) || (0 == len)
			|| (len > rb->Length)) {

		status = ETxRbRet_InvalidParams;
	}

	if (ETxRbRet_Ok == status) {

		size_t freeSpace = 0;
		(void) TxRbGetFreeSpace(rb, &freeSpace);
		/* Assert there is enough free space in the buffer */
		if (len > freeSpace) {

			status = ETxRbRet_WouldOverflow;
		}
	}

	if (ETxRbRet_Ok == status) {

		/* Test for overflow */
		if (rb->Head + len > rb->Length) {

			/* Fill the upper part of the ring buffer */
			size_t lenUpper = rb->Length - rb->Head;
			memcpy(TX_RB_HEAD(rb), data, lenUpper);

			/* Fill the lower part of the ring buffer */
			size_t lenLower = len - lenUpper;
			memcpy(rb->Buffer, &(data[lenUpper]), lenLower);

		} else {

			/* Copy the data into the buffer */
			memcpy(TX_RB_HEAD(rb), data, len);
		}

		/* Update the head */
		rb->Head = (rb->Head + len) % rb->Length;

		/* Set the dirty flag */
		rb->Dirty = true;
	}

	return status;
}

/**
 * @brief Commit all data stored in the buffer
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbFlush(STxRb *rb) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Assert valid parameters */
	if (!TX_RB_VALID(rb)) {

		status = ETxRbRet_InvalidParams;
	}

	if (ETxRbRet_Ok == status) {

		if (!rb->Dirty) {

			status = ETxRbRet_Empty;
		}
	}

	if (ETxRbRet_Ok == status) {

		if (rb->TransferInProgress) {

			status = ETxRbRet_Busy;
		}
	}

	if (ETxRbRet_Ok == status) {

		/* Test if the ring buffer has overflowed */
		if (!TX_RB_OVERFLOWED(rb)) {

			/* No overflow - send data directly from the ring buffer, lock the tail */
			status = TxRbCommitDirectlyFromRingbuffer(rb);

		} else {

			/* On overflow allocate a linear buffer and commit the data from there without locking the tail */
			status = TxRbCommitFromLinearBuffer(rb);
		}

		/* Regardless of commit status invalidate the buffer to avoid entering a corrupted state */
		rb->Tail = rb->Head;
		rb->Dirty = false;
	}

	if (ETxRbRet_Ok == status) {

		rb->TransferInProgress = true;
	}

	return status;
}

/**
 * @brief Callback on transfer completion
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbCallback(STxRb *rb) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Assert valid parameters */
	if (!TX_RB_VALID(rb)) {

		status = ETxRbRet_InvalidParams;
	}

	if (ETxRbRet_Ok == status) {

		/* Free the linear buffer if allocated */
		if (NULL != rb->LinearBuffer) {

			TxRbFreeBuffer(rb->LinearBuffer);
			rb->LinearBuffer = NULL;
		}

		/* Call user-defined callback if any was registered */
		if (rb->UserCallback) {

			rb->UserCallback();
		}

		/* Reset transfer in progress flag */
		rb->TransferInProgress = false;

		/* Move the locked tail to the current tail position */
		rb->LockedTail = rb->Tail;
	}

	return status;
}

/**
 * @brief Get the number of free bytes in the ring buffer
 * @param rb Ring buffer control block
 * @param space Pointer to pass the number of available bytes out of the function
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbGetFreeSpace(STxRb *rb, size_t *space) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Assert valid parameters */
	if (!TX_RB_VALID(rb) || NULL == space) {

		status = ETxRbRet_InvalidParams;
	}

	if (ETxRbRet_Ok == status) {

		/* Test if the ring buffer has overflowed */
		if (TX_RB_OVERFLOWED(rb)) {

			/* If the head pointer is behind the tail pointer (or in the same position, i.e. the buffer is full) */
			*space = rb->LockedTail - rb->Head;

		} else {

			/* If the head pointer is farther into the buffer than the tail pointer or if the buffer is empty */
			*space = rb->Length - (rb->Head - rb->LockedTail);
		}
	}

	return status;
}

/**
 * @brief Commit data directly form the ring buffer, lock the tail
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
static ETxRbRet TxRbCommitDirectlyFromRingbuffer(STxRb *rb) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Lock the tail */
	rb->LockedTail = rb->Tail;

	/* Route the data */
	status = rb->Router(TX_RB_TAIL(rb), rb->Head - rb->Tail);

	return status;
}

/**
 * @brief Allocate a linear buffer, copy the data into it and commit from there, locking the tail is not required
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
static ETxRbRet TxRbCommitFromLinearBuffer(STxRb *rb) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Allocate a linear buffer */
	size_t len = rb->Length - rb->Tail + rb->Head;
	rb->LinearBuffer = TxRbAllocateBuffer(len);

	if (NULL == rb->LinearBuffer) {

		status = ETxRbRet_Error;
	}

	if (ETxRbRet_Ok == status) {

		/* Copy the upper part of the ring buffer into the linear buffer */
		memcpy(rb->LinearBuffer, TX_RB_TAIL(rb),
				rb->Length - rb->Tail);

		/* Copy the lower part of the ring buffer into the linear buffer */
		memcpy(&(rb->LinearBuffer[rb->Length - rb->Tail]),
				rb->Buffer, rb->Head);

		/* Commit the data from the allocated buffer */
		status = rb->Router(rb->LinearBuffer, len);

		/* No need to lock the tail - data in the ring buffer can be invalidated or overwritten freely */
	}

	return status;
}

/**
 * @brief Memory allocator wrapper
 * @note When using FreeRTOS, it is better to call pvPortMalloc() from here instead of malloc()
 * @param bufferSize Size of the memory block to be allocated
 * @retval Pointer to the allocated block on success or NULL on failure
 */
static void* TxRbAllocateBuffer(size_t bufferSize) {

#if TX_RB_FREERTOS_IN_USE
	return pvPortMalloc(bufferSize);
#else /* !TX_RB_FREERTOS_IN_USE */
	return malloc(bufferSize);
#endif /* !TX_RB_FREERTOS_IN_USE */
}

/**
 * @brief Memory allocator wrapper
 * @note When using FreeRTOS, a dedicated high-priority garbage-collector task must be created to defer freeing the block to it
 * @param buffer Pointer to the previously allocated memory block
 * @retval None
 */
static void TxRbFreeBuffer(void *buffer) {

#if TX_RB_FREERTOS_IN_USE

	if (xPortIsInsideInterrupt()) {

		/* If in IRQ, defer handling of the freeing to the garbage-collector task */
		if (pdPASS != xQueueSendFromISR(garbageQueueHandle, &buffer, NULL)) {

			for(;;) {

				/* We must never get here as this implies a memory leak */
			}
		}

	} else {

		vPortFree(buffer);
	}
#else /* !TX_RB_FREERTOS_IN_USE */
	free(buffer);
#endif /* !TX_RB_FREERTOS_IN_USE */

}
