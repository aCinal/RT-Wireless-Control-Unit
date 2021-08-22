/**
 * @author Adrian Cinal
 * @file rt12e_libs_tx_ringbuffer.c
 * @brief Source file implementing the generic ring buffer utilities
 */

#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include "rt12e_libs_tx_ringbuffer.h"

#define likely(x)    __builtin_expect(!!(x), 1)
#define unlikely(x)  __builtin_expect(!!(x), 0)

#define TX_RB_VALID(RB)       ( (NULL != (RB)) && (NULL != (RB)->Buffer) && (0 != (RB)->Length) && (NULL != (RB)->Router) && (NULL != (RB)->MemAlloc) && (NULL != (RB)->MemUnref) )
#define TX_RB_OVERFLOWED(RB)  ( (RB)->Dirty && ( (RB)->Head <= (RB)->LockedTail ) )

static ETxRbRet TxRbCommitDirectlyFromRingbuffer(STxRb *rb);
static ETxRbRet TxRbCommitFromLinearBuffer(STxRb *rb);

/**
 * @brief Initialize the ring buffer control block
 * @param rb Uninitialized ring buffer control block
 * @param buffer Buffer to be used as the ring buffer
 * @param len Size of the buffer
 * @param router Router function used for committing/sending the data
 * @param callback User-defined callback on transfer completion
 * @param memalloc User-provided memory allocator
 * @param memunref User-provided memory deallocator
 * @note The memory deallocator callback will be called from TxRbCallback, i.e. likely from an ISR context
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbInit(STxRb *rb, uint8_t *buffer, size_t len, TTxRbRouter router,
		TTxRbUserCallback callback, TTxRbMemoryAllocator memalloc,
		TTxRbMemoryDeallocator memunref) {

	/* Assert valid parameters */
	ETxRbRet status = ETxRbRet_Ok;

	if (unlikely((NULL == rb) || (NULL == buffer) || (0 == len) || (NULL == router)
			|| (NULL == memalloc) || (NULL == memunref))) {

		status = ETxRbRet_InvalidParams;
	}

	if (likely(ETxRbRet_Ok == status)) {

		/* Save the buffer address and relevant callbacks */
		rb->Buffer = buffer;
		rb->Length = len;
		rb->Router = router;
		rb->UserCallback = callback;
		rb->MemAlloc = memalloc;
		rb->MemUnref = memunref;

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
	if (unlikely(!TX_RB_VALID(rb) || (NULL == data) || (0 == len)
			|| (len > rb->Length))) {

		status = ETxRbRet_InvalidParams;
	}

	if (likely(ETxRbRet_Ok == status)) {

		size_t freeSpace = 0;
		(void) TxRbGetFreeSpace(rb, &freeSpace);
		/* Assert there is enough free space in the buffer */
		if (len > freeSpace) {

			status = ETxRbRet_WouldOverflow;
		}
	}

	if (likely(ETxRbRet_Ok == status)) {

		/* Test for overflow */
		if (rb->Head + len > rb->Length) {

			/* Fill the upper part of the ring buffer */
			size_t lenUpper = rb->Length - rb->Head;
			(void) memcpy(&rb->Buffer[rb->Head], data, lenUpper);

			/* Fill the lower part of the ring buffer */
			size_t lenLower = len - lenUpper;
			(void) memcpy(rb->Buffer, &(data[lenUpper]), lenLower);

		} else {

			/* Copy the data into the buffer */
			(void) memcpy(&rb->Buffer[rb->Head], data, len);
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
	if (unlikely(!TX_RB_VALID(rb))) {

		status = ETxRbRet_InvalidParams;
	}

	if (likely(ETxRbRet_Ok == status)) {

		if (!rb->Dirty) {

			status = ETxRbRet_Empty;
		}
	}

	if (likely(ETxRbRet_Ok == status)) {

		if (rb->TransferInProgress) {

			status = ETxRbRet_Busy;
		}
	}

	if (likely(ETxRbRet_Ok == status)) {

		/* Test if the ring buffer has overflowed */
		if (!TX_RB_OVERFLOWED(rb)) {

			/* No overflow - send data directly from the ring buffer, lock the tail */
			status = TxRbCommitDirectlyFromRingbuffer(rb);

		} else {

			/* On overflow allocate a linear buffer and commit the data from there without locking the tail */
			status = TxRbCommitFromLinearBuffer(rb);
		}
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
	if (unlikely(!TX_RB_VALID(rb))) {

		status = ETxRbRet_InvalidParams;
	}

	if (likely(ETxRbRet_Ok == status)) {

		/* Free the linear buffer if allocated */
		if (NULL != rb->LinearBuffer) {

			rb->MemUnref(rb->LinearBuffer);
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
	if (unlikely(!TX_RB_VALID(rb) || NULL == space)) {

		status = ETxRbRet_InvalidParams;
	}

	if (likely(ETxRbRet_Ok == status)) {

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
	/* Pre-invalidate the buffer */
	rb->Tail = rb->Head;
	rb->Dirty = false;
	/* Pre-set transfer in progress flag */
	rb->TransferInProgress = true;

	/* Route the data */
	status = rb->Router(&(rb->Buffer[rb->LockedTail]),
			rb->Head - rb->LockedTail);

	if (unlikely(ETxRbRet_Ok != status)) {

		/* Restore previous state on routing failure */
		rb->Tail = rb->LockedTail;
		rb->Dirty = true;
		rb->TransferInProgress = false;
	}

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
	rb->LinearBuffer = rb->MemAlloc(len);

	if (unlikely(NULL == rb->LinearBuffer)) {

		status = ETxRbRet_Error;
	}

	if (likely(ETxRbRet_Ok == status)) {

		/* Save old tail to restore it on routing failure */
		size_t oldTail = rb->Tail;
		/* Pre-invalidate the buffer */
		rb->Tail = rb->Head;
		rb->Dirty = false;
		/* Pre-set transfer in progress flag */
		rb->TransferInProgress = true;

		/* Copy the upper part of the ring buffer into the linear buffer */
		(void) memcpy(rb->LinearBuffer, &rb->Buffer[oldTail], rb->Length - oldTail);

		/* Copy the lower part of the ring buffer into the linear buffer */
		(void) memcpy(&(rb->LinearBuffer[rb->Length - oldTail]), rb->Buffer, rb->Head);

		/* Commit the data from the allocated buffer */
		status = rb->Router(rb->LinearBuffer, len);

		/* Assert transfer initiated successfully */
		if (ETxRbRet_Ok != status) {

			/* Restore previous state on routing failure */
			rb->Tail = oldTail;
			rb->Dirty = true;
			rb->TransferInProgress = false;
			/* Cleanup */
			rb->MemUnref(rb->LinearBuffer);
			rb->LinearBuffer = NULL;
		}

		/* No need to lock the tail - data in the ring buffer can be invalidated or overwritten freely */
	}

	return status;
}
