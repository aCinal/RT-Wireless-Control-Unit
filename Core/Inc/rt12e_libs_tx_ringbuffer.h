/**
 * @author Adrian Cinal
 * @file rt12e_libs_tx_ringbuffer.h
 * @brief Header file containing the generic ring buffer interface
 */

#ifndef __RT12E_LIBS_TX_RINGBUFFER_H_
#define __RT12E_LIBS_TX_RINGBUFFER_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum ETxRbRet {
	ETxRbRet_Ok = 0, /* No errors occurred */
	ETxRbRet_InvalidParams, /* Invalid parameters were passed to the API */
	ETxRbRet_Error, /* Internal error occurred */
	ETxRbRet_Empty, /* Buffer is empty */
	ETxRbRet_Busy, /* Transfer is in progress */
	ETxRbRet_WouldOverflow /* Write failed, buffer would overflow */
} ETxRbRet;

/**
 * @brief Router function prototype typedef
 */
typedef ETxRbRet (*TTxRbRouter)(uint8_t*, size_t);

/**
 * @brief User-defined callback prototype typedef
 */
typedef void (*TTxRbUserCallback)(void);

/**
 * @brief User-defined memory allocator
 */
typedef void* (*TTxRbMemoryAllocator)(size_t);

/**
 * @brief User-defined memory deallocator
 */
typedef void (*TTxRbMemoryDeallocator)(void*);

/**
 * @brief Ring buffer control block
 * @note Members of this structure must not be modified directly, a dedicated API should be used instead
 */
typedef struct STxRb {
	uint8_t *Buffer; /* Pointer to the buffer */
	size_t Length; /* Buffer size in bytes */
	TTxRbRouter Router; /* Data router */
	TTxRbUserCallback UserCallback; /* User-defined callback */
	TTxRbMemoryAllocator MemAlloc; /* Memory allocator */
	TTxRbMemoryDeallocator MemUnref; /* Memory deallocator */
	volatile size_t Head; /* Position of the head */
	volatile size_t Tail; /* Position of the tail */
	volatile bool Dirty; /* Buffer dirty flag */
	volatile bool TransferInProgress; /* Transfer in progress flag */
	volatile size_t LockedTail; /* Position of the tail when transfer was initialized */
	uint8_t *LinearBuffer; /* Linear buffer */
} STxRb;

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
		TTxRbMemoryDeallocator memunref);

/**
 * @brief Write data to the ring buffer
 * @param rb Ring buffer control block
 * @param data Data buffer
 * @param len Length of the data
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbWrite(STxRb *rb, uint8_t *data, size_t len);

/**
 * @brief Commit all data stored in the buffer
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbFlush(STxRb *rb);

/**
 * @brief Callback on transfer completion
 * @param rb Ring buffer control block
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbCallback(STxRb *rb);

/**
 * @brief Get the number of free bytes in the ring buffer
 * @param rb Ring buffer control block
 * @param space Pointer to pass the number of available bytes out of the function
 * @retval ETxRbRet Status
 */
ETxRbRet TxRbGetFreeSpace(STxRb *rb, size_t *space);

#endif /* __RT12E_LIBS_TX_RINGBUFFER_H_ */

