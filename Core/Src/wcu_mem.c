/**
 * @author Adrian Cinal
 * @file wcu_mem.c
 * @brief Source file implementing memory management wrappers for the WCU application
 */

#include "wcu_mem.h"
#include "wcu_utils.h"
#include "wcu_events.h"
#include "FreeRTOS.h"

/**
 * @brief Allocate memory on the heap
 * @param size Memory block size
 * @retval void* Pointer to the allocated block size or NULL on failure
 */
void* WcuMemAlloc(size_t size) {

	return pvPortMalloc(size);
}

/**
 * @brief Return the allocated memory to the heap
 * @param memoryBlock Pointer to the previously allocated memory block
 * @retval None
 */
void WcuMemFree(void *memoryBlock) {

	if (xPortIsInsideInterrupt()) {

		/* If inside interrupt, defer the free to the event dispatcher as FreeRTOS does not allow
		 * calls to vPortFree() in interrupt context
		 */
		WCU_ASSERT(EWcuRet_Ok == WcuEventSend(EWcuEventType_DeferredMemoryUnref, memoryBlock, 0));

	} else {

		vPortFree(memoryBlock);
	}
}
