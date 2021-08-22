/**
 * @author Adrian Cinal
 * @file wcu_mem.h
 * @brief Header file containing memory management wrappers declarations for the WCU application
 */

#ifndef __WCU_MEM_H_
#define __WCU_MEM_H_

#include "wcu_defs.h"

/**
 * @brief Allocate memory on the heap
 * @param size Memory block size
 * @retval void* Pointer to the allocated block size or NULL on failure
 */
void *WcuMemAlloc(size_t size);

/**
 * @brief Return the allocated memory to the heap
 * @param memoryBlock Pointer to the previously allocated memory block
 * @retval None
 */
void WcuMemFree(void *memoryBlock);

/**
 * @brief Return the allocated memory to the heap
 * @param memoryBlock Pointer to the previously allocated memory block
 * @retval None
 */
void WcuMemHandleDeferredUnref(void *memoryBlock);

#endif /* __WCU_MEM_H_ */

