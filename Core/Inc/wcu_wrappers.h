/**
 * @author Adrian Cinal
 * @file wcu_wrappers.h
 * @brief Header file containing wrappers declarations for the WCU application
 */

#ifndef __WCU_WRAPPERS_H_
#define __WCU_WRAPPERS_H_

#include "wcu_defs.h"
#include <stdint.h>

/**
 * @brief Reload the IWDG counter
 * @retval None
 */
void WcuReloadWatchdogCounter(void);

/**
 * @brief Put the current thread to sleep
 * @param millisec Sleep time in milliseconds
 * @retval None
 */
void WcuSleep(uint32_t millisec);

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
 * @brief Return the MCU uptime in milliseconds
 * @retval None
 */
uint32_t WcuGetUptimeInMs(void);

/**
 * @brief Calculate the R3TP-compliant CRC code
 * @param buffer R3TP message buffer
 * @param len Message length in bytes
 * @retval uint16_t The calculated CRC
 */
uint16_t WcuGetR3tpCrc(uint8_t *buffer, uint32_t len);


#endif /* __WCU_WRAPPERS_H_ */
