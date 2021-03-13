/**
 * @author Adrian Cinal
 * @file wcu_event_wrappers.c
 * @brief Header file containing wrappers implementation for WCU
 */

#include "wcu_wrappers.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>

extern IWDG_HandleTypeDef hiwdg;
extern CRC_HandleTypeDef hcrc;

/**
 * @brief Reload the IWDG counter
 * @retval None
 */
void WcuReloadWatchdogCounter(void) {

	(void) HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @brief Put the current thread to sleep
 * @param millisec Sleep time in milliseconds
 * @retval None
 */
void WcuSleep(uint32_t millisec) {

	vTaskDelay(pdMS_TO_TICKS(millisec));
}

/**
 * @brief Allocate memory on the heap
 * @param size Memory block size
 * @retval void* Pointer to the allocated block size or NULL on failure
 */
void *WcuMemAlloc(size_t size) {

	return pvPortMalloc(size);
}

/**
 * @brief Return the allocated memory to the heap
 * @param memoryBlock Pointer to the previously allocated memory block
 * @retval None
 */
void WcuMemFree(void *memoryBlock) {

	vPortFree(memoryBlock);
}

/**
 * @brief Return the MCU uptime in milliseconds
 * @retval None
 */
uint32_t WcuGetUptimeInMs(void) {

	return HAL_GetTick();
}

/**
 * @brief Calculate the R3TP-compliant CRC code
 * @param buffer R3TP message buffer
 * @param len Message length in bytes
 * @retval uint16_t The calculated CRC
 */
uint16_t WcuGetR3tpCrc(uint8_t *buffer, uint32_t len) {

	/* Calculate the ful 32-bit CRC */
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) buffer,
			(len / sizeof(uint32_t)));

	/* Return the lower 16 bits */
	return (crc & 0xFFFF);
}
