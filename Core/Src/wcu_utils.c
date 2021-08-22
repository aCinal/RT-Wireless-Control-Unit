/**
 * @author Adrian Cinal
 * @file wcu_utils.c
 * @brief Source file implementing common utilities for the WCU application
 */

#include "wcu_utils.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

extern CRC_HandleTypeDef hcrc;

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
