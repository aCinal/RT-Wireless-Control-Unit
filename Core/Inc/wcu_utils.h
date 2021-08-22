/**
 * @author Adrian Cinal
 * @file wcu_utils.h
 * @brief Header file containing common utilities declarations for the WCU application
 */

#ifndef __WCU_UTILS_H_
#define __WCU_UTILS_H_

#include "wcu_defs.h"

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

/**
 * @brief Assertion macro
 */
#define WCU_ASSERT(x)  if ( unlikely( !(x) ) ) { for( ; /* ever */ ; ); }

#endif /* __WCU_UTILS_H_ */
