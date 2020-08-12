/**
 * @author Adrian Cinal
 * @file wcu_xbeetx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeTx task
 */

#ifndef __WCU_XBEETX_CALLS_H_
#define __WCU_XBEETX_CALLS_H_

#include "stm32f4xx_hal.h"

/**
 * @brief Requests and receives the RSSI value from the XBEE Pro device
 * @param[out] rssiBuff Buffer for the RSSI value
 * @retval HAL_StatusTypeDef HAL status
 */
HAL_StatusTypeDef xbeeTx_GetRssi(uint8_t *rssiBuff);

#endif /* __WCU_XBEETX_CALLS_H_ */
