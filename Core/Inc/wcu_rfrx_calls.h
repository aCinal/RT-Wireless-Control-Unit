/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.h
 * @brief Header file providing prototypes of functions called by the rfRx task
 */

#ifndef __WCU_RFRX_CALLS_H_
#define __WCU_RFRX_CALLS_H_

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
#define RF_SPI_HANDLE    (hspi1)
#define RF_SPI_INSTANCE  (SPI1)

/**
 * @brief Internal messages enumeration for internal communication between rfRx task and callbacks
 */
typedef enum ERfRxInternalMail {
	ERfRxInternalMail_DataReady = 0,
	ERfRxInternalMail_PeriodElapsed
} ERfRxInternalMail;

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void);

/**
 * @brief Handles nRF905 communications
 * @retval None
 */
void rfRx_HandleCom(void);

#endif /* __WCU_RFRX_CALLS_H_ */
