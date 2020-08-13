/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.h
 * @brief Header file providing prototypes of functions called by the rfRx task
 */

#ifndef __WCU_RFRX_CALLS_H_
#define __WCU_RFRX_CALLS_H_

#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
#define RF_SPI_HANDLE hspi1
#define RF_SPI_INSTANCE SPI1

#define WCU_RFRX_SPI_RX_BUFF_SIZE	32UL	/* SPI Rx buffer size */
#define WCU_RFRX_SPI_TX_TIMEOUT		50		/* SPI Tx timeout */

/**
 * @brief Listens for and handles the RF message
 * @retval None
 */
void rfRx_HandleMessage(void);

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void);

#endif /* __WCU_RFRX_CALLS_H_ */
