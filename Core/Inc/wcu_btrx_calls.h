/**
 * @author Adrian Cinal
 * @file wcu_btrx_calls.h
 * @brief Header file providing prototypes of functions called by the btRx task
 */

#ifndef __WCU_BTRX_CALLS_H_
#define __WCU_BTRX_CALLS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart1;
#define BT_UART_HANDLE (huart1)
#define BT_UART_INSTANCE (USART1)

extern osMutexId crcMutexHandle;
extern CRC_HandleTypeDef hcrc;

/**
 * @brief Listens for and handles the BT message
 * @retval None
 */
void btRx_HandleMessage(void);

#endif /* __WCU_BTRX_CALLS_H_ */
