/**
 * @author Adrian Cinal
 * @file wcu_btrx_calls.h
 * @brief Header file providing prototypes of functions called by the btRx task
 */

#ifndef __WCU_BTRX_CALLS_H_
#define __WCU_BTRX_CALLS_H_

#include "rt12e_libs_uartcircularbuffer.h"

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;
#define BT_UART_HANDLE    (huart1)
#define BT_UART_INSTANCE  (USART1)

/**
 * @brief Error code return value enumeration
 */
typedef enum EBtRxRet {
	EBtRxRet_Ok = 0,
	EBtRxRet_Error
} EBtRxRet;

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartCirBufRet Status
 */
EUartCirBufRet btRx_StartCircularBufferIdleDetectionRx(void);

/**
 * @brief Handles the BT message
 * @EBtRxRet Status
 */
EBtRxRet btRx_HandleCom(void);

#endif /* __WCU_BTRX_CALLS_H_ */
