/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.h
 * @brief Header file providing prototypes of functions called by the gnssRx task
 */

#ifndef __WCU_GNSSRX_CALLS_H_
#define __WCU_GNSSRX_CALLS_H_

#include "rt12e_libs_uartcircularbuffer.h"

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart3;
#define GNSS_UART_HANDLE	(huart3)
#define GNSS_UART_INSTANCE	(USART3)

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void);

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartCirBufRet Status
 */
EUartCirBufRet gnssRx_StartCircularBufferIdleDetectionRx(void);

/**
 * @brief Handles the GNSS message
 * @retval None
 */
void gnssRx_HandleMessage(void);

#endif /* __WCU_GNSSRX_CALLS_H_ */
