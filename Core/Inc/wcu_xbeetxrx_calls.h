/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeTxRx task
 */

#ifndef __WCU_XBEETXRX_CALLS_H_
#define __WCU_XBEETXRX_CALLS_H_

#include "rt12e_libs_uartcircularbuffer.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart4;
#define XBEE_UART_HANDLE    (huart4)
#define XBEE_UART_INSTANCE  (UART4)

/**
 * @brief Internal messages enumeration for internal communication between xbeeTxRx task and callbacks
 */
typedef enum EXbeeInternalMail {
	EXbeeInternalMail_PeriodElapsed = 0,
	EXbeeInternalMail_MessageReceived
} EXbeeInternalMail;

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartCirBufRet Status
 */
EUartCirBufRet xbeeTxRx_StartCircularBufferIdleDetectionRx(void);

/**
 * @brief Handles internal messages
 * @retval None
 */
void xbeeTxRx_HandleInternalMail(void);

/**
 * @brief Handles transmitting telemetry data
 * @retval None
 */
void xbeeTxRx_HandleOutgoingR3tpComms(void);

/**
 * @brief Configures the XBEE Pro device
 * @retval None
 */
void xbeeTxRx_DeviceConfig(void);

#endif /* __WCU_XBEETXRX_CALLS_H_ */
