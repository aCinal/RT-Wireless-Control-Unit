/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeTxRx task
 */

#ifndef __WCU_XBEETXRX_CALLS_H_
#define __WCU_XBEETXRX_CALLS_H_

#include "rt12e_libs_uartringbuffer.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart4;
#define XBEE_UART_HANDLE    (huart4)
#define XBEE_UART_INSTANCE  (UART4)

/**
 * @brief Error code return value enumeration
 */
typedef enum EXbeeTxRxRet {
	EXbeeTxRxRet_Ok = 0,
	EXbeeTxRxRet_Error
} EXbeeTxRxRet;

/**
 * @brief Internal messages enumeration for internal communication between xbeeTxRx task and callbacks
 */
typedef enum EXbeeTxRxInternalMail {
	EXbeeTxRxInternalMail_MessageReceived = 0,
	EXbeeTxRxInternalMail_PeriodElapsed
} EXbeeTxRxInternalMail;

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet xbeeTxRx_StartRingBufferIdleDetectionRx(void);

/**
 * @brief Handles internal messages
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet xbeeTxRx_HandleInternalMail(void);

/**
 * @brief Handles transmitting telemetry data
 * @retval None
 */
void xbeeTxRx_HandleOutgoingR3tpCom(void);

/**
 * @brief Configures the XBEE Pro device
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet xbeeTxRx_DeviceConfig(void);

#endif /* __WCU_XBEETXRX_CALLS_H_ */
