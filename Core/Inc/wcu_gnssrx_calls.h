/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.h
 * @brief Header file providing prototypes of functions called by the gnssRx task
 */

#ifndef __WCU_GNSSRX_CALLS_H_
#define __WCU_GNSSRX_CALLS_H_

#include "rt12e_libs_uartringbuffer.h"

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart3;
#define GNSS_UART_HANDLE    (huart3)
#define GNSS_UART_INSTANCE  (USART3)

/**
 * @brief Error code return value enumeration
 */
typedef enum EGnssRxRet {
	EGnssRxRet_Ok = 0,
	EGnssRxRet_Error
} EGnssRxRet;

/**
 * @brief Configures the Quectel L26 device
 * @retval EGnssRxRet Status
 */
EGnssRxRet gnssRx_DeviceConfig(void);

/**
 * @brief Starts listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet gnssRx_StartRingBufferIdleDetectionRx(void);

/**
 * @brief Handles the GNSS message
 * @retval Status
 */
EGnssRxRet gnssRx_HandleCom(void);

#endif /* __WCU_GNSSRX_CALLS_H_ */
