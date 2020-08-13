/**
 * @author Adrian Cinal
 * @file wcu_xbeerx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeRx task
 */

#ifndef __WCU_XBEERX_CALLS_H_
#define __WCU_XBEERX_CALLS_H_

#include "cmsis_os.h"

#define WCU_XBEETXRX_UART_RX_TIMEOUT					(500U)			/* UART Rx timeout */
#define WCU_XBEETXRX_UART_RX_CLEANUP_TIMEOUT			(10U)			/* UART Rx cleanup timeout */


/**
 * @brief Receives the warning for the driver via UART
 * @param buff UART Rx buffer of size R3TP_VER2_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveWarning(uint8_t buff[]);


#endif /* __WCU_XBEERX_CALLS_H_ */
