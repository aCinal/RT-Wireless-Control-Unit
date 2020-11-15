/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeTxRx task
 */

#ifndef __WCU_XBEETXRX_CALLS_H_
#define __WCU_XBEETXRX_CALLS_H_

#include "rt12e_libs_uartringbuffer.h"

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
 * @brief Configure the XBEE-Pro device
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet XbeeTxRxDeviceConfig(void);

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet XbeeTxRxStartRingBufferIdleDetectionRx(void);

/**
 * @brief Handle internal messages
 * @retval EXbeeTxRxRet Status
 */
EXbeeTxRxRet XbeeTxRxHandleInternalMail(void);

/**
 * @brief Handle transmitting telemetry data
 * @retval None
 */
void XbeeTxRxHandleOutgoingR3tpCom(void);

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void XbeeTxRxPeriodElapsedCallback(void);

#endif /* __WCU_XBEETXRX_CALLS_H_ */
