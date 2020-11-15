/**
 * @author Adrian Cinal
 * @file wcu_btrx_calls.h
 * @brief Header file providing prototypes of functions called by the btRx task
 */

#ifndef __WCU_BTRX_CALLS_H_
#define __WCU_BTRX_CALLS_H_

#include "rt12e_libs_uartringbuffer.h"

/**
 * @brief Error code return value enumeration
 */
typedef enum EBtRxRet {
	EBtRxRet_Ok = 0,
	EBtRxRet_Error
} EBtRxRet;

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet BtRxStartRingBufferIdleDetectionRx(void);

/**
 * @brief Handle the BT message
 * @retval EBtRxRet Status
 */
EBtRxRet BtRxHandleCom(void);

#endif /* __WCU_BTRX_CALLS_H_ */
