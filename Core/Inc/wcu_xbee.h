/**
 * @author Adrian Cinal
 * @file wcu_xbee.h
 * @brief Header file providing the external interface of the XBEE service
 */

#ifndef __WCU_XBEE_H_
#define __WCU_XBEE_H_

#include "wcu_defs.h"
#include "rt12e_libs_can.h"
#include <stdint.h>

/**
 * @brief XBEE service startup
 * @retval None
 */
void WcuXbeeStartup(void);

/**
 * @brief Handle sending/resending an acknowledge message
 * @param msgId ID of the message being acknowledged (R3TP VER byte)
 * @param seqNum Sequence number of the message being acknowledged
 * @retval None
 */
void WcuXbeeHandlePendingAcknowledge(uint8_t msgId, uint8_t seqNum);

/**
 * @brief Handle the pending RX message
 * @retval None
 */
void WcuXbeeHandlePendingRxMessage(void);

/**
 * @brief Handle the timer expired event
 * @retval None
 */
void WcuXbeeHandleTimerExpired(void);

/**
 * @brief Transmit CAN frame via R3TP to the XBEE receiver
 * @param canMessage Pointer to the CAN frame
 * @retval EWcuRet Status
 */
EWcuRet WcuXbeeSendTelemetryData(SCanMessage* canMessage);

#endif /* __WCU_XBEE_H_ */
