/**
 * @author Adrian Cinal
 * @file wcu_can.h
 * @brief Header file providing the external interface of the CAN service
 */

#ifndef __WCU_CAN_H_
#define __WCU_CAN_H_

#include "wcu_defs.h"
#include "rt12e_libs_can.h"

/**
 * @brief CAN service startup
 * @retval None
 */
void WcuCanStartup(void);

/**
 * @brief Handle pending CAN message
 * @param fifo CAN FIFO identifier
 * @retval None
 */
void WcuCanHandlePendingMessage(uint32_t fifo);

/**
 * @brief Send the message to the CAN bus
 * @param message CAN frame
 * @retval EWcuRet Status
 */
EWcuRet WcuCanMessageSend(SCanFrame* message);

/**
 * @brief Set the new telemetry subscription
 * @param messageIds Array of message IDs
 * @param count Number of IDs in the messageIds array
 * @retval EWcuRet Status
 */
void WcuCanSetNewSubscription(uint32_t *messageIds, uint32_t count);

#endif /* __WCU_CAN_H_ */
