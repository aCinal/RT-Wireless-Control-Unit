/**
 * @author Adrian Cinal
 * @file wcu_events.h
 * @brief Header file containing declarations of event dispatcher functions and relevant macros
 */

#ifndef __WCU_EVENTS_H_
#define __WCU_EVENTS_H_

#include "wcu_defs.h"

/**
 * @brief Event type
 */
typedef enum EWcuEventType {
	EWcuEventType_AdcConversionComplete = 0,
	EWcuEventType_BtRxMessagePending,
	EWcuEventType_CanError,
	EWcuEventType_CanRxMessagePending,
	EWcuEventType_GnssRxMessagePending,
	EWcuEventType_LogEntriesPending,
	EWcuEventType_TimerExpired,
	EWcuEventType_UartTxMessagePending,
	EWcuEventType_XbeeRxMessagePending,
	EWcuEventType_XbeeAcknowledgePending
} EWcuEventType;

/**
 * @brief Event structure
 */
typedef struct SWcuEvent {
	EWcuEventType type;
	void *paramPtr;
	uint32_t paramUint;
} SWcuEvent;

/**
 * @brief  Function implementing the EventDispatcher thread.
 * @param  argument: Not used
 * @retval None
 */
void WcuEventDispatcherEntryPoint(void const *argument);

/**
 * @brief Create and send event
 * @param type Event type
 * @param paramPtr Pointer parameter
 * @param paramUint Integer parameter
 * @retval EWcuRet Status
 */
EWcuRet WcuEventSend(EWcuEventType type, void *paramPtr, uint32_t paramUint);

#endif /* __WCU_EVENTS_H_ */
