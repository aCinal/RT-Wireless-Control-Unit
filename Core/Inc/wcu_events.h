/**
 * @author Adrian Cinal
 * @file wcu_events.h
 * @brief Header file containing declarations of event dispatcher functions and relevant macros
 */

#ifndef __WCU_EVENTS_H_
#define __WCU_EVENTS_H_

#include "wcu_defs.h"

/**
  * @brief  Function implementing the dispatcherThread thread.
  * @param  argument: Not used
  * @retval None
  */
void DispatcherThreadEntryPoint(void const * argument);

/**
 * @brief Create and send event
 * @param signal Event type
 * @param param Pointer to event parameters
 * @retval EWcuRet Status
 */
EWcuRet WcuEventSend(EWcuEventType signal, void *param);

#endif /* __WCU_EVENTS_H_ */
