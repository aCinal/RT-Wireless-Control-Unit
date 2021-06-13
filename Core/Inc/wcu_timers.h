/**
 * @author Adrian Cinal
 * @file wcu_timers.h
 * @brief Header file defining aliases for HW timer instances in use by the WCU application
 */

#ifndef __WCU_TIMERS_H_
#define __WCU_TIMERS_H_

#include "stm32f4xx_hal.h"

#define WCU_XBEE_STATUS_TIMER             (TIM7)
#define WCU_WATCHDOG_RELOAD_TIMER         (TIM11)
#define WCU_DIAGNOSTICS_SELFCHECK_TIMER   (TIM13)
#define WCU_DIAGNOSTICS_SNAPSHOT_TIMER    (TIM14)

#define WCU_EVENT_TIMER_INSTANCE(EVENTPARAM)  ( ( (TIM_HandleTypeDef*) (EVENTPARAM) )->Instance )

#endif /* __WCU_TIMERS_H_ */
