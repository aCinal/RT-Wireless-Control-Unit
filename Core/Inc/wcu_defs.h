/**
 * @author Adrian Cinal
 * @file wcu_defs.h
 * @brief Header file containing WCU definitions and macros
 */

#ifndef __WCU_DEFS_H_
#define __WCU_DEFS_H_

#include <stddef.h>

/* Exported macros -------------------------------------------------------------------------- */
#define WCU_IWDG_SLEEP_TIME  (5000)

/* Exported typedefs -------------------------------------------------------------------------- */
typedef float float32_t;
typedef double float64_t;

typedef enum EWcuRet {
    EWcuRet_Ok = 0,
    EWcuRet_InvalidParams,
    EWcuRet_Error
} EWcuRet;

typedef enum EWcuLogSeverityLevel {
	EWcuLogSeverityLevel_Info = 0,
	EWcuLogSeverityLevel_Error,
	EWcuLogSeverityLevel_Debug
} EWcuLogSeverityLevel;

typedef enum EWcuEventSignal {
	EWcuEventSignal_Init = 0,
	EWcuEventSignal_WatchdogWakeup,
	EWcuEventSignal_PendingLogEntry,
    EWcuEventSignal_CanPendingMessage,
    EWcuEventSignal_BtPendingMessage,
    EWcuEventSignal_GnssPendingMessage,
    EWcuEventSignal_XbeePendingMessage,
    EWcuEventSignal_XbeeStatusTimerExpired,
    EWcuEventSignal_DiagnosticsTimerExpired,
    EWcuEventSignal_AdcConversionComplete,
} EWcuEventSignal;

typedef struct SWcuEvent {
    EWcuEventSignal signal;
    void *param;
} SWcuEvent;

#endif /* __WCU_DEFS_H_ */
