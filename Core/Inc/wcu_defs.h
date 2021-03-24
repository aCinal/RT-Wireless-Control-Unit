/**
 * @author Adrian Cinal
 * @file wcu_defs.h
 * @brief Header file containing WCU definitions and macros
 */

#ifndef __WCU_DEFS_H_
#define __WCU_DEFS_H_

#include <stddef.h>

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
    EWcuEventSignal_AdcConversionComplete = 0,
    EWcuEventSignal_BtRxMessagePending,
    EWcuEventSignal_CanRxMessagePending,
    EWcuEventSignal_DiagnosticsTimerExpired,
    EWcuEventSignal_GnssRxMessagePending,
	EWcuEventSignal_LogEntryPending,
	EWcuEventSignal_UartTxMessagePending,
	EWcuEventSignal_WatchdogTimerExpired,
    EWcuEventSignal_XbeeRxMessagePending,
    EWcuEventSignal_XbeeStatusTimerExpired,
} EWcuEventSignal;

typedef struct SWcuEvent {
    EWcuEventSignal signal;
    void *param;
} SWcuEvent;

#endif /* __WCU_DEFS_H_ */
