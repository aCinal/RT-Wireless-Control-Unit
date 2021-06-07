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

/**
 * @brief WCU functions return value
 */
typedef enum EWcuRet {
    EWcuRet_Ok = 0,
    EWcuRet_InvalidParams,
    EWcuRet_Error
} EWcuRet;

/**
 * @brief Log entry severity level
 */
typedef enum EWcuLogSeverityLevel {
	EWcuLogSeverityLevel_Info = 0,
	EWcuLogSeverityLevel_Error,
	EWcuLogSeverityLevel_Debug
} EWcuLogSeverityLevel;

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
} EWcuEventType;

/**
 * @brief Event structure
 */
typedef struct SWcuEvent {
    EWcuEventType signal;
    void *param;
} SWcuEvent;

#endif /* __WCU_DEFS_H_ */
