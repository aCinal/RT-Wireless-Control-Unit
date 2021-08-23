/**
 * @author Adrian Cinal
 * @file wcu_logger.h
 * @brief Header file providing the external interface of the logger service
 */

#ifndef __WCU_LOGGER_H_
#define __WCU_LOGGER_H_

#include "wcu_defs.h"

/* Compilation flags -------------------------------------------------------------------------- */
#define WCU_ENABLE_INFO_PRINTS            1
#define WCU_ENABLE_ERROR_PRINTS           1
#define WCU_ENABLE_DEBUG_PRINTS           0
#define WCU_REDIRECT_LOGS_TO_SERIAL_PORT  0

/* Exported typedefs -------------------------------------------------------------------------- */
/**
 * @brief Log entry severity level
 */
typedef enum EWcuLogSeverityLevel {
	EWcuLogSeverityLevel_Info = 0,
	EWcuLogSeverityLevel_Error,
	EWcuLogSeverityLevel_Debug
} EWcuLogSeverityLevel;

/* Exported macros an definitions -------------------------------------------------------------------------- */
#if (WCU_ENABLE_INFO_PRINTS)
#define WcuLogInfo(msg, ...)     ( WcuLoggerPrint(EWcuLogSeverityLevel_Info, (msg), ##__VA_ARGS__) )
#else /* #if !(WCU_ENABLE_INFO_PRINTS) */
#define WcuLogInfo(msg, ...)
#endif /* !(WCU_ENABLE_INFO_PRINTS) */
#if (WCU_ENABLE_ERROR_PRINTS)
#define WcuLogError(msg, ...)     ( WcuLoggerPrint(EWcuLogSeverityLevel_Error, (msg), ##__VA_ARGS__) )
#else /* #if !(WCU_ENABLE_ERROR_PRINTS) */
#define WcuLogError(msg, ...)
#endif /* !(WCU_ENABLE_ERROR_PRINTS) */
#if (WCU_ENABLE_DEBUG_PRINTS)
#define WcuLogDebug(msg, ...)     ( WcuLoggerPrint(EWcuLogSeverityLevel_Debug, (msg), ##__VA_ARGS__) )
#else /* #if !(WCU_ENABLE_DEBUG_PRINTS) */
#define WcuLogDebug(msg, ...)
#endif /* !(WCU_ENABLE_DEBUG_PRINTS) */

/* Public functions prototypes -------------------------------------------------------------------------- */

/**
 * @brief Logger service startup
 * @retval None
 */
void WcuLoggerStartup(void);

/**
 * @brief Log an error message
 * @param severityLevel Severity level
 * @param messagePayloadTbl User payload
 * @param ... (optional parameters) Format specifiers
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel, const char *messagePayloadTbl, ...);

/**
 * @brief Flush the ringbuffer, thereby committing log entries to the SD card or to the serial port
 * @retval None
 */
void WcuLoggerFlushRingbuffer(void);

#endif /* __WCU_LOGGER_H_ */
