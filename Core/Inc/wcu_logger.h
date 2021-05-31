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
#define WCU_ENABLE_DEBUG_PRINTS           1
#define WCU_REDIRECT_LOGS_TO_SERIAL_PORT  1

/* Exported macros -------------------------------------------------------------------------- */
#if (WCU_ENABLE_INFO_PRINTS)
#define WcuLogInfo(msg)      ( WcuLoggerPrint(EWcuLogSeverityLevel_Info, (msg) ) )
#else /* #if !(WCU_ENABLE_INFO_PRINTS) */
#define WcuLogInfo(msg)     ( (void)(msg) )
#endif /* !(WCU_ENABLE_INFO_PRINTS) */
#if (WCU_ENABLE_ERROR_PRINTS)
#define WcuLogError(msg)     ( WcuLoggerPrint(EWcuLogSeverityLevel_Error, (msg) ) )
#else /* #if !(WCU_ENABLE_ERROR_PRINTS) */
#define WcuLogError(msg)     ( (void)(msg) )
#endif /* !(WCU_ENABLE_ERROR_PRINTS) */
#if (WCU_ENABLE_DEBUG_PRINTS)
#define WcuLogDebug(msg)     ( WcuLoggerPrint(EWcuLogSeverityLevel_Debug, (msg) ) )
#else /* #if !(WCU_ENABLE_DEBUG_PRINTS) */
#define WcuLogDebug(msg)     ( (void)(msg) )
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
 * @param messagePayloadTbl Error message
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel, const char *messagePayloadTbl);

/**
 * @brief Flush the ringbuffer, thereby committing log entries to the SD card or to the serial port
 * @retval None
 */
void WcuLoggerFlushRingBuffer(void);

#endif /* __WCU_LOGGER_H_ */
