/**
 * @author Adrian Cinal
 * @file wcu_diagnostics.h
 * @brief Header file providing the external interface of the diagnostic service
 */

#ifndef __WCU_DIAGNOSTICS_H_
#define __WCU_DIAGNOSTICS_H_

#include <stdint.h>

/**
 * @brief WCU runtime database for debugging
 */
typedef struct SWcuDiagnosticsDatabase {
	uint32_t EventsDispatched;
	uint32_t EventsNotSent;
	uint32_t CanMessagesReceived;
	uint32_t CanMessagesSent;
	uint32_t BtMessagesForwarded;
	uint32_t BtDroppedMessages;
	uint32_t GnssNmeaParserErrorCount;
	uint32_t GnssNmeaParserDataReadyCount;
	uint32_t GnssNmeaParserDataNotReadyCount;
	uint32_t GnssLargestBurstRead;
	uint32_t XbeeTelemetryMessagesSent;
	uint32_t XbeeAcknowledgeMessagesSent;
	uint32_t XbeeDriverWarningMessagesReceived;
	uint32_t XbeeNewSubscriptionMessagesReceived;
	uint32_t XbeeDroppedMessages;
	uint32_t LoggerEntriesQueued;
	uint32_t LoggerEntriesCommitted;
	uint32_t WatchdogRefreshCount;
} SWcuDiagnosticsDatabase;

extern SWcuDiagnosticsDatabase g_WcuDiagnosticsDatabase;

#define WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(COUNTER)  (g_WcuDiagnosticsDatabase.COUNTER++)
#define WCU_DIAGNOSTICS_DATABASE_STORE_IF_GREATER_THAN_CURRENT(FIELD, VALUE) \
	(g_WcuDiagnosticsDatabase.FIELD = ( (VALUE) > g_WcuDiagnosticsDatabase.FIELD ) ? (VALUE) : g_WcuDiagnosticsDatabase.FIELD )

/**
 * @brief Diagnostic service startup
 * @retval None
 */
void WcuDiagnosticsStartup(void);

/**
 * @brief Handle the timer expired event
 * @retval None
 */
void WcuDiagnosticsHandleTimerExpired(void);

/**
 * @brief Handle the ADC conversion complete event
 * @retval None
 */
void WcuDiagnosticsHandleAdcConversionComplete(void);

#endif /* __WCU_DIAGNOSTICS_H_ */
