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
	uint32_t CanMessagesDropped;
	uint32_t CanMessagesSent;
	uint32_t BtMessagesForwarded;
	uint32_t BtMessagesDropped;
	uint32_t GnssParserErrorCount;
	uint32_t GnssParserDataReadyCount;
	uint32_t GnssParserDataNotReadyCount;
	uint32_t XbeeDriverWarningMessagesReceived;
	uint32_t XbeeNewSubscriptionMessagesReceived;
	uint32_t XbeeMessagesDropped;
	uint32_t LoggerEntries;
	uint32_t WatchdogRefreshCount;
} SWcuDiagnosticsDatabase;

extern SWcuDiagnosticsDatabase g_WcuDiagnosticsDatabase;

#define WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(COUNTER)  (g_WcuDiagnosticsDatabase.COUNTER++)

/**
 * @brief Diagnostic service startup
 * @retval None
 */
void WcuDiagnosticsStartup(void);

/**
 * @brief Initiate the self-check
 * @retval None
 */
void WcuDiagnosticsStartSelfCheck(void);

/**
 * @brief Handle the ADC conversion complete event
 * @retval None
 */
void WcuDiagnosticsHandleAdcConversionComplete(void);

/**
 * @brief Log runtime database snapshot
 * @retval None
 */
void WcuDiagnosticsLogDatabaseSnapshot(void);

#endif /* __WCU_DIAGNOSTICS_H_ */
