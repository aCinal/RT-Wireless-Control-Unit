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
	uint32_t CanQueueStarvations;
	uint32_t CanMessagesSent;
	uint32_t CanErrors;

	uint32_t BtMessagesForwarded;
	uint32_t BtInvalidMessagesReceived;

	uint32_t GnssParserFramesCompleted;
	uint32_t GnssParserErrors;

	uint32_t XbeeMessagesSent;
	uint32_t XbeeTransmitRingbufferStarvations;
	uint32_t XbeeDriverWarningMessagesReceived;
	uint32_t XbeeNewSubscriptionMessagesReceived;
	uint32_t XbeeInvalidMessagesReceived;

	uint32_t LoggerEntriesQueued;
	uint32_t LoggerCommits;

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
