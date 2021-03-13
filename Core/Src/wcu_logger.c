/**
 * @author Adrian Cinal
 * @file wcu_logger.c
 * @brief Source file implementing the logger service
 */

#include "wcu_logger.h"
#include "wcu_wrappers.h"
#include "wcu_events.h"
#include <stdio.h>
#include <string.h>
#include "wcu_sdio.h"

#define LOG_TIMESTAMP_LENGTH      ( (uint32_t) 12 )                                           /* Length of the timestamp in decimal */
#define LOG_SEVERITY_TAG_LENGTH   ( (uint32_t) 4 )                                            /* Length of the severity level tag */
#define LOG_HEADER_LENGTH         ( LOG_TIMESTAMP_LENGTH + LOG_SEVERITY_TAG_LENGTH )          /* Total length of the log entry header */
#define LOG_TRAILER_LENGTH        ( (uint32_t) 3 )                                            /* <CR><LF><NUL> sequence length */
#define LOG_TIMESTAMP(log)        (log)                                                       /* Get pointer to the log entry timestamp */
#define LOG_SEVERITY_TAG(log)     ( &( (log)[LOG_TIMESTAMP_LENGTH] ) )                        /* Get pointer to the log entry severity tag */
#define LOG_PAYLOAD(log)          ( &( (log)[LOG_HEADER_LENGTH] ) )                           /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)  ( &( (log)[LOG_HEADER_LENGTH + (payLen)] ) )                /* Get pointer to the log entry trailer */

/**
 * @brief Log an error message
 * @param severityLevel Severity level
 * @param messagePayloadTbl Error message
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel, const char *messagePayloadTbl) {

	size_t payloadLength = strlen(messagePayloadTbl);
	size_t messageSize = LOG_HEADER_LENGTH + payloadLength + LOG_TRAILER_LENGTH;
	/* Allocate the memory for the error message */
	char *logEntryPtr = WcuMemAlloc(messageSize * sizeof(char));

	/* Assert successful memory allocation */
	if (logEntryPtr != NULL) {

		/* Write the timestamp to the memory block */
		(void) sprintf(LOG_TIMESTAMP(logEntryPtr), "<%010lu>", WcuGetUptimeInMs());

		/* Write the severity tag to the memory block */
		switch (severityLevel) {

		case EWcuLogSeverityLevel_Info:

			(void) sprintf(LOG_SEVERITY_TAG(logEntryPtr), "INF ");
			break;

		case EWcuLogSeverityLevel_Error:

			(void) sprintf(LOG_SEVERITY_TAG(logEntryPtr), "ERR ");
			break;

		case EWcuLogSeverityLevel_Debug:

			(void) sprintf(LOG_SEVERITY_TAG(logEntryPtr), "DBG ");
			break;

		default:

			(void) sprintf(LOG_SEVERITY_TAG(logEntryPtr), "--- ");
			break;
		}

		/* Write the message payload to the memory block */
		(void) sprintf(LOG_PAYLOAD(logEntryPtr), (messagePayloadTbl));
		/* Write the message trailer to the memory block */
		(void) sprintf(LOG_TRAILER(logEntryPtr, payloadLength), "\r\n");

		/* Send the event */
		if (EWcuRet_Ok != WcuEventSend(EWcuEventSignal_PendingLogEntry, logEntryPtr)) {

			/* Cleanup on failure */
			WcuMemFree(logEntryPtr);
		}
	}
}

/**
 * @brief Commit a log entry to the SD card or to the serial port
 * @param log Log entry
 * @retval None
 */
void WcuLoggerCommitEntry(char* log) {

	if(g_WcuLoggerReady) {

		UINT bytesWritten;
		/* Write the error message to the file */
		(void) f_write(&g_WcuLogfileFd, log, strlen(log),
				&bytesWritten);

		/* Free the allocated memory */
		WcuMemFree(log);
	}
}
