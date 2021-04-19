/**
 * @author Adrian Cinal
 * @file wcu_logger.c
 * @brief Source file implementing the logger service
 */

#include "wcu_logger.h"
#include "wcu_wrappers.h"
#include "wcu_events.h"
#include "wcu_sdio.h"
#include "wcu_diagnostics.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#if WCU_REDIRECT_LOGS_TO_SERIAL_PORT
#include "rt12e_libs_uartringbuffer_tx.h"
#endif /* WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

#define LOG_TIMESTAMP_LENGTH         ( (uint32_t) 12 )                                   /* Length of the timestamp in decimal */
#define LOG_SEVERITY_TAG_LENGTH      ( (uint32_t) 4 )                                    /* Length of the severity level tag */
#define LOG_HEADER_LENGTH            ( LOG_TIMESTAMP_LENGTH + LOG_SEVERITY_TAG_LENGTH )  /* Total length of the log entry header */
#define LOG_TRAILER_LENGTH           ( (uint32_t) 3 )                                    /* <CR><LF><NUL> sequence length */
#define LOG_TIMESTAMP(log)           (log)                                               /* Get pointer to the log entry timestamp */
#define LOG_SEVERITY_TAG(log)        ( &( (log)[LOG_TIMESTAMP_LENGTH] ) )                /* Get pointer to the log entry severity tag */
#define LOG_PAYLOAD(log)             ( &( (log)[LOG_HEADER_LENGTH] ) )                   /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)     ( &( (log)[LOG_HEADER_LENGTH + (payLen)] ) )        /* Get pointer to the log entry trailer */
#if WCU_REDIRECT_LOGS_TO_SERIAL_PORT
#define WCU_LOGGER_RING_BUFFER_SIZE  (512)                                               /* Logger ring buffer size */

SUartTxRb g_WcuLoggerTxRingBuffer;
static bool g_LoggerRbInitialized = false;
#endif /* WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

extern UART_HandleTypeDef huart2;

/**
 * @brief Log an error message
 * @param severityLevel Severity level
 * @param messagePayloadTbl Error message
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel,
		const char *messagePayloadTbl) {

	size_t payloadLength = strlen(messagePayloadTbl);
	size_t messageSize = LOG_HEADER_LENGTH + payloadLength + LOG_TRAILER_LENGTH;
	/* Allocate the memory for the error message */
	char *logEntryPtr = WcuMemAlloc(messageSize * sizeof(char));

	/* Assert successful memory allocation */
	if (logEntryPtr != NULL) {

		/* Write the timestamp to the memory block */
		(void) sprintf(LOG_TIMESTAMP(logEntryPtr), "<%010lu>",
				WcuGetUptimeInMs());

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
		if (EWcuRet_Ok
				== WcuEventSend(EWcuEventType_LogEntryPending, logEntryPtr)) {

			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(LoggerEntriesQueued);

		} else {

			/* Cleanup on failure to enqueue the event */
			WcuMemFree(logEntryPtr);
		}
	}
}

/**
 * @brief Commit a log entry to the SD card or to the serial port
 * @param log Log entry
 * @retval None
 */
void WcuLoggerCommitEntry(char *log) {

#if WCU_REDIRECT_LOGS_TO_SERIAL_PORT

	if (!g_LoggerRbInitialized) {

		static uint8_t ringbuffer[WCU_LOGGER_RING_BUFFER_SIZE];

		/* On first commit, initialize the ring buffer */
		if (EUartTxRbRet_Ok == UartTxRbInit(&g_WcuLoggerTxRingBuffer, &huart2, ringbuffer, sizeof(ringbuffer), NULL)) {

			g_LoggerRbInitialized = true;
		}
	}

	/* Write the error message to the ring buffer */
	(void) UartTxRbWrite(&g_WcuLoggerTxRingBuffer, (uint8_t*)log, strlen(log));

	/* Tell the dispatcher to initiate transmission */
	(void) WcuEventSend(EWcuEventType_UartTxMessagePending, &g_WcuLoggerTxRingBuffer);

	/* Free the allocated memory */
	WcuMemFree(log);

#else /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

	if (g_WcuLoggerReady) {

		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(LoggerEntriesCommitted);

		/* Write the error message to the file */
		(void) WcuSdioFileWrite(&g_WcuLogfileFd, (uint8_t*)log, strlen(log));

		/* Free the allocated memory */
		WcuMemFree(log);
	}

#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */
}
