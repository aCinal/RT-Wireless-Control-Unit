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
#include "rt12e_libs_tx_ringbuffer.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#define LOG_TIMESTAMP_LENGTH         ( (uint32_t) 12 )                                   /* Length of the timestamp in decimal */
#define LOG_SEVERITY_TAG_LENGTH      ( (uint32_t) 4 )                                    /* Length of the severity level tag */
#define LOG_HEADER_LENGTH            ( LOG_TIMESTAMP_LENGTH + LOG_SEVERITY_TAG_LENGTH )  /* Total length of the log entry header */
#define LOG_TRAILER_LENGTH           ( (uint32_t) 3 )                                    /* <CR><LF><NUL> sequence length */
#define LOG_TIMESTAMP(log)           (log)                                               /* Get pointer to the log entry timestamp */
#define LOG_SEVERITY_TAG(log)        ( &( (log)[LOG_TIMESTAMP_LENGTH] ) )                /* Get pointer to the log entry severity tag */
#define LOG_PAYLOAD(log)             ( &( (log)[LOG_HEADER_LENGTH] ) )                   /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)     ( &( (log)[LOG_HEADER_LENGTH + (payLen)] ) )        /* Get pointer to the log entry trailer */
#define WCU_LOGGER_RING_BUFFER_SIZE  (512)                                               /* Logger ring buffer size */

#if WCU_REDIRECT_LOGS_TO_SERIAL_PORT
extern UART_HandleTypeDef huart2;
#endif /* WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

STxRb g_WcuLoggerTxRingbuffer;

static inline EWcuRet WcuLoggerTxRingbufferInit(void);
static ETxRbRet WcuLoggerTxRingbufferRouter(uint8_t *data, size_t len);

/**
 * @brief Logger service startup
 * @retval None
 */
void WcuLoggerStartup(void) {

	/* Initialize the ring buffer */
	(void) WcuLoggerTxRingbufferInit();
}

/**
 * @brief Log an error message
 * @param severityLevel Severity level
 * @param messagePayloadTbl Error message
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel,
		const char *messagePayloadTbl) {

#if !WCU_REDIRECT_LOGS_TO_SERIAL_PORT
	/* If the SDIO service is not up, return as quickly as possible */
	if (g_WcuSdioReady)
#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */
	{
		size_t payloadLength = strlen(messagePayloadTbl);
		size_t messageSize = LOG_HEADER_LENGTH + payloadLength
				+ LOG_TRAILER_LENGTH;
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

				(void) sprintf(LOG_SEVERITY_TAG(logEntryPtr), "??? ");
				break;
			}

			/* Write the message payload to the memory block */
			(void) sprintf(LOG_PAYLOAD(logEntryPtr), (messagePayloadTbl));
			/* Write the message trailer to the memory block */
			(void) sprintf(LOG_TRAILER(logEntryPtr, payloadLength), "\r\n");

			/* Write the error message to the ring buffer */
			if (ETxRbRet_Ok
					== TxRbWrite(&g_WcuLoggerTxRingbuffer,
							(uint8_t*) logEntryPtr, strlen(logEntryPtr))) {

				/* If write was successful, send the event to the dispatcher to flush the ring buffer */
				(void) WcuEventSend(EWcuEventType_LogEntriesPending,
						&g_WcuLoggerTxRingbuffer, 0);
				WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(LoggerEntriesQueued);

			} else {

				/* Increment ringbuffer starvation counter */
				WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(
						LoggerRingbufferStarvations);
			}

			/* Free the memory */
			WcuMemFree(logEntryPtr);
		}
	}
}

/**
 * @brief Flush the ringbuffer, thereby committing log entries to the SD card or to the serial port
 * @retval None
 */
void WcuLoggerFlushRingbuffer(void) {

	ETxRbRet status = ETxRbRet_Ok;

	/* Flush the ring buffer */
	status = TxRbFlush(&g_WcuLoggerTxRingbuffer);

	if (ETxRbRet_Busy == status) {

		/* If the router is busy, enqueue the event again */
		(void) WcuEventSend(EWcuEventType_LogEntriesPending,
				&g_WcuLoggerTxRingbuffer, 0);
	}

#if !WCU_REDIRECT_LOGS_TO_SERIAL_PORT

	if (ETxRbRet_Ok == status) {

		/* When committing logs to an SD card, the transfer is blocking so call the callback directly here instead of in IRQ */
		(void) TxRbCallback(&g_WcuLoggerTxRingbuffer);
	}

#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */
}

/**
 * @brief Initialize the logger ring buffer
 * @retval EWcuRet Status
 */
static inline EWcuRet WcuLoggerTxRingbufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_LOGGER_RING_BUFFER_SIZE];

	/* Initialize the ring buffer */
	if (ETxRbRet_Ok
			!= TxRbInit(&g_WcuLoggerTxRingbuffer, ringbuffer,
					sizeof(ringbuffer), WcuLoggerTxRingbufferRouter, NULL)) {

		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief TX ring buffer router
 * @param data Data to be committed
 * @param len Length of the data
 * @retval ETxRbRet Status
 */
static ETxRbRet WcuLoggerTxRingbufferRouter(uint8_t *data, size_t len) {

	ETxRbRet status = ETxRbRet_Ok;

#if WCU_REDIRECT_LOGS_TO_SERIAL_PORT

	if (HAL_OK != HAL_UART_Transmit_DMA(&huart2, data, len)) {

		status = ETxRbRet_Error;
	}

#else /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

	FIL fd;
	/* Open the logfile for writing in append mode */
	if (EWcuRet_Ok != WcuSdioFileOpen(&fd, WCU_LOG_PATH,
	FA_WRITE | FA_OPEN_APPEND)) {

		status = ETxRbRet_Error;
	}

	if (ETxRbRet_Ok == status) {

		/* Write the error message to the file */
		if (EWcuRet_Ok != WcuSdioFileWrite(&fd, (uint8_t*) data, len)) {

			status = ETxRbRet_Error;
		}
	}

	/* Close the file on cleanup */
	(void) WcuSdioFileClose(&fd);

#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

	return status;
}

/**
 * @brief TX ring buffer callback
 * @retval None
 */
void WcuLoggerTxRingbufferCallback(void) {

	/* Increment statistics counter */
	WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(LoggerCommits);
}
