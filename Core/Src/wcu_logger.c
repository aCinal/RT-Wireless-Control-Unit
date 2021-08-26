/**
 * @author Adrian Cinal
 * @file wcu_logger.c
 * @brief Source file implementing the logger service
 */

#include "wcu_logger.h"
#include "wcu_events.h"
#include "wcu_sdio.h"
#include "wcu_diagnostics.h"
#include "wcu_mem.h"
#include "wcu_utils.h"
#include "rt12e_libs_tx_ringbuffer.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"

#define LOG_TIMESTAMP_LENGTH           ( (uint32_t) 12 )                                   /* Length of the timestamp in decimal */
#define LOG_SEVERITY_TAG_LENGTH        ( (uint32_t) 4 )                                    /* Length of the severity level tag */
#define LOG_HEADER_LENGTH              ( LOG_TIMESTAMP_LENGTH + LOG_SEVERITY_TAG_LENGTH )  /* Total length of the log entry header */
#define LOG_TRAILER_LENGTH             ( (uint32_t) 3 )                                    /* <CR><LF><NUL> sequence length */
#define WCU_LOGGER_RING_BUFFER_SIZE    (512)                                               /* Logger ring buffer size */
#define WCU_LOGGER_MAX_PAYLOAD_SIZE    (256)                                               /* Maximum size of user data in a single log entry */

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
 * @param messagePayloadTbl User payload
 * @param ... (optional parameters) Format specifiers
 * @retval None
 */
void WcuLoggerPrint(EWcuLogSeverityLevel severityLevel,
		const char *messagePayloadTbl, ...) {

#if !WCU_REDIRECT_LOGS_TO_SERIAL_PORT
	/* If the SDIO service is not up, return as quickly as possible */
	if (g_WcuSdioReady)
#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */
	{
		/* Stack-allocated buffer for sprintf use */
		char fmtbuf[WCU_LOGGER_MAX_PAYLOAD_SIZE];

		/* Prepare the log entry payload */
		va_list args;
		va_start(args, messagePayloadTbl);
		size_t payloadLength = snprintf(fmtbuf, WCU_LOGGER_MAX_PAYLOAD_SIZE,
				messagePayloadTbl, args);
		va_end(args);

		/* Assert null-terminated payload string */
		fmtbuf[WCU_LOGGER_MAX_PAYLOAD_SIZE - 1] = '\0';

		size_t messageSize = LOG_HEADER_LENGTH + payloadLength
				+ LOG_TRAILER_LENGTH;
		/* Allocate memory for the error message */
		char *logEntryPtr = WcuMemAlloc(messageSize * sizeof(char));

		/* Assert successful memory allocation */
		if (logEntryPtr != NULL) {

			const char *tag = NULL;
			/* Determine the severity tag */
			switch (severityLevel) {

			case EWcuLogSeverityLevel_Info:

				tag = "INF ";
				break;

			case EWcuLogSeverityLevel_Error:

				tag = "ERR ";
				break;

			case EWcuLogSeverityLevel_Debug:

				tag = "DBG ";
				break;

			default:

				tag = "??? ";
				break;
			}

			char *iter = logEntryPtr;
			/* Write the timestamp to the memory block */
			iter += sprintf(iter, "<%010lu>", WcuGetUptimeInMs());
			/* Write the severity tag to the memory block */
			iter += sprintf(iter, tag);
			/* Write the message payload to the memory block */
			iter += sprintf(iter, fmtbuf);
			/* Write the message trailer to the memory block */
			iter += sprintf(logEntryPtr, "\r\n");

			/* Write the error message to the ring buffer */
			if (ETxRbRet_Ok
					== TxRbWrite(&g_WcuLoggerTxRingbuffer,
							(uint8_t*) logEntryPtr, strlen(logEntryPtr))) {

				WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(LoggerEntriesQueued);

			} else {

				WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(
						LoggerRingbufferStarvations);
			}

			/* Send the event to the dispatcher to flush the ring buffer even if the write failed
			 * - this ensures the buffer gets flushed eventually even if the event queue was full
			 * when the buffer was first filled */
			(void) WcuEventSend(EWcuEventType_LogEntriesPending,
					&g_WcuLoggerTxRingbuffer, 0);

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
					sizeof(ringbuffer), WcuLoggerTxRingbufferRouter, NULL,
					WcuMemAlloc, WcuMemFree)) {

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
