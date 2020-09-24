/**
 * @author Adrian Cinal
 * @file wcu_common.c
 * @brief Source file implementing functions common to all WCU tasks
 */

#include "wcu_common.h"

#include "cmsis_os.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

extern osMessageQId sdioLogQueueHandle;

#define LOG_HEADER_LENGTH         ((uint32_t) 11)                       /* Length of the timestamp in decimal */
#define LOG_TRAILER_LENGTH        ((uint32_t) 3)                        /* <CR><LF><NUL> sequence length */
#define LOG_PAYLOAD(log)          (&((log)[LOG_HEADER_LENGTH]))           /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)  (&((log)[LOG_HEADER_LENGTH + (payLen)]))  /* Get pointer to the log entry trailer */

/**
 * @brief Logs an error message to the SD card
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(const char messagePayloadTbl[]) {

	size_t payloadLength = strlen(messagePayloadTbl);
	/* Allocate the memory for the error message */
	char *logEntryPtr = pvPortMalloc(
			(LOG_HEADER_LENGTH + payloadLength + LOG_TRAILER_LENGTH)
					* sizeof(char));
	/* Assert successful memory allocation */
	if (logEntryPtr != NULL) {

		/* Write the message header to the memory block */
		(void) sprintf(logEntryPtr, "%010lu ", HAL_GetTick());
		/* Write the message payload to the memory block */
		(void) sprintf(LOG_PAYLOAD(logEntryPtr), (messagePayloadTbl));
		/* Write the message trailer to the memory block */
		(void) sprintf(LOG_TRAILER(logEntryPtr, payloadLength), "\r\n");

		/* Push the pointer to the message to the logErrorQueue */
		if (pdPASS != xQueueSend(sdioLogQueueHandle, &logEntryPtr, 0)) {

			/* Cleanup on failure to push to queue */
			vPortFree(logEntryPtr);

		}

	}

}
