/**
 * @author Adrian Cinal
 * @file wcu_common.c
 * @brief Source file implementing functions common to all WCU tasks
 */

#include "wcu_common.h"

#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

extern osMessageQId sdioLogQueueHandle;

#define LOG_HEAD_LENGTH  ((uint32_t) 11)  /* Length of the timestamp in decimal */
#define LOG_TAIL_LENGTH  ((uint32_t) 3)   /* <CR><LF> and NULL character sequence */

/**
 * @brief Logs an error message to the SD card
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(const char messagePayloadTbl[]) {

	size_t payloadLength = strlen(messagePayloadTbl);
	/* Allocate the memory for the error message */
	char *logEntryPtr = pvPortMalloc(
			LOG_HEAD_LENGTH + payloadLength + LOG_TAIL_LENGTH);
	/* Assert successful memory allocation */
	if (logEntryPtr != NULL) {

		/* Write the message head to the memory block */
		(void) sprintf(logEntryPtr, "%010lu ", HAL_GetTick());
		/* Write the message payload to the memory block */
		(void) sprintf(logEntryPtr + LOG_HEAD_LENGTH, (messagePayloadTbl));
		/* Write the message tail to the memory block */
		(void) sprintf(logEntryPtr + LOG_HEAD_LENGTH + payloadLength, "\r\n");

		/* Push the pointer to the message to the logErrorQueue */
		if (pdPASS != xQueueSend(sdioLogQueueHandle, &logEntryPtr, 0)) {

			/* Cleanup on failure to push to queue */
			vPortFree(logEntryPtr);

		}

	}

}
