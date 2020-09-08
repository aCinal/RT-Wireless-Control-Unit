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

/**
 * @brief Logs an error message to the SD card
 * @param msgTbl Error message
 * @retval None
 */
void LogPrint(const char msgTbl[]) {

	/* Allocate the memory for the error message (timestamp length + message length + NULL character) */
	char* logEntryPtr = pvPortMalloc(11UL + strlen(msgTbl) + 1UL);
	/* Assert successful memory allocation */
	if(logEntryPtr != NULL) {

		/* Write the timestamp to the memory block */
		(void) sprintf(logEntryPtr, "%010lu ", HAL_GetTick());
		/* Write the message to the memory block */
		(void) sprintf(logEntryPtr + 11UL, (msgTbl));
		/* Push the pointer to the message to the logErrorQueue */
		if(pdPASS != xQueueSend(sdioLogQueueHandle, &logEntryPtr, 0)) {

			/* Cleanup on failure to push to queue */
			vPortFree(logEntryPtr);

		}

	}

}

