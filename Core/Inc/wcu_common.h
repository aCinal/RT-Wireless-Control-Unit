/**
 * @author Adrian Cinal
 * @file wcu_common.h
 * @brief Header file containing definitions and macros common to all WCU tasks
 */

#ifndef __WCU_COMMON_H_
#define __WCU_COMMON_H_

#include "cmsis_os.h"
#include "rt12e_libs_can.h"
#include <stdio.h>
#include <string.h>

/* External variables -------------------------------------------------------------------------- */
extern osMessageQId sdioLogQueueHandle;
extern osMessageQId canTxQueueHandle;

/* Exported typedefs -------------------------------------------------------------------------- */
typedef float float32_t;
typedef double float64_t;

/* Exported defines -------------------------------------------------------------------------- */
/**
 * @brief Definitions increasing code clarity
 */
#define CLEAR_NO_BITS_ON_ENTRY						(0x00000000UL)			/* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_NO_BITS_ON_EXIT						(0x00000000UL)			/* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_ENTRY						(0xFFFFFFFFUL)			/* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_EXIT						(0xFFFFFFFFUL)			/* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */

/* Exported macros -------------------------------------------------------------------------- */
/**
 * @brief Logs an error message to the SD card
 */
#define LogError(messageTable) do { \
	\
	/* Allocate the memory for the error message (timestamp length + message length + NULL character) */ \
	char* errMsgPtr = pvPortMalloc(11UL + strlen(messageTable) + 1UL); \
	/* Assert successful memory allocation */ \
	if(errMsgPtr != NULL) { \
		\
		/* Write the timestamp to the memory block */ \
		(void) sprintf(errMsgPtr, "%010lu ", HAL_GetTick()); \
		/* Write the message to the memory block */ \
		(void) sprintf(errMsgPtr + 11UL, messageTable); \
		/* Push the pointer to the message to the logErrorQueue */ \
		if(pdPASS != xQueueSend(sdioLogQueueHandle, &errMsgPtr, 0)) { \
			\
			/* Cleanup on failure to push to queue */ \
			vPortFree(errMsgPtr);\
			\
		} \
		\
	} \
	\
} while(0)

/**
 * @brief Pushes a CAN frame to the canTxQueue
 */
#define AddToCanTxQueue(canFramePtr, errMsgTable) do { \
	\
	/* Push the frame to the queue */ \
	if (pdPASS != xQueueSend(canTxQueueHandle, canFramePtr, 0)) { \
		\
		/* Log the error */ \
		LogError(errMsgTable); \
		\
	} \
	\
} while(0)

#endif /* __WCU_COMMON_H_ */
