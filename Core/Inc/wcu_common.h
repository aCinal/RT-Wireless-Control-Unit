/**
 * @author Adrian Cinal
 * @file wcu_common.h
 * @brief Header file containing definitions and macros common to all WCU tasks
 */

#ifndef __WCU_COMMON_H_
#define __WCU_COMMON_H_

#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"

#include "cmsis_os.h"
#include <stdint.h>

/* External variables -------------------------------------------------------------------------- */
extern osMessageQId canTxQueueHandle;
extern osMutexId crcMutexHandle;
extern CRC_HandleTypeDef hcrc;

/* Exported typedefs -------------------------------------------------------------------------- */
typedef float float32_t;
typedef double float64_t;

/* Exported defines -------------------------------------------------------------------------- */
/**
 * @brief Definitions increasing code clarity
 */
#define CLEAR_NO_BITS_ON_ENTRY   ((uint32_t) 0x00000000UL)  /* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_NO_BITS_ON_EXIT    ((uint32_t) 0x00000000UL)  /* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_ENTRY  ((uint32_t) 0xFFFFFFFFUL)  /* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_EXIT   ((uint32_t) 0xFFFFFFFFUL)  /* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */

/* Exported function prototypes */

/**
 * @brief Logs an error message to the SD card
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(const char messagePayloadTbl[]);

/* Exported macros -------------------------------------------------------------------------- */

/**
 * @brief Push a CAN frame to the canTxQueue
 */
#define AddToCanTxQueue(canFramePtr, errMsgTbl) do { \
	/* Push the frame to the queue */ \
	if (pdPASS != xQueueSend(canTxQueueHandle, canFramePtr, 0)) { \
		/* Log the error */ \
		LogPrint(errMsgTbl); \
	} \
} while(0)

/**
 * @brief Acquire the CRC semaphore
 */
#define CRC_SEM_WAIT() ((void) osMutexWait(crcMutexHandle, osWaitForever))

/**
 * @brief Release the CRC semaphore
 */
#define CRC_SEM_POST() ((void) osMutexRelease(crcMutexHandle))

/**
 * @brief Calculate the CRC
 */
#define GET_CRC(payload, bytes) (_bits0_15(HAL_CRC_Calculate(&hcrc, (uint32_t*)payload, bytes / 4U)))

#endif /* __WCU_COMMON_H_ */
