/**
 * @author Adrian Cinal
 * @file wcu_base.h
 * @brief Header file containing basic WCU definitions and macros
 */

#ifndef __WCU_BASE_H_
#define __WCU_BASE_H_

#include "cmsis_os.h"
#include "rt12e_libs_can.h"
#include <stdio.h>
#include <string.h>

/* External variables -------------------------------------------------------------------------- */
extern osMessageQId sdioLogQueueHandle;
extern osMessageQId canTxQueueHandle;

/* Exported defines -------------------------------------------------------------------------- */
/**
 * @brief Notification values
 * @note Name of each definition adheres to the format: WCU_NV_[taker]_[giver]_[description (optional)],
 *       where [taker] is the task receiving the notification, [giver] is the task/function sending the notification
 */
#define WCU_NV_IWDGGTKP_CANGTKP						(0x00000001UL)			/* canGtkp task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_BTRX						(0x00000002UL)			/* btRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_GNSSRX						(0x00000004UL)			/* gnssRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_RFRX						(0x00000008UL)			/* rfRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_XBEETXRX					(0x00000010UL)			/* xbeeTxRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL		(29UL)					/* Value to notify canGtkp that reading subscription from SD card failed */
#define WCU_NV_RFRX_SPI_RX_CPLT_CB					(0x00000001UL)			/* Value to notify rfRx from HAL_SPI_RxCpltCallback */

/**
 * @brief Definitions increasing code clarity
 */
#define CLEAR_NO_BITS_ON_ENTRY						(0x00000000UL)			/* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_NO_BITS_ON_EXIT						(0x00000000UL)			/* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_ENTRY						(0xFFFFFFFFUL)			/* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_EXIT						(0xFFFFFFFFUL)			/* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */

#define WCU_DEFAULT_TIMEOUT							(10)					/* Default timeout */
#define WCU_DEFAULT_TASKDELAY						(10)					/* Default task delay */

/* Exported macros -------------------------------------------------------------------------- */
/**
 * @brief Logs an error message to the SD card
 */
#define LOGERROR(messageTable) do { \
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
			/* Cleanup on failure to push to queue */ \
			vPortFree(errMsgPtr);\
		} \
		\
	} \
	\
} while(0)

/**
 * @brief Pushes a CAN frame to the canTxQueue
 */
#define ADDTOCANTXQUEUE(canFramePtr, errMsgTable) do { \
	\
	/* Push the frame to the queue */ \
	if (pdPASS != xQueueSend(canTxQueueHandle, canFramePtr, 0)) { \
		\
		/* Log the error */ \
		LOGERROR(errMsgTable); \
		\
	} \
	\
} while(0)

#endif /* __WCU_BASE_H_ */
