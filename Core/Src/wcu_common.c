/**
 * @author Adrian Cinal
 * @file wcu_common.c
 * @brief Source file implementing functions common to all WCU tasks
 */

#include "wcu_common.h"
#include "rt12e_libs_generic.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define CRC_SEM_WAIT()            ( (void) osMutexWait(crcMutexHandle, osWaitForever) )  /* Acquire the CRC semaphore */
#define CRC_SEM_POST()            ( (void) osMutexRelease(crcMutexHandle) )              /* Release the CRC semaphore */
#define LOG_HEADER_LENGTH         ( (uint32_t) 11 )                                      /* Length of the timestamp in decimal */
#define LOG_TRAILER_LENGTH        ( (uint32_t) 3 )                                       /* <CR><LF><NUL> sequence length */
#define LOG_PAYLOAD(log)          ( &( (log)[LOG_HEADER_LENGTH] ) )                      /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)  ( &( (log)[LOG_HEADER_LENGTH + (payLen)] ) )           /* Get pointer to the log entry trailer */
#if defined (USE_SERIAL_DEBUG_PRINTS)
#define DEBUG_UART_HANDLE         (huart2)                                               /* UART handle alias */
#define DEBUG_UART_INSTANCE       (USART2)                                               /* UART instance alias */

extern UART_HandleTypeDef DEBUG_UART_HANDLE;
#endif /* defined (USE_SERIAL_DEBUG_PRINTS) */
extern CRC_HandleTypeDef hcrc;
extern osMutexId crcMutexHandle;
extern osMessageQId sdioLogQueueHandle;

/**
 * @brief Log an error message to the SD card
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(const char *messagePayloadTbl) {

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

#if defined(USE_SERIAL_DEBUG_PRINTS)
	taskENTER_CRITICAL();

	HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)logEntryPtr, strlen(logEntryPtr), WCU_COMMON_TIMEOUT);
	vPortFree(logEntryPtr);

	taskEXIT_CRITICAL();
#else
		/* Push the pointer to the message to the logErrorQueue */
		if (pdPASS != xQueueSend(sdioLogQueueHandle, &logEntryPtr, 0)) {

			/* Cleanup on failure to push to queue */
			vPortFree(logEntryPtr);

		}
#endif /* !defined (USE_SERIAL_DEBUG_PRINTS) */

	}

}

/**
 * @brief Add the CAN frame to canTxQueue
 * @param canFramePtr Pointer to the CAN frame structure
 * @retval None
 */
void SendToCan(SCanFrame *canFramePtr) {

	/* Push the frame to the queue */
	if (pdPASS != xQueueSend(canTxQueueHandle, canFramePtr, WCU_COMMON_TIMEOUT)) {

		/* Log the error */
		LogPrint("SendToCan: Queue is full");

	}

}

/**
 * @brief Calculate the CRC of payload and return the 16 least significant bits
 * @param payloadPtr Payload
 * @param numOfBytes Number of bytes
 * @retval uint16_t 16 least significant bits of the CRC
 */
uint16_t GetR3tpCrc(uint8_t *payloadPtr, uint32_t numOfBytes) {

	/* Acquire the semaphore */
	CRC_SEM_WAIT();

	/* Calculate the CRC */
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) payloadPtr,
			numOfBytes / 4U);

	/* Release the semaphore */
	CRC_SEM_POST();

	return _bits0_15(crc);

}
