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

#define CRC_SEM_WAIT()            ( (void) osMutexWait(crcMutexHandle, osWaitForever) )       /* Acquire the CRC semaphore */
#define CRC_SEM_POST()            ( (void) osMutexRelease(crcMutexHandle) )                   /* Release the CRC semaphore */
#define LOG_TIMESTAMP_LENGTH      ( (uint32_t) 12 )                                           /* Length of the timestamp in decimal */
#define LOG_SEVERITY_TAG_LENGTH   ( (uint32_t) 4 )                                            /* Length of the severity level tag */
#define LOG_HEADER_LENGTH         ( LOG_TIMESTAMP_LENGTH + LOG_SEVERITY_TAG_LENGTH )          /* Total length of the log entry header */
#define LOG_TRAILER_LENGTH        ( (uint32_t) 3 )                                            /* <CR><LF><NUL> sequence length */
#define LOG_TIMESTAMP(log)        (log)                                                       /* Get pointer to the log entry timestamp */
#define LOG_SEVERITY_TAG(log)     ( &( (log)[LOG_TIMESTAMP_LENGTH] ) )                        /* Get pointer to the log entry severity tag */
#define LOG_PAYLOAD(log)          ( &( (log)[LOG_HEADER_LENGTH] ) )                           /* Get pointer to the log entry payload */
#define LOG_TRAILER(log, payLen)  ( &( (log)[LOG_HEADER_LENGTH + (payLen)] ) )                /* Get pointer to the log entry trailer */
#if (REDIRECT_LOGS_TO_SERIAL_PORT)
#define DBSERIAL_SEM_WAIT()       ( (void) osMutexWait(dbSerialMutexHandle, osWaitForever) )  /* Acquire the debug serial semaphore */
#define DBSERIAL_SEM_POST()       ( (void) osMutexRelease(dbSerialMutexHandle) )              /* Release the debug serial semaphore */
#define DEBUG_UART_HANDLE         (huart2)                                                    /* UART handle alias */
#define DEBUG_UART_INSTANCE       (USART2)                                                    /* UART instance alias */

extern UART_HandleTypeDef DEBUG_UART_HANDLE;
extern osMutexId dbSerialMutexHandle;
#endif /* (REDIRECT_LOGS_TO_SERIAL_PORT) */
extern CRC_HandleTypeDef hcrc;
extern osMutexId crcMutexHandle;
extern osMessageQId sdioLogQueueHandle;

/**
 * @brief Log an error message to the SD card
 * @param severityLevel Severity level
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(EWcuLogSeverityLevel severityLevel, const char *messagePayloadTbl) {

	size_t payloadLength = strlen(messagePayloadTbl);
	/* Allocate the memory for the error message */
	char *logEntryPtr = pvPortMalloc(
			(LOG_HEADER_LENGTH + payloadLength + LOG_TRAILER_LENGTH)
					* sizeof(char));

	/* Assert successful memory allocation */
	if (logEntryPtr != NULL) {

		/* Write the timestamp to the memory block */
		(void) sprintf(LOG_TIMESTAMP(logEntryPtr), "<%010lu>", HAL_GetTick());

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

#if !(REDIRECT_LOGS_TO_SERIAL_PORT)

		/* Push the pointer to the message to the logErrorQueue */
		if (pdPASS != xQueueSend(sdioLogQueueHandle, &logEntryPtr, 0)) {

			/* Cleanup on failure to push to queue */
			vPortFree(logEntryPtr);

		}

#else /* #if (REDIRECT_LOGS_TO_SERIAL_PORT) */

		/* Acquire the debug serial port semaphore */
		DBSERIAL_SEM_WAIT();

		/* Transmit the log entry via the serial port */
		HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*) logEntryPtr,
				strlen(logEntryPtr), WCU_COMMON_TIMEOUT);

		/* Release the debug serial port semaphore */
		DBSERIAL_SEM_POST();

		/* Cleanup */
		vPortFree(logEntryPtr);

#endif /* (REDIRECT_LOGS_TO_SERIAL_PORT) */

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
		LogError("SendToCan: Queue is full");

	}

}

/**
 * @brief Calculate the CRC of payload and return the 16 least significant bits
 * @param payloadPtr Payload
 * @param numOfBytes Number of bytes
 * @retval uint16_t 16 least significant bits of the CRC
 */
uint16_t GetR3tpCrc(uint8_t *payloadPtr, uint32_t numOfBytes) {

	/* Acquire the CRC semaphore */
	CRC_SEM_WAIT();

	/* Calculate the CRC */
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*) payloadPtr,
			numOfBytes / 4U);

	/* Release the CRC semaphore */
	CRC_SEM_POST();

	/* Return the lower 16 bits */
	return (crc & 0xFFFF);

}

