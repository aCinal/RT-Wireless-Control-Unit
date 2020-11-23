/**
 * @author Adrian Cinal
 * @file wcu_common.h
 * @brief Header file containing definitions and macros common to all WCU tasks
 */

#ifndef __WCU_COMMON_H_
#define __WCU_COMMON_H_

#include "rt12e_libs_can.h"

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/* External variables -------------------------------------------------------------------------- */
extern osMessageQId canTxQueueHandle;

/* Exported typedefs -------------------------------------------------------------------------- */
typedef float float32_t;
typedef double float64_t;

typedef enum EWcuLogSeverityLevel {
	EWcuLogSeverityLevel_Info = 0,
	EWcuLogSeverityLevel_Error,
	EWcuLogSeverityLevel_Debug
} EWcuLogSeverityLevel;

/* Exported defines -------------------------------------------------------------------------- */
#define WCU_COMMON_TIMEOUT         ( (TickType_t) pdMS_TO_TICKS(50) )   /* Common timeout */
#define WCU_COMMON_TASK_DELAY      ( (TickType_t) pdMS_TO_TICKS(1) )    /* Common task delay */

#define CLEAR_NO_BITS_ON_ENTRY     ( (uint32_t) 0x00000000UL )        /* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_NO_BITS_ON_EXIT      ( (uint32_t) 0x00000000UL )        /* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_ENTRY    ( (uint32_t) 0xFFFFFFFFUL )        /* Value to pass as ulBitsToClearOnEntry to xTaskNotifyWait */
#define CLEAR_ALL_BITS_ON_EXIT     ( (uint32_t) 0xFFFFFFFFUL )        /* Value to pass as ulBitsToClearOnExit to xTaskNotifyWait */

/* Conditional compilation flags */
#define RT11
#define RT12e
#undef RT12e
#if defined(RT11) && defined(RT12e)
#error Both RT11 and RT12e defined
#endif /* defined(RT11) && defined(RT12e) */

/**
 * @brief Set the LOG_REDIRECT_TO_SERIAL_PORT macro to redirect selected logs to the serial port
 */
#define REDIR_INF  (0x01)
#define REDIR_ERR  (0x02)
#define REDIR_DBG  (0x04)
#define LOG_REDIRECT_TO_SERIAL_PORT (0 | REDIR_INF | REDIR_ERR | REDIR_DBG)
#if !defined (LOG_REDIRECT_TO_SERIAL_PORT)
#error If log redirection to the serial port is not used, set LOG_REDIRECT_TO_SERIAL_PORT to zero
#endif /* !defined (LOG_REDIRECT_TO_SERIAL_PORT) */

/* Exported macros -------------------------------------------------------------------------- */
/* Set/reset pins based on the label */
#define SET_PIN(label)    ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET) )
#define RESET_PIN(label)  ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET) )
#define LogInfo(msg)      ( LogPrint(EWcuLogSeverityLevel_Info, (msg) ) )
#define LogError(msg)     ( LogPrint(EWcuLogSeverityLevel_Error, (msg) ) )
#define LogDebug(msg)     ( LogPrint(EWcuLogSeverityLevel_Debug, (msg) ) )

/* Exported function prototypes -------------------------------------------------------------------------- */
/**
 * @brief Log an error message to the SD card
 * @param severityLevel Severity level
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(EWcuLogSeverityLevel severityLevel, const char *messagePayloadTbl);

/**
 * @brief Add the CAN frame to canTxQueue
 * @param canFramePtr Pointer to the CAN frame structure
 * @retval None
 */
void SendToCan(SCanFrame *canFramePtr);

/**
 * @brief Calculate the CRC of payload and return the 16 least significant bits
 * @param payloadPtr Payload
 * @param numOfBytes Number of bytes
 * @retval uint16_t 16 least significant bits of the CRC
 */
uint16_t GetR3tpCrc(uint8_t *payloadPtr, uint32_t numOfBytes);

#endif /* __WCU_COMMON_H_ */
