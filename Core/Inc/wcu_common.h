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
#define USE_SERIAL_DEBUG_PRINTS
//#undef USE_SERIAL_DEBUG_PRINTS

/* Exported macros -------------------------------------------------------------------------- */
/* Set/reset pins based on the label */
#define SET_PIN(label)    ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET) )
#define RESET_PIN(label)  ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET) )

/* Exported function prototypes -------------------------------------------------------------------------- */
/**
 * @brief Log an error message to the SD card
 * @param messagePayloadTbl Error message
 * @retval None
 */
void LogPrint(const char *messagePayloadTbl);

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
