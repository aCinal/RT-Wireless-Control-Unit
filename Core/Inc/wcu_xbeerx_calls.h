/**
 * @author Adrian Cinal
 * @file wcu_xbeerx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeRx task
 */

#ifndef __WCU_XBEERX_CALLS_H_
#define __WCU_XBEERX_CALLS_H_

#include "cmsis_os.h"

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeRx_WaitSubscriptionFromSdioGtkp(void);

/**
 * @brief Receives and handles the telemetry subscription via UART
 * @param buff UART Rx buffer of size R3TP_VER1_MAX_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveSubscription(uint8_t buff[]);

/**
 * @brief Receives the warning for the driver via UART
 * @param buff UART Rx buffer of size R3TP_VER2_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveWarning(uint8_t buff[]);

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count);

#endif /* __WCU_XBEERX_CALLS_H_ */
