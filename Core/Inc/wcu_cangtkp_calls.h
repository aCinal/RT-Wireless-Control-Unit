/**
 * @author Adrian Cinal
 * @file wcu_cangtkp_calls.h
 * @brief Header file providing prototypes of functions called by the canGtkp task
 */

#ifndef __WCU_CANGTKP_CALLS_H_
#define __WCU_CANGTKP_CALLS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid telemetry subscription stored on the SD card
 * @retval None
 */
void canGtkp_WaitSubscriptionFromSdioGtkp(void);

/**
 * @brief Handles the CAN outgoing messages
 * @retval None
 */
void canGtkp_HandleOutbox(void);

/**
 * @brief Handles the CAN incoming messages
 * @retval None
 */
void canGtkp_HandleInbox(void);

/**
 * @brief Handles setting CAN filters according to the new telemetry subscription
 * @retval None
 */
void canGtkp_HandleNewSubscription(void);

#endif /* __WCU_CANGTKP_CALLS_H_ */
