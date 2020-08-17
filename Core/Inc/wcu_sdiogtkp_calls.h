/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.h
 * @brief Header file providing prototypes of functions called by the sdioGtkp task
 */

#ifndef __WCU_SDIOGTKP_CALLS_H_
#define __WCU_SDIOGTKP_CALLS_H_

#include "cmsis_os.h"

extern osMessageQId sdioSubQueueHandle;
extern osMessageQId sdioLogQueueHandle;

#define WCU_SDIOGTKP_LOGFILE_PATH					("ERRLOG.TXT")			/* Error log file path */
#define WCU_SDIOGTKP_SUBFILE_PATH					("SUBSCR")				/* Subscription file path */

/**
 * @brief Handles the error logger
 * @retval None
 */
void sdioGtkp_HandleLogger(void);

/**
 * @brief Handles saving the new telemetry subscription to the SD card
 */
void sdioGtkp_HandleNewSubscription(void);

/**
 * @brief Tries loading the telemetry subscription from the SD card and pushing it to an appropriate queue
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(void);

#endif /* __WCU_SDIOGTKP_CALLS_H_ */