/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.h
 * @brief Header file providing prototypes of functions called by the sdioGtkp task
 */

#ifndef __WCU_SDIOGTKP_CALLS_H_
#define __WCU_SDIOGTKP_CALLS_H_

#include <inttypes.h>

/**
 * @brief Error code return value enumeration
 */
typedef enum ESdioGtkpRet {
	ESdioGtkpRet_Ok = 0,
	ESdioGtkpRet_Error
} ESdioGtkpRet;

/**
 * @brief Try loading the telemetry subscription from the SD card and forwarding it to CAN gatekeeper
 * @retval ESdioGtkpRet Status
 */
ESdioGtkpRet SdioGtkpLoadTelemetrySubscription(void);

/**
 * @brief Handle the error logger
 * @retval None
 */
void SdioGtkpHandleLogger(void);

/**
 * @brief Handle saving the new telemetry subscription to the SD card
 * @retval ESdioGtkpRet Status
 */
ESdioGtkpRet SdioGtkpHandleNewSubscription(void);

#endif /* __WCU_SDIOGTKP_CALLS_H_ */
