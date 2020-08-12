/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.h
 * @brief Header file providing prototypes of functions called by the sdioGtkp task
 */

#ifndef __WCU_SDIOGTKP_CALLS_H_
#define __WCU_SDIOGTKP_CALLS_H_

#include "fatfs.h"

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(FIL *fp, const TCHAR *path);

#endif /* __WCU_SDIOGTKP_CALLS_H_ */
