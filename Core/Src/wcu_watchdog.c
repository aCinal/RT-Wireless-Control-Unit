/**
 * @author Adrian Cinal
 * @file wcu_watchdog.c
 * @brief Source file implementing the watchdog service
 */

#include "wcu_watchdog.h"
#include "wcu_diagnostics.h"
#include "stm32f4xx_hal.h"

extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim11;

/**
 * @brief Watchdog service startup
 * @retval None
 */
void WcuWatchdogStartup(void) {

	/* Start the timer */
	(void) HAL_TIM_Base_Start_IT(&htim11);
}

/**
 * @brief Reload the IWDG counter
 * @retval None
 */
void WcuWatchdogReload(void) {

	(void) HAL_IWDG_Refresh(&hiwdg);
	WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(WatchdogRefreshCount);
}
