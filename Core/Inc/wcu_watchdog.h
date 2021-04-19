/**
 * @author Adrian Cinal
 * @file wcu_watchdog.h
 * @brief Header file providing the external interface of the watchdog service
 */

#ifndef __WCU_WATCHDOG_H
#define __WCU_WATCHDOG_H

/**
 * @brief Watchdog service startup
 * @retval None
 */
void WcuWatchdogStartup(void);

/**
 * @brief Reload the IWDG counter
 * @retval None
 */
void WcuWatchdogReload(void);

#endif /* __WCU_WATCHDOG_H */
