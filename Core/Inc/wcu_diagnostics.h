/**
 * @author Adrian Cinal
 * @file wcu_diagnostics.h
 * @brief Header file providing the external interface of the diagnostic service
 */

#ifndef __WCU_DIAGNOSTICS_H_
#define __WCU_DIAGNOSTICS_H_

/**
 * @brief Diagnostic service startup
 * @retval None
 */
void WcuDiagnosticsStartup(void);

/**
 * @brief Handle the timer expired event
 * @retval None
 */
void WcuDiagnosticsHandleTimerExpired(void);

/**
 * @brief Handle the ADC conversion complete event
 * @retval None
 */
void WcuDiagnosticsHandleAdcConversionComplete(void);

#endif /* __WCU_DIAGNOSTICS_H_ */
