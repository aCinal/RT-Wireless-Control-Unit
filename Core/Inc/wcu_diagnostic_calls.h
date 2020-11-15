/**
 * @author Adrian Cinal
 * @file wcu_Diagnosticcalls.h
 * @brief Header file providing prototypes of functions called by the diagnostic task
 */

#ifndef __WCU_DIAGNOSTIC_CALLS_H_
#define __WCU_DIAGNOSTIC_CALLS_H_

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
#define DIAGNOSTIC_ADC_HANDLE    (hadc1)
#define DIAGNOSTIC_ADC_INSTANCE  (ADC1)

/**
 * @brief Run diagnostics on the MCU and transmit the data via CAN bus
 * @retval None
 */
void DiagnosticRunDiagnostics(void);

/**
 * @brief Callback on ADC conversion complete
 * @retval None
 */
void DiagnosticAdcConvCpltcallback(void);

#endif /* __WCU_DIAGNOSTIC_CALLS_H_ */
