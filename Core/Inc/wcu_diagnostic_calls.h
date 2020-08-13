/**
 * @author Adrian Cinal
 * @file wcu_diagnostic_calls.h
 * @brief Header file providing prototypes of functions called by the diagnostic task
 */

#ifndef __WCU_DIAGNOSTIC_CALLS_H_
#define __WCU_DIAGNOSTIC_CALLS_H_

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
#define DIAGNOSTIC_ADC_HANDLE hadc1
#define DIAGNOSTIC_ADC_INSTANCE	ADC1

extern UART_HandleTypeDef huart4;
#define XBEE_UART_HANDLE huart4
#define XBEE_UART_INSTANCE UART4

#define WCU_CAN_ID_WCU_DIAG		(uint32_t)(0x733UL)		/* CAN ID: _733_WCU_DIAG */

/**
 * @brief Runs diagnostics on the MCU and transmits the data via CAN bus
 */
void diagnostic_RunDiagnostics(void);

#endif /* __WCU_DIAGNOSTIC_CALLS_H_ */
