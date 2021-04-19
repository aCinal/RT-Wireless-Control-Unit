/**
 * @author Adrian Cinal
 * @file wcu_diagnostics.c
 * @brief Source file implementing the diagnostic service
 */

#include "wcu_diagnostics.h"
#include "wcu_wrappers.h"
#include "wcu_defs.h"
#include "wcu_can.h"
#include "rt12e_libs_generic.h"

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>

#define CAN_ID_WCU_DIAG   ( (uint32_t) 0x733 )            /* CAN ID: _733_WCU_DIAG */
#define CAN_DLC_WCU_DIAG  ( (uint32_t) 4 )

#define VDD               ( (float32_t) 3.3 )             /* Supply voltage */
#define V25               ( (float32_t) 0.76 )            /* Temperature sensor's voltage at 25 degrees Celsius */
#define AVG_SLOPE         ( (float32_t) (2.5 / 1000.0) )  /* Average slope: 2.5 mV/degreeC */

/* Convert 12-bit ADC value to floating-point input voltage */
#define Adc12BitToVoltage(val)  ( (float32_t) (VDD * ( (float32_t)(val) / (float32_t) 0x0FFFU ) ) )
/* Convert sensed voltage to temperature in degrees Celsius */
#define SensedVoltageToDegreesCelsius(volt)  ( (float32_t) ( ( ( (float32_t)(volt) - V25 ) / AVG_SLOPE ) + 25.0F ) )

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim11;

static uint16_t g_AdcBuffer;

SWcuDiagnosticsDatabase g_WcuDiagnosticsDatabase;

/**
 * @brief Diagnostic service startup
 * @retval None
 */
void WcuDiagnosticsStartup(void) {

	/* Start the timer */
	(void) HAL_TIM_Base_Start_IT(&htim11);
}

/**
 * @brief Handle the timer expired event
 * @retval None
 */
void WcuDiagnosticsHandleTimerExpired(void) {

	/* Start the ADC conversion and return */
	(void) HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&g_AdcBuffer, sizeof(g_AdcBuffer));
}

/**
 * @brief Handle the ADC conversion complete event
 * @retval None
 */
void WcuDiagnosticsHandleAdcConversionComplete(void) {

	SCanMessage canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = CAN_DLC_WCU_DIAG;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = CAN_ID_WCU_DIAG;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Calculate the sensed voltage */
	float32_t Vsense = Adc12BitToVoltage(g_AdcBuffer);

	/* Calculate the MCU temperature based on the sensed voltage */
	float32_t floatTemperature = SensedVoltageToDegreesCelsius(Vsense);

	/* Normalize the temperature to fit it in the CAN frame */
	int16_t mcuTemperature = (int16_t) lround(floatTemperature * 10.0);

	/* Calculate the MCU uptime in seconds */
	uint16_t mcuUptime = (uint16_t) (WcuGetUptimeInMs() / 1000UL);

	/* Write the MCU temperature to the frame payload */
	canMessage.PayloadTbl[0] = _getbyte(mcuTemperature, 1);
	canMessage.PayloadTbl[1] = _getbyte(mcuTemperature, 0);

	/* Write the MCU uptime to the frame payload */
	canMessage.PayloadTbl[2] = _getbyte(mcuUptime, 1);
	canMessage.PayloadTbl[3] = _getbyte(mcuUptime, 0);

	/* Transmit the frame */
	(void) WcuCanMessageSend(&canMessage);
}
