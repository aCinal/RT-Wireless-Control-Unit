/**
 * @author Adrian Cinal
 * @file wcu_diagnostic_calls.c
 * @brief Source file defining functions called by the diagnostic task
 */

#include "wcu_diagnostic_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"

#include <math.h>

#define CAN_ID_WCU_DIAG   ( (uint32_t) 0x733 )            /* CAN ID: _733_WCU_DIAG */
#define CAN_DLC_WCU_DIAG  ( (uint32_t) 4 )

#define VDD               ( (float32_t) 3.3 )             /* Supply voltage */
#define V25               ( (float32_t) 0.76)             /* Temperature sensor's voltage at 25 degreeC */
#define AVG_SLOPE         ( (float32_t) (2.5 / 1000.0) )  /* Average slope: 2.5 mV/degreeC */

/* Convert 12-bit ADC value to floating-point input voltage */
#define _adc12BitToVoltage(val)  ( (float32_t) (val * (VDD / (float32_t) 0x0FFFU ) ) )

extern osThreadId diagnosticHandle;

/**
 * @brief Run diagnostics on the MCU and transmit the data via CAN bus
 * @retval None
 */
void diagnostic_RunDiagnostics(void) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = CAN_DLC_WCU_DIAG;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_WCU_DIAG;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	static uint16_t temperatureSensorAdcBuff;
	/* Start the ADC */
	(void) HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &temperatureSensorAdcBuff, 1);

	/* Wait for the conversion complete callback to notify the task */
	if (0UL < ulTaskNotifyTake(pdTRUE,
	portMAX_DELAY)) {

		/* Calculate the sensed voltage */
		float32_t Vsense = _adc12BitToVoltage(temperatureSensorAdcBuff);

		/* Calculate the MCU temperature based on the sensed voltage */
		float32_t floatTemperature = ((Vsense - V25) / AVG_SLOPE) + 25.0;

		/* Normalize the temperature to fit it in the CAN frame */
		int16_t mcuTemperature = (int16_t) lround(floatTemperature * 10.0);

		/* Calculate the MCU uptime in seconds */
		uint16_t mcuUptime = (uint16_t) (HAL_GetTick() / 1000UL);

		/* Write the MCU temperature to the frame payload */
		canFrame.PayloadTbl[0] = _bits8_15(mcuTemperature);
		canFrame.PayloadTbl[1] = _bits0_7(mcuTemperature);

		/* Write the MCU uptime to the frame payload */
		canFrame.PayloadTbl[2] = _bits8_15(mcuUptime);
		canFrame.PayloadTbl[3] = _bits0_7(mcuUptime);

		/* Transmit the frame */
		SendToCan(&canFrame);

	}

}

/**
 * @brief Callback on ADC conversion complete
 * @retval None
 */
void diagnostic_AdcConvCpltcallback(void) {

	/* Notify the diagnostic task */
	vTaskNotifyGiveFromISR(diagnosticHandle, NULL);

}
