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

#define CAN_ID_WCU_DIAG  ((uint32_t) 0x733)  /* CAN ID: _733_WCU_DIAG */

/**
 * @brief Runs diagnostics on the MCU and transmits the data via CAN bus
 */
void diagnostic_RunDiagnostics(void) {

	static SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 4;
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

		/* Temperature sensor characteristics */
		static const float32_t VDD = 3.3; /* Supply voltage */
		static const float32_t V25 = 0.76; /* Voltage at 25 degreeC */
		static const float32_t AVG_SLOPE = 2.5 / 1000.0; /* Average slope: 2.5 mV/degreeC */

		/* Calculate the sensed voltage */
		float32_t Vsense = temperatureSensorAdcBuff / 4095.0 * VDD;

		/* Calculate the MCU temperature based on the sensed voltage */
		float32_t floatTemperature = ((Vsense - V25) / AVG_SLOPE) + 25.0;

		static int16_t mcuTemperature;
		/* Normalize the temperature to fit it in the CAN frame */
		mcuTemperature = (int16_t) lround(floatTemperature * 10.0);

		static uint16_t mcuUptime;
		/* Calculate the MCU uptime in seconds */
		mcuUptime = (uint16_t) (HAL_GetTick() / 1000UL);

		/* Write the MCU temperature to the frame payload */
		canFrame.PayloadTbl[0] = _bits8_15(mcuTemperature);
		canFrame.PayloadTbl[1] = _bits0_7(mcuTemperature);

		/* Write the MCU uptime to the frame payload */
		canFrame.PayloadTbl[2] = _bits8_15(mcuUptime);
		canFrame.PayloadTbl[3] = _bits0_7(mcuUptime);

		/* Transmit the frame */
		AddToCanTxQueue(&canFrame,
				"diagnostic failed to send to canTxQueue");

	}

}
