/**
 * @author Adrian Cinal
 * @file wcu_diagnostics.c
 * @brief Source file implementing the diagnostic service
 */

#include "wcu_diagnostics.h"
#include "wcu_defs.h"
#include "wcu_can.h"
#include "wcu_logger.h"
#include "rt12e_libs_generic.h"

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <wcu_utils.h>

#define WCU_CAN_ID_WCU_DIAG   ( (uint32_t) 0x733 )        /* CAN ID: _733_WCU_DIAG */
#define WCU_CAN_DLC_WCU_DIAG  ( (uint32_t) 4 )

#define WCU_DATABASE_SNAPSHOT_SW_PRESCALER  ( (uint32_t) 3 )

#define VDD               ( (float32_t) 3.3 )             /* Supply voltage */
#define V25               ( (float32_t) 0.76 )            /* Temperature sensor's voltage at 25 degrees Celsius */
#define AVG_SLOPE         ( (float32_t) (2.5 / 1000.0) )  /* Average slope: 2.5 mV/degreeC */
/* Convert 12-bit ADC value to floating-point input voltage */
#define Adc12BitToVoltage(val)  ( (float32_t) (VDD * ( (float32_t)(val) / (float32_t) 0x0FFFU ) ) )
/* Convert sensed voltage to temperature in degrees Celsius */
#define SensedVoltageToDegreesCelsius(volt)  ( (float32_t) ( ( ( (float32_t)(volt) - V25 ) / AVG_SLOPE ) + 25.0F ) )

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

static uint16_t s_AdcBuffer;

SWcuDiagnosticsDatabase g_WcuDiagnosticsDatabase;

/**
 * @brief Diagnostic service startup
 * @retval None
 */
void WcuDiagnosticsStartup(void) {

	/* Start the timers */
	(void) HAL_TIM_Base_Start_IT(&htim13);
	(void) HAL_TIM_Base_Start_IT(&htim14);
}

/**
 * @brief Initiate the self-check
 * @retval None
 */
void WcuDiagnosticsStartSelfCheck(void) {

	/* Start the ADC conversion and return */
	(void) HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &s_AdcBuffer,
			sizeof(s_AdcBuffer));
}

/**
 * @brief Handle the ADC conversion complete event
 * @retval None
 */
void WcuDiagnosticsHandleAdcConversionComplete(void) {

	SCanMessage canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = WCU_CAN_DLC_WCU_DIAG;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = WCU_CAN_ID_WCU_DIAG;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Calculate the sensed voltage */
	float32_t Vsense = Adc12BitToVoltage(s_AdcBuffer);

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

/**
 * @brief Log runtime database snapshot
 * @retval None
 */
void WcuDiagnosticsLogDatabaseSnapshot(void) {

#if (WCU_ENABLE_DEBUG_PRINTS)

	static uint32_t counter = 0;

	counter += 1U;
	if (counter == WCU_DATABASE_SNAPSHOT_SW_PRESCALER) {

		/* Print the snapshot */
		WcuLogDebug("EVD=%lu,ENS=%lu,CRX=%lu,CSV=%lu,CTX=%lu,CER=%lu,BOK=%lu,BNK=%lu,GRD=%lu,GER=%lu,"
				"XTX=%lu,XSV=%lu,XWA=%lu,XSU=%lu,XNK=%lu,LEQ=%lu,LSV=%lu,LCM=%lu,WRC=%lu,DUC=%lu",
				g_WcuDiagnosticsDatabase.EventsDispatched,
				g_WcuDiagnosticsDatabase.EventQueueStarvations,
				g_WcuDiagnosticsDatabase.CanMessagesReceived,
				g_WcuDiagnosticsDatabase.CanReceiveQueueStarvations,
				g_WcuDiagnosticsDatabase.CanMessagesSent,
				g_WcuDiagnosticsDatabase.CanErrors,
				g_WcuDiagnosticsDatabase.BtMessagesForwarded,
				g_WcuDiagnosticsDatabase.BtInvalidMessagesReceived,
				g_WcuDiagnosticsDatabase.GnssParserFramesCompleted,
				g_WcuDiagnosticsDatabase.GnssParserErrors,
				g_WcuDiagnosticsDatabase.XbeeMessagesSent,
				g_WcuDiagnosticsDatabase.XbeeTransmitRingbufferStarvations,
				g_WcuDiagnosticsDatabase.XbeeDriverWarningMessagesReceived,
				g_WcuDiagnosticsDatabase.XbeeNewSubscriptionMessagesReceived,
				g_WcuDiagnosticsDatabase.XbeeInvalidMessagesReceived,
				g_WcuDiagnosticsDatabase.LoggerEntriesQueued,
				g_WcuDiagnosticsDatabase.LoggerRingbufferStarvations,
				g_WcuDiagnosticsDatabase.LoggerCommits,
				g_WcuDiagnosticsDatabase.WatchdogRefreshCount,
				g_WcuDiagnosticsDatabase.DeferredUnrefCount);

		/* Reset the frequency scaling counter */
		counter = 0;
	}

#endif /* (WCU_ENABLE_DEBUG_PRINTS) */
}
