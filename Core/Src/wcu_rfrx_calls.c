/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"

#include "wcu_common.h"
#include "s2lp_api.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CAN_ID_TPMS_1       ( (uint32_t) 0x411 )       /* CAN ID: _411_WDTM_TPMS_1 */
#define FREQ_XO             ( (uint32_t) 16 )          /* S2-LP crystal oscillator frequency in MHz */
#define FREQ_TPMS           ( (uint32_t) 433 )         /* TPMS radio frequency in MHz */
#define RFRX_READ_BUF_SIZE  ( (uint32_t) 8 )           /* RX buffer size */
#define TPMS_FL_ID          ( (uint32_t) 0x00000000 )  /* Front-left TPMS sensor ID */
#define TPMS_FR_ID          ( (uint32_t) 0x00000001 )  /* Front-right TPMS sensor ID */
#define TPMS_RL_ID          ( (uint32_t) 0x00000002 )  /* Rear-left TPMS sensor ID */
#define TPMS_RR_ID          ( (uint32_t) 0x00000003 )  /* Rear-right TPMS sensor ID */

extern osThreadId rfRxHandle;
extern osMessageQId rfRxEventQueueHandle;

/**
 * @brief TPMS data structure
 */
typedef struct STpmsData {
	uint8_t tireAirPressure;
	uint8_t tireAirTemperature;
	uint32_t sensorId;
} STpmsData;

static void RfRxParsePayload(STpmsData *tpmsDataPtr, uint8_t *payloadPtr);
static void RfRxAddDataToCanFrame(SCanFrame *canFramePtr,
		STpmsData *tpmsDataPtr);

/**
 * @brief Configure the S2-LP device
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxDeviceConfig(void) {

	ERfRxRet status = ERfRxRet_Ok;

	/* TODO: Run S2-LP config */

	return status;

}

/**
 * @brief Handle event messages
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxHandleEvents(void) {

	ERfRxRet status = ERfRxRet_Ok;

	/* TODO: Implement event handling */

	return status;

}

/**
 * @brief Callback on external interrupt
 * @retval None
 */
void RfRxExtiCallback(void) {

	ERfRxEvent mail = ERfRxEvent_Exti;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxEventQueueHandle, &mail, NULL);

}

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void RfRxPeriodElapsedCallback(void) {

	ERfRxEvent event = ERfRxEvent_PeriodElapsed;
	/* Notify the task */
	(void) xQueueSendFromISR(rfRxEventQueueHandle, &event, NULL);

}

/**
 * @brief Parse the TPMS sensor payload
 * @param tpmsDataPtr Buffer for the parsed TPMS data
 * @param payloadPtr Payload buffer
 * @retval None
 */
static void RfRxParsePayload(STpmsData *tpmsDataPtr, uint8_t *payloadPtr) {

	/* TODO: Parse the data */

}

/**
 * @brief Add the TPMS data to the CAN frame
 * @param canFramePtr CAN frame structure
 * @param tpmsDataPtr TPMS data structure
 * @retval None
 */
static void RfRxAddDataToCanFrame(SCanFrame *canFramePtr,
		STpmsData *tpmsDataPtr) {

	/* Identify the sensor */
	switch (tpmsDataPtr->sensorId) {

	case TPMS_FL_ID:

		canFramePtr->PayloadTbl[0] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[4] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_FR_ID:

		canFramePtr->PayloadTbl[1] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[5] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_RL_ID:

		canFramePtr->PayloadTbl[2] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[6] = tpmsDataPtr->tireAirTemperature;
		break;

	case TPMS_RR_ID:

		canFramePtr->PayloadTbl[3] = tpmsDataPtr->tireAirPressure;
		canFramePtr->PayloadTbl[7] = tpmsDataPtr->tireAirTemperature;
		break;

	default:

		LogError("RfRxAddDataToCanFrame: Unknown ID");
		break;

	}

}
