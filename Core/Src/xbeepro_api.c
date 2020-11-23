/**
 * @file xbeepro_api.h
 * @author Adrian Cinal
 * @brief XBee-PRO API source file
 */

#include "xbeepro_api.h"

#include "xbeepro_lld.h"
#include "xbeepro_config.h"

#include "cmsis_os.h"
#include <stdio.h>

#define XBEEPROAPI_DELAY(millisec)  vTaskDelay( pdMS_TO_TICKS(millisec) )  /* Delay macro */
#define XBEEPRO_GT_DEFAULT          ( (uint16_t) 0x0CE4 )                  /* XBEE Pro Guard Times default value */

static uint16_t gGuardTimes = XBEEPRO_GT_DEFAULT;

/**
 * @brief Return the local value of the XBee-PRO Guard Times parameter
 * @retval uint16_t Local value of the guard time
 */
uint16_t XbeeProApiReadLocalGuardTime(void) {

	return gGuardTimes;

}

/**
 * @brief Set the Guard Times parameter of the XBee-PRO device
 * @param gt Guard Times parameter
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiSetGuardTimes(uint16_t gt) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Wait the current guard time */
	XBEEPROAPI_DELAY(gGuardTimes);

	/* Enter command mode */
	if (EXbeeProLldRet_Ok != XbeeProLld_EnterCommandMode()) {

		status = EXbeeProApiRet_Error;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait the current guard time */
		XBEEPROAPI_DELAY(gGuardTimes);

		/* Print the parameter value to the command string */
		char SET_GT[] = "ATGT0000\r";
		(void) sprintf(&(SET_GT[4]), "%04X\r", gt);
		/* Send command */
		if (EXbeeProLldRet_Ok != XbeeProLld_SendCommand(SET_GT)) {

			status = EXbeeProApiRet_Error;

		}

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Save the new guard time */
		gGuardTimes = gt;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Exit command mode */
		if (EXbeeProLldRet_Ok != XbeeProLld_ExitCommandMode()) {

			status = EXbeeProApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Read the RSSI value of the last transmission received
 * @param rssiPtr Pointer to pass the received value out of the function
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiReadRssi(uint8_t *rssiPtr) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Assert valid parameters */
	if (NULL == rssiPtr) {

		status = EXbeeProApiRet_InvalidParams;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait the guard time */
		XBEEPROAPI_DELAY(gGuardTimes);

		/* Enter command mode */
		if (EXbeeProLldRet_Ok != XbeeProLld_EnterCommandMode()) {

			status = EXbeeProApiRet_Error;

		}

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait the guard time */
		XBEEPROAPI_DELAY(gGuardTimes);

		/* Send command */
		if (EXbeeProLldRet_Ok != XbeeProLld_SendCommand("ATDB\r")) {

			status = EXbeeProApiRet_Error;

		}

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Receive RSSI */
		if (EXbeeProLldRet_Ok != XbeeProLld_Receive(rssiPtr, 1)) {

			status = EXbeeProApiRet_Error;

		}

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Exit command mode */
		if (EXbeeProLldRet_Ok != XbeeProLld_ExitCommandMode()) {

			status = EXbeeProApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Transmit payload to be transmitted by XBee-PRO to the device
 * @param payloadPtr Payload buffer
 * @param numOfBytes Size of the payload buffer in bytes
 * @retval EXbeeProLldRet Status
 */
EXbeeProApiRet XbeeProApiSendPayload(uint8_t* payloadPtr, uint32_t numOfBytes) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Assert valid parameters */
	if( ( NULL == payloadPtr ) || ( 0 == numOfBytes ) ) {

		status = EXbeeProApiRet_InvalidParams;

	}

	if(EXbeeProApiRet_Ok == status) {

		/* Transmit the payload */
		if(EXbeeProLldRet_Ok != XbeeProLld_Transmit(payloadPtr, numOfBytes)) {

			status = EXbeeProApiRet_Ok;

		}

	}

	return status;

}
