/**
 * @file xbeepro_api.h
 * @author Adrian Cinal
 * @brief XBee-PRO API source file
 */

#include "xbeepro_api.h"

#include "xbeepro_lld.h"
#include "xbeepro_config.h"

#include "cmsis_os.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define XBEEPROAPI_DELAY(millisec)    vTaskDelay( pdMS_TO_TICKS(millisec) )  /* Delay macro */
#define XBEEPROAPI_WAIT_GUARD_TIME()  ( XBEEPROAPI_DELAY(gGuardTimes ))      /* Wait the guard time */
#define XBEEPRO_GT_DEFAULT            ( (uint16_t) 0x03E8 )                  /* XBEE Pro Guard Times default value */

static uint16_t gGuardTimes = XBEEPRO_GT_DEFAULT;

static EXbeeProApiRet XbeeProApiSendCommand(const char *command);
static EXbeeProApiRet XbeeProApiEnterCommandMode(void);
static EXbeeProApiRet XbeeProApiExitCommandMode(void);
static EXbeeProApiRet XbeeProApiWaitForResponse(void);

/**
 * @brief Transmit payload to be transmitted by XBee-PRO to the device
 * @param payloadPtr Payload buffer
 * @param numOfBytes Size of the payload buffer in bytes
 * @retval EXbeeProLldRet Status
 */
EXbeeProApiRet XbeeProApiSendPayload(uint8_t *payloadPtr, uint32_t numOfBytes) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Assert valid parameters */
	if (( NULL == payloadPtr) || (0 == numOfBytes)) {

		status = EXbeeProApiRet_InvalidParams;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Transmit the payload */
		if (EXbeeProLldRet_Ok != XbeeProLldTransmit(payloadPtr, numOfBytes)) {

			status = EXbeeProApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set the Guard Times parameter of the XBee-PRO device
 * @param gt Guard Times parameter
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiSetGuardTimes(uint16_t gt) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Enter command mode */
	status = XbeeProApiEnterCommandMode();

	if (EXbeeProApiRet_Ok == status) {

		/* Wait for OK response */
		status = XbeeProApiWaitForResponse();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Print the parameter value to the command string */
		char SET_GT[] = "ATGT0000\r";
		(void) sprintf(&(SET_GT[4]), "%04X\r", gt);
		/* Send command */
		status = XbeeProApiSendCommand(SET_GT);

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Apply changes */
		status = XbeeProApiApplyChanges();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait for OK response */
		status = XbeeProApiWaitForResponse();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Save the new guard time */
		gGuardTimes = gt;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Exit command mode */
		status = XbeeProApiExitCommandMode();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait for OK response */
		status = XbeeProApiWaitForResponse();

	}

	return status;

}

/**
 * @brief Read the RSSI value of the last transmission received
 * @param rssiPtr Pointer to pass the received value out of the function
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiGetRssi(uint8_t *rssiPtr) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;
	char respMsg[3] = { };

	/* Assert valid parameters */
	if (NULL == rssiPtr) {

		status = EXbeeProApiRet_InvalidParams;

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Enter command mode */
		status = XbeeProApiEnterCommandMode();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait for OK response */
		status = XbeeProApiWaitForResponse();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Send command */
		status = XbeeProApiSendCommand("ATDB\r");

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Receive RSSI */
		status = XbeeProLldReceive(respMsg, sizeof(respMsg));

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Exit command mode */
		status = XbeeProApiExitCommandMode();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* Wait for OK response */
		status = XbeeProApiWaitForResponse();

	}

	if (EXbeeProApiRet_Ok == status) {

		/* On success parse the response message and pass the parsed value out of the function */
		*rssiPtr = strtoul(respMsg, NULL, 16);

	}

	return status;

}

/**
 * @brief Apply changes made to the XBee-PRO configuration registers
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiApplyChanges(void) {

	return XbeeProApiSendCommand("ATAC\r");

}

/**
 * @brief Transmit a command to the XBee-PRO device in command mode
 * @param commandStr Command string
 * @retval EXbeeProApiRet Status
 */
static EXbeeProApiRet XbeeProApiSendCommand(const char *commandStr) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	/* Send command */
	if (EXbeeProLldRet_Ok
			!= XbeeProLldTransmit((uint8_t*) commandStr, strlen(commandStr))) {

		status = EXbeeProApiRet_Error;

	}

	return status;

}

/**
 * @brief Enter command mode
 * @retval EXbeeProApiRet Status
 */
static EXbeeProApiRet XbeeProApiEnterCommandMode(void) {

	/* Observe the guard times before and after the command sequence */
	XBEEPROAPI_WAIT_GUARD_TIME();

	/* Send the command sequence */
	EXbeeProApiRet status = XbeeProApiSendCommand("+++");

	XBEEPROAPI_WAIT_GUARD_TIME();

	return status;

}

/**
 * @brief Exit command mode
 * @retval EXbeeProApiRet Status
 */
static EXbeeProApiRet XbeeProApiExitCommandMode(void) {

	return XbeeProApiSendCommand("ATCN\r");

}

/**
 * @brief Wait for an OK response from the XBee-PRO device
 * @retval EXbeeProApiRet Status
 */
static EXbeeProApiRet XbeeProApiWaitForResponse(void) {

	EXbeeProApiRet status = EXbeeProApiRet_Ok;

	char dummy;
	if (EXbeeProLldRet_Ok != XbeeProLldReceive(&dummy, 1)) {

		status = EXbeeProApiRet_Error;

	}

	return status;

}
