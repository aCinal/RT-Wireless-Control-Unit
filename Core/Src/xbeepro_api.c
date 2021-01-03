/**
 * @file xbeepro_api.h
 * @author Adrian Cinal
 * @brief XBee-PRO API source file
 */

#include "xbeepro_api.h"

#include "xbeepro_config.h"

#define XBEEPROAPI_UART_TIMEOUT ( (uint32_t) 500 )

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

	/* Transmit the payload */
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE, payloadPtr, numOfBytes,
			XBEEPROAPI_UART_TIMEOUT)) {

		status = EXbeeProApiRet_Error;

	}

	return status;

}

