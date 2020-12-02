/**
 * @file xbeepro_lld.c
 * @author Adrian Cinal
 * @brief XBee-PRO LLD source file
 */

#include "xbeepro_lld.h"

#include "xbeepro_config.h"

#define XBEEPROLLD_UART_TIMEOUT  ( (uint32_t) 500 )

/**
 * @brief Transmit data to the XBee-PRO device
 * @param bufPtr Payload buffer
 * @param numOfBytes Number of bytes to be transmitted
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldTransmit(uint8_t *bufPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	if (HAL_OK != HAL_UART_Transmit(&XBEEPRO_UART_HANDLE, bufPtr, numOfBytes,
	XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}

/**
 * @brief Receive XBee-PRO register contents
 * @param respMsgPtr Buffer to pass the received response message out of the function
 * @param numOfBytes Number of bytes to be received
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldReceive(char *respMsgPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	/* Receive data */
	if (HAL_OK
			!= HAL_UART_Receive(&XBEEPRO_UART_HANDLE, (uint8_t*) respMsgPtr,
					numOfBytes,
					XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}
