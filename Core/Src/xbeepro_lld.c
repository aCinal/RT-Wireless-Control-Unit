/**
 * @file xbeepro_lld.c
 * @author Adrian Cinal
 * @brief XBee-PRO LLD source file
 */

#include "xbeepro_lld.h"

#include "xbeepro_config.h"

#include <stdint.h>

#define STATIC_STRLEN(str)       ( (uint32_t)( ( sizeof(str) / sizeof(str[0]) ) - 1U ) )
#define XBEEPROLLD_UART_TIMEOUT  ( (uint32_t) 500 )

/**
 * @brief Transmit data to the XBee-PRO device
 * @param bufPtr Payload buffer
 * @param numOfBytes Number of bytes to be transmitted
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldTransmit(uint8_t *bufPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE, bufPtr, numOfBytes,
					XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}

/**
 * @brief Transmit a command to the XBee-PRO device in command mode
 * @param command Command string
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldSendCommand(const char *command) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	/* Send command */
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE, (uint8_t*) command,
					STATIC_STRLEN(command), XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}

/**
 * @brief Enter command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldEnterCommandMode(void) {

	return XbeeProLldSendCommand("+++");

}

/**
 * @brief Exit command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldExitCommandMode(void) {

	return XbeeProLldSendCommand("ATCN\r");

}

/**
 * @brief Apply changes to the configuration command registers
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldApplyChanges(void) {

	return XbeeProLldSendCommand("ATAC\r");

}

/**
 * @brief Receive XBee-PRO register contents
 * @param bufPtr Buffer to pass the register contents out of the function
 * @param numOfBytes Number of bytes to be received
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldReceive(uint8_t *bufPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	/* Receive data */
	if (HAL_OK != HAL_UART_Receive(&XBEEPRO_UART_HANDLE, bufPtr, numOfBytes,
	XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}


