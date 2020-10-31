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
 * @brief Enter command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_EnterCommandMode(void) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	const char ENTER_COMMAND_MODE[] = "+++";
	/* Enter command mode */
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE,
					(uint8_t*) ENTER_COMMAND_MODE,
					STATIC_STRLEN(ENTER_COMMAND_MODE),
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
EXbeeProLldRet XbeeProLld_SendCommand(const char *command) {

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
 * @brief Exit command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_ExitCommandMode(void) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	const char EXIT_COMMAND_MODE[] = "ATCN\r";
	/* Exit command mode */
	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE,
					(uint8_t*) EXIT_COMMAND_MODE,
					STATIC_STRLEN(EXIT_COMMAND_MODE),
					XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}

/**
 * @brief Receive XBee-PRO register contents
 * @param bufPtr Buffer to pass the register contents out of the function
 * @param numOfBytes Number of bytes to be received
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_Receive(uint8_t *bufPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	/* Receive data */
	if (HAL_OK != HAL_UART_Receive(&XBEEPRO_UART_HANDLE, bufPtr, numOfBytes,
	XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}

/**
 * @brief Transmit data to the XBee-PRO device
 * @param bufPtr Payload buffer
 * @param numOfBytes Number of bytes to be transmitted
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_Transmit(uint8_t *bufPtr, size_t numOfBytes) {

	EXbeeProLldRet status = EXbeeProLldRet_Ok;

	if (HAL_OK
			!= HAL_UART_Transmit(&XBEEPRO_UART_HANDLE, bufPtr, numOfBytes,
					XBEEPROLLD_UART_TIMEOUT)) {

		status = EXbeeProLldRet_Error;

	}

	return status;

}
