/**
 * @author Adrian Cinal
 * @file wcu_xbeetx_calls.c
 * @brief Source file defining functions called by the xbeeTx task
 */

#include "wcu_xbeetx_calls.h"
#include "wcu_basic.h"

/**
 * @brief Requests and receives the RSSI value from the XBEE Pro device
 * @param[out] rssiBuff Buffer for the RSSI value
 * @retval HAL_StatusTypeDef HAL status
 */
HAL_StatusTypeDef xbeeTx_GetRssi(uint8_t *rssiBuff) {

	HAL_StatusTypeDef ret = HAL_OK;

	/* Enter AT command mode */
	const char COMMAND_SEQUENCE[] = "+++";
	ret = HAL_UART_Transmit(&XBEE_UART_HANDLE, (uint8_t*) COMMAND_SEQUENCE, 3,
	WCU_XBEETX_UART_TX_TIMEOUT);
	if (HAL_OK != ret) {

		Error_Handler();
		return ret;

	}

	/* Request RSSI value */
	const char DB[] = "ATDB\r";
	ret = HAL_UART_Transmit(&XBEE_UART_HANDLE, (uint8_t*) COMMAND_SEQUENCE,
			sizeof(DB), WCU_XBEETX_UART_TX_TIMEOUT);
	if (HAL_OK != ret) {

		Error_Handler();
		return ret;

	}

	/* Receive the RSSI value */
	ret = HAL_UART_Receive(&XBEE_UART_HANDLE, rssiBuff, 1, 0xFFFF);
	if (HAL_OK != ret) {

		Error_Handler();

	}

	return ret;

}
