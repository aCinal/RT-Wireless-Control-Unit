/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"
#include "wcu_basic.h"

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void) {

	/* Set CSN low to start configuring the device */
	HAL_GPIO_WritePin(RF_SPI1_CSN_GPIO_Port, RF_SPI1_CSN_Pin, GPIO_PIN_RESET);

	/*
	 * TODO:
	 */

	/* Power up the chip by driving PWR_UP high */
	HAL_GPIO_WritePin(RF_PWR_UP_GPIO_Port, RF_PWR_UP_Pin, GPIO_PIN_SET);

	/* Select ShockBurst(TM) RX mode by driving TX_EN low */
	HAL_GPIO_WritePin(RF_TX_EN_GPIO_Port, RF_TX_EN_Pin, GPIO_PIN_RESET);

	/* Enable the chip for receive by driving TRX_CE high */
	HAL_GPIO_WritePin(RF_TRX_CE_GPIO_Port, RF_TRX_CE_Pin, GPIO_PIN_SET);

}
