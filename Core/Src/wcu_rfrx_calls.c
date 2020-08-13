/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"
#include "wcu_base.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief Listens for and handles the RF message
 * @retval None
 */
void rfRx_HandleMessage(void) {

	static uint8_t spiRxBuff[WCU_RFRX_SPI_RX_BUFF_SIZE]; /* SPI Rx buffer */

	/* Poll the DR pin until it is set high */
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(RF_DR_GPIO_Port, RF_DR_Pin)) {

		/* Set the TRX_CE pin low to enter standby mode */
		HAL_GPIO_WritePin(RF_TRX_CE_GPIO_Port, RF_TRX_CE_Pin, GPIO_PIN_RESET);

		/* Set the CSN pin low to start the SPI transmission */
		HAL_GPIO_WritePin(RF_SPI1_CSN_GPIO_Port, RF_SPI1_CSN_Pin,
				GPIO_PIN_RESET);

		static const uint8_t R_TX_PAYLOAD = 0b00100100; /* nRF905 SPI instruction to read the payload */
		/* Request the payload */
		(void) HAL_SPI_Transmit(&RF_SPI_HANDLE, (uint8_t*) &R_TX_PAYLOAD, 1,
		WCU_RFRX_SPI_TX_TIMEOUT);

		/* Receive the payload */
		(void) HAL_SPI_Receive_DMA(&hspi1, spiRxBuff,
		WCU_RFRX_SPI_RX_BUFF_SIZE);

		/* Assert the payload was received */
		if (0UL == ulTaskNotifyTake(pdTRUE,
		portMAX_DELAY)) {

			/* Log the error */
			LOGERROR("SPI receive timeout in rfRx\r\n");

		}

		/* Set the CSN pin high to end the SPI transmission */
		HAL_GPIO_WritePin(RF_SPI1_CSN_GPIO_Port, RF_SPI1_CSN_Pin, GPIO_PIN_SET);

		/* Enable the chip for receive by driving TRX_CE high */
		HAL_GPIO_WritePin(RF_TRX_CE_GPIO_Port, RF_TRX_CE_Pin, GPIO_PIN_SET);

	}

}

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
