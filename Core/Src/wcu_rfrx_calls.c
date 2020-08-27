/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_generic.h"

#include "main.h"
#include "cmsis_os.h"

/* Set/reset pins based on the label */
#define SET_PIN(label) (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET))
#define RESET_PIN(label) (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET))

#define RFRX_SPI_RX_BUFF_SIZE	(uint32_t)(8)			/* SPI Rx buffer size */
#define RFRX_SPI_TX_TIMEOUT		(uint32_t)(500)			/* SPI Tx timeout */

#define TPMS_ID_FL				(uint32_t)(0x00000000)	/* TBD experimentally */
#define TPMS_ID_FR				(uint32_t)(0x00000000)	/* TBD experimentally */
#define TPMS_ID_RL				(uint32_t)(0x00000000)	/* TBD experimentally */
#define TPMS_ID_RR				(uint32_t)(0x00000000)	/* TBD experimentally */
#define TPMS_CRC_INIT			(uint8_t)(0x08)			/* TPMS CRC initial value */
#define TPMS_CRC_POLY			(uint8_t)(0x07)			/* TPMS CRC polynomial */
#define TPMS_PRES_SCAL_FAC		(float32_t)(0.25)		/* TPMS pressure scaling factor */
#define TPMS_PRES_OFFSET		(float32_t)(-7.)		/* TPMS pressure offset */

#define _bits0_6(x)				(uint8_t)(x & 0x7F)
#define _bit7(x)				(uint8_t)((x >> 7) & 0x01)
#define XOR(x, y)				(x ^ y)

/* Macros for scrambling the TPMS data from the bits of the message */
#define TPMS_SCRMBL_PRES_NONINVERTED(buff)	(uint8_t)((_bits0_6(buff[4]) << 1) | _bit7(buff[5]))
#define TPMS_SCRMBL_TEMP(buff)				(uint8_t)((_bits0_6(buff[5]) << 1) | _bit7(buff[6]))

/**
 * @brief TPMS data structure
 */
typedef struct STpmsData {

	uint32_t Id; /* TPMS ID */

	uint8_t PressurePsi; /* Pressure in PSI */

	uint8_t TemperatureC; /* Temperature in degrees Celsius */

} STpmsData;

/**
 * @brief TPMS data decoder return value
 */
typedef enum ETpmsDecoderRet {

	ETpmsDecoderRet_Ok = 0,

	ETpmsDecoderRet_InvalidCrc,

	ETpmsDecoderRet_PressureCheckError

} ETpmsDecoderRet;

typedef uint8_t TByte;
typedef TByte TInstruction;
typedef TByte TParameter;

static void rfRx_WriteConfig(TInstruction instruction, TParameter parameter);
static ETpmsDecoderRet rfRx_DecodeMessage(uint8_t buff[], STpmsData* tpmsData);
static uint8_t rfRx_CalculateCrc(uint8_t buff[]);

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void) {

	/*
	 * TODO Set the RX payload width to 8 bytes, test CRC compatibility with TPMS
	 */

	/* Power up the chip by driving PWR_UP high */
	SET_PIN(RF_PWR_UP);
	/* Select ShockBurst(TM) RX mode by driving TX_EN low */
	RESET_PIN(RF_TX_EN);
	/* Enable the chip for receive by driving TRX_CE high */
	SET_PIN(RF_TRX_CE);

}

/**
 * @brief Listens for and handles the RF message
 * @retval None
 */
void rfRx_HandleMessage(void) {

	static uint8_t rxBufTbl[RFRX_SPI_RX_BUFF_SIZE]; /* SPI Rx buffer */

	/* Poll the DR pin until it is set high */
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(RF_DR_GPIO_Port, RF_DR_Pin)) {

		/* Set the TRX_CE pin low to enter standby mode */
		RESET_PIN(RF_TRX_CE);
		/* Set the CSN pin low to start the SPI transmission */
		RESET_PIN(RF_SPI1_CSN);

		static const uint8_t R_TX_PAYLOAD = 0b00100100; /* nRF905 SPI instruction to read the payload */
		/* Request the payload */
		if(HAL_OK != HAL_SPI_Transmit(&RF_SPI_HANDLE, (uint8_t*) &R_TX_PAYLOAD, 1,
		RFRX_SPI_TX_TIMEOUT)) {

			/* Log the error */
			LogError("SPI transmit failed in rfRx\r\n");
			/* Cleanup */
			SET_PIN(RF_SPI1_CSN);
			SET_PIN(RF_TRX_CE);
			return;

		}

		/* Receive the payload */
		(void) HAL_SPI_Receive_DMA(&hspi1, rxBufTbl,
		RFRX_SPI_RX_BUFF_SIZE);

		/* Assert the payload was received */
		if (0UL == ulTaskNotifyTake(pdTRUE,
		portMAX_DELAY)) {

			/* Log the error */
			LogError("SPI receive timeout in rfRx\r\n");
			/* Cleanup */
			SET_PIN(RF_SPI1_CSN);
			SET_PIN(RF_TRX_CE);
			return;

		}

		/* Set the CSN pin high to end the SPI transmission */
		SET_PIN(RF_SPI1_CSN);
		/* Enable the chip for receive by driving TRX_CE high */
		SET_PIN(RF_TRX_CE);

		STpmsData tpmsData;
		/* Clear the data buffer */
		memset(&tpmsData, 0x00, sizeof(tpmsData));
		/* Parse the message */
		if(ETpmsDecoderRet_Ok != rfRx_DecodeMessage(rxBufTbl, &tpmsData)) {

			/* Log the error */
			LogError("TPMS decoder error\r\n");
			return;

		}

		/* Handle the CAN frame */
		/*
		 * TODO: Assert all four sensors transmitted their readings
		 */

	}

}

/**
 * @brief Sends a configuration instruction to the device
 * @param instruction Instruction
 * @param parameter Instruction argument
 * @retval None
 */
static void rfRx_WriteConfig(TInstruction instruction, TParameter parameter) {

	RESET_PIN(RF_SPI1_CSN);

	/* Transmit instruction */
	if(HAL_OK != HAL_SPI_Transmit(&RF_SPI_HANDLE, (uint8_t*)&instruction, 1, RFRX_SPI_TX_TIMEOUT)) {

		/* Log the error */
		LogError("SPI transmit failed in rfRx\r\n");
		/* Cleanup */
		SET_PIN(RF_SPI1_CSN);
		return;

	}

	/* Transmit parameter */
	if(HAL_OK != HAL_SPI_Transmit(&RF_SPI_HANDLE, (uint8_t*)&instruction, 1, RFRX_SPI_TX_TIMEOUT)) {

		/* Log the error */
		LogError("SPI transmit failed in rfRx\r\n");
		/* Cleanup */
		SET_PIN(RF_SPI1_CSN);
		return;

	}

	SET_PIN(RF_SPI1_CSN);
	return;

}

/**
 * @brief Decodes the TPMS data message
 * @param[in] buff Message buffer of size RFRX_SPI_RX_BUFF_SIZE
 * @param[out] tpmsData TPMS data structure
 * @retval ETpmsDecoderRet Status
 */
static ETpmsDecoderRet rfRx_DecodeMessage(uint8_t buff[], STpmsData* tpmsData) {

	ETpmsDecoderRet ret = ETpmsDecoderRet_Ok; /* Return value */

	/* Pressure check */
	uint8_t pressureNonInverted = TPMS_SCRMBL_PRES_NONINVERTED(buff);
	uint8_t pressureInverted = buff[7];
	if(pressureNonInverted != ~pressureInverted) {

		ret = ETpmsDecoderRet_PressureCheckError;

	} else {

		/* Read the TPMS ID */
		tpmsData->Id = _join32bits(buff[0], buff[1], buff[2], buff[3]);
		tpmsData->PressurePsi = TPMS_PRES_SCAL_FAC * pressureNonInverted + TPMS_PRES_OFFSET;
		tpmsData->TemperatureC = TPMS_SCRMBL_TEMP(buff);

	}

	return ret;

}

/**
 * @brief Calculates the CRC of the TPMS data message
 * @param buff Message buffer of size RFRX_SPI_RX_BUFF_SIZE
 * @retval uint8_t CRC8 value
 */
static uint8_t rfRx_CalculateCrc(uint8_t buff[]) {

	uint8_t ret = TPMS_CRC_INIT;

	for(uint8_t byteCtr = 0; byteCtr < RFRX_SPI_RX_BUFF_SIZE; byteCtr += 1U) {

		ret = XOR(ret, buff[byteCtr]);
		for(uint8_t bitCtr = 0; bitCtr < 8U; bitCtr += 1U) {

			if(0 != (ret & 0x80)) {

				ret = XOR((ret << 1), TPMS_CRC_POLY);

			} else {

				ret = ret << 1;

			}

		}

	}

	return ret;

}
