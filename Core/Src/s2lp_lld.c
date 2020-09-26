/**
 * @file s2lp_lld.c
 * @author Adrian Cinal
 * @brief S2-LP LLD source file
 */

#include "s2lp_lld.h"
#include "s2lp_config.h"

/* Set/reset pins based on the label */
#define SET_PIN(label)             (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET))
#define RESET_PIN(label)           (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET))

#define S2LP_SPI_TIMEOUT  ((uint32_t)500U)  /* SPI timeout */

/**
 * @brief Write data to S2-LP's registers
 * @param address S2-LP memory map address
 * @param bufPtr Data to be written
 * @param numOfBytes Number of bytes to be written
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_WriteReg(TByte address, TByte* bufPtr, TSize numOfBytes) {

	ES2lpLldRet status = ES2lpLldRet_Ok;

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header = 0x00;
	/* Transmit the header */
	if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &header, 1, S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;

	}

	if(ES2lpLldRet_Ok == status) {

		/* Transmit the address */
		if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &address, 1, S2LP_SPI_TIMEOUT)) {

			status = ES2lpLldRet_Error;

		}

	}

	if(ES2lpLldRet_Ok == status) {

		/* Transmit the data */
		if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, bufPtr, numOfBytes, S2LP_SPI_TIMEOUT)) {

			status = ES2lpLldRet_Error;

		}

	}

	/* Drive chip-select pin high */
	SET_PIN(S2LP_CSn);

	return status;

}

/**
 * @brief Read data from S2-LP's registers
 * @param address S2-LP memory map address
 * @param[out] bufPtr Buffer for the read data
 * @param numOfBytes Number of bytes to be read
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_ReadReg(TByte address, TByte* bufPtr, TSize numOfBytes) {

	ES2lpLldRet status = ES2lpLldRet_Ok;

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header = 0x01;
	/* Transmit the header */
	if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &header, 1, S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;

	}

	if(ES2lpLldRet_Ok == status) {

		/* Transmit the address */
		if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &address, 1, S2LP_SPI_TIMEOUT)) {

			status = ES2lpLldRet_Error;

		}

	}

	if(ES2lpLldRet_Ok == status) {

		/* Receive the data */
		if(HAL_OK != HAL_SPI_Receive(&S2LP_SPI_HANDLE, bufPtr, numOfBytes, S2LP_SPI_TIMEOUT)) {

			status = ES2lpLldRet_Error;

		}

	}

	/* Drive chip-select pin high */
	SET_PIN(S2LP_CSn);

	return status;

}

/**
 * @brief Send a command to S2-LP via SPI
 * @param command S2-LP command code
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_Command(TByte command) {

	ES2lpLldRet status = ES2lpLldRet_Ok;

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header = 0x80;
	/* Transmit the header */
	if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &header, 1, S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;

	}

	if(ES2lpLldRet_Ok == status) {

		/* Transmit the address */
		if(HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, &command, 1, S2LP_SPI_TIMEOUT)) {

			status = ES2lpLldRet_Error;

		}

	}

	/* Drive chip-select pin high */
	SET_PIN(S2LP_CSn);

	return status;

}

