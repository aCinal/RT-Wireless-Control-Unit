/**
 * @file s2lp_lld.c
 * @author Adrian Cinal
 * @brief S2-LP LLD source file
 */

#include "s2lp_lld.h"
#include "s2lp_config.h"

#include <stddef.h>

/* Set/reset pins based on the label */
#define SET_PIN(label)    ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET) )
#define RESET_PIN(label)  ( HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET) )

#define DELAY(ms)  ( HAL_Delay(ms) )     /* Wrapper for delay */
#define S2LP_SPI_TIMEOUT  ( (uint32_t) 500U )  /* SPI timeout */

#define S2LP_WRITE_HEADER    ( (TByte) 0x00 )
#define S2LP_READ_HEADER     ( (TByte) 0x01 )
#define S2LP_COMMAND_HEADER  ( (TByte) 0x80 )

/**
 * @brief Reset the device
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_Reset(void) {

	ES2lpLldRet status = ES2lpLldRet_Ok;

	/* Hard reset */
	SET_PIN(S2LP_SDN);
	DELAY(1);
	RESET_PIN(S2LP_SDN);
	DELAY(1);

	/* Soft reset */
	status = S2lpLld_SendCommand(S2LP_SRES_COMMAND, NULL);
	DELAY(2);

	return status;
}

/**
 * @brief Write data to S2-LP's registers
 * @param address S2-LP memory map address
 * @param bufPtr Data to be written
 * @param numOfBytes Number of bytes to be written
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_WriteReg(TByte address, TByte *bufPtr, TSize numOfBytes,
		THalfWord *s2lpStatusBitsBufPtr) {

	ES2lpLldRet status = ES2lpLldRet_Ok;
	TByte s2lpStatusBits[2] = { };

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header[2] = { S2LP_WRITE_HEADER, address };
	/* Transmit the header and receive the status bits */
	if (HAL_OK
			!= HAL_SPI_TransmitReceive(&S2LP_SPI_HANDLE, header, s2lpStatusBits,
					2, S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;
	}

	if (ES2lpLldRet_Ok == status) {

		if (NULL != s2lpStatusBitsBufPtr) {

			/* Save the status bits in the provided buffer in the correct order */
			*s2lpStatusBitsBufPtr = (s2lpStatusBits[0] << 8U)
					| s2lpStatusBits[1];
		}
	}

	if (ES2lpLldRet_Ok == status) {

		/* Transmit the data */
		if (HAL_OK != HAL_SPI_Transmit(&S2LP_SPI_HANDLE, bufPtr, numOfBytes,
		S2LP_SPI_TIMEOUT)) {

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
 * @param bufPtr Buffer for the read data
 * @param numOfBytes Number of bytes to be read
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_ReadReg(TByte address, TByte *bufPtr, TSize numOfBytes,
		THalfWord *s2lpStatusBitsBufPtr) {

	ES2lpLldRet status = ES2lpLldRet_Ok;
	TByte s2lpStatusBits[2] = { };

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header[2] = { S2LP_READ_HEADER, address };
	/* Transmit the header and receive the status bits */
	if (HAL_OK
			!= HAL_SPI_TransmitReceive(&S2LP_SPI_HANDLE, header, s2lpStatusBits,
					2, S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;
	}

	if (ES2lpLldRet_Ok == status) {

		if (NULL != s2lpStatusBitsBufPtr) {

			/* Save the status bits in the provided buffer in the correct order */
			*s2lpStatusBitsBufPtr = (s2lpStatusBits[0] << 8U)
					| s2lpStatusBits[1];
		}
	}

	if (ES2lpLldRet_Ok == status) {

		/* Receive the data */
		if (HAL_OK != HAL_SPI_Receive(&S2LP_SPI_HANDLE, bufPtr, numOfBytes,
		S2LP_SPI_TIMEOUT)) {

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
 * @param s2lpStatusBitsBufPtr Buffer to pass the status register contents out of the function (should be NULL if not used)
 * @retval ES2lpLldRet Status
 */
ES2lpLldRet S2lpLld_SendCommand(TByte command, THalfWord *s2lpStatusBitsBufPtr) {

	ES2lpLldRet status = ES2lpLldRet_Ok;
	TByte s2lpStatusBits[2] = { };

	/* Drive chip-select pin low */
	RESET_PIN(S2LP_CSn);

	TByte header[2] = { S2LP_COMMAND_HEADER, command };
	/* Transmit the header and receive the status bits */
	if (HAL_OK
			!= HAL_SPI_TransmitReceive(&S2LP_SPI_HANDLE, header, s2lpStatusBits,
					2,
					S2LP_SPI_TIMEOUT)) {

		status = ES2lpLldRet_Error;
	}

	if (ES2lpLldRet_Ok == status) {

		if (NULL != s2lpStatusBitsBufPtr) {

			/* Save the status bits in the provided buffer in the correct order */
			*s2lpStatusBitsBufPtr = (s2lpStatusBits[0] << 8U)
					| s2lpStatusBits[1];
		}
	}

	/* Drive chip-select pin high */
	SET_PIN(S2LP_CSn);

	return status;
}
