/**
 * @file nrf905lld.c
 * @author Adrian Cinal
 * @brief Low-level drivers source for the nRF905 device
 */

#include "nrf905lld.h"
#include "nrf905lldconfig.h"

#include <stddef.h>

/* Set/reset pins based on the label */
#define SET_PIN(label)             (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_SET))
#define RESET_PIN(label)           (HAL_GPIO_WritePin(label##_GPIO_Port, label##_Pin, GPIO_PIN_RESET))

/* Set bits in a register based on a mask */
#define SET_BITS(REG, MASK, BITS)  ((REG) = (((TByte)(REG) & ~(TByte)(MASK)) | ((TByte)(MASK) & ((TByte)(BITS)))))

/* Instruction set for the nRF905 SPI */
#define W_TX_PAYLOAD               ((TByte) 0b00100000)
#define R_TX_PAYLOAD               ((TByte) 0b00100001)
#define W_TX_ADDRESS               ((TByte) 0b00100010)
#define R_TX_ADDRESS               ((TByte) 0b00100011)
#define R_RX_PAYLOAD               ((TByte) 0b00100100)
#define W_CONFIG(offset)           ((TByte) (0x0F & (TByte)(offset)))
#define R_CONFIG(offset)           ((TByte) (0x10 | (0x0F & (TByte)(offset))))

/* Default contents of the configuration register */
#define DEFAULT_CONFIG_0           ((TByte) 0x6C)
#define DEFAULT_CONFIG_1           ((TByte) 0x00)
#define DEFAULT_CONFIG_2           ((TByte) 0x44)
#define DEFAULT_CONFIG_3           ((TByte) 0x20)
#define DEFAULT_CONFIG_4           ((TByte) 0x20)
#define DEFAULT_CONFIG_5           ((TByte) 0xE7)
#define DEFAULT_CONFIG_6           ((TByte) 0xE7)
#define DEFAULT_CONFIG_7           ((TByte) 0xE7)
#define DEFAULT_CONFIG_8           ((TByte) 0xE7)
#define DEFAULT_CONFIG_9           ((TByte) 0xE7)

/* Boundary sizes */
#define MAXADDRWIDTH               ((TSize) 4U)      /* Maximum TX/RX address width */
#define MAXPAYLWIDTH               ((TSize) 32U)     /* Maximum TX/RX payload width */

/* Communication parameters */
#define SPITXTIMEOUT               ((uint32_t)500U)  /* SPI Tx timeout */
#define SPIRXTIMEOUT               ((uint32_t)500U)  /* SPI Rx timeout */

static ENrf905LldRet Nrf905Lld_GenericWrite(TByte instruction, TByte* parametersPtr, TSize parametersSize);
static ENrf905LldRet Nrf905Lld_GenericRead(TByte instruction, TByte* buffer, TSize dataSize);

/**
 * @brief Sets center frequence together with SetPllMode
 * @param chNo CH_NO parameter value
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetChannel(TWord chNo) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_0_1[2]; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(0), configReg_0_1, 2);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_0_1[0], 0xFF, chNo & 0xFF);
		SET_BITS(configReg_0_1[1], 0x01, ((chNo & 0x100) >> 8));
		status = Nrf905Lld_GenericWrite(W_CONFIG(0), configReg_0_1, 2);

	}

	return status;

}

/**
 * @brief Sets frequency band
 * @param band Frequency band
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetBand(ENrf905LldBand band) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_1; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(1), &configReg_1, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		switch(band) {

		case ENrf905LldBand_433MHz:

			SET_BITS(configReg_1, 0x02, 0x00);
			break;

		case ENrf905LldBand_868_915MHz:

			SET_BITS(configReg_1, 0x02, 0x02);
			break;


		default:

			status = ENrf905LldRet_InvalidParams;
			break;

		}

		if(ENrf905LldRet_Ok == status) {

			status = Nrf905Lld_GenericWrite(W_CONFIG(1), &configReg_1, 1);

		}

	}

	return status;

}

/**
 * @brief Sets the output power
 * @param outputPower Output power
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetOutputPower(ENrf905LldOutputPower outputPower) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_1; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(1), &configReg_1, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		switch(outputPower) {

		case ENrf905LldOutputPower_n10dBm:

			SET_BITS(configReg_1, 0x0C, 0x00);
			break;

		case ENrf905LldOutputPower_n2dBm:

			SET_BITS(configReg_1, 0x0C, 0x04);
			break;

		case ENrf905LldOutputPower_p6dBm:

			SET_BITS(configReg_1, 0x0C, 0x08);
			break;

		case ENrf905LldOutputPower_p10dBm:

			SET_BITS(configReg_1, 0x0C, 0x0C);
			break;

		default:

			status = ENrf905LldRet_InvalidParams;
			break;

		}

		if(ENrf905LldRet_Ok == status) {

			status = Nrf905Lld_GenericWrite(W_CONFIG(1), &configReg_1, 1);

		}

	}

	return status;

}

/**
 * @brief Reduces current in RX mode by 1.6mA. Sensitivity is reduced
 * @param enable True to reduce power, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableReducedCurrentRx(bool enable) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_1; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(1), &configReg_1, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_1, 0x10, (enable ? 0x10 : 0x00));
		status = Nrf905Lld_GenericWrite(W_CONFIG(1), &configReg_1, 1);

	}

	return status;

}

/**
 * @brief Retransmit contents in TX register if TRX_CE and TX_EN are high
 * @param enable True to enable retransmission of data packet, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableAutoRetran(bool enable) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_1; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(1), &configReg_1, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_1, 0x20, (enable ? 0x20 : 0x00));
		status = Nrf905Lld_GenericWrite(W_CONFIG(1), &configReg_1, 1);

	}

	return status;

}

/**
 * @brief Sets RX address width
 * @param addressWidth RX address width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetRxAddressWidth(TSize addressWidth) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_2; /* Config register contents buffer */

	/* Assert valid parameters */
	if(addressWidth > MAXADDRWIDTH) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		/* Pull the configuration register contents */
		status = Nrf905Lld_GenericRead(R_CONFIG(2), &configReg_2, 1);

	}

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_2, 0x07, addressWidth);
		status = Nrf905Lld_GenericWrite(W_CONFIG(2), &configReg_2, 1);

	}

	return status;

}

/**
 * @brief Sets TX address width
 * @param addressWidth TX address width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetTxAddressWidth(TSize addressWidth) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_2; /* Config register contents buffer */

	/* Assert valid parameters */
	if(addressWidth > MAXADDRWIDTH) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		/* Pull the configuration register contents */
		status = Nrf905Lld_GenericRead(R_CONFIG(2), &configReg_2, 1);

	}

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_2, 0x70, (addressWidth << 4));
		status = Nrf905Lld_GenericWrite(W_CONFIG(2), &configReg_2, 1);

	}

	return status;

}

/**
 * @brief Sets RX payload width
 * @param payloadWidth RX payload width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetRxPayloadWidth(TSize payloadWidth) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_3; /* Config register contents buffer */

	/* Assert valid parameters */
	if(payloadWidth > MAXPAYLWIDTH) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		/* Pull the configuration register contents */
		status = Nrf905Lld_GenericRead(R_CONFIG(3), &configReg_3, 1);

	}

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_3, 0x3F, payloadWidth);
		status = Nrf905Lld_GenericWrite(W_CONFIG(3), &configReg_3, 1);

	}

	return status;

}

/**
 * @brief Sets TX payload width
 * @param payloadWidth TX payload width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetTxPayloadWidth(TSize payloadWidth) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_4; /* Config register contents buffer */

	/* Assert valid parameters */
	if(payloadWidth > MAXPAYLWIDTH) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		/* Pull the configuration register contents */
		status = Nrf905Lld_GenericRead(R_CONFIG(4), &configReg_4, 1);

	}

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_4, 0x3F, payloadWidth);
		status = Nrf905Lld_GenericWrite(W_CONFIG(4), &configReg_4, 1);

	}

	return status;

}

/**
 * @brief Sets the device's RX address
 * @param addPtr RX address
 * @param size Address size
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetDeviceIdentity(TByte* addPtr, TSize size) {

	ENrf905LldRet status = ENrf905LldRet_Ok;

	if(size > 4) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		status = Nrf905Lld_GenericWrite(W_CONFIG(5), addPtr, size);

	}

	return status;

}

/**
 * @brief Sets output clock frequency
 * @param freq Output clock frequency
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetOutputClockFrequency(ENrf905LldOutputClkFreq freq) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_9; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(9), &configReg_9, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		switch(freq) {

		case ENrf905LldOutputClkFreq_4MHz:

			SET_BITS(configReg_9, 0x03, 0x00);
			break;

		case ENrf905LldOutputClkFreq_2MHz:

			SET_BITS(configReg_9, 0x03, 0x01);
			break;

		case ENrf905LldOutputClkFreq_1MHz:

			SET_BITS(configReg_9, 0x03, 0x02);
			break;

		case ENrf905LldOutputClkFreq_500kHz:

			SET_BITS(configReg_9, 0x03, 0x03);
			break;

		default:

			status = ENrf905LldRet_InvalidParams;
			break;

		}

		if(ENrf905LldRet_Ok == status) {

			status = Nrf905Lld_GenericWrite(W_CONFIG(9), &configReg_9, 1);

		}

	}

	return status;

}

/**
 * @brief Enables external clock signal
 * @param enable True to enable external clock signal, false if no external clock signal available
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableOutputClock(bool enable) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_9; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(9), &configReg_9, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_9, 0x04, (enable ? 0x04 : 0x00));
		status = Nrf905Lld_GenericWrite(W_CONFIG(9), &configReg_9, 1);

	}

	return status;

}

/**
 * @brief Sets crystal oscillator frequency
 * @param xof Crystal oscillator frequency
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetXof(ENrf905LldXof xof) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_9; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(9), &configReg_9, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		switch(xof) {

		case ENrf905LldXof_4MHz:

			SET_BITS(configReg_9, 0x38, 0x00);
			break;

		case ENrf905LldXof_8MHz:

			SET_BITS(configReg_9, 0x38, 0x08);
			break;

		case ENrf905LldXof_12MHz:

			SET_BITS(configReg_9, 0x38, 0x10);
			break;

		case ENrf905LldXof_16MHz:

			SET_BITS(configReg_9, 0x38, 0x18);
			break;

		case ENrf905LldXof_20MHz:

			SET_BITS(configReg_9, 0x38, 0x20);
			break;

		default:

			status = ENrf905LldRet_InvalidParams;
			break;

		}

		if(ENrf905LldRet_Ok == status) {

			status = Nrf905Lld_GenericWrite(W_CONFIG(9), &configReg_9, 1);

		}

	}

	return status;

}

/**
 * @brief Enables CRC check
 * @param enable True to enable CRC check enable, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableCrcCheck(bool enable) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_9; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(9), &configReg_9, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		SET_BITS(configReg_9, 0x40, (enable ? 0x40 : 0x00));
		status = Nrf905Lld_GenericWrite(W_CONFIG(9), &configReg_9, 1);

	}

	return status;

}

/**
 * @brief Sets CRC mode
 * @param crcMode CRC mode
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetCrcMode(ENrf905LldCrcMode crcMode) {

	ENrf905LldRet status = ENrf905LldRet_Ok;
	TByte configReg_9; /* Config register contents buffer */

	/* Pull the configuration register contents */
	status = Nrf905Lld_GenericRead(R_CONFIG(9), &configReg_9, 1);

	if(ENrf905LldRet_Ok == status) {

		/* Update and push back */
		switch(crcMode) {

		case ENrf905LldCrcMode_Crc8:

			SET_BITS(configReg_9, 0x80, 0x00);
			break;

		case ENrf905LldCrcMode_Crc16:

			SET_BITS(configReg_9, 0x80, 0x80);
			break;

		default:

			status = ENrf905LldRet_InvalidParams;
			break;

		}

		if(ENrf905LldRet_Ok == status) {

			status = Nrf905Lld_GenericWrite(W_CONFIG(9), &configReg_9, 1);

		}

	}

	return status;

}

/**
 * @brief Powers up the device and puts it in standby mode
 * @retval None
 */
void Nrf905Lld_PwrUp(void) {

	SET_PIN(NRF_PWR_UP);

}

/**
 * @brief Powers down the device
 * @retval None
 */
void Nrf905Lld_PwrDown(void) {

	RESET_PIN(NRF_PWR_UP);

}

/**
 * @brief Transmit/Receive enable
 * @retval None
 */
void Nrf905Lld_TransmissionEnable(void) {

	SET_PIN(NRF_TRX_CE);

}

/**
 * @brief Transmit/Receive disable
 * @retval None
 */
void Nrf905Lld_TransmissionDisable(void) {

	RESET_PIN(NRF_TRX_CE);

}

/**
 * @brief Puts the device in ShockBurst(tm) RX mode
 * @retval None
 */
void Nrf905Lld_ModeSelectRx(void) {

	RESET_PIN(NRF_TX_EN);

}

/**
 * @brief Puts the device in ShockBurst(tm) TX mode
 * @retval None
 */
void Nrf905Lld_ModeSelectTx(void) {

	SET_PIN(NRF_TX_EN);

}

/**
 * @brief Writes TX payload to the device
 * @param payloadPtr Payload
 * @param size Size of the payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_WriteTxPayload(TByte* payloadPtr, TSize size) {

	return Nrf905Lld_GenericWrite(W_TX_PAYLOAD, payloadPtr, size);

}

/**
 * @brief Reads TX payload from the device
 * @param[out] bufPtr Buffer for the payload
 * @param[in] size Size of the expected payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadTxPayload(TByte* bufPtr, TSize size) {

	return Nrf905Lld_GenericRead(R_TX_PAYLOAD, bufPtr, size);

}

/**
 * @brief Writes TX address to the device
 * @param addPtr Address
 * @param size Size of the address
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_WriteTxAddress(TByte* addPtr, TSize size) {

	return Nrf905Lld_GenericWrite(W_TX_ADDRESS, addPtr, size);

}

/**
 * @brief Reads TX address from the device
 * @param[out] bufPtr Buffer for the address
 * @param[in] size Size of the expected address
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadTxAddress(TByte* bufPtr, TSize size) {

	return Nrf905Lld_GenericRead(R_TX_ADDRESS, bufPtr, size);

}

/**
 * @brief Reads RX payload from the device
 * @param[out] bufPtr Buffer for the payload
 * @param[in] size Size of the expected payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadRxPayload(TByte* bufPtr, TSize size) {

	return Nrf905Lld_GenericRead(R_RX_PAYLOAD, bufPtr, size);

}

/**
 * @brief Dumps the configuration register contents into a buffer
 * @param bufPtr Buffer
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_DumpConfigReg(TByte* bufPtr) {

	ENrf905LldRet status = ENrf905LldRet_Ok;

	/* Assert valid pointer dereference */
	if(NULL == bufPtr) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		status = Nrf905Lld_GenericRead(R_CONFIG(0), bufPtr, 10);

	}

	return status;

}

/**
 * @brief Restores the default values of the configuration register
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_RestoreDefaultConfig(void) {

	uint8_t buf[10];
	buf[0] = DEFAULT_CONFIG_0;
	buf[1] = DEFAULT_CONFIG_1;
	buf[2] = DEFAULT_CONFIG_2;
	buf[3] = DEFAULT_CONFIG_3;
	buf[4] = DEFAULT_CONFIG_4;
	buf[5] = DEFAULT_CONFIG_5;
	buf[6] = DEFAULT_CONFIG_6;
	buf[7] = DEFAULT_CONFIG_7;
	buf[8] = DEFAULT_CONFIG_8;
	buf[9] = DEFAULT_CONFIG_9;

	return Nrf905Lld_GenericWrite(W_CONFIG(0), buf, sizeof(buf));

}

/**
 * @brief Writes data to the device's registers
 * @param instruction SPI instruction
 * @param parametersPtr Instruction parameters in little endian format
 * @param parametersSize Size of the parameters array
 * @retval ENrf905LldRet Status
 */
static ENrf905LldRet Nrf905Lld_GenericWrite(TByte instruction, TByte* parametersPtr, TSize parametersSize) {

	ENrf905LldRet status = ENrf905LldRet_Ok;

	RESET_PIN(NRF_CSN);

	/* Transmit instruction */
	if(HAL_OK != HAL_SPI_Transmit(&NRF_SPI_HANDLE, &instruction, 1, SPITXTIMEOUT)) {

		status = ENrf905LldRet_Error;

	}

	if(ENrf905LldRet_Ok == status) {

		/* If parameters included */
		if(parametersSize > 0) {

			/* Assert valid pointer dereference */
			if(NULL == parametersPtr) {

				status = ENrf905LldRet_InvalidParams;

			}

			if(ENrf905LldRet_Ok == status) {

				/* Transmit parameters */
				if(HAL_OK != HAL_SPI_Transmit(&NRF_SPI_HANDLE, parametersPtr, parametersSize, SPITXTIMEOUT)) {

					status = ENrf905LldRet_Error;

				}

			}

		}

	}

	SET_PIN(NRF_CSN);

	return status;

}

/**
 * @brief Reads data from the device's registers
 * @param[in] instruction SPI instruction
 * @param[out] buffer Buffer for the read data
 * @param[in] dataSize Size of the expected data
 * @retval ENrf905LldRet Status
 */
static ENrf905LldRet Nrf905Lld_GenericRead(TByte instruction, TByte* buffer, TSize dataSize) {

	ENrf905LldRet status = ENrf905LldRet_Ok;

	RESET_PIN(NRF_CSN);

	/* Assert valid pointer dereference */
	if(NULL == buffer) {

		status = ENrf905LldRet_InvalidParams;

	}

	if(ENrf905LldRet_Ok == status) {

		/* Transmit instruction */
		if(HAL_OK != HAL_SPI_Transmit(&NRF_SPI_HANDLE, &instruction, 1, SPITXTIMEOUT)) {

			status = ENrf905LldRet_Error;

		}

	}

	if(ENrf905LldRet_Ok == status) {

		/* Read data */
		if(HAL_OK != HAL_SPI_Receive(&NRF_SPI_HANDLE, buffer, dataSize, SPIRXTIMEOUT)) {

			status = ENrf905LldRet_Error;

		}

	}

	SET_PIN(NRF_CSN);

	return status;

}
