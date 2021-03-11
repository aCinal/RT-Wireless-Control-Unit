/**
 * @file s2lp_api.c
 * @author Adrian Cinal
 * @brief S2-LP API source file
 */

#include "s2lp_api.h"
#include "s2lp_lld.h"

#include <stddef.h>

/**
 * @brief Start up the device
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_Start(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Reset()) {
		status = ES2lpApiRet_Error;
	}

	/* TODO: Run default configuration */

	return status;

}

/**
 * @brief Send a command to the S2-LP device
 * @param command Command to be transmitted
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SendCommand(TByte command) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Assert valid command */
	if (!IS_S2LP_COMMAND(command)) {

		status = ES2lpApiRet_InvalidParams;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Transmit the command */
		if (ES2lpLldRet_Ok != S2lpLld_SendCommand(command, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Enable IRQ generation on a given event
 * @param events Interrupt event(s) bit(s) (bitwise ORed if multiple)
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableInterrupt(TWord events) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TWord irqMask;

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register word IRQ_MASK */
		if (ES2lpLldRet_Ok
				!= S2lpLld_ReadReg(S2LP_IRQ_MASK3_ADDR, (TByte*) &irqMask, 4,
				NULL)) {

			status = ES2lpApiRet_Error;

		}

		/* Swap bytes to little endian */
		irqMask = SWAP_ENDIAN_32(irqMask);

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the INT_MASK field */
		irqMask |= events;
		/* Swap bytes back to big endian */
		irqMask = SWAP_ENDIAN_32(irqMask);
		/* Write back to register IRQ_MASK */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_IRQ_MASK3_ADDR, (TByte*) &irqMask, 4,
				NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Disable IRQ generation on a given event
 * @param events Interrupt event(s) bit(s) (bitwise ORed if multiple)
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_DisableInterrupt(TWord events) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TWord irqMask;

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register word IRQ_MASK */
		if (ES2lpLldRet_Ok
				!= S2lpLld_ReadReg(S2LP_IRQ_MASK3_ADDR, (TByte*) &irqMask, 4,
				NULL)) {

			status = ES2lpApiRet_Error;

		}

		/* Swap bytes to little endian */
		irqMask = SWAP_ENDIAN_32(irqMask);

	}

	if (ES2lpApiRet_Ok == status) {

		/* Reset the INT_MASK field */
		irqMask = ~((~irqMask) | ((TWord) events));
		/* Swap bytes back to big endian */
		irqMask = SWAP_ENDIAN_32(irqMask);
		/* Write back to register IRQ_MASK */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_IRQ_MASK3_ADDR, (TByte*) &irqMask, 4,
				NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}


/**
 * @brief Test if a given interrupt event has occurred and clear the IRQ_STATUS register
 * @param irqStatusPtr Pointer to pass the IRQ_STATUS register contents out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FetchInterruptStatus(TWord* irqStatusPtr) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TWord irqStatus;

	/* Assert valid parameters */
	if (NULL == irqStatusPtr) {

		status = ES2lpApiRet_InvalidParams;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register word IRQ_STATUS */
		if (ES2lpLldRet_Ok
				!= S2lpLld_ReadReg(S2LP_IRQ_STATUS3_ADDR, (TByte*) &irqStatus,
						4,
						NULL)) {

			status = ES2lpApiRet_Error;

		}

		/* Swap bytes to little endian */
		irqStatus = SWAP_ENDIAN_32(irqStatus);

	}

	if (ES2lpApiRet_Ok == status) {

		/* Test the interrupt status bit and clear it */
		*irqStatusPtr = irqStatus;

	}

	return status;

}

/**
 * @brief Specify GPIO I/O signal
 * @param gpio GPIO to configure
 * @param sig GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioSignal sig) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte addr;
	TByte reg;

	/* Identify the GPIO */
	switch (gpio) {

	case ES2lpApiGpioPin_0:

		addr = S2LP_GPIO0_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_1:

		addr = S2LP_GPIO1_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_2:

		addr = S2LP_GPIO2_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_3:

		addr = S2LP_GPIO3_CONF_ADDR;
		break;

	default:

		status = ES2lpApiRet_InvalidParams;
		break;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the GPIO_SELECT and GPIO_MODE fields */
		reg = (TByte) sig;

		/* Write to register GPIOx_CONF */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(addr, &reg, 1, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Select modulation type
 * @param type Modulation type
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SelectModulationType(ES2lpApiModulationType type) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte mod2;

	/* Fetch register MOD2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(S2LP_MOD2_ADDR, &mod2, 1, NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Clear the MOD_TYPE field */
		mod2 &= ~0xF0;
		/* Set the MOD_TYPE field */
		mod2 |= (TByte) type;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register MOD2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(S2LP_MOD2_ADDR, &mod2, 1, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set a fixed packet length
 * @param length Packet length
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetPacketLength(THalfWord length) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte pcktCtrl2;
	TByte pcktLen[2];

	/* Fetch register PCKTCTRL2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(S2LP_PCKTCTRL2_ADDR, &pcktCtrl2, 1, NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set FIX_VAR_LEN field */
		SET_BITS(pcktCtrl2, 0x01, 0x00);
		/* Write back to register PCKTCTRL2 */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_PCKTCTRL2_ADDR, &pcktCtrl2, 1, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set PCKTLEN1 field */
		pcktLen[0] = (length >> 8) & 0xFF;
		/* Set PCKTLEN0 field */
		pcktLen[1] = length & 0xFF;
		/* Write to register PCKTLEN */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_PCKTLEN1_ADDR, pcktLen, sizeof(pcktLen), NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Sets a fixed packet length
 * @param enable True to enable Manchester encoding, false otherwise
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableManchester(bool enable) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte pcktCtrl2;

	/* Fetch register PCKTCTRL2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(S2LP_PCKTCTRL2_ADDR, &pcktCtrl2, 1, NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set MANCHESTER_EN field */
		SET_BITS(pcktCtrl2, 0x02, (enable ? 0x02 : 0x00));
		/* Write back to register PCKTCTRL2 */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_PCKTCTRL2_ADDR, &pcktCtrl2, 1, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set synchronization word
 * @param sync Synchronization word
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetSyncWord(TWord sync) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Endianness must not be changed if the host MCU is little-endian!
	 * By writing to address S2LP_SYNC3_ADDR with increment we change the endianness to big-endian,
	 * but since S2-LP matches the sync word most-significant-bit-first starting from the least significant
	 * byte (SYNC0), so in little-endian, this works out exactly as expected with no manual byte swapping
	 * required. */

	/* Write to register SYNC */
	if (ES2lpLldRet_Ok
			!= S2lpLld_WriteReg(S2LP_SYNC3_ADDR, (TByte*) &sync, 4, NULL)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Set data rate
 * @param datarate_m DATARATE_M (mantissa) in the data rate equation
 * @param datarate_e DATARATE_E (exponent) in the data rate equation
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetDataRate(THalfWord datarate_m, TByte datarate_e) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte mod4_2[3];

	/* Assert valid datarate_e value */
	if (0 != (datarate_e & 0xF0)) {

		return ES2lpApiRet_InvalidParams;

	}

	/* Fetch registers MOD4, MOD3 and MOD2 */
	if (ES2lpLldRet_Ok
			!= S2lpLld_ReadReg(S2LP_MOD4_ADDR, mod4_2, sizeof(mod4_2), NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Place the datarate_m MSB in register MOD4 */
		mod4_2[0] = (datarate_m & 0xFF00) >> 8;
		/* Place the datarate_m LSB in register MOD3 */
		mod4_2[1] = datarate_m & 0x00FF;

		/* Clear the DATARATE_E field in register MOD2 */
		mod4_2[2] &= ~0x0F;
		/* Set the DATARATE_E field */
		mod4_2[2] |= datarate_e;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to registers MOD4, MOD3 and MOD2 */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_MOD4_ADDR, mod4_2, sizeof(mod4_2), NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set frequency deviation
 * @param fdev_m FDEV_M (mantissa) in the frequency deviation equation
 * @param fdev_e FDEV_E (exponent) in the frequency deviation equation
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetFrequencyDeviation(TByte fdev_m, TByte fdev_e) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	TByte mod1_0[2];

	/* Assert valid fdev_e value */
	if (0 != (fdev_e & 0xF0)) {

		return ES2lpApiRet_InvalidParams;

	}

	/* Fetch registers MOD1 and MOD0 */
	if (ES2lpLldRet_Ok
			!= S2lpLld_ReadReg(S2LP_MOD1_ADDR, mod1_0, sizeof(mod1_0), NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Clear the FDEV_E field in register MOD1 */
		mod1_0[0] &= ~0x0F;
		/* Set the FDEV_E field */
		mod1_0[0] |= fdev_e;

		/* Write fdev_m to register MOD0 */
		mod1_0[1] = fdev_m;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to registers MOD1 and MOD0 */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_MOD1_ADDR, mod1_0, sizeof(mod1_0), NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set channel filter bandwidth
 * @param chflt_m CHFLT_M (mantissa) in the channel filter equation/table
 * @param chflt_e CHLFT_E (exponent) in the channel filter equation/table
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetChannelFilterBandwidth(TByte chflt_m, TByte chflt_e) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	TByte chflt;

	/* Assert valid chflt_m and chflt_e values */
	if (0 != (chflt_m & 0xF0) || 0 != (chflt_e & 0xF0)) {

		return ES2lpApiRet_InvalidParams;

	}

	/* Fetch register CHFLT */
	if (ES2lpLldRet_Ok
			!= S2lpLld_ReadReg(S2LP_CHFLT_ADDR, &chflt, 1, NULL)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write the chflt_m and chflt_e values to the CHFLT register */
		chflt = (chflt_m << 4) | chflt_e;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register CHFLT */
		if (ES2lpLldRet_Ok
				!= S2lpLld_WriteReg(S2LP_CHFLT_ADDR, &chflt, 1, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;
}

/**
 * @brief Read the RSSI level captured at the end of the SYNC word detection of the received packet
 * @param rssiPtr Buffer to pass the RSSI value out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRssi(TByte *rssiPtr) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok
			!= S2lpLld_ReadReg(S2LP_RSSI_LEVEL_ADDR, rssiPtr, 1, NULL)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Read RX payload
 * @param bufPtr Buffer to pass the payload out of the function
 * @param numOfBytes Number of bytes to read
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRxPayload(TByte *bufPtr, TSize numOfBytes) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Assert valid parameters */
	if ((NULL == bufPtr) || (0 == numOfBytes)) {

		status = ES2lpApiRet_InvalidParams;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Read from the RX FIFO */
		if (ES2lpLldRet_Ok
				!= S2lpLld_ReadReg(S2LP_FIFO_ADDR, bufPtr, numOfBytes, NULL)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}
