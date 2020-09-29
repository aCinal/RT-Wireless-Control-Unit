/**
 * @file s2lp_api.c
 * @author Adrian Cinal
 * @brief S2-LP API source file
 */

#include "s2lp_api.h"
#include "s2lp_lld.h"

#include <stddef.h>

/* Set bits in a register based on a mask */
#define SET_BITS(REG, MASK, BITS)  ((REG) = (((TByte)(REG) & ~(TByte)(MASK)) | ((TByte)(MASK) & ((TByte)(BITS)))))

/**
 * @brief Send the S2-LP to TX state for transmission
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToTxState(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(TX_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Send the S2-LP to RX state for reception
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToRxState(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(RX_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Go to READY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToReadyState(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(READY_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Go to STANDBY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToStandbyState(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(STANDBY_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Go to SLEEP state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToSleepState(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(SLEEP_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Exit from TX or RX states and go to READY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_AbortTransmission(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(SABORT_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Reset the S2-LP state machine and registers values
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_Reset(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(SRES_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Clean the RX FIFO
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FlushRxFifo(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(FLUSHRXFIFO_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Clean the TX FIFO
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FlushTxFifo(void) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	if (ES2lpLldRet_Ok != S2lpLld_Command(FLUSHTXFIFO_COMMAND)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Specify GPIO I/O signal
 * @param gpio GPIO to configure
 * @param mode GPIO mode
 * @param sig GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioMode mode,
		ES2lpApiGpioSignal sig) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte addr;
	TByte reg;

	/* Identify the GPIO */
	switch (gpio) {

	case ES2lpApiGpioPin_0:

		addr = GPIO0_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_1:

		addr = GPIO1_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_2:

		addr = GPIO2_CONF_ADDR;
		break;

	case ES2lpApiGpioPin_3:

		addr = GPIO3_CONF_ADDR;
		break;

	default:

		status = ES2lpApiRet_InvalidParams;
		break;

	}

	if (ES2lpApiRet_Ok == status) {

		reg = 0;
		/* Set the GPIO_MODE field */
		switch (mode) {

		case ES2lpApiGpioMode_DigitalInput:

			SET_BITS(reg, 0x03, 0x01);
			break;

		case ES2lpApiGpioMode_DigitalOutputLowPower:

			SET_BITS(reg, 0x03, 0x02);
			break;

		case ES2lpApiGpioMode_DigitalOutputHighPower:

			SET_BITS(reg, 0x03, 0x03);
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the GPIO_SELECT field */
		if (ES2lpApiGpioMode_DigitalInput == mode) {

			/* Select GPIO digital input signal */
			switch (sig) {

			case ES2lpApiGpioSignal_i_TxCommand:

				SET_BITS(reg, 0xF8, 0x00);
				break;

			case ES2lpApiGpioSignal_i_RxCommand:

				SET_BITS(reg, 0xF8, 0x08);
				break;

			case ES2lpApiGpioSignal_i_TxDataInputForDirectModulation:

				SET_BITS(reg, 0xF8, 0x10);
				break;

			case ES2lpApiGpioSignal_i_WakeUpFromExternalInput:

				SET_BITS(reg, 0xF8, 0x18);
				break;

			case ES2lpApiGpioSignal_i_ExternalClockForLdcModesTiming:

				SET_BITS(reg, 0xF8, 0x20);
				break;

			default:

				status = ES2lpApiRet_InvalidParams;
				break;

			}

		} else {

			/* Select GPIO digital output signal */
			switch (sig) {

			case ES2lpApiGpioSignal_o_Irq:

				SET_BITS(reg, 0xF8, 0x00);
				break;

			case ES2lpApiGpioSignal_o_PorInverted:

				SET_BITS(reg, 0xF8, 0x08);
				break;

			case ES2lpApiGpioSignal_o_WakeUpTimerExpiration:

				SET_BITS(reg, 0xF8, 0x10);
				break;

			case ES2lpApiGpioSignal_o_LowBatteryDetection:

				SET_BITS(reg, 0xF8, 0x18);
				break;

			case ES2lpApiGpioSignal_o_TxDataInternalClockOutput:

				SET_BITS(reg, 0xF8, 0x20);
				break;

			case ES2lpApiGpioSignal_o_CommandInfoFromRadioTxBlock:

				SET_BITS(reg, 0xF8, 0x28);
				break;

			case ES2lpApiGpioSignal_o_FifoAlmostEmptyFlag:

				SET_BITS(reg, 0xF8, 0x30);
				break;

			case ES2lpApiGpioSignal_o_FifoAlmostFullFlag:

				SET_BITS(reg, 0xF8, 0x38);
				break;

			case ES2lpApiGpioSignal_o_RxDataOutput:

				SET_BITS(reg, 0xF8, 0x40);
				break;

			case ES2lpApiGpioSignal_o_RxClockOutput:

				SET_BITS(reg, 0xF8, 0x48);
				break;

			case ES2lpApiGpioSignal_o_RxStateIndication:

				SET_BITS(reg, 0xF8, 0x50);
				break;

			case ES2lpApiGpioSignal_o_DeviceNotInSleepOrStandbyState:

				SET_BITS(reg, 0xF8, 0x58);
				break;

			case ES2lpApiGpioSignal_o_DeviceInStandbyState:

				SET_BITS(reg, 0xF8, 0x60);
				break;

			case ES2lpApiGpioSignal_o_AntennaSwitchSignal:

				SET_BITS(reg, 0xF8, 0x68);
				break;

			case ES2lpApiGpioSignal_o_ValidPreambleDetectedFlag:

				SET_BITS(reg, 0xF8, 0x70);
				break;

			case ES2lpApiGpioSignal_o_SyncWordDetectedFlag:

				SET_BITS(reg, 0xF8, 0x78);
				break;

			case ES2lpApiGpioSignal_o_RssiAboveThreshold:

				SET_BITS(reg, 0xF8, 0x80);
				break;

			case ES2lpApiGpioSignal_o_TxRxModeIndicator:

				SET_BITS(reg, 0xF8, 0x90);
				break;

			case ES2lpApiGpioSignal_o_Vdd:

				SET_BITS(reg, 0xF8, 0x98);
				break;

			case ES2lpApiGpioSignal_o_Gnd:

				SET_BITS(reg, 0xF8, 0xA0);
				break;

			case ES2lpApiGpioSignal_o_ExternalSmpsEnableSignal:

				SET_BITS(reg, 0xF8, 0xA8);
				break;

			case ES2lpApiGpioSignal_o_DeviceInSleepState:

				SET_BITS(reg, 0xF8, 0xB0);
				break;

			case ES2lpApiGpioSignal_o_DeviceInReadyState:

				SET_BITS(reg, 0xF8, 0xB8);
				break;

			case ES2lpApiGpioSignal_o_DeviceInLockState:

				SET_BITS(reg, 0xF8, 0xC0);
				break;

			case ES2lpApiGpioSignal_o_DeviceWaitingForHighLockDetectorSignal:

				SET_BITS(reg, 0xF8, 0xC8);
				break;

			case ES2lpApiGpioSignal_o_TxDataOokSignal:

				SET_BITS(reg, 0xF8, 0xD0);
				break;

			case ES2lpApiGpioSignal_o_DeviceWaitingForHighReady2SignalFromXo:

				SET_BITS(reg, 0xF8, 0xD8);
				break;

			case ES2lpApiGpioSignal_o_DeviceWaitingForTimerExpirationToAllowPmBlockSettling:

				SET_BITS(reg, 0xF8, 0xE0);
				break;

			case ES2lpApiGpioSignal_o_DeviceWaitingForEndOfVcoCalibration:

				SET_BITS(reg, 0xF8, 0xE8);
				break;

			case ES2lpApiGpioSignal_o_DeviceEnablesTheFullSynthBlockCircuitry:

				SET_BITS(reg, 0xF8, 0xF0);
				break;

			default:

				status = ES2lpApiRet_InvalidParams;
				break;

			}

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write to register GPIOx_CONF */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(addr, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set charge pump current
 * @param icp Charge pump current
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetChargePumpCurrent(ES2lpApiIcp icp) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	/* Fetch register SYNT3 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNT3_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the PLL_CP_ISEL field */
		switch (icp) {

		case ES2lpApiIcp_120uA:

			SET_BITS(reg, 0xE0, 0x40);
			break;

		case ES2lpApiIcp_200uA:

			SET_BITS(reg, 0xE0, 0x20);
			break;

		case ES2lpApiIcp_140uA:

			SET_BITS(reg, 0xE0, 0x60);
			break;

		case ES2lpApiIcp_240uA:

			SET_BITS(reg, 0xE0, 0x40);
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register SYNT3 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register SYNTH_CONFIG2 */
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNTH_CONFIG2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the PLL_PFD_SPLIT_EN field */
		switch (icp) {

		case ES2lpApiIcp_120uA:

			SET_BITS(reg, 0x04, 0x00);
			break;

		case ES2lpApiIcp_200uA:

			SET_BITS(reg, 0x04, 0x04);
			break;

		case ES2lpApiIcp_140uA:

			SET_BITS(reg, 0x04, 0x00);
			break;

		case ES2lpApiIcp_240uA:

			SET_BITS(reg, 0x04, 0x04);
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register SYNTH_CONFIG2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNTH_CONFIG2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set base frequency
 * @param config Configuration structure
 * @retval ES2lpApiRet Status
 * @note f_base = (f_xo * SYNT) / 2^(BANDSELECT + REFDIV + 21)
 */
ES2lpApiRet S2lpApi_SetBaseFrequency(SS2lpApiBaseFrequencyConfig config) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	/* Fetch register SYNT3 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNT3_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the BS field */
		SET_BITS(reg, 0x10, (config.BANDSELECT << 4));
		/* Set the SYNT[27:24] field */
		SET_BITS(reg, 0x0F, (config.SYNT >> 24));
		/* Write back to register SYNT3  */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the SYNT[23:16] field */
		reg = (config.SYNT >> 16) & 0xFF;
		/* Write to register SYNT2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the SYNT[15:8] field */
		reg = (config.SYNT >> 8) & 0xFF;
		/* Write to register SYNT2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT1_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the SYNT[7:0] field */
		reg = config.SYNT & 0xFF;
		/* Write to register SYNT2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register XO_RCO_CONF0 */
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(XO_RCO_CONF0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the REFDIV field */
		SET_BITS(reg, 0x80, (config.REFDIV << 3));
		/* Write back to register XO_RCO_CONF0  */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(XO_RCO_CONF0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Select the RF channel
 * @param chspace Value to write to register CHSPACE
 * @param chnum Value to write to register CHNUM
 * @retval ES2lpApiRet Status
 * @note frequency = f_base + f_xo / (2^15 * CHSPACE) * CHNUM
 */
ES2lpApiRet S2lpApi_SetCenterFrequency(TByte chspace, TByte chnum) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Write to register CHSPACE */
	if (ES2lpLldRet_Ok != S2lpLld_WriteReg(CHSPACE_ADDR, &chspace, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write to register CHNUM */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(CHNUM_ADDR, &chnum, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set the data rate by configuring the DATARATE_M and DATARATE_E registers
 * @param mantissa 16-bit mantissa value of the data rate equation
 * @param exponent 4-bit exponent value of the data rate equation
 * @retval ES2lpApiRet Status
 * @note Data rate formula:
 *           DataRate = f_dig * DATARATE_M/2^32 if DATARATE_E = 0
 *           DataRate = f_dig * (2^16 + DATARATE_M) * 2^DATARATE_E / 2^33 if DATARATE_E >0
 *           DataRate = f_dig / (8 * DATARATE_M)
 *       where f_dig is the digital clock frequency
 */
ES2lpApiRet S2lpApi_SetDataRate(TWord mantissa, TByte exponent) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	reg = (mantissa >> 8) & 0xFF;
	/* Write to register MOD4 */
	if (ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD4_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		reg = mantissa & 0xFF;
		/* Write to register MOD3 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register MOD2 */
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(MOD2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the DATARATE_E field */
		SET_BITS(reg, 0x0F, exponent);
		/* Write back to register MOD2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set modulation type
 * @param type Modulation type
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetModulationType(ES2lpApiModulationType type) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	/* Fetch register MOD2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(MOD2_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the MOD_TYPE field */
		switch (type) {

		case ES2lpApiModulationType_2_FSK:

			SET_BITS(reg, 0xF0, 0x00);
			break;

		case ES2lpApiModulationType_4_FSK:

			SET_BITS(reg, 0xF0, 0x10);
			break;

		case ES2lpApiModulationType_2_GFSK_BT_1:

			SET_BITS(reg, 0xF0, 0x20);
			break;

		case ES2lpApiModulationType_4_GFSK_BT_1:

			SET_BITS(reg, 0xF0, 0x30);
			break;

		case ES2lpApiModulationType_ASK_OOK:

			SET_BITS(reg, 0xF0, 0x50);
			break;

		case ES2lpApiModulationType_PolarMode:

			SET_BITS(reg, 0xF0, 0x60);
			break;

		case ES2lpApiModulationType_Unmodulated:

			SET_BITS(reg, 0xF0, 0x70);
			break;

		case ES2lpApiModulationType_2_GFSK_BT_0_5:

			SET_BITS(reg, 0xF0, 0xA0);
			break;

		case ES2lpApiModulationType_4_GFSK_BT_0_5:

			SET_BITS(reg, 0xF0, 0xB0);
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register MOD2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set signal detect threshold
 * @param rssiThreshold Value to write to register RSSI_TH
 * @retval ES2lpApiRet Status
 * @note Threshold in dBm is (RSSI_TH - 146)
 */
ES2lpApiRet S2lpApi_SetSignalDetectThreshold(TByte rssiThreshold) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Write to register RSSI_TH */
	if (ES2lpLldRet_Ok != S2lpLld_WriteReg(RSSI_TH_ADDR, &rssiThreshold, 1)) {

		status = ES2lpApiRet_Error;

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
	TByte reg;

	/* Fetch register PCKTCTRL2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(PCKTCTRL2_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set FIX_VAR_LEN field */
		SET_BITS(reg, 0x01, 0x00);
		/* Write back to register PCKTCTRL2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(PCKTCTRL2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set PCKTLEN1 field */
		reg = (length >> 8) & 0xFF;
		/* Write to register PCKTLEN1 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(PCKTLEN1_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set PCKTLEN0 field */
		reg = length & 0xFF;
		/* Write to register PCKTLEN0 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(PCKTLEN0_ADDR, &reg, 1)) {

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
	TByte reg;

	/* Fetch register PCKTCTRL2 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(PCKTCTRL2_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set MANCHESTER_EN field */
		SET_BITS(reg, 0x02, (enable ? 0x02 : 0x00));
		/* Write back to register PCKTCTRL2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(PCKTCTRL2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Configure CRC calculation
 * @param mode CRC mode
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigCrc(ES2lpApiCrcMode mode) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	/* Fetch register PCKTCTRL1 */
	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(PCKTCTRL1_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set CRC_MODE field */
		switch (mode) {

		case ES2lpApiCrcMode_NoCRCField:

			SET_BITS(reg, 0xE0, 0x00);
			break;

		case ES2lpApiCrcMode_Poly0x07:

			SET_BITS(reg, 0xE0, 0x20);
			break;

		case ES2lpApiCrcMode_Poly0x8005:

			SET_BITS(reg, 0xE0, 0x40);
			break;

		case ES2lpApiCrcMode_Poly0x1021:

			SET_BITS(reg, 0xE0, 0x60);
			break;

		case ES2lpApiCrcMode_Poly0x864CBF:

			SET_BITS(reg, 0xE0, 0x80);
			break;

		case ES2lpApiCrcMode_Poly0x04C011BB7:

			SET_BITS(reg, 0xE0, 0xA0);
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register PCKTCTRL2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(PCKTCTRL1_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		if (ES2lpApiCrcMode_NoCRCField != mode) {

			/* Fetch register PCKT_FLT_OPTIONS */
			if (ES2lpLldRet_Ok
					!= S2lpLld_ReadReg(PCKT_FLT_OPTIONS_ADDR, &reg, 1)) {

				status = ES2lpApiRet_Error;

			}

		}

	}

	if (ES2lpApiRet_Ok == status) {

		if (ES2lpApiCrcMode_NoCRCField != mode) {

			/* Set the CRC_FLT field */
			SET_BITS(reg, 0x01, 0x01);

			/* Write back to register PCKT_FLT_OPTIONS */
			if (ES2lpLldRet_Ok
					!= S2lpLld_WriteReg(PCKT_FLT_OPTIONS_ADDR, &reg, 1)) {

				status = ES2lpApiRet_Error;

			}

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
	TByte reg;

	reg = (sync >> 24) & 0xFF;
	/* Write to register SYNC3 */
	if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNC3_ADDR, &reg, 4)) {

		status = ES2lpApiRet_Error;

	}

	if (ES2lpApiRet_Ok == status) {

		reg = (sync >> 16) & 0xFF;
		/* Write to register SYNC2 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNC2_ADDR, &reg, 4)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		reg = (sync >> 8) & 0xFF;
		/* Write to register SYNC1 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNC1_ADDR, &reg, 4)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		reg = sync & 0xFF;
		/* Write to register SYNC0 */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNC0_ADDR, &reg, 4)) {

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

	if (ES2lpLldRet_Ok != S2lpLld_ReadReg(RSSI_LEVEL_ADDR, rssiPtr, 1)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Enable/disable IRQ generation on a given event
 * @param event Interrupt event
 * @param enable True to enable IRQ generation, false otherwise
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableInterrupt(ES2lpApiInterruptEvent event, bool enable) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte addr;
	TByte reg;

	/* Identify the event */
	switch (event) {

	case ES2lpApiInterruptEvent_RxDataReady:
	case ES2lpApiInterruptEvent_RxDataDiscarded:
	case ES2lpApiInterruptEvent_TxDataSent:
	case ES2lpApiInterruptEvent_MaxReTxReached:
	case ES2lpApiInterruptEvent_CrcError:
	case ES2lpApiInterruptEvent_TxFifoOverflowUnderflowError:
	case ES2lpApiInterruptEvent_RxFifoOverflowUnderflowError:
	case ES2lpApiInterruptEvent_TxFifoAlmostFull:

		addr = IRQ_MASK0_ADDR;
		break;

	case ES2lpApiInterruptEvent_TxFifoAlmostEmpty:
	case ES2lpApiInterruptEvent_RxFifoAlmostFull:
	case ES2lpApiInterruptEvent_RxFifoAlmostEmpty:
	case ES2lpApiInterruptEvent_MaxNoOfBackOffDuringCca:
	case ES2lpApiInterruptEvent_ValidPreambleDetected:
	case ES2lpApiInterruptEvent_SyncWordDetected:
	case ES2lpApiInterruptEvent_RssiAboveThreshold:
	case ES2lpApiInterruptEvent_WakeUpTimeoutInLdcrMode:

		addr = IRQ_MASK1_ADDR;
		break;

	case ES2lpApiInterruptEvent_Ready:
	case ES2lpApiInterruptEvent_StandbyStateSwitchingInProgress:
	case ES2lpApiInterruptEvent_LowBatteryLevel:
	case ES2lpApiInterruptEvent_PowerOnReset:

		addr = IRQ_MASK2_ADDR;
		break;

	case ES2lpApiInterruptEvent_RxTimerTimeout:
	case ES2lpApiInterruptEvent_SniffTimerTimeout:

		addr = IRQ_MASK3_ADDR;
		break;

	default:

		status = ES2lpApiRet_InvalidParams;
		break;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register IRQ_MASKx */
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(addr, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Set the INT_MASK field */
		switch (event) {

		case ES2lpApiInterruptEvent_RxDataReady:

			SET_BITS(reg, 0x01, (enable ? 0x01 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RxDataDiscarded:

			SET_BITS(reg, 0x02, (enable ? 0x02 : 0x00));
			break;

		case ES2lpApiInterruptEvent_TxDataSent:

			SET_BITS(reg, 0x04, (enable ? 0x04 : 0x00));
			break;

		case ES2lpApiInterruptEvent_MaxReTxReached:

			SET_BITS(reg, 0x08, (enable ? 0x08 : 0x00));
			break;

		case ES2lpApiInterruptEvent_CrcError:

			SET_BITS(reg, 0x10, (enable ? 0x10 : 0x00));
			break;

		case ES2lpApiInterruptEvent_TxFifoOverflowUnderflowError:

			SET_BITS(reg, 0x20, (enable ? 0x20 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RxFifoOverflowUnderflowError:

			SET_BITS(reg, 0x40, (enable ? 0x40 : 0x00));
			break;

		case ES2lpApiInterruptEvent_TxFifoAlmostFull:

			SET_BITS(reg, 0x80, (enable ? 0x80 : 0x00));
			break;

		case ES2lpApiInterruptEvent_TxFifoAlmostEmpty:

			SET_BITS(reg, 0x01, (enable ? 0x01 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RxFifoAlmostFull:

			SET_BITS(reg, 0x02, (enable ? 0x02 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RxFifoAlmostEmpty:

			SET_BITS(reg, 0x04, (enable ? 0x04 : 0x00));
			break;

		case ES2lpApiInterruptEvent_MaxNoOfBackOffDuringCca:

			SET_BITS(reg, 0x08, (enable ? 0x08 : 0x00));
			break;

		case ES2lpApiInterruptEvent_ValidPreambleDetected:

			SET_BITS(reg, 0x10, (enable ? 0x10 : 0x00));
			break;

		case ES2lpApiInterruptEvent_SyncWordDetected:

			SET_BITS(reg, 0x20, (enable ? 0x20 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RssiAboveThreshold:

			SET_BITS(reg, 0x40, (enable ? 0x40 : 0x00));
			break;

		case ES2lpApiInterruptEvent_WakeUpTimeoutInLdcrMode:

			SET_BITS(reg, 0x80, (enable ? 0x80 : 0x00));
			break;

		case ES2lpApiInterruptEvent_Ready:

			SET_BITS(reg, 0x01, (enable ? 0x01 : 0x00));
			break;

		case ES2lpApiInterruptEvent_StandbyStateSwitchingInProgress:

			SET_BITS(reg, 0x02, (enable ? 0x02 : 0x00));
			break;

		case ES2lpApiInterruptEvent_LowBatteryLevel:

			SET_BITS(reg, 0x04, (enable ? 0x04 : 0x00));
			break;

		case ES2lpApiInterruptEvent_PowerOnReset:

			SET_BITS(reg, 0x08, (enable ? 0x08 : 0x00));
			break;

		case ES2lpApiInterruptEvent_RxTimerTimeout:

			SET_BITS(reg, 0x10, (enable ? 0x10 : 0x00));
			break;

		case ES2lpApiInterruptEvent_SniffTimerTimeout:

			SET_BITS(reg, 0x20, (enable ? 0x20 : 0x00));
			break;

		default:

			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Write back to register IRQ_MASKx */
		if (ES2lpLldRet_Ok != S2lpLld_WriteReg(addr, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Test if a given interrupt event occured
 * @param event Interrupt event
 * @param booleanPtr Buffer to pass the Boolean value out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_TestInterrupt(ES2lpApiInterruptEvent event,
bool *booleanPtr) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte addr;
	TByte reg;

	/* Assert valid parameters */
	if (NULL == booleanPtr) {

		status = ES2lpApiRet_InvalidParams;

	}

	if (ES2lpApiRet_Ok == status) {

		/* Identify the event */
		switch (event) {

		case ES2lpApiInterruptEvent_RxDataReady:
		case ES2lpApiInterruptEvent_RxDataDiscarded:
		case ES2lpApiInterruptEvent_TxDataSent:
		case ES2lpApiInterruptEvent_MaxReTxReached:
		case ES2lpApiInterruptEvent_CrcError:
		case ES2lpApiInterruptEvent_TxFifoOverflowUnderflowError:
		case ES2lpApiInterruptEvent_RxFifoOverflowUnderflowError:
		case ES2lpApiInterruptEvent_TxFifoAlmostFull:

			addr = IRQ_STATUS0_ADDR;
			break;

		case ES2lpApiInterruptEvent_TxFifoAlmostEmpty:
		case ES2lpApiInterruptEvent_RxFifoAlmostFull:
		case ES2lpApiInterruptEvent_RxFifoAlmostEmpty:
		case ES2lpApiInterruptEvent_MaxNoOfBackOffDuringCca:
		case ES2lpApiInterruptEvent_ValidPreambleDetected:
		case ES2lpApiInterruptEvent_SyncWordDetected:
		case ES2lpApiInterruptEvent_RssiAboveThreshold:
		case ES2lpApiInterruptEvent_WakeUpTimeoutInLdcrMode:

			addr = IRQ_STATUS1_ADDR;
			break;

		case ES2lpApiInterruptEvent_Ready:
		case ES2lpApiInterruptEvent_StandbyStateSwitchingInProgress:
		case ES2lpApiInterruptEvent_LowBatteryLevel:
		case ES2lpApiInterruptEvent_PowerOnReset:

			addr = IRQ_STATUS2_ADDR;
			break;

		case ES2lpApiInterruptEvent_RxTimerTimeout:
		case ES2lpApiInterruptEvent_SniffTimerTimeout:

			addr = IRQ_STATUS3_ADDR;
			break;

		default:

			status = ES2lpApiRet_InvalidParams;
			break;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Fetch register IRQ_MASKx */
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(addr, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if (ES2lpApiRet_Ok == status) {

		/* Test the interrupt status bit */
		switch (event) {

		case ES2lpApiInterruptEvent_RxDataReady:

			*booleanPtr = (reg & 0x01) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RxDataDiscarded:

			*booleanPtr = (reg & 0x02) ? true : false;
			break;

		case ES2lpApiInterruptEvent_TxDataSent:

			*booleanPtr = (reg & 0x04) ? true : false;
			break;

		case ES2lpApiInterruptEvent_MaxReTxReached:

			*booleanPtr = (reg & 0x08) ? true : false;
			break;

		case ES2lpApiInterruptEvent_CrcError:

			*booleanPtr = (reg & 0x10) ? true : false;
			break;

		case ES2lpApiInterruptEvent_TxFifoOverflowUnderflowError:

			*booleanPtr = (reg & 0x20) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RxFifoOverflowUnderflowError:

			*booleanPtr = (reg & 0x40) ? true : false;
			break;

		case ES2lpApiInterruptEvent_TxFifoAlmostFull:

			*booleanPtr = (reg & 0x80) ? true : false;
			break;

		case ES2lpApiInterruptEvent_TxFifoAlmostEmpty:

			*booleanPtr = (reg & 0x01) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RxFifoAlmostFull:

			*booleanPtr = (reg & 0x02) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RxFifoAlmostEmpty:

			*booleanPtr = (reg & 0x04) ? true : false;
			break;

		case ES2lpApiInterruptEvent_MaxNoOfBackOffDuringCca:

			*booleanPtr = (reg & 0x08) ? true : false;
			break;

		case ES2lpApiInterruptEvent_ValidPreambleDetected:

			*booleanPtr = (reg & 0x10) ? true : false;
			break;

		case ES2lpApiInterruptEvent_SyncWordDetected:

			*booleanPtr = (reg & 0x20) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RssiAboveThreshold:

			*booleanPtr = (reg & 0x40) ? true : false;
			break;

		case ES2lpApiInterruptEvent_WakeUpTimeoutInLdcrMode:

			*booleanPtr = (reg & 0x80) ? true : false;
			break;

		case ES2lpApiInterruptEvent_Ready:

			*booleanPtr = (reg & 0x01) ? true : false;
			break;

		case ES2lpApiInterruptEvent_StandbyStateSwitchingInProgress:

			*booleanPtr = (reg & 0x02) ? true : false;
			break;

		case ES2lpApiInterruptEvent_LowBatteryLevel:

			*booleanPtr = (reg & 0x04) ? true : false;
			break;

		case ES2lpApiInterruptEvent_PowerOnReset:

			*booleanPtr = (reg & 0x08) ? true : false;
			break;

		case ES2lpApiInterruptEvent_RxTimerTimeout:

			*booleanPtr = (reg & 0x10) ? true : false;
			break;

		case ES2lpApiInterruptEvent_SniffTimerTimeout:

			*booleanPtr = (reg & 0x20) ? true : false;
			break;

		default:

			break;

		}

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
		if (ES2lpLldRet_Ok != S2lpLld_ReadReg(FIFO_ADDR, bufPtr, numOfBytes)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}
