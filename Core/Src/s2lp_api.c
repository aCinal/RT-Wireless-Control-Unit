/**
 * @file s2lp_api.c
 * @author Adrian Cinal
 * @brief S2-LP API source file
 */

#include "s2lp_api.h"
#include "s2lp_lld.h"

/* Set bits in a register based on a mask */
#define SET_BITS(REG, MASK, BITS)  ((REG) = (((TByte)(REG) & ~(TByte)(MASK)) | ((TByte)(MASK) & ((TByte)(BITS)))))

/**
 * @brief Specify GPIO I/O signal
 * @param gpio GPIO to configure
 * @param mode GPIO mode
 * @param signal GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigureGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioMode mode, ES2lpApiGpioSignal signal) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte addr;
	TByte reg;

	/* Identify the GPIO */
	switch(gpio) {

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

	if(ES2lpApiRet_Ok == status) {

		reg = 0;
		/* Set the GPIO_MODE field */
		switch(mode) {

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

	if(ES2lpApiRet_Ok == status) {

		/* Set the GPIO_SELECT field */
		if(ES2lpApiGpioMode_DigitalInput == mode) {

			/* Select GPIO digital input signal */
			switch(signal) {

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
			switch(signal) {

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

	if(ES2lpApiRet_Ok == status) {

		/* Write to register GPIOx_CONF */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(addr, &reg, 1)) {

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
	if(ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNT3_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the PLL_CP_ISEL field */
		switch(icp) {

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

	if(ES2lpApiRet_Ok == status) {

		/* Write back to register SYNT3 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Fetch register SYNTH_CONFIG2 */
		if(ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNTH_CONFIG2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the PLL_PFD_SPLIT_EN field */
		switch(icp) {

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

	if(ES2lpApiRet_Ok == status) {

		/* Write back to register SYNTH_CONFIG2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNTH_CONFIG2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set base frequency
 * @param config Configuration structure
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetBaseFrequency(SS2lpApiBaseFrequencySettings config) {

	ES2lpApiRet status = ES2lpApiRet_Ok;
	TByte reg;

	/* Fetch register SYNT3 */
	if(ES2lpLldRet_Ok != S2lpLld_ReadReg(SYNT3_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the BS field */
		SET_BITS(reg, 0x10, (config.BANDSELECT << 4));
		/* Set the SYNT[27:24] field */
		SET_BITS(reg, 0x0F, (config.SYNT >> 24));
		/* Write back to register SYNT3  */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the SYNT[23:16] field */
		reg = (config.SYNT >> 16) & 0xFF;
		/* Write to register SYNT2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the SYNT[15:8] field */
		reg = (config.SYNT >> 8) & 0xFF;
		/* Write to register SYNT2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT1_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the SYNT[7:0] field */
		reg = config.SYNT & 0xFF;
		/* Write to register SYNT2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(SYNT0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Fetch register XO_RCO_CONF0 */
		if(ES2lpLldRet_Ok != S2lpLld_ReadReg(XO_RCO_CONF0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the REFDIV field */
		SET_BITS(reg, 0x80, (config.REFDIV << 3));
		/* Write back to register XO_RCO_CONF0  */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(XO_RCO_CONF0_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}

/**
 * @brief Set intermediate frequency for analog RF synthesizer
 * @param ifOffsetAna Value to write to register IF_OFFSET_ANA
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetIntFreqForAnalogRFSynth(TByte ifOffsetAna) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Write to register IF_OFFSET_ANA */
	if(ES2lpLldRet_Ok != S2lpLld_WriteReg(IF_OFFSET_ANA_ADDR, &ifOffsetAna, 1)) {

		status = ES2lpApiRet_Error;

	}

	return status;

}

/**
 * @brief Set intermediate frequency for digital shift-to-baseband circuits
 * @param ifOffsetAna Value to write to register IF_OFFSET_DIG
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetIntFreqForDigitalShiftToBbCircuits(TByte ifOffsetDig) {

	ES2lpApiRet status = ES2lpApiRet_Ok;

	/* Write to register IF_OFFSET_DIG */
	if(ES2lpLldRet_Ok != S2lpLld_WriteReg(IF_OFFSET_DIG_ADDR, &ifOffsetDig, 1)) {

		status = ES2lpApiRet_Error;

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
	if(ES2lpLldRet_Ok != S2lpLld_WriteReg(CHSPACE_ADDR, &chspace, 1)) {

		status = ES2lpApiRet_Error;

	}

	if(ES2lpApiRet_Ok == status) {

		/* Write to register CHNUM */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(CHNUM_ADDR, &chnum, 1)) {

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
	if(ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD4_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if(ES2lpApiRet_Ok == status) {

		reg = mantissa & 0xFF;
		/* Write to register MOD3 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD3_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Fetch register MOD2 */
		if(ES2lpLldRet_Ok != S2lpLld_ReadReg(MOD2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	if(ES2lpApiRet_Ok == status) {

		/* Set the DATARATE_E field */
		SET_BITS(reg, 0x0F, (exponent & 0x0F));
		/* Write back to register MOD2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD2_ADDR, &reg, 1)) {

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
	if(ES2lpLldRet_Ok != S2lpLld_ReadReg(MOD2_ADDR, &reg, 1)) {

		status = ES2lpApiRet_Error;

	}

	if(ES2lpApiRet_Ok == status) {

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

	if(ES2lpApiRet_Ok == status) {

		/* Write back to register MOD2 */
		if(ES2lpLldRet_Ok != S2lpLld_WriteReg(MOD2_ADDR, &reg, 1)) {

			status = ES2lpApiRet_Error;

		}

	}

	return status;

}



