/**
 * @file s2lp_api.h
 * @author Adrian Cinal
 * @brief S2-LP API header file
 */

#ifndef __S2LP_API_H_
#define __S2LP_API_H_

/*---------------------------------------------- Includes ----------------------------------------------*/

#include "s2lp_lld.h"
#include <stdbool.h>

/*---------------------------------------------- Defines ----------------------------------------------*/

#define BIT_FIELD(name, size) unsigned int name : size  /* Declare bit field */

/*---------------------------------------------- Typedefs ----------------------------------------------*/

/**
 * @brief API functions return value enumeration
 */
typedef enum ES2lpApiRet {
	ES2lpApiRet_Ok = 0,
	ES2lpApiRet_InvalidParams,
	ES2lpApiRet_Error
} ES2lpApiRet;

/**
 * @brief S2-LP GPIOs enumeration
 */
typedef enum ES2lpApiGpioPin {
	ES2lpApiGpioPin_0 = 0,
	ES2lpApiGpioPin_1,
	ES2lpApiGpioPin_2,
	ES2lpApiGpioPin_3
} ES2lpApiGpioPin;

/**
 * @brief S2-LP GPIO modes enumeration
 */
typedef enum ES2lpApiGpioMode {
	ES2lpApiGpioMode_DigitalInput = 0,
	ES2lpApiGpioMode_DigitalOutputLowPower,
	ES2lpApiGpioMode_DigitalOutputHighPower
} ES2lpApiGpioMode;

/**
 * @brief S2-LP GPIO signals enumeration
 */
typedef enum ES2lpApiGpioSignal {
	ES2lpApiGpioSignal_o_Irq = 0,
	ES2lpApiGpioSignal_o_PorInverted,
	ES2lpApiGpioSignal_o_WakeUpTimerExpiration,
	ES2lpApiGpioSignal_o_LowBatteryDetection,
	ES2lpApiGpioSignal_o_TxDataInternalClockOutput,
	ES2lpApiGpioSignal_o_CommandInfoFromRadioTxBlock,
	ES2lpApiGpioSignal_o_FifoAlmostEmptyFlag,
	ES2lpApiGpioSignal_o_FifoAlmostFullFlag,
	ES2lpApiGpioSignal_o_RxDataOutput,
	ES2lpApiGpioSignal_o_RxClockOutput,
	ES2lpApiGpioSignal_o_RxStateIndication,
	ES2lpApiGpioSignal_o_DeviceNotInSleepOrStandbyState,
	ES2lpApiGpioSignal_o_DeviceInStandbyState,
	ES2lpApiGpioSignal_o_AntennaSwitchSignal,
	ES2lpApiGpioSignal_o_ValidPreambleDetectedFlag,
	ES2lpApiGpioSignal_o_SyncWordDetectedFlag,
	ES2lpApiGpioSignal_o_RssiAboveThreshold,
	ES2lpApiGpioSignal_o_TxRxModeIndicator,
	ES2lpApiGpioSignal_o_Vdd,
	ES2lpApiGpioSignal_o_Gnd,
	ES2lpApiGpioSignal_o_ExternalSmpsEnableSignal,
	ES2lpApiGpioSignal_o_DeviceInSleepState,
	ES2lpApiGpioSignal_o_DeviceInReadyState,
	ES2lpApiGpioSignal_o_DeviceInLockState,
	ES2lpApiGpioSignal_o_DeviceWaitingForHighLockDetectorSignal,
	ES2lpApiGpioSignal_o_TxDataOokSignal,
	ES2lpApiGpioSignal_o_DeviceWaitingForHighReady2SignalFromXo,
	ES2lpApiGpioSignal_o_DeviceWaitingForTimerExpirationToAllowPmBlockSettling,
	ES2lpApiGpioSignal_o_DeviceWaitingForEndOfVcoCalibration,
	ES2lpApiGpioSignal_o_DeviceEnablesTheFullSynthBlockCircuitry,
	ES2lpApiGpioSignal_i_TxCommand,
	ES2lpApiGpioSignal_i_RxCommand,
	ES2lpApiGpioSignal_i_TxDataInputForDirectModulation,
	ES2lpApiGpioSignal_i_WakeUpFromExternalInput,
	ES2lpApiGpioSignal_i_ExternalClockForLdcModesTiming
} ES2lpApiGpioSignal;

/**
 * @brief S2-LP charge pump current enumeration
 */
typedef enum ES2lpApiIcp {
	ES2lpApiIcp_120uA = 0,
	ES2lpApiIcp_200uA,
	ES2lpApiIcp_140uA,
	ES2lpApiIcp_240uA
} ES2lpApiIcp;

/**
 * @brief S2-LP base frequency settings structure
 * @note f_base = (f_xo * SYNT) / 2^(BANDSELECT + REFDIV + 21)
 */
typedef struct SS2lpApiBaseFrequencySettings {
	BIT_FIELD(SYNT, 28);
	BIT_FIELD(BANDSELECT, 1);
	BIT_FIELD(REFDIV, 1);
} SS2lpApiBaseFrequencySettings;

/**
 * @brief S2-LP modulation type enumeration
 */
typedef enum ES2lpApiModulationType {
	ES2lpApiModulationType_2_FSK = 0,
	ES2lpApiModulationType_4_FSK,
	ES2lpApiModulationType_2_GFSK_BT_1,
	ES2lpApiModulationType_4_GFSK_BT_1,
	ES2lpApiModulationType_ASK_OOK,
	ES2lpApiModulationType_PolarMode,
	ES2lpApiModulationType_Unmodulated,
	ES2lpApiModulationType_2_GFSK_BT_0_5,
	ES2lpApiModulationType_4_GFSK_BT_0_5
} ES2lpApiModulationType;

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Specify GPIO I/O signal
 * @param gpio GPIO to configure
 * @param mode GPIO mode
 * @param signal GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigureGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioMode mode, ES2lpApiGpioSignal signal);

/**
 * @brief Set charge pump current
 * @param icp Charge pump current
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetChargePumpCurrent(ES2lpApiIcp icp);

/**
 * @brief Set base frequency
 * @param config Configuration structure
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetBaseFrequency(SS2lpApiBaseFrequencySettings config);

/**
 * @brief Set intermediate frequency for analog RF synthesizer
 * @param ifOffsetAna Value to write to register IF_OFFSET_ANA
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetIntFreqForAnalogRFSynth(TByte ifOffsetAna);

/**
 * @brief Set intermediate frequency for digital shift-to-baseband circuits
 * @param ifOffsetAna Value to write to register IF_OFFSET_DIG
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetIntFreqForDigitalShiftToBbCircuits(TByte ifOffsetDig);

/**
 * @brief Select the RF channel
 * @param chspace Value to write to register CHSPACE
 * @param chnum Value to write to register CHNUM
 * @retval ES2lpApiRet Status
 * @note f = f_base + f_xo / (2^15 * CHSPACE) * CHNUM
 */
ES2lpApiRet S2lpApi_SetCenterFrequency(TByte chspace, TByte chnum);

/**
 * @brief Set the data rate by configuring the DATARATE_M and DATARATE_E registers
 * @param mantissa 16-bit mantissa value of the data rate equation (DATARATE_M)
 * @param exponent 4-bit exponent value of the data rate equation (DATARATE_E)
 * @retval ES2lpApiRet Status
 * @note Data rate formula:
 *           DataRate = f_dig * DATARATE_M/2^32 if DATARATE_E = 0
 *           DataRate = f_dig * (2^16 + DATARATE_M) * 2^DATARATE_E / 2^33 if DATARATE_E >0
 *           DataRate = f_dig / (8 * DATARATE_M)
 *       where f_dig is the digital clock frequency
 */
ES2lpApiRet S2lpApi_SetDataRate(TWord mantissa, TByte exponent);

/**
 * @brief Set modulation type
 * @param type Modulation type
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetModulationType(ES2lpApiModulationType type);


#endif /* __S2LP_API_H_ */
