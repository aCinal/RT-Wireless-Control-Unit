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
 * @brief Charge pump current enumeration
 */
typedef enum ES2lpApiIcp {
	ES2lpApiIcp_120uA = 0,
	ES2lpApiIcp_200uA,
	ES2lpApiIcp_140uA,
	ES2lpApiIcp_240uA
} ES2lpApiIcp;

/**
 * @brief Base frequency configuration structure
 * @note f_base = (f_xo * SYNT) / 2^(BANDSELECT + REFDIV + 21)
 */
typedef struct SS2lpApiBaseFrequencyConfig {
	BIT_FIELD(SYNT, 28);
	BIT_FIELD(BANDSELECT, 1);
	BIT_FIELD(REFDIV, 1);
} SS2lpApiBaseFrequencyConfig;

/**
 * @brief Modulation type enumeration
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

/**
 * @brief Carrier sense mode enumeration
 */
typedef enum ES2lpApiCarrierSenseMode {
	ES2lpApiCarrierSenseMode_Static = 0,
	ES2lpApiCarrierSenseMode_Dynamic_6db,
	ES2lpApiCarrierSenseMode_Dynamic_12db,
	ES2lpApiCarrierSenseMode_Dynamic_18db
} ES2lpApiCarrierSenseMode;

/**
 * @brief Packet format enumeration mode
 */
typedef enum ES2lpApiPacketFormat {
	ES2lpApiPacketFormat_Basic = 0,
	ES2lpApiPacketFormat_802_15_4g,
	ES2lpApiPacketFormat_UART_OTA,
	ES2lpApiPacketFormat_Stack
} ES2lpApiPacketFormat;

/**
 * @brief RX mode enumeration
 */
typedef enum ES2lpApiRxMode {
	ES2lpApiRxMode_NormalMode = 0,
	ES2lpApiRxMode_DirectThroughFifo,
	ES2lpApiRxMode_DirectThroughGpio
} ES2lpApiRxMode;

/**
 * @brief CRC mode enumeration
 */
typedef enum ES2lpApiCrcMode {
	ES2lpApiCrcMode_NoCRCField = 0,
	ES2lpApiCrcMode_Poly0x07,
	ES2lpApiCrcMode_Poly0x8005,
	ES2lpApiCrcMode_Poly0x1021,
	ES2lpApiCrcMode_Poly0x864CBF,
	ES2lpApiCrcMode_Poly0x04C011BB7
} ES2lpApiCrcMode;

/**
 * @brief Packet control configuration structure
 */
typedef struct SS2lpApiPacketControlConfig {
	BIT_FIELD(SYNC_LEN, 6);          /* The number of bits used for the SYNC field in the packet */
	BIT_FIELD(PREAMBLE_LEN, 10);     /* Number of '01' or '10' of the preamble of the packet */
	BIT_FIELD(LEN_WID, 1);           /* The number of bytes used for the length field: 0: 1 byte, 1: 2 bytes */
	BIT_FIELD(ADDRESS_LEN, 1);       /* 1: include the ADDRESS field in the packet */
	ES2lpApiPacketFormat PCKT_FRMT;  /* Packet format */
	ES2lpApiRxMode RX_MODE;          /* RX mode */
	BIT_FIELD(BYTE_SWAP, 1);         /* Send theFIFO bytes in 0: MSBit first, 1: LSBit first*/
	BIT_FIELD(PREAMBLE_SEL, 2);      /* Select the preamble pattern (Table 52. in S2-LP datasheet)  */
	BIT_FIELD(MANCHESTER_EN, 1);     /* Enable Manchester encoding */
	BIT_FIELD(FIX_VAR_LEN, 1);       /* Packet length mode 0: fixed, 1: variable */
	ES2lpApiCrcMode CRC_MODE;
	THalfWord PCKTLEN;
	TWord SYNC;
	TByte PCKT_PSTMBL;
} SS2lpApiPacketControlConfig;

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Send the S2-LP to TX state for transmission
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToTxState(void);

/**
 * @brief Send the S2-LP to RX state for reception
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToRxState(void);

/**
 * @brief Go to READY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToReadyState(void);

/**
 * @brief Go to STANDBY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToStandbyState(void);

/**
 * @brief Go to SLEEP state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_GoToSleepState(void);

/**
 * @brief Exit from TX or RX states and go to READY state
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_AbortTransmission(void);

/**
 * @brief Reset the S2-LP state machine and registers values
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_Reset(void);

/**
 * @brief Clean the RX FIFO
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FlushRxFifo(void);

/**
 * @brief Clean the TX FIFO
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FlushTxFifo(void);

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
ES2lpApiRet S2lpApi_SetBaseFrequency(SS2lpApiBaseFrequencyConfig config);

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

/**
 * @brief Set the bandwidth of the receiver channel filter
 * @param mantissa 4-bit mantissa value of the receiver channel filter
 * @param exponent 4-bit exponent value of the receiver channel filter
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetRxChannelFilterBandwidth(TByte mantissa, TByte exponent);

/**
 * @brief Set the gain of the RSSI filter
 * @param gain 4-bit gain value
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetRssiFilterGain(TByte gain);

/**
 * @brief Select carrier sense mode
 * @param mode Carrier sense mode
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SelectCarrierSenseMode(ES2lpApiCarrierSenseMode mode);

/**
 * @brief Set signal detect threshold
 * @param rssiThreshold Value to write to register RSSI_TH
 * @retval ES2lpApiRet Status
 * @note Threshold in dBm is (RSSI_TH - 146)
 */
ES2lpApiRet S2lpApi_SetSignalDetectThreshold(TByte rssiThreshold);

/**
 * @brief Configure packet control
 * @param config Configuration structure
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigPacketControl(SS2lpApiPacketControlConfig config);


#endif /* __S2LP_API_H_ */
