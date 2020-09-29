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
 * @brief Interrupt event enumeration
 */
typedef enum ES2lpApiInterruptEvent {
	ES2lpApiInterruptEvent_RxDataReady = 0,
	ES2lpApiInterruptEvent_RxDataDiscarded,
	ES2lpApiInterruptEvent_TxDataSent,
	ES2lpApiInterruptEvent_MaxReTxReached,
	ES2lpApiInterruptEvent_CrcError,
	ES2lpApiInterruptEvent_TxFifoOverflowUnderflowError,
	ES2lpApiInterruptEvent_RxFifoOverflowUnderflowError,
	ES2lpApiInterruptEvent_TxFifoAlmostFull,
	ES2lpApiInterruptEvent_TxFifoAlmostEmpty,
	ES2lpApiInterruptEvent_RxFifoAlmostFull,
	ES2lpApiInterruptEvent_RxFifoAlmostEmpty,
	ES2lpApiInterruptEvent_MaxNoOfBackOffDuringCca,
	ES2lpApiInterruptEvent_ValidPreambleDetected,
	ES2lpApiInterruptEvent_SyncWordDetected,
	ES2lpApiInterruptEvent_RssiAboveThreshold,
	ES2lpApiInterruptEvent_WakeUpTimeoutInLdcrMode,
	ES2lpApiInterruptEvent_Ready,
	ES2lpApiInterruptEvent_StandbyStateSwitchingInProgress,
	ES2lpApiInterruptEvent_LowBatteryLevel,
	ES2lpApiInterruptEvent_PowerOnReset,
	ES2lpApiInterruptEvent_RxTimerTimeout,
	ES2lpApiInterruptEvent_SniffTimerTimeout
} ES2lpApiInterruptEvent;

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
 * @param sig GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioMode mode, ES2lpApiGpioSignal sig);

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
 * @note f_base = (f_xo * SYNT) / 2^(BANDSELECT + REFDIV + 21)
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
 * @brief Set signal detect threshold
 * @param rssiThreshold Value to write to register RSSI_TH
 * @retval ES2lpApiRet Status
 * @note Threshold in dBm is (RSSI_TH - 146)
 */
ES2lpApiRet S2lpApi_SetSignalDetectThreshold(TByte rssiThreshold);

/**
 * @brief Set a fixed packet length
 * @param length Packet length
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetPacketLength(THalfWord length);

/**
 * @brief Enable/disable Manchester encoding
 * @param enable True to enable Manchester encoding, false otherwise
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableManchester(bool enable);

/**
 * @brief Configure CRC calculation
 * @param mode CRC mode
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigCrc(ES2lpApiCrcMode mode);

/**
 * @brief Set synchronization word
 * @param sync Synchronization word
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetSyncWord(TWord sync);

/**
 * @brief Read the RSSI level captured at the end of the SYNC word detection of the received packet
 * @param rssiPtr Buffer to pass the RSSI value out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRssi(TByte* rssiPtr);

/**
 * @brief Enable/disable IRQ generation on a given event
 * @param event Interrupt event
 * @param enable True to enable IRQ generation, false otherwise
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableInterrupt(ES2lpApiInterruptEvent event, bool enable);

/**
 * @brief Test if a given interrupt event occured
 * @param event Interrupt event
 * @param booleanPtr Buffer to pass the Boolean value out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_TestInterrupt(ES2lpApiInterruptEvent event, bool* booleanPtr);

/**
 * @brief Read RX payload
 * @param bufPtr Buffer to pass the payload out of the function
 * @param numOfBytes Number of bytes to read
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRxPayload(TByte* bufPtr, TSize numOfBytes);


#endif /* __S2LP_API_H_ */
