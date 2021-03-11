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
 * @brief S2-LP GPIO signals enumeration
 */
typedef enum ES2lpApiGpioSignal {
	ES2lpApiGpioSignal_OutputIrq = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_NIRQ),
	ES2lpApiGpioSignal_OutputPorInverted = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_POR_INVERTED),
	ES2lpApiGpioSignal_OutputWakeUpTimerExpiration = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_WUT_EXPIRATION),
	ES2lpApiGpioSignal_OutputLowBatteryDetection = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_LOW_BATTERY),
	ES2lpApiGpioSignal_OutputTxDataInternalClockOutput = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_DATA_INTERNAL_CLOCK),
	ES2lpApiGpioSignal_OutputCommandInfoFromRadioTxBlock = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_STATE_RADIO_TX),
	ES2lpApiGpioSignal_OutputFifoAlmostEmptyFlag = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_RX_FIFO_ALMOST_EMPTY),
	ES2lpApiGpioSignal_OutputFifoAlmostFullFlag = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_RX_FIFO_ALMOST_FULL),
	ES2lpApiGpioSignal_OutputRxDataOutput = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_RX_DATA_OUTPUT),
	ES2lpApiGpioSignal_OutputRxClockOutput = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_RX_CLOCK_OUTPUT),
	ES2lpApiGpioSignal_OutputRxStateIndication = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_RX_STATE_INDICATION),
	ES2lpApiGpioSignal_OutputDeviceNotInSleepOrStandbyState = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_NOT_SLEEP_OR_STANDBY),
	ES2lpApiGpioSignal_OutputDeviceInStandbyState = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_DEVICE_IN_STANDBY_STATE),
	ES2lpApiGpioSignal_OutputAntennaSwitchSignal = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_ANTENNA_SWITCH_SIGNAL),
	ES2lpApiGpioSignal_OutputValidPreambleDetectedFlag = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_VALID_PREAMBLE_DETECTED),
	ES2lpApiGpioSignal_OutputSyncWordDetectedFlag = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_SYNC_WORD_DETECTED),
	ES2lpApiGpioSignal_OutputRssiAboveThreshold = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_RSSI_ABOVE_THRESHOLD),
	ES2lpApiGpioSignal_OutputTxRxModeIndicator = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_RX_MODE_INDICATOR),
	ES2lpApiGpioSignal_OutputVdd = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_VDD),
	ES2lpApiGpioSignal_OutputGnd = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_GND),
	ES2lpApiGpioSignal_OutputExternalSmpsEnableSignal = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_SMPS_ENABLE_SIGNAL),
	ES2lpApiGpioSignal_OutputDeviceInSleepState = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_DEVICE_IN_SLEEP_STATE),
	ES2lpApiGpioSignal_OutputDeviceInReadyState = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_DEVICE_IN_READY_STATE),
	ES2lpApiGpioSignal_OutputDeviceInLockState = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_DEVICE_IN_LOCK_STATE),
	ES2lpApiGpioSignal_OutputDeviceWaitingForHighLockDetectorSignal = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_WAIT_FOR_LOCK_DETECTOR),
	ES2lpApiGpioSignal_OutputTxDataOokSignal = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_TX_DATA_OOK_SIGNAL),
	ES2lpApiGpioSignal_OutputDeviceWaitingForHighReady2SignalFromXo = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_WAIT_FOR_XO_READY_SIGNAL),
	ES2lpApiGpioSignal_OutputDeviceWaitingForTimerExpirationToAllowPmBlockSettling = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_WAIT_FOR_TIMER_EXP),
	ES2lpApiGpioSignal_OutputDeviceWaitingForEndOfVcoCalibration = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_WAIT_FOR_VCO_CALIB),
	ES2lpApiGpioSignal_OutputDeviceEnablesTheFullSynthBlockCircuitry = (S2LP_GPIO_MODE_OUTPUT_LOW_POWER | S2LP_GPIO_SEL_SYNTH_BLOCK_ENABLED),
	ES2lpApiGpioSignal_InputTxCommand = (S2LP_GPIO_MODE_INPUT | S2LP_GPIO_SEL_TX_COMMAND),
	ES2lpApiGpioSignal_InputRxCommand = (S2LP_GPIO_MODE_INPUT | S2LP_GPIO_SEL_RX_COMMAND),
	ES2lpApiGpioSignal_InputTxDataInputForDirectModulation = (S2LP_GPIO_MODE_INPUT | S2LP_GPIO_SEL_TX_DATA_INPUT),
	ES2lpApiGpioSignal_InputWakeUpFromExternalInput = (S2LP_GPIO_MODE_INPUT | S2LP_GPIO_SEL_WAKE_UP_FROM_EXT_INPUT),
	ES2lpApiGpioSignal_InputExternalClockForLdcModesTiming = (S2LP_GPIO_MODE_INPUT | S2LP_GPIO_SEL_EXT_CLOCK_AT_34_7_Hz)
} ES2lpApiGpioSignal;

/**
 * @brief Modulation type enumeration
 */
typedef enum ES2lpApiModulationType {
	ES2lpApiModulationType_2_FSK = S2LP_MOD_2_FSK,
	ES2lpApiModulationType_4_FSK = S2LP_MOD_4_FSK,
	ES2lpApiModulationType_2_GFSK = S2LP_MOD_2_GFSK,
	ES2lpApiModulationType_4_GFSK = S2LP_MOD_4_GFSK,
	ES2lpApiModulationType_ASK_OOK = S2LP_MOD_ASK_OOK,
	ES2lpApiModulationType_Unmodulated = S2LP_MOD_CW
} ES2lpApiModulationType;

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Initialize the device
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_Start(void);

/**
 * @brief Send a command to the S2-LP device
 * @param command Command to be transmitted
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SendCommand(TByte command);

/**
 * @brief Enable IRQ generation on a given event
 * @param events Interrupt event(s) bit(s) (bitwise ORed if multiple)
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_EnableInterrupt(TWord events);

/**
 * @brief Disable IRQ generation on a given event
 * @param events Interrupt event(s) bit(s) (bitwise ORed if multiple)
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_DisableInterrupt(TWord events);

/**
 * @brief Test if a given interrupt event has occurred and clear the IRQ_STATUS register
 * @param irqStatusPtr Pointer to pass the IRQ_STATUS register contents out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_FetchInterruptStatus(TWord* irqStatusPtr);

/**
 * @brief Specify GPIO I/O signal
 * @param gpio GPIO to configure
 * @param sig GPIO I/O signal
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ConfigGpio(ES2lpApiGpioPin gpio, ES2lpApiGpioSignal sig);

/**
 * @brief Select modulation type
 * @param type Modulation type
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SelectModulationType(ES2lpApiModulationType type);

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
 * @brief Set synchronization word
 * @param sync Synchronization word
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetSyncWord(TWord sync);

/**
 * @brief Set data rate
 * @param datarate_m DATARATE_M (mantissa) in the data rate equation
 * @param datarate_e DATARATE_E (exponent) in the data rate equation
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetDataRate(THalfWord datarate_m, TByte datarate_e);

/**
 * @brief Set frequency deviation
 * @param fdev_m FDEV_M (mantissa) in the frequency deviation equation
 * @param fdev_e FDEV_E (exponent) in the frequency deviation equation
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetFrequencyDeviation(TByte fdev_m, TByte fdev_e);

/**
 * @brief Set channel filter bandwidth
 * @param chflt_m CHFLT_M (mantissa) in the channel filter equation/table
 * @param chflt_e CHLFT_E (exponent) in the channel filter equation/table
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_SetChannelFilterBandwidth(TByte chflt_m, TByte chflt_e);

/**
 * @brief Read the RSSI level captured at the end of the SYNC word detection of the received packet
 * @param rssiPtr Buffer to pass the RSSI value out of the function
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRssi(TByte* rssiPtr);

/**
 * @brief Read RX payload
 * @param bufPtr Buffer to pass the payload out of the function
 * @param numOfBytes Number of bytes to read
 * @retval ES2lpApiRet Status
 */
ES2lpApiRet S2lpApi_ReadRxPayload(TByte* bufPtr, TSize numOfBytes);


/*---------------------------------------------- Macros ----------------------------------------------*/

/**
 * @brief Send the S2-LP to TX state for transmission
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_GoToTxState()        S2lpApi_SendCommand(S2LP_TX_COMMAND)

/**
 * @brief Send the S2-LP to RX state for reception
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_GoToRxState()        S2lpApi_SendCommand(S2LP_RX_COMMAND)

/**
 * @brief Go to READY state
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_GoToReadyState()     S2lpApi_SendCommand(S2LP_READY_COMMAND)

/**
 * @brief Go to STANDBY state
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_GoToStandbyState()   S2lpApi_SendCommand(S2LP_STANDBY_COMMAND)

/**
 * @brief Go to SLEEP state
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_GoToSleepState()     S2lpApi_SendCommand(S2LP_SLEEP_COMMAND)

/**
 * @brief Exit from TX or RX states and go to READY state
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_AbortTransmission()  S2lpApi_SendCommand(S2LP_SABORT_COMMAND)

/**
 * @brief Reset the S2-LP state machine and registers values
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_Reset()              S2lpApi_SendCommand(S2LP_SRES_COMMAND)

/**
 * @brief Clean the RX FIFO
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_FlushRxFifo()        S2lpApi_SendCommand(S2LP_FLUSHRXFIFO_COMMAND)

/**
 * @brief Clean the TX FIFO
 * @retval ES2lpApiRet Status
 */
#define S2lpApi_FlushTxFifo()        S2lpApi_SendCommand(S2LP_FLUSHRXFIFO_COMMAND)


#endif /* __S2LP_API_H_ */
