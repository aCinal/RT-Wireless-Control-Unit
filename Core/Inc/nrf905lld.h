/**
 * @file nrf905lld.h
 * @author Adrian Cinal
 * @brief Low-level drivers interface for the nRF905 device
 */

#ifndef __NRF905LLD_H_
#define __NRF905LLD_H_

#include <inttypes.h>
#include <stddef.h>
#include <stdbool.h>

typedef uint8_t TByte;
typedef uint32_t TWord;
typedef uint32_t TSize;

/**
 * @brief nRF905 low-level drivers return value enumeration
 */
typedef enum ENrf905LldRet {
	ENrf905LldRet_Ok = 0,
	ENrf905LldRet_InvalidParams,
	ENrf905LldRet_Error
} ENrf905LldRet;

/**
 * @brief nRf905 frequency band enumeration
 */
typedef enum ENrf905LldBand {
	ENrf905LldBand_433MHz = 0,
	ENrf905LldBand_868_915MHz
} ENrf905LldBand;

/**
 * @brief nRF905 output power enumeration
 */
typedef enum ENrf905LldOutputPower {
	ENrf905LldOutputPower_n10dBm = 0,
	ENrf905LldOutputPower_n2dBm,
	ENrf905LldOutputPower_p6dBm,
	ENrf905LldOutputPower_p10dBm
} ENrf905LldOutputPower;

/**
 * @brief nRF905 output clock frequency enumeration
 */
typedef enum ENrf905LldOutputClkFreq {
	ENrf905LldOutputClkFreq_4MHz = 0,
	ENrf905LldOutputClkFreq_2MHz,
	ENrf905LldOutputClkFreq_1MHz,
	ENrf905LldOutputClkFreq_500kHz
} ENrf905LldOutputClkFreq;

/**
 * @brief nRF905 crystal oscillator frequency enumeration
 */
typedef enum ENrf905LldXof {
	ENrf905LldXof_4MHz = 0,
	ENrf905LldXof_8MHz,
	ENrf905LldXof_12MHz,
	ENrf905LldXof_16MHz,
	ENrf905LldXof_20MHz
} ENrf905LldXof;

/**
 * @brief nRF905 CRC check mode enumeration
 */
typedef enum ENrf905LldCrcMode {
	ENrf905LldCrcMode_Crc8 = 0,
	ENrf905LldCrcMode_Crc16
} ENrf905LldCrcMode;

/**
 * @brief Sets center frequence together with SetPllMode
 * @param chNo CH_NO parameter value
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetChannel(TWord chNo);

/**
 * @brief Sets frequency band
 * @param band Frequency band
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetBand(ENrf905LldBand band);

/**
 * @brief Sets the output power
 * @param outputPower Output power
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetOutputPower(ENrf905LldOutputPower outputPower);

/**
 * @brief Reduces current in RX mode by 1.6mA. Sensitivity is reduced
 * @param enable True to reduce power, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableReducedCurrentRx(bool enable);

/**
 * @brief Retransmit contents in TX register if TRX_CE and TX_EN are high
 * @param enable True to enable retransmission of data packet, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableAutoRetran(bool enable);

/**
 * @brief Sets RX address width
 * @param addressWidth RX address width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetRxAddressWidth(TSize addressWidth);

/**
 * @brief Sets TX address width
 * @param addressWidth TX address width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetTxAddressWidth(TSize addressWidth);

/**
 * @brief Sets RX payload width
 * @param payloadWidth RX payload width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetRxPayloadWidth(TSize payloadWidth);

/**
 * @brief Sets TX payload width
 * @param payloadWidth TX payload width
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetTxPayloadWidth(TSize payloadWidth);

/**
 * @brief Sets the device's RX address
 * @param addPtr RX address
 * @param size Address size
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetDeviceIdentity(TByte* addPtr, TSize size);

/**
 * @brief Sets output clock frequency
 * @param freq Output clock frequency
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetOutputClockFrequency(ENrf905LldOutputClkFreq freq);

/**
 * @brief Enables external clock signal
 * @param enable True to enable external clock signal, false if no external clock signal available
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableOutputClock(bool enable);

/**
 * @brief Sets crystal oscillator frequency
 * @param xof Crystal oscillator frequency
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetXof(ENrf905LldXof xof);

/**
 * @brief Enables CRC check
 * @param enable True to enable CRC check enable, false otherwise
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_EnableCrcCheck(bool enable);

/**
 * @brief Sets CRC mode
 * @param crcMode CRC mode
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_SetCrcMode(ENrf905LldCrcMode crcMode);

/**
 * @brief Powers up the device and puts it in standby mode
 * @retval None
 */
void Nrf905Lld_PwrUp(void);

/**
 * @brief Powers down the device
 * @retval None
 */
void Nrf905Lld_PwrDown(void);

/**
 * @brief Transmit/Receive enable
 * @retval None
 */
void Nrf905Lld_TransmissionEnable(void);

/**
 * @brief Transmit/Receive disable
 * @retval None
 */
void Nrf905Lld_TransmissionDisable(void);

/**
 * @brief Puts the device in ShockBurst(tm) RX mode
 * @retval None
 */
void Nrf905Lld_ModeSelectRx(void);

/**
 * @brief Puts the device in ShockBurst(tm) TX mode
 * @retval None
 */
void Nrf905Lld_ModeSelectTx(void);

/**
 * @brief Writes TX payload to the device
 * @param payloadPtr Payload
 * @param size Size of the payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_WriteTxPayload(TByte* payloadPtr, TSize size);

/**
 * @brief Reads TX payload from the device
 * @param[out] bufPtr Buffer for the payload
 * @param[in] size Size of the expected payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadTxPayload(TByte* bufPtr, TSize size);

/**
 * @brief Writes TX address to the device
 * @param addPtr Address
 * @param size Size of the address
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_WriteTxAddress(TByte* addPtr, TSize size);

/**
 * @brief Reads TX address from the device
 * @param[out] bufPtr Buffer for the address
 * @param[in] size Size of the expected address
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadTxAddress(TByte* bufPtr, TSize size);

/**
 * @brief Reads RX payload from the device
 * @param[out] bufPtr Buffer for the payload
 * @param[in] size Size of the expected payload
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_ReadRxPayload(TByte* bufPtr, TSize size);

/**
 * @brief Dumps the configuration register contents into a buffer
 * @param bufPtr Buffer
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_DumpConfigReg(TByte* bufPtr);

/**
 * @brief Restores the default values of the configuration register
 * @retval ENrf905LldRet Status
 */
ENrf905LldRet Nrf905Lld_RestoreDefaultConfig(void);

#endif /* __NRF905LLD_H_ */
