/**
 * @file xbeepro_api.h
 * @author Adrian Cinal
 * @brief XBee-PRO API header file
 */

#ifndef __XBEEPRO_API_H
#define __XBEEPRO_API_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief API functions return value enumeration
 */
typedef enum EXbeeProApiRet {
	EXbeeProApiRet_Ok = 0,
	EXbeeProApiRet_InvalidParams,
	EXbeeProApiRet_Error
} EXbeeProApiRet;

/**
 * @brief Return the local value of the XBee-PRO Guard Times parameter
 * @retval uint16_t Local value of the guard time
 */
uint16_t XbeeProApiReadLocalGuardTime(void);

/**
 * @brief Set the Guard Times parameter of the XBee-PRO device
 * @param gt Guard Times parameter
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiSetGuardTimes(uint16_t gt);

/**
 * @brief Read the RSSI value of the last transmission received
 * @param rssiPtr Pointer to pass the received value out of the function
 * @retval EXbeeProApiRet Status
 */
EXbeeProApiRet XbeeProApiReadRssi(uint8_t* rssiPtr);

/**
 * @brief Transmit payload to be transmitted by XBee-PRO to the device
 * @param payloadPtr Payload buffer
 * @param numOfBytes Size of the payload buffer in bytes
 * @retval EXbeeProLldRet Status
 */
EXbeeProApiRet XbeeProApiSendPayload(uint8_t* payloadPtr, uint32_t numOfBytes);

#endif /* __XBEEPRO_API_H */
