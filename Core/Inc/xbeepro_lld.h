/**
 * @file xbeepro_lld.h
 * @author Adrian Cinal
 * @brief XBee-PRO LLD header file
 */

#ifndef __XBEEPRO_LLD_H
#define __XBEEPRO_LLD_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief LLD functions return value enumeration
 */
typedef enum EXbeeProLldRet {
	EXbeeProLldRet_Ok = 0,
	EXbeeProLldRet_Error
} EXbeeProLldRet;

/**
 * @brief Transmit data to the XBee-PRO device
 * @param bufPtr Payload buffer
 * @param numOfBytes Number of bytes to be transmitted
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldTransmit(uint8_t* bufPtr, size_t numOfBytes);

/**
 * @brief Receive XBee-PRO register contents
 * @param respMsgPtr Buffer to pass the received response message out of the function
 * @param numOfBytes Number of bytes to be received
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLldReceive(char *respMsgPtr, size_t numOfBytes);

#endif /* __XBEEPRO_LLD_H */

