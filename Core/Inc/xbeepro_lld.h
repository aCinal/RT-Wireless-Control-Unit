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
 * @brief Enter command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_EnterCommandMode(void);

/**
 * @brief Transmit a command to the XBee-PRO device in command mode
 * @param command Command string
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_SendCommand(const char* command);

/**
 * @brief Exit command mode
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_ExitCommandMode(void);

/**
 * @brief Receive XBee-PRO register contents
 * @param bufPtr Buffer to pass the register contents out of the function
 * @param numOfBytes Number of bytes to be received
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_Receive(uint8_t* bufPtr, size_t numOfBytes);

/**
 * @brief Transmit data to the XBee-PRO device
 * @param bufPtr Payload buffer
 * @param numOfBytes Number of bytes to be transmitted
 * @retval EXbeeProLldRet Status
 */
EXbeeProLldRet XbeeProLld_Transmit(uint8_t* bufPtr, size_t numOfBytes);

#endif /* __XBEEPRO_LLD_H */

