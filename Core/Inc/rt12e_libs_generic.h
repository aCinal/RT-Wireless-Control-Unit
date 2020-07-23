/**
 * @author Adrian Cinal
 * @file rt12e_libs_generic.h
 * @brief Header file providing macros facilitating data manipulation
 */

#ifndef __RT12E_LIBS_GENERIC_H_
#define __RT12E_LIBS_GENERIC_H_

/**
 * @brief Converts two separate 8-bit values to a single 16-bit value
 */
#define READ16(highbyte, lowbyte) (uint16_t)(((uint8_t)highbyte << 8U) | (uint8_t)lowbyte)

/**
 * @brief Converts four separate 8-bit values to a single 32-bit value
 */
#define READ32(highbyte, midhighbyte, midlowbyte, lowbyte) (uint32_t)(((uint8_t)highbyte << 24U) | ((uint8_t)midhighbyte << 16U) | ((uint8_t)midlowbyte << 8U) | (uint8_t)lowbyte)

/**
 * @brief Retrieves the least significant byte of a 16-bit value
 */
#define LSB16(value) (uint8_t)((uint16_t)value & 0xFF)

/**
 * @brief Retrieves the most significant byte of a 16-bit value
 */
#define MSB16(value) (uint8_t)(((uint16_t)value >> 8U) & 0xFF)

/**
 * @brief Retrieves two least significant bytes (16 least significant bits) of a value
 */
#define TWOLOWBYTES(value) (uint16_t)(value & 0xFFFF)

/**
 * @brief Retrieves the least significant byte of a 32-bit value
 */
#define LSB32(value) (uint32_t)((uint32_t)value & 0xFF)

/**
 * @brief Retrieves the low mid (second least significant) byte of a 32-bit value
 */
#define LOWMID32(value) (uint32_t)(((uint32_t)value >> 8U) & 0xFF)

/**
 * @brief Retrieves the high mid (second most significant) byte of a 32-bit value
 */
#define HIGHMID32(value) (uint32_t)(((uint32_t)value >> 16U) & 0xFF)

/**
 * @brief Retrieves the most significant byte of a 32-bit value
 */
#define MSB32(value) (uint32_t)(((uint32_t)value >> 24U) & 0xFF)


#endif /* __RT12E_LIBS_GENERIC_H_ */
