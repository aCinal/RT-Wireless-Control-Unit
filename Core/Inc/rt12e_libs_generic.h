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
#define READ16(highbyte, lowbyte) (((0xFF & highbyte) << 8) | (0xFF & lowbyte))

/**
 * @brief Converts four separate 8-bit values to a single 32-bit value
 */
#define READ32(highbyte, midhighbyte, midlowbyte, lowbyte) (((0xFF & highbyte) << 24U) | ((0xFF & midhighbyte) << 16U) | ((0xFF & midlowbyte) << 8U) | (0xFF & lowbyte))

/**
 * @brief Retrieves the least significant byte of a value
 */
#define LSB(value) (0xFF & value)

/**
 * @brief Retrieves the most significant byte of a 16-bit value
 */
#define MSB16(value) (0xFF & ((0xFFFF & value) >> 8U))

/**
 * @brief Retrieves two least significant bytes (16 least significant bits) of a value
 */
#define TWOLOWBYTES(value) (0xFFFF & value)

/**
 * @brief Retrieves the low mid (second least significant) byte of a 32-bit value
 */
#define LOWMID32(value) (0xFF & ((0xFFFFFFFF & value) >> 8U))

/**
 * @brief Retrieves the high mid (second most significant) byte of a 32-bit value
 */
#define HIGHMID32(value) (0xFF & ((0xFFFFFFFF & value) >> 16U))

/**
 * @brief Retrieves the most significant byte of a 32-bit value
 */
#define MSB32(value) (0xFF & ((0xFFFFFFFF & value) >> 24U))


#endif /* __RT12E_LIBS_GENERIC_H_ */
