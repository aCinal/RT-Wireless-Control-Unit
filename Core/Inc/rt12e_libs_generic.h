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
#define READAS16BIT(highbyte, lowbyte) (uint16_t)(((uint8_t)highbyte << 8) | (uint8_t)lowbyte)

/**
 * @brief Converts four separate 8-bit values to a single 32-bit value
 */
#define READAS32BIT(highbyte, midhighbyte, midlowbyte, lowbyte) (uint32_t)(((uint8_t)highbyte << 24) | ((uint8_t)midhighbyte << 16) | ((uint8_t)midlowbyte << 8) | (uint8_t)lowbyte)

/**
 * @brief Retrieves the least significant byte of a 16-bit value
 */
#define GETLSBOF16(value) (uint8_t)((uint16_t)value & 0xFF)

/**
 * @brief Retrieves the most significant byte of a 16-bit value
 */
#define GETMSBOF16(value) (uint8_t)(((uint16_t)value >> 8) & 0xFF)

/**
 * @brief Retrieves two least significant bytes (16 least significant bits) of a value
 */
#define GET16LSBITS(value) (uint16_t)(value & 0xFFFF)

#endif /* __RT12E_LIBS_GENERIC_H_ */
