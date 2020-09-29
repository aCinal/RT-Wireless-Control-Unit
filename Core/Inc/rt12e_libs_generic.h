/**
 * @author Adrian Cinal
 * @file rt12e_libs_generic.h
 * @brief Header file providing macros facilitating data manipulation
 */

#ifndef __RT12E_LIBS_GENERIC_H_
#define __RT12E_LIBS_GENERIC_H_

/**
 * @brief Reinterpret two 8-bit values as a single 16-bit value
 */
#define _reinterpret16bits(high, low)  (((0xFF & (high)) << 8) | (0xFF & (low)))

/**
 * @brief Reinterpret four 8-bit values as a single 32-bit value
 */
#define _reinterpret32bits(high, midh, midl, low)  (((0xFF & (high)) << 24) | ((0xFF & (midh)) << 16) | ((0xFF & (midl)) << 8) | (0xFF & (low)))

/**
 * @brief Access bits 0-7 (least significant byte) of a value
 */
#define _bits0_7(x)  (0xFF & (x))

/**
 * @brief Access bits 0-15 (16 least significant bits) of a value
 */
#define _bits0_15(x)  (0xFFFF & (x))

/**
 * @brief Access bits 8-15 (second least significant byte) of a value
 */
#define _bits8_15(x)  (0xFF & ((x) >> 8))

/**
 * @brief Access bits 8-15 (third least significant byte) of a value
 */
#define _bits16_23(x)  (0xFF & ((x) >> 16))

/**
 * @brief Access bits 8-15 (fourth least significant byte) of a value
 */
#define _bits24_31(x)  (0xFF & ((x) >> 24))

#endif /* __RT12E_LIBS_GENERIC_H_ */
