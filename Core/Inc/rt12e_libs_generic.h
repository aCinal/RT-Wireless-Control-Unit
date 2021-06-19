/**
 * @author Adrian Cinal
 * @file rt12e_libs_generic.h
 * @brief Header file providing macros facilitating data manipulation
 */

#ifndef __RT12E_LIBS_GENERIC_H_
#define __RT12E_LIBS_GENERIC_H_

/**
 * @brief Reinterpret two 8-bit values as a single big-endian 16-bit value
 */
#define _reinterpret16bitsbe(high, low)  ( ( ( 0xFF & (high) ) << 8 ) | ( 0xFF & (low) ) )

/**
 * @brief Reinterpret two 8-bit values as a single little-endian 16-bit value
 */
#define _reinterpret16bitsle(low, high)  ( ( ( 0xFF & (high) ) << 8 ) | ( 0xFF & (low) ) )

/**
 * @brief Reinterpret four 8-bit values as a single big-endian 32-bit value
 */
#define _reinterpret32bitsbe(high, midh, midl, low)  ( ( ( 0xFF & (high) ) << 24 ) | ( ( 0xFF & (midh) ) << 16 ) | ( ( 0xFF & (midl) ) << 8 ) | ( 0xFF & (low) ) )

/**
 * @brief Reinterpret four 8-bit values as a single little-endian 32-bit value
 */
#define _reinterpret32bitsle(low, midl, midh, high)  ( ( ( 0xFF & (high) ) << 24 ) | ( ( 0xFF & (midh) ) << 16 ) | ( ( 0xFF & (midl) ) << 8 ) | ( 0xFF & (low) ) )

/**
 * @brief Access a specific byte of a value
 */
#define _getbyte(value, byte) ( 0xFF & ( (value) >> (8 * byte) ) )

#endif /* __RT12E_LIBS_GENERIC_H_ */
