/**
 * @author Adrian Cinal
 * @file rt12e_libs_r3tp.h
 * @brief Header file containing Racing Team Telemetry Transport Protocol (R3TP) defines and macros
 */

#ifndef __RT12E_LIBS_R3TP_H_
#define __RT12E_LIBS_R3TP_H_

/* VER 0 defines - telemetry data */
#define R3TP_VER0_VER_BYTE        ( (uint8_t) 0x00 )          /* VER 0 protocol version byte */
#define R3TP_VER0_FRAME_SIZE      ( (uint32_t) 20 )           /* R3TP version 0 frame size in bytes */

/* VER 1 defines - new telemetry subscription */
#define R3TP_VER1_VER_BYTE        ( (uint8_t) 0x01 )          /* VER 1 protocol version byte */
#define R3TP_VER1_HEADER_SIZE     ( (uint32_t) 8U )           /* R3TP header size */
#define R3TP_VER1_MAX_FRAME_NUM   ( (uint32_t) 28 )           /* Maximum number of frames in a subscription */
/* R3TP version 1 max frame size in bytes */
#define R3TP_VER1_MAX_FRAME_SIZE  ( (uint32_t) (12U + 4U * R3TP_VER1_MAX_FRAME_NUM) )

/* VER 2 defines - driver warning */
#define R3TP_VER2_VER_BYTE        ( (uint8_t) 0x02 )          /* VER 2 protocol version byte */
#define R3TP_VER2_FRAME_SIZE      ( (uint32_t) 8 )            /* R3TP version 2 frame size in bytes */
#define R3TP_GREEN_WARNING_BYTE   ( (uint8_t) 0x00 )          /* R3TP VER 2 green warning byte */
#define R3TP_RED_WARNING_BYTE     ( (uint8_t) 0x01 )          /* R3TP VER 2 red warning byte */

/* VER 3 defines - acknowledge message */
#define R3TP_VER3_VER_BYTE        ( (uint8_t) 0x03 )          /* VER 3 protocol version byte */
#define R3TP_VER3_FRAME_SIZE      ( (uint32_t) 8 )            /* R3TP version 3 frame size in bytes */

/* Common defines */
#define R3TP_END_SEQ_LOW_BYTE     ( (uint8_t) 0xDE )          /* R3TP end sequence low byte */
#define R3TP_END_SEQ_HIGH_BYTE    ( (uint8_t) 0xED )          /* R3TP end sequence high byte */
#define R3TP_MAX_FRAME_SIZE       (R3TP_VER1_MAX_FRAME_SIZE)  /* R3TP max frame size (of all versions) in bytes */

/* Exported macros */

/** @brief Return the protocol version of the message pointed to by base */
#define R3TP_PROTOCOL_VERSION(base)      ( (uint8_t)(base)[0] )

/** @brief Return the address at which the payload of an R3TP version 1 frame begins in reference to the base */
#define R3TP_VER1_PAYLOAD(base)          ( (uint8_t*)&( ( (uint8_t*)(base) )[R3TP_VER1_HEADER_SIZE] ) )

/** @brief Calculate the total length of an R3TP version 1 frame based on the number of subscription frames */
#define R3TP_VER1_MESSAGE_LENGTH(frNum)  ( ( (uint32_t)(frNum) ) * 4U + 12U )

/** @brief Read CRC checksum from the message buffer */
#define R3TP_READ_CRC(base)              ( (uint16_t) ( ( (base)[2] ) | ( ( (base)[3] << 8 ) ) ) )

/** @brief Validate R3TP end sequence */
#define R3TP_VALID_END_SEQ(base, length) ( (R3TP_END_SEQ_LOW_BYTE == (base)[length - 2U]) \
                                		    && (R3TP_END_SEQ_HIGH_BYTE == (base)[length - 1U]) )

#endif /* __RT12E_LIBS_R3TP_H_ */
