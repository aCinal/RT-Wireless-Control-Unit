/**
 * @author Adrian Cinal
 * @file rt12e_libs_r3tp.h
 * @brief Header file containing Racing Team Telemetry Transport Protocol (R3TP) defines and macros
 */

#ifndef __RT12E_LIBS_R3TP_H_
#define __RT12E_LIBS_R3TP_H_

/* VER 0 defines - telemetry data */
#define R3TP_VER0_VER_BYTE			(uint8_t)(0x00U)			/* VER 0 protocol version byte */
#define R3TP_VER0_FRAME_SIZE		(uint32_t)(20UL)			/* R3TP version 0 frame size in bytes */

/* VER 1 defines - new telemetry subscription */
#define R3TP_VER1_VER_BYTE			(uint8_t)(0x01U)			/* VER 1 protocol version byte */
#define R3TP_VER1_HEADER_SIZE		(uint32_t)(8UL)				/* R3TP header size */
#define R3TP_VER1_MAX_FRAME_NUM		(uint32_t)(28UL)			/* Maximum number of frames in a subscription */
/* R3TP version 1 max frame size in bytes */
#define R3TP_VER1_MAX_FRAME_SIZE	(uint32_t)(12UL + 4UL * R3TP_VER1_MAX_FRAME_NUM)

/* VER 2 defines - driver warning */
#define R3TP_VER2_VER_BYTE			(uint8_t)(0x02U)			/* VER 2 protocol version byte */
#define R3TP_VER2_FRAME_SIZE		(uint32_t)(8UL)				/* R3TP version 2 frame size in bytes */
#define R3TP_GREEN_WARNING_BYTE		(uint8_t)(0x00U)			/* R3TP VER 2 green warning byte */
#define R3TP_RED_WARNING_BYTE		(uint8_t)(0x01U)			/* R3TP VER 2 red warning byte */

/* Common defines */
#define R3TP_END_SEQ_LOW_BYTE		(uint8_t)(0xDEU)			/* R3TP end sequence low byte */
#define R3TP_END_SEQ_HIGH_BYTE		(uint8_t)(0xEDU)			/* R3TP end sequence high byte */
#define R3TP_MAX_FRAME_SIZE			R3TP_VER1_MAX_FRAME_SIZE	/* R3TP max frame size (of all versions) in bytes */

/* Exported macros */
/**
 * @brief Returns the address at which the payload of an R3TP version 1 frame begins in reference to the start
 */
#define R3TP_VER1_PAYLOAD(start) ((uint8_t*)&(((uint8_t*)start)[R3TP_VER1_HEADER_SIZE]))

/**
 * @brief Calculates the total length of an R3TP version 1 frame based on the number of subscription frames
 */
#define R3TP_VER1_MESSAGE_LENGTH(frNum) ((uint32_t)((uint32_t)frNum * 4UL + 12UL))

#endif /* __RT12E_LIBS_R3TP_H_ */
