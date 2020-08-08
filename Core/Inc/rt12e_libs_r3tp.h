/**
 * @author Adrian Cinal
 * @file rt12e_libs_r3tp.h
 * @brief Header file containing Racing Team Telemetry Transport Protocol (R3TP) defines and macros
 */

#ifndef __RT12E_LIBS_R3TP_H_
#define __RT12E_LIBS_R3TP_H_

/* Exported defines ------------------------------------------------------------*/
#define R3TP_VER0_VER_BYTE			(0x00U)									/* VER 0 protocol version byte */
#define R3TP_VER1_VER_BYTE			(0x01U)									/* VER 1 protocol version byte */
#define R3TP_VER2_VER_BYTE			(0x02U)									/* VER 2 protocol version byte */

#define R3TP_VER0_FRAME_SIZE		(20U)									/* R3TP version 0 frame size in bytes */
#define R3TP_VER2_FRAME_SIZE		(8U)									/* R3TP version 2 frame size in bytes */

#define R3TP_VER1_MAX_FRAME_NUM		(28U)									/* Maximum number of frames in a subscription */
#define R3TP_VER1_MAX_FRAME_SIZE	(12U + 4U * R3TP_VER1_MAX_FRAME_NUM)	/* R3TP version 1 max frame size in bytes */

#define R3TP_END_SEQ_LOW_BYTE		(0xDEU)									/* R3TP end sequence low byte */
#define R3TP_END_SEQ_HIGH_BYTE		(0xEDU)									/* R3TP end sequence high byte */

#define R3TP_GREEN_WARNING			(0x00U)									/* R3TP VER 2 green warning */
#define R3TP_RED_WARNING			(0x01U)									/* R3TP VER 2 red warning */

/* Exported macros ------------------------------------------------------------*/
/**
 * @brief Returns the address at which the payload of an R3TP version 1 frame begins in reference to the start
 */
#define R3TP_VER1_PAYLOAD_BEGIN(start) (uint8_t*)((uint8_t*)start + 8UL)

/**
 * @brief Returns the address offset from the start of the payload of an R3TP version 1 frame
 */
#define R3TP_VER1_PAYLOAD_BEGIN_OFFSET(start, offset) (uint8_t*)((uint8_t*)start + 8UL + (uint32_t)offset)

/**
 * @brief Returns the address at which the epilogue (frame align and END SEQ) of an R3TP version 1 frame begins
 */
#define R3TP_VER1_EPILOGUE_BEGIN(start, frameNum) (uint8_t*)((uint8_t*)start + 8UL + (uint32_t)frameNum * 4UL)

/**
 * @brief Calculates the total length of an R3TP version 1 frame based on the number of subscription frames
 */
#define R3TP_VER1_MESSAGE_LENGTH(frameNum) (uint32_t)((uint32_t)frameNum * 4UL + 12UL)

#endif /* __RT12E_LIBS_R3TP_H_ */
