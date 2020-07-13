/**
 * @author Adrian Cinal
 * @file rt12e_libs_r3tp.h
 * @brief Header file containing Racing Team Telemetry Transport Protocol (R3TP) defines and macros
 */

#ifndef __RT12E_LIBS_R3TP_H_
#define __RT12E_LIBS_R3TP_H_

/* Exported defines ------------------------------------------------------------*/
#define R3TP_VER0_FRAME_SIZE 20U									/* R3TP version 0 frame size in bytes */
#define R3TP_VER1_MAX_FRAME_NUM 28U									/* Maximum number of frames in a subscription */
#define R3TP_VER1_MAX_FRAME_SIZE 12 + 4 * R3TP_VER1_MAX_FRAME_NUM	/* R3TP version 1 max frame size in bytes */
#define R3TP_VER0_VER_RES_SEQ_BYTE 0x00								/* VER 0 protocol version byte */
#define R3TP_VER1_VER_RES_SEQ_BYTE 0x20								/* VER 1 protocol version byte */
#define R3TP_END_SEQ_LOW_BYTE 0xDE									/* R3TP end sequence low byte */
#define R3TP_END_SEQ_HIGH_BYTE 0xED									/* R3TP end sequence high byte */

/* Exported macro ------------------------------------------------------------*/
/**
 * @brief This macro returns the address at which the payload of an R3TP version 1 frame begins in reference to the buffer
 */
#define R3TP_VER1_PAYLOAD_BEGIN(buff) (uint8_t*)((uint8_t*)buff + (uint32_t)8)

/**
 * @brief This macro returns the address offset from the start of the payload of an R3TP version 1 frame
 */
#define R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, offset) (uint8_t*)((uint8_t*)buff + 8U + (uint32_t)offset)

/**
 * @brief This macro returns the address at which the epilogue (frame align and END SEQ) of an R3TP version 1 frame begins
 */
#define R3TP_VER1_EPILOGUE_BEGIN(buff, frameNum) (uint8_t*)((uint8_t*)buff + 8U + (uint32_t)frameNum * 4U)

/**
 * @brief This macro calculates the total length of an R3TP version 1 frame based on the number of subscription frames
 */
#define R3TP_VER1_MESSAGE_LENGTH(frameNum) (uint32_t)((uint32_t)frameNum * 4U + 12U)

#endif /* __RT12E_LIBS_R3TP_H_ */
