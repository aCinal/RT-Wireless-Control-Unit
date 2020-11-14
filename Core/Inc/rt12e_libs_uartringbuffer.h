/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer.h
 * @brief Header file containing the interface for using the ring buffer with UART DMA
 */

#ifndef __RT12E_LIBS_UARTRINGBUFFER_H_
#define __RT12E_LIBS_UARTRINGBUFFER_H_

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <inttypes.h>
#include <stdbool.h>

/* Exported typedef ------------------------------------------------------------*/
/**
 * @brief API functions return value enumeration
 */
typedef enum EUartRingBufRet {
	EUartRingBufRet_Ok = 0,                /* No error */
	EUartRingBufRet_InvalidParams,         /* Invalid function parameters */
	EUartRingBufRet_HalError,              /* HAL error */
	EUartRingBufRet_BufferEmpty            /* Ring buffer empty */
} EUartRingBufRet;

/**
 * @brief Ring buffer internal state structure
 */
typedef struct SUartRingBufInternalState {
	volatile size_t Head;                 /* Position of the head */
	volatile size_t Tail;                 /* Position of the tail */
	volatile bool Dirty;                  /* Buffer dirty flag */
} SUartRingBufInternalState;

/**
 * @brief Ring buffer structure
 */
typedef struct SUartRingBuf {
	uint8_t *BufferPtr;                   /* Pointer to the buffer */
	size_t BufferSize;                    /* Buffer size in bytes */
	UART_HandleTypeDef *PeriphHandlePtr;  /* Peripheral handle */
	void (*Callback)(void);               /* Callback */
	SUartRingBufInternalState State;      /* Internal state */
} SUartRingBuf;

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Enable interrupts and start the data transfer to the ring buffer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_Start(SUartRingBuf *ringBufPtr);

/**
 * @brief Disable interrupts and stop the data transfer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_Stop(SUartRingBuf *ringBufPtr);

/**
 * @brief ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_IrqHandlerCallback(SUartRingBuf *ringBufPtr);

/**
 * @brief Test if there is unread data in the buffer
 * @param ringBufPtr Pointer to the ring buffer structure
 * @retval bool True if there is new data in the buffer, false otherwise
 */
bool UartRingBuf_IsDataReady(SUartRingBuf *ringBufPtr);

/**
 * @brief Move the data from the ring buffer to the destination
 * @param ringBufPtr Pointer to the ring buffer structure
 * @param dstBufPtr Destination address
 * @param dstBufSize Size of the destination buffer
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet UartRingBuf_Read(SUartRingBuf *ringBufPtr,
		uint8_t *dstBufPtr, size_t dstBufSize);

#endif /* __RT12E_LIBS_UARTRINGBUFFER_H_ */
