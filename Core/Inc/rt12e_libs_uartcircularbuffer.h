/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartcircularbuffer.h
 * @brief Header file containing the interface for using the circular buffer with UART DMA
 */

#ifndef __RT12E_LIBS_UARTCIRCULARBUFFER_H_
#define __RT12E_LIBS_UARTCIRCULARBUFFER_H_

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <inttypes.h>

/* Exported typedef ------------------------------------------------------------*/
/**
 * @brief API functions return value enumeration
 */
typedef enum EUartCirBufRet {
	EUartCirBufRet_Ok = 0,                /* No error */
	EUartCirBufRet_InvalidParams,         /* Invalid function parameters */
	EUartCirBufRet_HalError,              /* HAL error */
	EUartCirBufRet_BufferEmpty            /* Ring buffer empty */
} EUartCirBufRet;

/**
 * @brief Circular buffer structure
 */
typedef struct SUartCirBuf {
	uint8_t *BufferPtr;                   /* Pointer to the buffer */
	size_t BufferSize;                    /* Buffer size in bytes */
	volatile size_t Head;                 /* Position of the head */
	volatile size_t Tail;                 /* Position of the tail */
	UART_HandleTypeDef *PeriphHandlePtr;  /* Peripheral handle */
	void (*Callback)(void);               /* Callback */
} SUartCirBuf;

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval EUartCirBufRet Error code
 */
EUartCirBufRet uartCirBuf_start(SUartCirBuf *cirBufPtr);

/**
 * @brief Disables interrupts and stops the data transfer
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval EUartCirBufRet Error code
 */
EUartCirBufRet uartCirBuf_stop(SUartCirBuf *cirBufPtr);

/**
 * @brief ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param cirBufPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCirBuf_irqHandlerCallback(SUartCirBuf *cirBufPtr);

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param cirBufPtr Pointer to the circular buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartCirBufRet Error code
 */
EUartCirBufRet uartCirBuf_read(SUartCirBuf *cirBufPtr,
		uint8_t *dstBuffPtr, size_t dstBuffSize);

#endif /* __RT12E_LIBS_UARTCIRCULARBUFFER_H_ */
