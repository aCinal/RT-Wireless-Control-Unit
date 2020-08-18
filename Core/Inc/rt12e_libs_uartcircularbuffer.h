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
typedef enum EUartCircularBufferStatus {

	EUartCircularBufferStatus_OK = 0, /* No error */

	EUartCircularBufferStatus_InvalidParams, /* Invalid function parameters */

	EUartCircularBufferStatus_HalError, /* HAL error */

	EUartCircularBufferStatus_BufferEmpty /* Ring buffer empty */

} EUartCircularBufferStatus;

/**
 * @brief Ring buffer structure
 */
typedef struct SCircularBuffer {

	uint8_t *BufferPtr; /* Pointer to the buffer */

	size_t BufferSize; /* Buffer size in bytes */

	volatile size_t Head; /* Position of the head */

	volatile size_t Tail; /* Position of the tail */

	UART_HandleTypeDef *PeriphHandlePtr; /* Peripheral handle */

	void (*Callback)(void); /* Callback */

} SUartCircularBuffer;

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Enables interrupts and starts the data transfer to the ring buffer
 * @param rbPtr Pointer to the circular buffer structure
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_start(SUartCircularBuffer *rbPtr);

/**
 * @brief Disables interrupts and stops the data transfer
 * @param rbPtr Pointer to the circular buffer structure
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_stop(SUartCircularBuffer *rbPtr);

/**
 * @brief ISR callback
 * @note This function must be called from the USARTx_IRQHandler
 * @param rbPtr Pointer to the circular buffer structure
 * @retval None
 */
void uartCircularBuffer_irqHandlerCallback(SUartCircularBuffer *rbPtr);

/**
 * @brief Moves the data from the ring buffer to the destination
 * @param rbPtr Pointer to the circular buffer structure
 * @param dstBuffPtr Destination address
 * @param dstBuffSize Size of the destination buffer
 * @retval EUartCircularBufferStatus Error code
 */
EUartCircularBufferStatus uartCircularBuffer_read(SUartCircularBuffer *rbPtr,
		uint8_t *dstBuffPtr, size_t dstBuffSize);

#endif /* __RT12E_LIBS_UARTCIRCULARBUFFER_H_ */
