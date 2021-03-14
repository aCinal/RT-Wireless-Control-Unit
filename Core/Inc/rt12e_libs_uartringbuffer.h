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
typedef enum EUartRbRet {
	EUartRbRet_Ok = 0,                    /* No error */
	EUartRbRet_InvalidParams,             /* Invalid function parameters */
	EUartRbRet_HalError,                  /* HAL error */
	EUartRbRet_BufferEmpty                /* Ring buffer empty */
} EUartRbRet;

/**
 * @brief Ring buffer control block
 * @note Members of this structure must not be modified directly, a dedicated API should be used instead
 */
typedef struct SUartRb {
	uint8_t *BufferPtr;                   /* Pointer to the buffer */
	size_t BufferSize;                    /* Buffer size in bytes */
	UART_HandleTypeDef *PeriphHandlePtr;  /* Peripheral handle */
	void (*Callback)(void);               /* Callback */
	volatile size_t Head;                 /* Position of the head */
	volatile size_t Tail;                 /* Position of the tail */
	volatile bool Dirty;                  /* Buffer dirty flag */
} SUartRb;

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param callback Function to be called on message received
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbInit(SUartRb *rb, UART_HandleTypeDef *uartHandle,
		uint8_t *buffer, size_t bufferSize, void (*callback)(void));

/**
 * @brief Enable interrupts and start the data transfer to the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbStart(SUartRb *rb);

/**
 * @brief Disable interrupts and stop the data transfer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbStop(SUartRb *rb);

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbIsr(SUartRb *rb);

/**
 * @brief Move the data from the ring buffer to the destination
 * @param rb Pointer to the ring buffer control block
 * @param buffer Destination buffer
 * @param bufferSize Size of the destination buffer
 * @param bytesRead Number of bytes read from the buffer
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbRead(SUartRb *rb, uint8_t *buffer, size_t bufferSize, size_t* bytesRead);

/**
 * @brief Test if the buffer is dirty
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status: EUartRbRet_Ok if the buffer is dirty, EUartRbRet_BufferEmpty otherwise
 */
EUartRbRet UartRbDirty(SUartRb *rb);

/**
 * @brief Flush the buffer and reset the dirty flag
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRbRet Status
 */
EUartRbRet UartRbInvalidate(SUartRb *rb);

#endif /* __RT12E_LIBS_UARTRINGBUFFER_H_ */
