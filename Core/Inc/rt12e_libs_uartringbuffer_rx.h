/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer_rx.h
 * @brief Header file containing the interface for using the ring buffer with UART DMA for reception
 */

#ifndef __RT12E_LIBS_UARTRINGBUFFER_RX_H_
#define __RT12E_LIBS_UARTRINGBUFFER_RX_H_

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <inttypes.h>
#include <stdbool.h>

/* Exported typedefs ------------------------------------------------------------*/
/**
 * @brief API functions return value enumeration
 */
typedef enum EUartRxRbRet {
	EUartRxRbRet_Ok = 0,                    /* No error */
	EUartRxRbRet_InvalidParams,             /* Invalid function parameters */
	EUartRxRbRet_Error,                     /* Internal error */
	EUartRxRbRet_BufferEmpty                /* Empty buffer */
} EUartRxRbRet;

/**
 * @brief RX ring buffer control block
 * @note Members of this structure must not be modified directly, a dedicated API should be used instead
 */
typedef struct SUartRxRb {
	uint8_t *BufferPtr;                 /* Pointer to the buffer */
	size_t BufferSize;                  /* Buffer size in bytes */
	UART_HandleTypeDef *UartHandlePtr;  /* Peripheral handle */
	void (*MsgReceivedCallback)(void);  /* Callback on transfer complete */
	volatile size_t Head;               /* Position of the head */
	volatile size_t Tail;               /* Position of the tail */
	volatile bool Dirty;                /* Buffer dirty flag */
} SUartRxRb;



/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param msgReceivedCallback Function to be called on message received event
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbInit(SUartRxRb *rb, UART_HandleTypeDef *uartHandle, uint8_t *buffer, size_t bufferSize,
		void (*msgReceivedCallback)(void));

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbIsr(SUartRxRb *rb);

/**
 * @brief Enable interrupts and start the data transfer to the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbRecv(SUartRxRb *rb);

/**
 * @brief Disable interrupts and stop the data transfer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbAbort(SUartRxRb *rb);

/**
 * @brief Move the data from the ring buffer to the destination
 * @param rb Pointer to the ring buffer control block
 * @param buffer Destination buffer
 * @param bufferSize Size of the destination buffer
 * @param bytesRead Number of bytes read from the buffer
 * @retval EUartRxRbRet Status
 */
EUartRxRbRet UartRxRbRead(SUartRxRb *rb, uint8_t *buffer, size_t bufferSize, size_t* bytesRead);

#endif /* __RT12E_LIBS_UARTRINGBUFFER_RX_H_ */
