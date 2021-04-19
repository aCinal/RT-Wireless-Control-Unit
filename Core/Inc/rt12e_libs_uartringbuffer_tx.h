/**
 * @author Adrian Cinal
 * @file rt12e_libs_uartringbuffer_tx.h
 * @brief Header file providing the interface for using the ring buffer with UART DMA for transmission
 */

#ifndef __RT12E_LIBS_UARTRINGBUFFER_TX_H_
#define __RT12E_LIBS_UARTRINGBUFFER_TX_H_

#include "stm32f4xx_hal.h"

#include <stddef.h>
#include <inttypes.h>
#include <stdbool.h>

/* Exported typedefs ------------------------------------------------------------*/
/**
 * @brief API functions return value enumeration
 */
typedef enum EUartTxRbRet {
	EUartTxRbRet_Ok = 0,                /* No error */
	EUartTxRbRet_InvalidParams,         /* Invalid function parameters */
	EUartTxRbRet_Error,                 /* Internal error */
	EUartTxRbRet_BufferEmpty,           /* Empty buffer */
	EUartTxRbRet_Again,                 /* Transmission is in progress, try again later */
	EUartTxRbRet_WouldOverflow          /* Buffer would overflow if the write were to proceed */
} EUartTxRbRet;

/**
 * @brief TX ring buffer control block
 * @note Members of this structure must not be modified directly, a dedicated API should be used instead
 */
typedef struct SUartTxRb {
	uint8_t *BufferPtr;                 /* Pointer to the buffer*/
	size_t BufferSize;                  /* Buffer size in bytes*/
	UART_HandleTypeDef *UartHandlePtr;  /* Peripheral handle */
	void (*MsgSentCallback)(void);      /* Callback on transfer complete */
	size_t Head;                        /* Position of the head */
	size_t Tail;                        /* Position of the tail */
	bool Dirty;                         /* Buffer dirty flag */
	bool TransferInProgress;            /* Transfer in progress flag */
	uint8_t *LinearBuffer;              /* Dynamically allocated buffer for handling ring buffer overflow */
} SUartTxRb;

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Initialize the ring buffer control block
 * @param rb Pointer to the ring buffer control block
 * @param uartHandle UART peripheral handle
 * @param buffer User-defined buffer
 * @param bufferSize Size of the user-defined buffer
 * @param msgSentCallback Function to be called on message sent event
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbInit(SUartTxRb *rb, UART_HandleTypeDef *uartHandle, uint8_t *buffer, size_t bufferSize,
		void (*msgSentCallback)(void));

/**
 * @brief Idle line detection interrupt service routine
 * @note This function must be called from the USARTx_IRQHandler
 * @param rb Pointer to the ring buffer control block
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbIsr(SUartTxRb *rb);

/**
 * @brief Write a message into the ring buffer
 * @param rb Pointer to the ring buffer control block
 * @param data Data buffer
 * @param len Size of the data buffer
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbWrite(SUartTxRb *rb, uint8_t *data, size_t len);

/**
 * @brief Transmit data sitting in the buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbSend(SUartTxRb *rb);

/**
 * @brief Get number of free bytes in the buffer
 * @param rb Pointer to the ring buffer control block
 * @param freeSpace Pointer to pass the number of available bytes out of the funcion
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbGetFreeSpace(SUartTxRb *rb, size_t *freeSpace);

/**
 * @brief Invalidate data in the buffer
 * @param rb Pointer to the ring buffer control block
 * @retval EUartTxRbRet Status
 */
EUartTxRbRet UartTxRbInvalidate(SUartTxRb *rb);

#endif /* __RT12E_LIBS_UARTRINGBUFFER_TX_H_ */
