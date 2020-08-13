/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.h
 * @brief Header file providing prototypes of functions called by the gnssRx task
 */

#ifndef __WCU_GNSSRX_CALLS_H_
#define __WCU_GNSSRX_CALLS_H_

#include "stm32f4xx_hal.h"
#include "quectel_l26_gnss_parser.h"

extern UART_HandleTypeDef huart3;
#define GNSS_UART_HANDLE (huart3)
#define GNSS_UART_INSTANCE (USART3)

#define WCU_CAN_ID_GPS_POS				(uint32_t)(0x500UL)				/* CAN ID: _500_GPS_POS */
#define WCU_CAN_ID_GPS_POS2				(uint32_t)(0x501UL)				/* CAN ID: _501_GPS_POS2 */
#define WCU_CAN_ID_GPS_STATUS			(uint32_t)(0x502UL)				/* CAN ID: _502_GPS_STATUS */

#define WCU_GNSSRX_UART_RX_BUFF_SIZE	(uint32_t)(100UL)				/* UART Rx buffer size */

extern void Error_Handler(void);

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void);

/**
 * @brief Listens for and handles the GNSS message
 * @retval None
 */
void gnssRx_HandleMessage(void);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS(GnssDataTypedef *pData);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS2(GnssDataTypedef *pData);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_STATUS(GnssDataTypedef *pData);

#endif /* __WCU_GNSSRX_CALLS_H_ */
