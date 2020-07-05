/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define XBEE_RSSI_Pin GPIO_PIN_2
#define XBEE_RSSI_GPIO_Port GPIOC
#define XBEE_RESET_Pin GPIO_PIN_3
#define XBEE_RESET_GPIO_Port GPIOC
#define XBEE_UART3_TX_Pin GPIO_PIN_0
#define XBEE_UART3_TX_GPIO_Port GPIOA
#define XBEE_UART3_RX_Pin GPIO_PIN_1
#define XBEE_UART3_RX_GPIO_Port GPIOA
#define RF_SPI1_CSN_Pin GPIO_PIN_4
#define RF_SPI1_CSN_GPIO_Port GPIOA
#define RF_SP1_SCK_Pin GPIO_PIN_5
#define RF_SP1_SCK_GPIO_Port GPIOA
#define RF_SPI1_MISO_Pin GPIO_PIN_6
#define RF_SPI1_MISO_GPIO_Port GPIOA
#define RF_SPI1_MOSI_Pin GPIO_PIN_7
#define RF_SPI1_MOSI_GPIO_Port GPIOA
#define RF_DR_Pin GPIO_PIN_4
#define RF_DR_GPIO_Port GPIOC
#define RF_AM_Pin GPIO_PIN_5
#define RF_AM_GPIO_Port GPIOC
#define RF_CD_Pin GPIO_PIN_0
#define RF_CD_GPIO_Port GPIOB
#define RF_uPCLK_Pin GPIO_PIN_1
#define RF_uPCLK_GPIO_Port GPIOB
#define RF_PWR_UP_Pin GPIO_PIN_2
#define RF_PWR_UP_GPIO_Port GPIOB
#define GNSS_UART4_TX_Pin GPIO_PIN_10
#define GNSS_UART4_TX_GPIO_Port GPIOB
#define GNSS_UART4_RX_Pin GPIO_PIN_11
#define GNSS_UART4_RX_GPIO_Port GPIOB
#define RF_TRX_CE_Pin GPIO_PIN_12
#define RF_TRX_CE_GPIO_Port GPIOB
#define RF_TX_EN_Pin GPIO_PIN_13
#define RF_TX_EN_GPIO_Port GPIOB
#define BT_UART1_TX_Pin GPIO_PIN_9
#define BT_UART1_TX_GPIO_Port GPIOA
#define BT_UART1_RX_Pin GPIO_PIN_10
#define BT_UART1_RX_GPIO_Port GPIOA
#define GNSS_1PPS_Pin GPIO_PIN_3
#define GNSS_1PPS_GPIO_Port GPIOB
#define GNSS_FORCE_ON_Pin GPIO_PIN_4
#define GNSS_FORCE_ON_GPIO_Port GPIOB
#define GNSS_RESET_Pin GPIO_PIN_5
#define GNSS_RESET_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/**
 * @brief Generic defines
 */
#define _WCU_DEFAULT_TASK_DELAY							1U							/* Default task delay */
#define _WCU_DEFAULT_TIMEOUT							portMAX_DELAY				/* Default timeout */
#define _WCU_CRC_MUTEX_TIMEOUT							_WCU_DEFAULT_TIMEOUT		/* crcMutex acquire timeout */
#define _WCU_REPORTTOWATCHDOG_QUEUE_SEND_TIMEOUT		_WCU_DEFAULT_TIMEOUT		/* reportToWatchdog queue send timeout */
#define _WCU_REPORTTOWATCHDOG_QUEUE_RECEIVE_TIMEOUT		0U							/* reportToWatchdog queue receive timeout */
#define _WCU_CANTRANSMIT_QUEUE_SEND_TIMEOUT				_WCU_DEFAULT_TIMEOUT		/* canTransmit queue send timeout */
#define _WCU_CANTRANSMIT_QUEUE_RECEIVE_TIMEOUT			0U							/* canTransmit queue receive timeout */
#define _WCU_CANSUBBEDFRAMES_QUEUE_SEND_TIMEOUT			_WCU_DEFAULT_TIMEOUT		/* canTlmtryFrames queue send timeout */
#define _WCU_CANSUBBEDFRAMES_QUEUE_RECEIVE_TIMEOUT		0U							/* canTlmtryFrames queue receive timeout */
#define _WCU_NUMBER_OF_WATCHED_THREADS					(uint8_t)5U					/* Number of threads watched by the IWDG */
#define _WCU_CAN_TX_BUFF_SIZE							8U							/* CAN Tx buffer size in bytes */
#define _WCU_CAN_RX_BUFF_SIZE							8U							/* CAN Rx buffer size in bytes */

/**
 * @brief R3TP defines
 */
#define _R3TP_VER0_FRAME_SIZE				20U											/* R3TP version 0 frame size in bytes */
#define _R3TP_TLMTRY_SUBSCR_MAX_FRAME_NUM	28U											/* Maximum number of frames in a subscription */
#define _R3TP_VER1_MAX_FRAME_SIZE			10 + 4 * _R3TP_TLMTRY_SUBSCR_MAX_FRAME_NUM	/* R3TP version 1 max frame size in bytes */
#define _R3TP_VER0_VER_RES_SEQ_BYTE			0x00										/* VER 0 protocol version byte */
#define _R3TP_VER1_VER_RES_SEQ_BYTE			0x20										/* VER 1 protocol version byte */
#define _R3TP_END_SEQ_LOW_BYTE				0xDE										/* R3TP end sequence low byte */
#define _R3TP_END_SEQ_HIGH_BYTE				0xED										/* R3TP end sequence high byte */

/**
 * @brief BT (WDTS) defines
 */
#define _WCU_BT_UART_RX_BUFF_SIZE				_R3TP_VER0_FRAME_SIZE	/* UART Rx buffer size in bytes */
#define _WCU_BT_UART_RX_NOTIFY_TAKE_TIMEOUT		_WCU_DEFAULT_TIMEOUT	/* UART Rx wait for notification timeout */

/**
 * @brief Xbee (telemetry) defines
 */
#define _WCU_XBEE_UART_RX_NOTIFY_TAKE_TIMEOUT	_WCU_DEFAULT_TIMEOUT	/* UART Rx wait for notification timeout */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
