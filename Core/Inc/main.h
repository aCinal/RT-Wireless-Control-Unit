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

#include <rt12e_libs_r3tp.h>

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
#define WCU_NUMBER_OF_WATCHED_THREADS					(uint8_t)5U					/* Number of threads watched by the IWDG */
#define WCU_CAN_PAYLOAD_SIZE							8U							/* CAN payload size in bytes */
#define WCU_DEFAULT_TASK_DELAY							1U							/* Default task delay */
#define WCU_DEFAULT_TIMEOUT								portMAX_DELAY				/* Default timeout */
#define WCU_REPORTTOWATCHDOGQUEUE_SEND_TIMEOUT			WCU_DEFAULT_TIMEOUT			/* reportToWatchdogQueue send timeout */
#define WCU_REPORTTOWATCHDOGQUEUE_RECEIVE_TIMEOUT		0U							/* reportToWatchdogQueue receive timeout */
#define WCU_CANTRANSMITQUEUE_SEND_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* canTransmitQueue send timeout */
#define WCU_CANTRANSMITQUEUE_RECEIVE_TIMEOUT			0U							/* canTransmitQueue receive timeout */
#define WCU_CANRECEIVEQUEUE_SEND_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* canReceiveQueue send timeout */
#define WCU_CANRECEIVEQUEUE_RECEIVE_TIMEOUT				0U							/* canReceiveQueue receive timeout */
#define WCU_SDIOLOGERRORQUEUE_SEND_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* sdioLogErrorQueue send timeout */
#define WCU_SDIOLOGERRORQUEUE_RECEIVE_TIMEOUT			0U							/* sdioLogErrorQueue receive timeout */
#define WCU_CRCMUTEX_TIMEOUT							WCU_DEFAULT_TIMEOUT			/* crcMutex acquire timeout */
#define WCU_ERROR_LOG_TIMESTAMP_LENGTH					11							/* Length of the error log timestamp */

/**
 * @brief BT (WDTS) defines
 */
#define WCU_BT_UART_RX_BUFF_SIZE				R3TP_VER0_FRAME_SIZE	/* UART Rx buffer size in bytes */
#define WCU_BT_UART_RX_NOTIFY_TAKE_TIMEOUT		WCU_DEFAULT_TIMEOUT		/* UART Rx wait for notification timeout */

/**
 * @brief Xbee (telemetry) defines
 */
#define WCU_XBEE_UART_TX_TIMEOUT				WCU_DEFAULT_TIMEOUT		/* UART Tx timeout */
#define WCU_XBEE_UART_RX_TIMEOUT				WCU_DEFAULT_TIMEOUT		/* UART Rx timeout */

/**
 * @brief SD/FATFS defines
 */
#define WCU_ERROR_LOG_PATH						"ERRORLOG.TXT"			/* Error log file path */
#define WCU_SUBSCRIPTION_PATH					"SUBSCRIPTION.CSV"		/* Subscription file path */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
