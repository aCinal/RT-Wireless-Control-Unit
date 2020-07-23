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

#define WCU_IWDGHANDLER_NOTIFICATIONVALUE_BTRECEIVE				0x00000001UL				/* btReceive task's unique notification value for checking in with the watchdog */
#define WCU_IWDGHANDLER_NOTIFICATIONVALUE_XBEESEND				0x00000002UL				/* xbeeSend task's unique notification value for checking in with the watchdog */
#define WCU_IWDGHANDLER_NOTIFICATIONVALUE_GNSSRECEIVE			0x00000004UL				/* gnssReceive task's unique notification value for checking in with the watchdog */
#define WCU_IWDGHANDLER_NOTIFICATIONVALUE_RFRECEIVE				0x00000008UL				/* rfReceive task's unique notification value for checking in with the watchdog */
#define WCU_IWDGHANDLER_NOTIFICATIONVALUE_CANGATEKEEPER			0x00000010UL				/* canGatekeeper task's unique notification value for checking in with the watchdog */

#define WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FOPENFAILED			29UL						/* Value to notify xbeeSubscribe that f_open failed */
#define WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED			30UL						/* Value to notify xbeeSubscribe that f_read failed */
#define WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM		31UL						/* Value to notify xbeeSubscribe that the frame count was invalid */
#define WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_XQUEUESENDFAILED	32UL						/* Value to notify xbeeSubscribe that xQueueSend failed */

#define WCU_DEFAULT_TASK_DELAY									1U							/* Default task delay */
#define WCU_DEFAULT_TIMEOUT										portMAX_DELAY				/* Default timeout */
#define WCU_IWDGGATEKEEPER_XTASKNOTIFYWAIT_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* iwdgGatekeeper xTaskNotifyWait timeout */
#define WCU_CANTRANSMITQUEUE_SEND_TIMEOUT						WCU_DEFAULT_TIMEOUT			/* canTransmitQueue send timeout */
#define WCU_CANTRANSMITQUEUE_RECEIVE_TIMEOUT					0U							/* canTransmitQueue receive timeout */
#define WCU_CANRECEIVEQUEUE_SEND_TIMEOUT						WCU_DEFAULT_TIMEOUT			/* canReceiveQueue send timeout */
#define WCU_CANRECEIVEQUEUE_RECEIVE_TIMEOUT						0U							/* canReceiveQueue receive timeout */
#define WCU_SDIOLOGERRORQUEUE_SEND_TIMEOUT						WCU_DEFAULT_TIMEOUT			/* sdioLogErrorQueue send timeout */
#define WCU_SDIOLOGERRORQUEUE_RECEIVE_TIMEOUT					0U							/* sdioLogErrorQueue receive timeout */
#define WCU_SDIOSUBSCRIPTIONQUEUE_SEND_TIMEOUT					WCU_DEFAULT_TIMEOUT			/* sdioSubscriptionQueue send timeout */
#define WCU_SDIOSUBSCRIPTIONQUEUE_RECEIVE_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* sdioSubscriptionQueue receive timeout */
#define WCU_CRCMUTEX_TIMEOUT									WCU_DEFAULT_TIMEOUT			/* crcMutex acquire timeout */
#define WCU_BTRECEIVE_ULTASKNOTIFYTAKE_TIMEOUT					WCU_DEFAULT_TIMEOUT			/* btReceive ulTaskNotifyTake timeout */
#define WCU_XBEESEND_ULTASKNOTIFYTAKE_TIMEOUT					WCU_DEFAULT_TIMEOUT			/* xbeeSend ulTaskNotifyTake timeout */
#define WCU_XBEESUBSCRIBE_UART_TIMEOUT							500U						/* xbeeSubscribe UART Rx timeout */
#define WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT					10U							/* xbeeSubscribe UART Rx cleanup timeout */
#define WCU_XBEESUBSCRIBE_ULTASKNOTIFYTAKE_TIMEOUT				WCU_DEFAULT_TIMEOUT			/* xbeeSubscribe ulTaskNotifyTake timeout */
#define WCU_XBEESUBSCRIBE_XTASKNOTIFYWAIT_TIMEOUT				5000U						/* xbeeSubscribe xTaskNotifyWait timeout */
#define WCU_SDIOGATEKEEPER_XTASKNOTIFYWAIT_TIMEOUT				0U							/* sdioGateekeper xTaskNotifyWait timeout */

#define WCU_CAN_PAYLOAD_SIZE									8U							/* CAN payload size in bytes */
#define WCU_ERROR_LOG_TIMESTAMP_SIZE							11U							/* Length of the error log timestamp */

#define WCU_SDIOGATEKEEPER_ERRLOG_PATH							"ERRLOG.TXT"				/* Error log file path */
#define WCU_SDIOGATEEKEPER_SUBSCR_PATH							"SUBSCR"					/* Subscription file path */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
