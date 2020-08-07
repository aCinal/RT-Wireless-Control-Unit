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
#define RF_SP1_CSN_Pin GPIO_PIN_4
#define RF_SP1_CSN_GPIO_Port GPIOA
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

#define WCU_IWDGGTKP_NV_BTRX						0x00000001UL			/* btRx task's unique notification value for checking in with the watchdog */
#define WCU_IWDGGTKP_NV_XBEETX						0x00000002UL			/* xbeeTx task's unique notification value for checking in with the watchdog */
#define WCU_IWDGGTKP_NV_GNSSRX						0x00000004UL			/* gnssRx task's unique notification value for checking in with the watchdog */
#define WCU_IWDGGTKP_NV_RFRX						0x00000008UL			/* rfRx task's unique notification value for checking in with the watchdog */
#define WCU_IWDGGTKP_NV_CANGTKP						0x00000010UL			/* canGtkp task's unique notification value for checking in with the watchdog */

#define WCU_XBEERX_NV_FOPENFAILED					29UL					/* Value to notify xbeeRx that f_open failed */
#define WCU_XBEERX_NV_FREADFAILED					30UL					/* Value to notify xbeeRx that f_read failed */
#define WCU_XBEERX_NV_INVALIDFRAMENUM				31UL					/* Value to notify xbeeRx that the frame count was invalid */
#define WCU_XBEERX_NV_XQUEUESENDFAILED				32UL					/* Value to notify xbeeRx that xQueueSend failed */

#define WCU_DEFAULT_TASK_DELAY						1U						/* Default task delay */
#define WCU_DEFAULT_TIMEOUT							portMAX_DELAY			/* Default timeout */
#define WCU_DIAG_TASK_DELAY							pdMS_TO_TICKS(1000)		/* diag task delay */
#define WCU_IWDGGTKP_XTASKNOTIFYWAIT_TIMEOUT		WCU_DEFAULT_TIMEOUT		/* iwdgGtkp xTaskNotifyWait timeout */
#define WCU_CANTXQUEUE_XQUEUESEND_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* canTxQueue xQueueSend timeout */
#define WCU_CANTXQUEUE_XQUEUERECEIVE_TIMEOUT		0U						/* canTxQueue xQueueReceive timeout */
#define WCU_CANRXQUEUE_XQUEUESEND_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* canRxQueue xQueueSend timeout */
#define WCU_CANRXQUEUE_XQUEUERECEIVE_TIMEOUT		0U						/* canRxQueue xQueueReceive timeout */
#define WCU_SDIOLOGQUEUE_XQUEUESEND_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* sdioLogQueue xQueueSend timeout */
#define WCU_SDIOLOGQUEUE_XQUEUERECEIVE_TIMEOUT		0U						/* sdioLogQueue xQueueReceive timeout */
#define WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* sdioSubQueue xQueueSend timeout */
#define WCU_SDIOSUBQUEUE_XQUEUERECEIVE_TIMEOUT		WCU_DEFAULT_TIMEOUT		/* sdioSubQueue xQueueReceive timeout */
#define WCU_CRCMUTEX_TIMEOUT						WCU_DEFAULT_TIMEOUT		/* crcMutex acquire timeout */
#define WCU_BTRX_ULTASKNOTIFYTAKE_TIMEOUT			0U						/* btRx ulTaskNotifyTake timeout */
#define WCU_XBEETX_ULTASKNOTIFYTAKE_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* xbeeTx ulTaskNotifyTake timeout */
#define WCU_XBEERX_UART_TIMEOUT						500U					/* xbeeRx UART Rx timeout */
#define WCU_XBEERX_UART_CLEANUP_TIMEOUT				10U						/* xbeeRx UART Rx cleanup timeout */
#define WCU_XBEERX_ULTASKNOTIFYTAKE_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* xbeeRx ulTaskNotifyTake timeout */
#define WCU_XBEERX_XTASKNOTIFYWAIT_TIMEOUT			pdMS_TO_TICKS(1000)		/* xbeeRx xTaskNotifyWait timeout */
#define WCU_SDIOGTKP_XTASKNOTIFYWAIT_TIMEOUT		0U						/* sdioGtkp xTaskNotifyWait timeout */
#define WCU_GNSSRX_ULTASKNOTIFYTAKE_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* gnssRx ulTaskNotifyTake timeout */
#define WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT		WCU_DEFAULT_TIMEOUT		/* gnssRx_DeviceConfig UART Tx timeout */
#define WCU_GNSSRX_DEVICECONFIG_SETUP_DELAY			1000U					/* gnssRx_DeviceConfig device setup delay */
#define WCU_RFRX_ULTASKNOTIFYTAKE_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* rfRx ulTaskNotifyTake timeout */
#define WCU_IWDGGTKP_INIT_DELAY						1000U					/* iwdgGtkp IWDG init delay */
#define WCU_DIAG_ULTASKNOTIFYTAKE_TIMEOUT			WCU_DEFAULT_TIMEOUT		/* diag ulTaskNotifyTake timeout */

#define WCU_LOGGER_TIMESTAMP_SIZE					11U						/* Length of the error log timestamp */
#define WCU_GNSSRX_UARTRXBUFF_SIZE					100U
#define WCU_RFRX_SPIRXBUFF_SIZE						20U/* TODO */

#define WCU_SDIOGTKP_LOGFILE_PATH					"ERRLOG.TXT"				/* Error log file path */
#define WCU_SDIOGTKP_SUBFILE_PATH					"SUBSCR"					/* Subscription file path */

#define WCU_CANID_GPS_POS							0x500UL						/* CAN ID: _500_GPS_POS */
#define WCU_CANID_GPS_POS2							0x501UL						/* CAN ID: _501_GPS_POS2 */
#define WCU_CANID_GPS_STATUS						0x502UL						/* CAN ID: _502_GPS_STATUS */
#define WCU_CANID_WCU_DIAG							0x733UL						/* CAN ID: _733_WCU_DIAG */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
