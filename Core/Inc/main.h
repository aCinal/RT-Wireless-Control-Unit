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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
