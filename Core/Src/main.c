/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <rt12e_libs_generic.h>
#include <rt12e_libs_can.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Intuitive names for UART instances */
#define BT_UART_INSTANCE	USART1
#define GNSS_UART_INSTANCE	USART3
#define XBEE_UART_INSTANCE	UART4

/* Intuitive names for UART handles */
#define BT_UART_HANDLE		huart1
#define GNSS_UART_HANDLE	huart3
#define XBEE_UART_HANDLE	huart4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief Logs an error message to the SD card
 */
#define LOGERROR(message) do { \
	/* Allocate the memory for the error message */ \
	char* errMsg = pvPortMalloc(WCU_ERROR_LOG_TIMESTAMP_SIZE + strlen(message) + 1U); \
	/* Assert successful memory allocation */ \
	if(errMsg != NULL) { \
		/* Write the timestamp to the memory block */ \
		sprintf(errMsg, "%010lu ", HAL_GetTick()); \
		/* Write the message to the memory block */ \
		sprintf(errMsg + WCU_ERROR_LOG_TIMESTAMP_SIZE, message); \
		/* Push the pointer to the message to the logErrorQueue */ \
		if(pdPASS != xQueueSend(sdioLogErrorQueueHandle, &errMsg, WCU_SDIOLOGERRORQUEUE_SEND_TIMEOUT)) { \
			/* Cleanup on failure to push to queue */ \
			vPortFree(errMsg);\
		} \
	} \
} while(0)

/**
 * @brief Checks in with the watchdog thread
 */
#define WATCHDOG_CHECKIN(notificationValue) xTaskNotify((TaskHandle_t)iwdgGatekeeperHandle, notificationValue, eSetBits)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId iwdgGatekeeperHandle;
osThreadId btReceiveHandle;
osThreadId xbeeSendHandle;
osThreadId xbeeSubscribeHandle;
osThreadId gnssReceiveHandle;
osThreadId rfReceiveHandle;
osThreadId canGatekeeperHandle;
osThreadId sdioGatekeeperHandle;
osMessageQId canTransmitQueueHandle;
osMessageQId canReceiveQueueHandle;
osMessageQId sdioLogErrorQueueHandle;
osMessageQId sdioSubscriptionQueueHandle;
osMutexId crcMutexHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartIwdgGatekeeperTask(void const * argument);
void StartBtReceiveTask(void const * argument);
void StartXbeeSendTask(void const * argument);
void StartXbeeSubscribeTask(void const * argument);
void StartGnssReceiveTask(void const * argument);
void StartRfReceiveTask(void const * argument);
void StartCanGatekeeperTask(void const * argument);
void StartSdioGatekeeperTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of crcMutex */
  osMutexDef(crcMutex);
  crcMutexHandle = osMutexCreate(osMutex(crcMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of canTransmitQueue */
  osMessageQDef(canTransmitQueue, 16, CanFrameTypedef);
  canTransmitQueueHandle = osMessageCreate(osMessageQ(canTransmitQueue), NULL);

  /* definition and creation of canReceiveQueue */
  osMessageQDef(canReceiveQueue, 16, CanFrameTypedef);
  canReceiveQueueHandle = osMessageCreate(osMessageQ(canReceiveQueue), NULL);

  /* definition and creation of sdioLogErrorQueue */
  osMessageQDef(sdioLogErrorQueue, 16, const char*);
  sdioLogErrorQueueHandle = osMessageCreate(osMessageQ(sdioLogErrorQueue), NULL);

  /* definition and creation of sdioSubscriptionQueue */
  osMessageQDef(sdioSubscriptionQueue, 32, uint32_t);
  sdioSubscriptionQueueHandle = osMessageCreate(osMessageQ(sdioSubscriptionQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of iwdgGatekeeper */
  osThreadDef(iwdgGatekeeper, StartIwdgGatekeeperTask, osPriorityNormal, 0, 128);
  iwdgGatekeeperHandle = osThreadCreate(osThread(iwdgGatekeeper), NULL);

  /* definition and creation of btReceive */
  osThreadDef(btReceive, StartBtReceiveTask, osPriorityNormal, 0, 128);
  btReceiveHandle = osThreadCreate(osThread(btReceive), NULL);

  /* definition and creation of xbeeSend */
  osThreadDef(xbeeSend, StartXbeeSendTask, osPriorityNormal, 0, 128);
  xbeeSendHandle = osThreadCreate(osThread(xbeeSend), NULL);

  /* definition and creation of xbeeSubscribe */
  osThreadDef(xbeeSubscribe, StartXbeeSubscribeTask, osPriorityNormal, 0, 128);
  xbeeSubscribeHandle = osThreadCreate(osThread(xbeeSubscribe), NULL);

  /* definition and creation of gnssReceive */
  osThreadDef(gnssReceive, StartGnssReceiveTask, osPriorityNormal, 0, 128);
  gnssReceiveHandle = osThreadCreate(osThread(gnssReceive), NULL);

  /* definition and creation of rfReceive */
  osThreadDef(rfReceive, StartRfReceiveTask, osPriorityNormal, 0, 128);
  rfReceiveHandle = osThreadCreate(osThread(rfReceive), NULL);

  /* definition and creation of canGatekeeper */
  osThreadDef(canGatekeeper, StartCanGatekeeperTask, osPriorityNormal, 0, 128);
  canGatekeeperHandle = osThreadCreate(osThread(canGatekeeper), NULL);

  /* definition and creation of sdioGatekeeper */
  osThreadDef(sdioGatekeeper, StartSdioGatekeeperTask, osPriorityNormal, 0, 1024);
  sdioGatekeeperHandle = osThreadCreate(osThread(sdioGatekeeper), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_SPI1_CSN_GPIO_Port, RF_SPI1_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_PWR_UP_Pin|RF_TRX_CE_Pin|RF_TX_EN_Pin|GNSS_FORCE_ON_Pin 
                          |GNSS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : XBEE_RSSI_Pin RF_DR_Pin RF_AM_Pin */
  GPIO_InitStruct.Pin = XBEE_RSSI_Pin|RF_DR_Pin|RF_AM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : XBEE_RESET_Pin */
  GPIO_InitStruct.Pin = XBEE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XBEE_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_SPI1_CSN_Pin */
  GPIO_InitStruct.Pin = RF_SPI1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SPI1_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_CD_Pin RF_uPCLK_Pin GNSS_1PPS_Pin */
  GPIO_InitStruct.Pin = RF_CD_Pin|RF_uPCLK_Pin|GNSS_1PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_PWR_UP_Pin RF_TRX_CE_Pin RF_TX_EN_Pin GNSS_FORCE_ON_Pin 
                           GNSS_RESET_Pin */
  GPIO_InitStruct.Pin = RF_PWR_UP_Pin|RF_TRX_CE_Pin|RF_TX_EN_Pin|GNSS_FORCE_ON_Pin 
                          |GNSS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
__weak void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (XBEE_UART_INSTANCE == huart->Instance) {
		/* Resume the xbeeSend task */
		vTaskNotifyGiveFromISR((TaskHandle_t) xbeeSendHandle, NULL);
	}
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	switch ((uint32_t) huart->Instance) {
	case (uint32_t) BT_UART_INSTANCE:
		/* Notify btReceive task */
		vTaskNotifyGiveFromISR((TaskHandle_t) btReceiveHandle, NULL);
		break;

	case (uint32_t) GNSS_UART_INSTANCE:
		/* Notify gnssReceive task */
		vTaskNotifyGiveFromISR((TaskHandle_t) gnssReceiveHandle, NULL);
		break;

	case (uint32_t) XBEE_UART_INSTANCE:
		/* Notify xbeeSubscribe task */
		vTaskNotifyGiveFromISR((TaskHandle_t) xbeeSubscribeHandle, NULL);
		break;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartIwdgGatekeeperTask */
/**
 * @brief  Function implementing the iwdgGatekeeper thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIwdgGatekeeperTask */
void StartIwdgGatekeeperTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	static uint32_t notificationValue; /* Buffer to pass the notification value out of the xTaskNotifyWait function */

	/* Initialize the watchdog */
	HAL_IWDG_Init(&hiwdg);

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Wait for notification */
		if (pdTRUE
				== xTaskNotifyWait(0x00000000UL, 0x00000000UL,
						&notificationValue,
						WCU_IWDGGATEKEEPER_XTASKNOTIFYWAIT_TIMEOUT)) {
			/* If all tasks checked in */
			if (notificationValue
					== (WCU_IWDGHANDLER_NOTIFICATIONVALUE_BTRECEIVE
							| WCU_IWDGHANDLER_NOTIFICATIONVALUE_XBEESEND
							| WCU_IWDGHANDLER_NOTIFICATIONVALUE_GNSSRECEIVE |
							WCU_IWDGHANDLER_NOTIFICATIONVALUE_RFRECEIVE
							| WCU_IWDGHANDLER_NOTIFICATIONVALUE_CANGATEKEEPER)) {
				/* Refresh the counter */
				HAL_IWDG_Refresh(&hiwdg);
				/* Clear the notification value */
				(void) xTaskNotifyWait(0xFFFFFFFFUL, 0xFFFFFFFFUL,
						&notificationValue, 0U);
			}
		}
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartBtReceiveTask */
/**
 * @brief Function implementing the btReceive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBtReceiveTask */
void StartBtReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartBtReceiveTask */
	static CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */
	static uint8_t btUartRxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Rx buffer */
	static uint16_t readCrc; /* Buffer for the transmitted CRC */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Listen for the message */
		HAL_UART_Receive_DMA(&BT_UART_HANDLE, btUartRxBuff,
		R3TP_VER0_FRAME_SIZE);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_BTRECEIVE_ULTASKNOTIFYTAKE_TIMEOUT)) {
			/* Validate the VER and RES/SEQ field */
			if (R3TP_VER0_VER_RES_SEQ_BYTE != btUartRxBuff[0U]) {
				LOGERROR("Invalid VER/RES/SEQ in btReceive\r\n");
				continue;
			}

			/* Validate the END SEQ field */
			if ((R3TP_END_SEQ_LOW_BYTE
					!= btUartRxBuff[R3TP_VER0_FRAME_SIZE - 2U])
					|| (R3TP_END_SEQ_HIGH_BYTE
							!= btUartRxBuff[R3TP_VER0_FRAME_SIZE - 1U])) {
				LOGERROR("Invalid END SEQ in btReceive\r\n");
				continue;
			}

			/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
			readCrc = READ16(btUartRxBuff[3U], btUartRxBuff[2U]);

			/* Clear the CHECKSUM field */
			btUartRxBuff[2U] = 0x00U;
			btUartRxBuff[3U] = 0x00U;

			/* Calculate the CRC */
			if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {
				calculatedCrc =
						TWOLOWBYTES(
								HAL_CRC_Calculate(&hcrc, (uint32_t* )btUartRxBuff, R3TP_VER0_FRAME_SIZE / 4U));
				osMutexRelease(crcMutexHandle);
			} else {
				LOGERROR("crcMutex timeout in btReceive\r\n");
				continue;
			}

			/* Validate the CRC */
			if (readCrc != calculatedCrc) {
				LOGERROR("Invalid CRC in btReceive\r\n");
				continue;
			}

			/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
			canFrame.Header.Tx.StdId = READ32(btUartRxBuff[7U],
					btUartRxBuff[6U], btUartRxBuff[5U], btUartRxBuff[4U]);
			/* Read the Data Length Code */
			canFrame.Header.Tx.DLC = (uint32_t) (
					btUartRxBuff[8U] < WCU_CAN_PAYLOAD_SIZE ?
							btUartRxBuff[8U] : WCU_CAN_PAYLOAD_SIZE);

			/* Read the payload */
			for (uint8_t i = 0; i < canFrame.Header.Tx.DLC; i += 1) {
				canFrame.Payload[i] = btUartRxBuff[9U + i];
			}

			/* Push CAN frame to queue */
			if (pdTRUE
					!= xQueueSend(canTransmitQueueHandle, &canFrame,
							WCU_CANTRANSMITQUEUE_SEND_TIMEOUT)) {
				LOGERROR("btReceive failed to send to canTransmitQueue\r\n");
			}
		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGHANDLER_NOTIFICATIONVALUE_BTRECEIVE);
	}
  /* USER CODE END StartBtReceiveTask */
}

/* USER CODE BEGIN Header_StartXbeeSendTask */
/**
 * @brief Function implementing the xbeeSend thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeSendTask */
void StartXbeeSendTask(void const * argument)
{
  /* USER CODE BEGIN StartXbeeSendTask */
	static CanFrameTypedef frameBuff; /* CAN frame buffer */
	static uint8_t xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Tx buffer */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */
	static uint8_t seqNum = 0U; /* Sequence number */

	/* Activate XBEE Pro by driving the XBEE_RESET pin high */
	HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		if (pdTRUE == xQueueReceive(canReceiveQueueHandle, xbeeUartTxBuff,
		WCU_CANRECEIVEQUEUE_RECEIVE_TIMEOUT)) {
			/* Assert valid data direction */
			if (RX != frameBuff.DataDirection) {
				/* Log error */
				LOGERROR("Invalid DataDirection in xbeeSend\r\n");
				continue;
			}

			/* Clear the buffer */
			memset(xbeeUartTxBuff, 0x00U, R3TP_VER0_FRAME_SIZE);

			/* Set VER and RES/SEQ field */
			xbeeUartTxBuff[0U] = R3TP_VER0_VER_RES_SEQ_BYTE;

			/* Set the SEQ NUM field */
			xbeeUartTxBuff[1U] = seqNum;
			/* Increment the sequence number */
			seqNum = (seqNum < 255U) ? seqNum + 1U : 0U;

			/* Set the END SEQ field */
			xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 2U] =
			R3TP_END_SEQ_LOW_BYTE;
			xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 1U] =
			R3TP_END_SEQ_HIGH_BYTE;

			/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
			xbeeUartTxBuff[4U] = LSB16(frameBuff.Header.Rx.StdId);
			xbeeUartTxBuff[5U] = MSB16(frameBuff.Header.Rx.StdId);

			/* Set the DLC field */
			xbeeUartTxBuff[8U] = (uint8_t) frameBuff.Header.Rx.DLC;

			/* Set the DATA field */
			for (uint8_t i = 0U; i < frameBuff.Header.Rx.DLC; i += 1U) {
				xbeeUartTxBuff[9U + i] = frameBuff.Payload[i];
			}

			/* Calculate the CRC */
			if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {
				calculatedCrc =
						TWOLOWBYTES(
								HAL_CRC_Calculate(&hcrc, (uint32_t*)xbeeUartTxBuff, R3TP_VER0_FRAME_SIZE / 4U));
				osMutexRelease(crcMutexHandle);

				/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
				xbeeUartTxBuff[2] = LSB16(calculatedCrc);
				xbeeUartTxBuff[3] = MSB16(calculatedCrc);
			} else {
				/* Log error */
				LOGERROR("crcMutex timeout in xbeeSend\r\n");
				continue;
			}

			/* Transmit frame */
			HAL_UART_Transmit_DMA(&XBEE_UART_HANDLE, xbeeUartTxBuff,
			R3TP_VER0_FRAME_SIZE);

			/* Wait for the transmission to end */
			if (0UL < ulTaskNotifyTake(pdTRUE,
			WCU_XBEESEND_ULTASKNOTIFYTAKE_TIMEOUT)) {
				/* Log error */
				LOGERROR(
						"xbeeSend failed to receive notification from TxCpltCallback\r\n");
			}
		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGHANDLER_NOTIFICATIONVALUE_XBEESEND);
	}
  /* USER CODE END StartXbeeSendTask */
}

/* USER CODE BEGIN Header_StartXbeeSubscribeTask */
/**
 * @brief Function implementing the xbeeSubscribe thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeSubscribeTask */
void StartXbeeSubscribeTask(void const * argument)
{
  /* USER CODE BEGIN StartXbeeSubscribeTask */
	static uint8_t xbeeUartRxBuff[R3TP_VER1_MAX_FRAME_SIZE]; /* UART Rx buffer */
	static uint16_t readCrc; /* Buffer for the transmitted CRC */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */
	static uint32_t frameNum; /* Number of frames in a subscription */
	static uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	static uint32_t notificationValue; /* Buffer for the notification value */
	enum {
		XBEESUBSCRIBE_OK = 0U, XBEESUBSCRIBE_ERROR
	} Status = XBEESUBSCRIBE_OK; /* Status flag */

	/* Wait for sdioGatekeeper to notify the task if there is a valid subscription stored on the SD card */
	if (pdTRUE
			== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &notificationValue,
					WCU_XBEESUBSCRIBE_XTASKNOTIFYWAIT_TIMEOUT)) {
		if (notificationValue <= 28UL) {
			Status = XBEESUBSCRIBE_OK; /* Reset the status flag */
			/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
			frameNum = notificationValue;

			for (uint32_t i = 0UL; i < frameNum; i += 1UL) {
				if (pdTRUE
						!= xQueueReceive(sdioSubscriptionQueueHandle,
								subscription + i,
								WCU_SDIOSUBSCRIPTIONQUEUE_RECEIVE_TIMEOUT)) {
					/* Log error and break */
					LOGERROR(
							"xbeeSubscribe failed to receive from sdioSubscriptionQueue\r\n");
					Status = XBEESUBSCRIBE_ERROR;
					break;
				}
			}

			/* If no error occured */
			if (XBEESUBSCRIBE_OK == Status) {
				/* Set the CAN filters */
				setCanFilterList(&hcan1, subscription, frameNum);
			}
		} else {
			/* Log error */
			switch (notificationValue) {
			case WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FOPENFAILED:
				LOGERROR(
						"sdioGatekeeper failed to open the subscription file\r\n");
				break;
			case WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED:
				LOGERROR(
						"sdioGatekeeper failed to read from the subscription file\r\n");
				break;
			case WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM:
				LOGERROR("Invalid FRAME NUM in the subscription file\r\n");
				break;
			case WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_XQUEUESENDFAILED:
				LOGERROR(
						"sdioGatekeeper failed to send to sdioSubscriptionQueue\r\n");
				break;
			default:
				LOGERROR("Invalid FRAME NUM in xbeeSubscribe (SD)\r\n");
				break;
			}
		}
	}

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Listen for the subscription frame VER octet */
		HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, xbeeUartRxBuff, 1U);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_XBEESUBSCRIBE_ULTASKNOTIFYTAKE_TIMEOUT)) {
			/* Validate the VER and RES/SEQ field */
			if (R3TP_VER1_VER_RES_SEQ_BYTE != xbeeUartRxBuff[0U]) {
				/* Log the error */
				LOGERROR("Invalid VER/RES/SEQ in xbeeSubscribe\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				while (HAL_OK
						== HAL_UART_Receive(&XBEE_UART_HANDLE, xbeeUartRxBuff,
								1U, WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT)) {
					__NOP();
				}
				continue;
			}

			/* On valid version byte, receive SEQ NUM, CHECKSUM and FRAME NUM */
			if (HAL_OK
					!= HAL_UART_Receive(&XBEE_UART_HANDLE, xbeeUartRxBuff + 1U,
							7, WCU_XBEESUBSCRIBE_UART_TIMEOUT)) {
				/* Log error */
				LOGERROR(
						"Failed to receive SEQ NUM, CHECKSUM and FRAME NUM in xbeeSubscribe\r\n");
				continue;
			}

			/* Read the FRAME NUM field */
			frameNum = READ32(xbeeUartRxBuff[7U], xbeeUartRxBuff[6U],
					xbeeUartRxBuff[5U], xbeeUartRxBuff[4U]);

			/* Assert the payload won't overflow the buffer */
			if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {
				/* Log error */
				LOGERROR("Invalid FRAME NUM in xbeeSubscribe\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				while (HAL_OK
						== HAL_UART_Receive(&XBEE_UART_HANDLE, xbeeUartRxBuff,
								1U, WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT)) {
					__NOP();
				}
				continue;
			}

			/* Receive the payload */
			if (HAL_OK
					!= HAL_UART_Receive(&XBEE_UART_HANDLE,
							R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff),
							frameNum * 4U, WCU_XBEESUBSCRIBE_UART_TIMEOUT)) {
				/* Log error */
				LOGERROR("Failed to receive the payload in xbeeSubscribe\r\n");
				continue;
			}

			/* Receive the frame align bytes (two) and END SEQ (also two bytes) */
			if (HAL_OK
					!= HAL_UART_Receive(&XBEE_UART_HANDLE,
							R3TP_VER1_EPILOGUE_BEGIN(xbeeUartRxBuff, frameNum),
							4U, WCU_XBEESUBSCRIBE_UART_TIMEOUT)) {
				/* Log error */
				LOGERROR("Failed to receive END SEQ in xbeeSubscribe\r\n");
				continue;
			}

			/* Validate the END SEQ field */
			if ((R3TP_END_SEQ_LOW_BYTE
					!= xbeeUartRxBuff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 2U])
					|| (R3TP_END_SEQ_HIGH_BYTE
							!= xbeeUartRxBuff[R3TP_VER1_MESSAGE_LENGTH(frameNum)
									- 1U])) {
				/* Log error */
				LOGERROR("Invalid END SEQ in xbeeSubscribe\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				while (HAL_OK
						== HAL_UART_Receive(&XBEE_UART_HANDLE, xbeeUartRxBuff,
								1U, WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT)) {
					__NOP();
				}
				continue;
			}

			/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
			readCrc = READ16(xbeeUartRxBuff[3U], xbeeUartRxBuff[2U]);

			/* Clear the CHECKSUM field */
			xbeeUartRxBuff[2U] = 0x00U;
			xbeeUartRxBuff[3U] = 0x00U;

			/* Calculate the CRC */
			if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {
				calculatedCrc =
						TWOLOWBYTES(
								HAL_CRC_Calculate(&hcrc, (uint32_t* )xbeeUartRxBuff, R3TP_VER1_MESSAGE_LENGTH(frameNum)/4U));
				osMutexRelease(crcMutexHandle);
			} else {
				/* Log error */
				LOGERROR("crcMutex timeout in xbeeSubscribe\r\n");
				continue;
			}

			/* Validate the CRC */
			if (readCrc != calculatedCrc) {
				/* Log error */
				LOGERROR("Invalid CRC in xbeeSubscribe\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				while (HAL_OK
						== HAL_UART_Receive(&XBEE_UART_HANDLE, xbeeUartRxBuff,
								1U, WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT)) {
					__NOP();
				}
				continue;
			}

			/* Read the payload */
			for (uint32_t i = 0UL; i < frameNum; i += 1UL) {
				subscription[i] =
						READ32(
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 3U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 2U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 1U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 4U * i)));
			}

			/* Write subscription to sdioSubscriptionQueue */
			Status = XBEESUBSCRIBE_OK; /* Reset the status flag */
			for (uint32_t i = 0UL; i < frameNum; i += 1UL) {
				/* Send the frame to the queue */
				if (pdTRUE
						!= xQueueSend(sdioSubscriptionQueueHandle,
								subscription + i,
								WCU_SDIOSUBSCRIPTIONQUEUE_SEND_TIMEOUT)) {
					/* Log error */
					LOGERROR(
							"xbeeSubscribe failed to send to sdioSubscriptionQueue\r\n");
					/* Cleanup */
					xQueueReset(sdioSubscriptionQueueHandle);
					Status = XBEESUBSCRIBE_ERROR;
					break;
				}
			}

			/* If no error occured while pushing the subscription to sdioSubscriptionQueue */
			if (XBEESUBSCRIBE_OK == Status) {
				/* Notify sdioGatekeeper */
				(void) xTaskNotify(sdioGatekeeperHandle, frameNum,
						eSetValueWithOverwrite);
			}

			/* Set the CAN filters */
			setCanFilterList(&hcan1, subscription, frameNum);
		}
	}
  /* USER CODE END StartXbeeSubscribeTask */
}

/* USER CODE BEGIN Header_StartGnssReceiveTask */
/**
 * @brief Function implementing the gnssReceive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGnssReceiveTask */
void StartGnssReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartGnssReceiveTask */
	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGHANDLER_NOTIFICATIONVALUE_GNSSRECEIVE);
	}
  /* USER CODE END StartGnssReceiveTask */
}

/* USER CODE BEGIN Header_StartRfReceiveTask */
/**
 * @brief Function implementing the rfReceive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRfReceiveTask */
void StartRfReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartRfReceiveTask */
	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGHANDLER_NOTIFICATIONVALUE_RFRECEIVE);
	}
  /* USER CODE END StartRfReceiveTask */
}

/* USER CODE BEGIN Header_StartCanGatekeeperTask */
/**
 * @brief Function implementing the canGatekeeper thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanGatekeeperTask */
void StartCanGatekeeperTask(void const * argument)
{
  /* USER CODE BEGIN StartCanGatekeeperTask */
	static CanFrameTypedef frameBuff; /* CAN frame buffer */
	static uint32_t dummy; /* CAN Tx mailbox */

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Check for outgoing messages */
		if (pdTRUE == xQueueReceive(canTransmitQueueHandle, &frameBuff,
		WCU_CANTRANSMITQUEUE_RECEIVE_TIMEOUT)) {
			/* Validate the DataDirection member */
			if (TX == frameBuff.DataDirection) {
				/* Send the message */
				HAL_CAN_AddTxMessage(&hcan1, &frameBuff.Header.Tx,
						frameBuff.Payload, &dummy);
			} else {
				/* Log error */
				LOGERROR("Invalid DataDirection in canGatekeeper\r\n");
			}
		}

		/* Check for incoming messages */
		if (0U < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {
			/* Receive the message */
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &frameBuff.Header.Rx,
					frameBuff.Payload);
			/* Set the DataDirection member in the CAN frame struct */
			frameBuff.DataDirection = RX;
			/* Send the frame to the telemetry queue */
			if (pdTRUE
					!= xQueueSend(canReceiveQueueHandle, &frameBuff,
							WCU_CANRECEIVEQUEUE_SEND_TIMEOUT)) {
				/* Log error */
				LOGERROR("canGatekeeper failed to send to canReceiveQueue\r\n");
			}
		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGHANDLER_NOTIFICATIONVALUE_CANGATEKEEPER);
	}
  /* USER CODE END StartCanGatekeeperTask */
}

/* USER CODE BEGIN Header_StartSdioGatekeeperTask */
/**
 * @brief Function implementing the sdioGatekeeper thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdioGatekeeperTask */
void StartSdioGatekeeperTask(void const * argument)
{
  /* USER CODE BEGIN StartSdioGatekeeperTask */
	static FATFS fatFs; /* File system object structure */
	static FIL errorLogFile; /* Error log file object structure */
	static FIL subscriptionFile; /* Telemetry subscription file object structure */
	static char *errorLogBuff; /* Buffer for the pointer to the error message */
	static uint32_t notificationValue; /* Buffer for the notification value */

	/* Mount the logical drive */
	while (FR_OK != f_mount(&fatFs, SDPath, 0x01U)) {
		__NOP();
	}

	/* Try loading the telemetry subscription from the SD card */
	if (FR_OK
			== f_open(&subscriptionFile, WCU_SDIOGATEEKEPER_SUBSCR_PATH,
					FA_READ | FA_OPEN_EXISTING)) {

		enum {
			SDIOGATEKEEPER_OK = 0U, SDIOGATEKEEPER_ERROR
		} Status; /* Status flag */
		uint32_t frameNum; /* Buffer for the number of frames */
		uint32_t frameBuff; /* Buffer for a subscription frame */
		uint8_t temp[4U]; /* Temporary buffer for four bytes to be read as a single 32-bit little endian value */
		UINT bytesRead; /* Buffer for the number of bytes read */

		/* Read the number of frames */
		if (FR_OK == f_read(&subscriptionFile, temp, 4U, &bytesRead)) {
			/* Parse the number of frames */
			frameNum = READ32(temp[3U], temp[2U], temp[1U], temp[0U]);

			/* Assert valid number of frames */
			if (frameNum <= R3TP_VER1_MAX_FRAME_NUM) {
				Status = SDIOGATEKEEPER_OK; /* Reset the status flag */

				/* Read the payload and push it to the queue */
				for (uint32_t i = 0U; i < frameNum; i += 1U) {
					if (FR_OK
							== f_read(&subscriptionFile, temp, 4U,
									&bytesRead)) {
						/* Assert end of file was not reached */
						if (bytesRead < 4U) {
							/* On invalid number of frames */
							(void) xTaskNotify(xbeeSubscribeHandle,
									WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM,
									eSetValueWithOverwrite);
							Status = SDIOGATEKEEPER_ERROR;
							break;
						}

						/* Parse the frame */
						frameBuff = READ32(temp[3U], temp[2U], temp[1U],
								temp[0U]);

						/* Send the frame to the queue */
						if (pdPASS
								!= xQueueSend(sdioSubscriptionQueueHandle,
										&frameBuff,
										WCU_SDIOSUBSCRIPTIONQUEUE_SEND_TIMEOUT)) {
							/* If failed to send to queue */
							(void) xTaskNotify(xbeeSubscribeHandle,
									WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_XQUEUESENDFAILED,
									eSetValueWithOverwrite);
							Status = SDIOGATEKEEPER_ERROR;
							/* Cleanup */
							xQueueReset(sdioSubscriptionQueueHandle);
							break;
						}
					} else {
						/* If failed to read the frame */
						(void) xTaskNotify(xbeeSubscribeHandle,
								WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED,
								eSetValueWithOverwrite);
						Status = SDIOGATEKEEPER_ERROR;
						break;
					}
				}

				/* If no error occured */
				if (SDIOGATEKEEPER_OK == Status) {
					/* Notify the xbeeSubscribe task to start reading from the queue */
					(void) xTaskNotify(xbeeSubscribeHandle, frameNum,
							eSetValueWithOverwrite);
				}

			} else {
				/* On invalid number of frames */
				(void) xTaskNotify(xbeeSubscribeHandle,
						WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM,
						eSetValueWithOverwrite);
			}
		} else {
			/* If failed to read the number of frames */
			(void) xTaskNotify(xbeeSubscribeHandle,
					WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED,
					eSetValueWithOverwrite);
		}

		/* Close the file */
		f_close(&subscriptionFile);

	} else {
		/* If failed to open the file */
		(void) xTaskNotify(xbeeSubscribeHandle,
				WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FOPENFAILED,
				eSetValueWithOverwrite);
	}

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Listen on incoming error messages */
		if (pdTRUE == xQueueReceive(sdioLogErrorQueueHandle, &errorLogBuff,
		WCU_SDIOLOGERRORQUEUE_RECEIVE_TIMEOUT)) {
			/* Try opening the file */
			if (FR_OK == f_open(&errorLogFile, WCU_SDIOGATEKEEPER_ERRLOG_PATH,
			FA_WRITE | FA_OPEN_APPEND)) {
				UINT bytesWritten; /* Buffer for the number of bytes written */
				(void) f_write(&errorLogFile, errorLogBuff,
						strlen(errorLogBuff), &bytesWritten);
				/* Close the file */
				(void) f_close(&errorLogFile);
				/* Free the allocated memory */
				vPortFree(errorLogBuff);
				errorLogBuff = NULL;
			}
		}

		/* Listen on notification from xbeeSubscribe */
		if (pdTRUE
				== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL,
						&notificationValue,
						WCU_SDIOGATEKEEPER_XTASKNOTIFYWAIT_TIMEOUT)) {
			if (notificationValue <= 28UL) {
				/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
				uint32_t frameBuff; /* Buffer for a subscription frame */
				uint8_t temp[4U]; /* Temporary buffer to facilitate transmitting a 32-bit little endian value */
				UINT bytesWritten; /* Buffer for the number of bytes written */

				/* Try opening the file */
				if (FR_OK
						== f_open(&subscriptionFile,
								WCU_SDIOGATEEKEPER_SUBSCR_PATH,
								FA_WRITE | FA_CREATE_ALWAYS)) {
					/* Print the number of frames to the SD card */
					temp[0U] = LSB32(notificationValue);
					temp[1U] = LOWMID32(notificationValue);
					temp[2U] = HIGHMID32(notificationValue);
					temp[3U] = MSB32(notificationValue);
					(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

					/* Print the subscription to the file */
					for (uint32_t i = 0UL; i < notificationValue; i += 1UL) {
						if (pdTRUE
								!= xQueueReceive(sdioSubscriptionQueueHandle,
										&frameBuff,
										WCU_SDIOSUBSCRIPTIONQUEUE_RECEIVE_TIMEOUT)) {
							/* Log error */
							LOGERROR(
									"sdioGatekeeper failed to receive from sdioSubscriptionQueue\r\n");
							break;
						}

						/* Print the frame to the SD card */
						temp[0U] = LSB32(frameBuff);
						temp[1U] = LOWMID32(frameBuff);
						temp[2U] = HIGHMID32(frameBuff);
						temp[3U] = MSB32(frameBuff);
						(void) f_write(&subscriptionFile, temp, 4U,
								&bytesWritten);
					}

					/* Close the file */
					f_close(&subscriptionFile);

				} else {
					/* If failed to open the file */
					/* Log error */
					LOGERROR(
							"sdioGatekeeper failed to open the subscription file\r\n");
				}

			} else {
				/* Log error */
				LOGERROR("Invalid notification value in sdioGatekeeper\r\n");
			}
		}
	}
  /* USER CODE END StartSdioGatekeeperTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
