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
#include <quectel_l26_gnss_parser.h>
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

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeSubscribe_WaitSubscriptionFromSD(void);

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssReceive_DeviceConfig(void);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_POS(GnssDataTypedef *pData);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_POS2(GnssDataTypedef *pData);

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_STATUS(GnssDataTypedef *pData);

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGatekeeper_LoadTelemetrySubscription(FIL *fp, const TCHAR *path);

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
  osThreadDef(gnssReceive, StartGnssReceiveTask, osPriorityAboveNormal, 0, 128);
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  huart3.Init.BaudRate = 9600;
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
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
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
		vTaskNotifyGiveFromISR(xbeeSendHandle, NULL);
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
		vTaskNotifyGiveFromISR(btReceiveHandle, NULL);
		break;

	case (uint32_t) GNSS_UART_INSTANCE:
		/* Notify gnssReceive task */
		vTaskNotifyGiveFromISR(gnssReceiveHandle, NULL);
		break;

	case (uint32_t) XBEE_UART_INSTANCE:
		/* Notify xbeeSubscribe task */
		vTaskNotifyGiveFromISR(xbeeSubscribeHandle, NULL);
		break;
	}
}

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeSubscribe_WaitSubscriptionFromSD(void) {
	uint32_t frameNum; /* Number of frames in a subscription */
	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	uint32_t notificationValue; /* Buffer for the notification value */

	/* Wait for sdioGatekeeper to notify the task if there is a valid subscription stored on the SD card */
	if (pdTRUE
			== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &notificationValue,
			WCU_XBEESUBSCRIBE_XTASKNOTIFYWAIT_TIMEOUT)) {
		if (notificationValue <= 28UL) {
			/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
			frameNum = notificationValue;

			enum {
				SUBSCRIPTIONREAD_OK = 0U, SUBSCRIPTIONREAD_ERROR
			} readStatus = SUBSCRIPTIONREAD_OK; /* Status flag */

			for (uint32_t i = 0; i < frameNum; i += 1UL) {
				if (pdTRUE
						!= xQueueReceive(sdioSubscriptionQueueHandle,
								subscription + i,
								WCU_SDIOSUBSCRIPTIONQUEUE_RECEIVE_TIMEOUT)) {
					/* Log error and break */
					LOGERROR(
							"xbeeSubscribe failed to receive from sdioSubscriptionQueue\r\n");
					readStatus = SUBSCRIPTIONREAD_ERROR;
					break;
				}
			}

			/* If no error occured */
			if (SUBSCRIPTIONREAD_OK == readStatus) {
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
}

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssReceive_DeviceConfig(void) {
	/* Wait for the device to turn on and set up */
	vTaskDelay(WCU_GNSSRECEIVE_DEVICECONFIG_SETUP_DELAY);

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	const char PMTK_SET_POS_FIX[] = "$PMTK220,100*1F\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) PMTK_SET_POS_FIX,
					sizeof(PMTK_SET_POS_FIX),
					WCU_GNSSRECEIVE_DEVICECONFIG_UART_TIMEOUT)) {
		Error_Handler();
	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*2B\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE,
					(uint8_t*) PMTK_API_SET_GNSS_SEARCH_MODE,
					sizeof(PMTK_API_SET_GNSS_SEARCH_MODE),
					WCU_GNSSRECEIVE_DEVICECONFIG_UART_TIMEOUT)) {
		Error_Handler();
	}
}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_POS(GnssDataTypedef *pData) {
	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_POS;

	/* Write the longitude to the frame payload */
	int32_t longitude = normalizeCoordinate(pData->Longitude, pData->LonDir);
	canFrame.Payload[0] = MSB32(longitude);
	canFrame.Payload[1] = HIGHMID32(longitude);
	canFrame.Payload[2] = LOWMID32(longitude);
	canFrame.Payload[3] = LSB(longitude);

	/* Write the latitude to the frame payload */
	int32_t latitude = normalizeCoordinate(pData->Latitude, pData->LatDir);
	canFrame.Payload[4] = MSB32(latitude);
	canFrame.Payload[5] = HIGHMID32(longitude);
	canFrame.Payload[6] = LOWMID32(longitude);
	canFrame.Payload[7] = LSB(longitude);

	/* Push CAN frame to queue */
	if (pdTRUE
			!= xQueueSend(canTransmitQueueHandle, &canFrame,
					WCU_CANTRANSMITQUEUE_SEND_TIMEOUT)) {
		LOGERROR(
				"gnssReceive_Send_GPS_POS failed to send to canTransmitQueue\r\n");
	}
}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_POS2(GnssDataTypedef *pData) {
	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 6;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_POS2;

	/* Write the speed to the frame payload */
	uint16_t speed = normalizeSpeed(pData->Speed);
	canFrame.Payload[0] = MSB16(speed);
	canFrame.Payload[1] = LSB(speed);

	/* Write the direction to the frame payload */
	uint16_t direction = normalizeDirection(pData->COG);
	canFrame.Payload[2] = MSB16(direction);
	canFrame.Payload[3] = LSB(direction);

	/* Write the altitude to the frame payload */
	uint16_t altitude = normalizeAltitude(pData->Altitude);
	canFrame.Payload[4] = MSB16(altitude);
	canFrame.Payload[5] = LSB(altitude);

	/* Push CAN frame to queue */
	if (pdTRUE
			!= xQueueSend(canTransmitQueueHandle, &canFrame,
					WCU_CANTRANSMITQUEUE_SEND_TIMEOUT)) {
		LOGERROR(
				"gnssReceive_Send_GPS_POS2 failed to send to canTransmitQueue\r\n");
	}
}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssReceive_Send_GPS_STATUS(GnssDataTypedef *pData) {
	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_STATUS;

	/* Write the satellites visible count to the frame payload */
	canFrame.Payload[0] = pData->SatellitesInView;
	/* Clear two most significant bits */
	canFrame.Payload[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canFrame.Payload[0] |= (uint8_t) pData->FixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canFrame.Payload[1] = pData->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = normalizeTime(pData->Time);
	canFrame.Payload[2] = MSB32(time);
	canFrame.Payload[3] = HIGHMID32(time);
	canFrame.Payload[4] = LOWMID32(time);
	canFrame.Payload[5] = LSB(time);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canFrame.Payload[5] |= (MSB32(pData->Date) & 0xF) << 4U;
	canFrame.Payload[6] = 0xFF & (pData->Date >> 20U);
	canFrame.Payload[7] = 0xFF & (pData->Date >> 12U);

	/* Push CAN frame to queue */
	if (pdTRUE
			!= xQueueSend(canTransmitQueueHandle, &canFrame,
					WCU_CANTRANSMITQUEUE_SEND_TIMEOUT)) {
		LOGERROR(
				"gnssReceive_Send_GPS_STATUS failed to send to canTransmitQueue\r\n");
	}
}

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGatekeeper_LoadTelemetrySubscription(FIL *fp, const TCHAR *path) {
	/* Try opening the file */
	if (FR_OK == f_open(fp, path,
	FA_READ | FA_OPEN_EXISTING)) {

		uint32_t frameNum; /* Buffer for the number of frames */
		uint32_t frameBuff; /* Buffer for a subscription frame */
		uint8_t temp[4]; /* Temporary buffer for four bytes to be read as a single 32-bit little endian value */
		UINT bytesRead; /* Buffer for the number of bytes read */

		/* Read the number of frames */
		if (FR_OK == f_read(fp, temp, 4U, &bytesRead)) {
			/* Parse the number of frames */
			frameNum = READ32(temp[3], temp[2], temp[1], temp[0]);

			/* Assert valid number of frames */
			if (frameNum <= R3TP_VER1_MAX_FRAME_NUM) {
				/* Read the payload and push it to the queue */
				for (uint32_t i = 0; i < frameNum; i += 1UL) {
					if (FR_OK == f_read(fp, temp, 4, &bytesRead)) {
						/* Assert end of file was not reached */
						if (bytesRead < 4U) {
							/* On invalid number of frames */
							/* Close the file */
							f_close(fp);
							/* Queue cleanup */
							xQueueReset(sdioSubscriptionQueueHandle);
							return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM;
						}

						/* Parse the frame */
						frameBuff = READ32(temp[3], temp[2], temp[1], temp[0]);

						/* Send the frame to the queue */
						if (pdPASS
								!= xQueueSend(sdioSubscriptionQueueHandle,
										&frameBuff,
										WCU_SDIOSUBSCRIPTIONQUEUE_SEND_TIMEOUT)) {
							/* If failed to send to queue */
							/* Close the file */
							f_close(fp);
							/* Queue cleanup */
							xQueueReset(sdioSubscriptionQueueHandle);
							return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_XQUEUESENDFAILED;
						}
					} else {
						/* If failed to read the frame */
						/* Close the file */
						f_close(fp);
						/* Queue cleanup */
						xQueueReset(sdioSubscriptionQueueHandle);
						return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED;
					}
				}

				/* On success */
				/* Close the file */
				f_close(fp);
				/* Return the number of frames read */
				return frameNum;

			} else {
				/* On invalid number of frames */
				/* Close the file */
				f_close(fp);
				return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_INVALIDFRAMENUM;
			}
		} else {
			/* If failed to read the number of frames */
			/* Close the file */
			f_close(fp);
			return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FREADFAILED;
		}

		/* Close the file */
		f_close(fp);

	} else {
		/* If failed to open the file */
		return WCU_XBEESUBSCRIBE_NOTIFICATIONVALUE_FOPENFAILED;
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

	/* Give the other tasks time to set up */
	vTaskDelay(WCU_IWDGGATEEKEEPER_INIT_DELAY);

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
						&notificationValue, 0);
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
			if (R3TP_VER0_VER_RES_SEQ_BYTE != btUartRxBuff[0]) {
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
			readCrc = READ16(btUartRxBuff[3], btUartRxBuff[2]);

			/* Clear the CHECKSUM field */
			btUartRxBuff[2] = 0x00U;
			btUartRxBuff[3] = 0x00U;

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
			canFrame.Header.Tx.StdId = READ32(btUartRxBuff[7], btUartRxBuff[6],
					btUartRxBuff[5], btUartRxBuff[4]);
			/* Read the Data Length Code */
			canFrame.Header.Tx.DLC = (uint32_t)btUartRxBuff[8];
			if(CAN_PAYLOAD_SIZE < canFrame.Header.Tx.DLC) {
				LOGERROR("Invalid DLC in btReceive\r\n");
				continue;
			}

			/* Read the payload */
			for (uint8_t i = 0U; i < canFrame.Header.Tx.DLC; i += 1U) {
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
			xbeeUartTxBuff[0] = R3TP_VER0_VER_RES_SEQ_BYTE;

			/* Set the SEQ NUM field */
			xbeeUartTxBuff[1] = seqNum;
			/* Increment the sequence number */
			seqNum = (seqNum < 255U) ? seqNum + 1U : 0U;

			/* Set the END SEQ field */
			xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 2U] =
			R3TP_END_SEQ_LOW_BYTE;
			xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 1U] =
			R3TP_END_SEQ_HIGH_BYTE;

			/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
			xbeeUartTxBuff[4] = LSB(frameBuff.Header.Rx.StdId);
			xbeeUartTxBuff[5] = MSB16(frameBuff.Header.Rx.StdId);

			/* Set the DLC field */
			xbeeUartTxBuff[8] = (uint8_t) frameBuff.Header.Rx.DLC;

			/* Set the DATA field */
			for (uint8_t i = 0; i < frameBuff.Header.Rx.DLC; i += 1U) {
				xbeeUartTxBuff[9U + i] = frameBuff.Payload[i];
			}

			/* Calculate the CRC */
			if (osOK == osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {
				calculatedCrc =
						TWOLOWBYTES(
								HAL_CRC_Calculate(&hcrc, (uint32_t*)xbeeUartTxBuff, R3TP_VER0_FRAME_SIZE / 4U));
				osMutexRelease(crcMutexHandle);

				/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
				xbeeUartTxBuff[2] = LSB(calculatedCrc);
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

	/* Wait for SDIO gatekeeper to test if there is a valid subscription stored on the SD card */
	xbeeSubscribe_WaitSubscriptionFromSD();

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);

		/* Listen for the subscription frame VER octet */
		HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, xbeeUartRxBuff, 1);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_XBEESUBSCRIBE_ULTASKNOTIFYTAKE_TIMEOUT)) {
			/* Validate the VER and RES/SEQ field */
			if (R3TP_VER1_VER_RES_SEQ_BYTE != xbeeUartRxBuff[0]) {
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
			frameNum = READ32(xbeeUartRxBuff[7], xbeeUartRxBuff[6],
					xbeeUartRxBuff[5], xbeeUartRxBuff[4]);

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
			readCrc = READ16(xbeeUartRxBuff[3], xbeeUartRxBuff[2]);

			/* Clear the CHECKSUM field */
			xbeeUartRxBuff[2] = 0x00U;
			xbeeUartRxBuff[3] = 0x00U;

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
								1, WCU_XBEESUBSCRIBE_UART_CLEANUP_TIMEOUT)) {
					__NOP();
				}
				continue;
			}

			/* Read the payload */
			for (uint32_t i = 0; i < frameNum; i += 1UL) {
				subscription[i] =
						READ32(
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 3U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 2U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 1U + 4U*i)),
								*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(xbeeUartRxBuff, 4U * i)));
			}

			/* Write subscription to sdioSubscriptionQueue */
			enum {
				SUBSCRIPTIONWRITE_OK = 0U, SUBSCRIPTIONWRITE_ERROR
			} writeStatus = SUBSCRIPTIONWRITE_OK; /* Status flag */
			for (uint32_t i = 0; i < frameNum; i += 1UL) {
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
					writeStatus = SUBSCRIPTIONWRITE_ERROR;
					break;
				}
			}

			/* If no error occured while pushing the subscription to sdioSubscriptionQueue */
			if (SUBSCRIPTIONWRITE_OK == writeStatus) {
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
	static uint8_t gnssUartRxBuff[WCU_GNSSRECEIVE_UARTRXBUFF_SIZE]; /* UART Rx buffer */
	static GnssDataTypedef dataBuff; /* GNSS data buffer */

	/* Configure the device */
	gnssReceive_DeviceConfig();

	/* Listen for the message */
	HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, gnssUartRxBuff,
	WCU_GNSSRECEIVE_UARTRXBUFF_SIZE);

	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);


		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_GNSSRECEIVE_ULTASKNOTIFYTAKE_TIMEOUT)) {
			/* Try parsing the message */
			switch(parseMessage(&dataBuff, (char*)gnssUartRxBuff, WCU_GNSSRECEIVE_UARTRXBUFF_SIZE)) {
			case GNSS_DATA_READY: /* If the data is ready */
				/* Send the data to CAN */
				gnssReceive_Send_GPS_POS(&dataBuff);
				gnssReceive_Send_GPS_POS2(&dataBuff);
				gnssReceive_Send_GPS_STATUS(&dataBuff);

				/* Clear the data buffer */
				memset(&dataBuff, 0x00, sizeof(dataBuff));

				/* Listen for the next message */
				HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, gnssUartRxBuff,
				WCU_GNSSRECEIVE_UARTRXBUFF_SIZE);
				break;

			case GNSS_DATA_PENDING: /* If the data is not complete */
				/* Listen for the next message */
				HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, gnssUartRxBuff,
				WCU_GNSSRECEIVE_UARTRXBUFF_SIZE);
				break;

			case GNSS_DATA_ERROR: /* If the parser failed */
				LOGERROR("parseMessage failed in gnssReceive\r\n");
				/* Listen for the next message */
				HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, gnssUartRxBuff,
				WCU_GNSSRECEIVE_UARTRXBUFF_SIZE);
			}
		}

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

	/* Try mounting the logical drive */
	if (FR_OK != f_mount(&fatFs, SDPath, 1)) {
		/* On f_mount failure, suspend the task */
		vTaskSuspend(NULL);
	}

	/* Try loading the telemetry subscription from the SD card */
	notificationValue = sdioGatekeeper_LoadTelemetrySubscription(
			&subscriptionFile, WCU_SDIOGATEEKEPER_SUBSCR_PATH);
	/* Notify xbeeSubscribe of the result */
	(void) xTaskNotify(xbeeSubscribeHandle, notificationValue,
			eSetValueWithOverwrite);

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
				uint8_t temp[4]; /* Temporary buffer to facilitate transmitting a 32-bit little endian value */
				UINT bytesWritten; /* Buffer for the number of bytes written */

				/* Try opening the file */
				if (FR_OK == f_open(&subscriptionFile,
				WCU_SDIOGATEEKEPER_SUBSCR_PATH,
				FA_WRITE | FA_CREATE_ALWAYS)) {

					/* Print the number of frames to the SD card */
					temp[0] = LSB(notificationValue);
					temp[1] = LOWMID32(notificationValue);
					temp[2] = HIGHMID32(notificationValue);
					temp[3] = MSB32(notificationValue);
					(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

					/* Print the subscription to the file */
					for (uint32_t i = 0; i < notificationValue; i += 1UL) {
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
						temp[0] = LSB(frameBuff);
						temp[1] = LOWMID32(frameBuff);
						temp[2] = HIGHMID32(frameBuff);
						temp[3] = MSB32(frameBuff);
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
