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

#include "wcu_common.h"
#include "wcu_cangtkp_calls.h"
#include "wcu_sdiogtkp_calls.h"
#include "wcu_btrx_calls.h"
#include "wcu_gnssrx_calls.h"
#include "wcu_rfrx_calls.h"
#include "wcu_diagnostic_calls.h"
#include "wcu_xbeetxrx_calls.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define WCU_DEFAULT_TASK_DELAY  ((TickType_t) 10)          /* Default task delay */

#define TIM_1s_INSTANCE         (TIM7)                     /* Alias for the TIM7 timer instance */
#define TIM_1s_HANDLE           (htim7)                    /* Alias for the TIM7 timer handle */

/**
 * @brief Watchdog task notification values
 */
#define NV_IWDGGTKP_CANGTKP     ((uint32_t) 0x00000001UL)  /* canGtkp task's unique notification value for checking in with the watchdog */
#define NV_IWDGGTKP_BTRX        ((uint32_t) 0x00000002UL)  /* btRx task's unique notification value for checking in with the watchdog */
#define NV_IWDGGTKP_GNSSRX      ((uint32_t) 0x00000004UL)  /* gnssRx task's unique notification value for checking in with the watchdog */
#define NV_IWDGGTKP_RFRX        ((uint32_t) 0x00000008UL)  /* rfRx task's unique notification value for checking in with the watchdog */
#define NV_IWDGGTKP_XBEETXRX    ((uint32_t) 0x00000010UL)  /* xbeeTxRx task's unique notification value for checking in with the watchdog */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief Checks in with the watchdog thread
 */
#define WATCHDOG_CHECKIN(nv)  ((void) xTaskNotify(iwdgGtkpHandle, (nv), eSetBits))

/**
 * @brief Clears the notification value bits based on the provided mask
 */
#define CLEARNVBITS(nvMask)  ((void) xTaskNotifyWait((nvMask), 0, NULL, 0))

/**
 * @brief Clears the notification value
 */
#define CLEARNV()  (CLEARNVBITS(CLEAR_ALL_BITS_ON_ENTRY))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId canGtkpHandle;
osThreadId sdioGtkpHandle;
osThreadId iwdgGtkpHandle;
osThreadId btRxHandle;
osThreadId gnssRxHandle;
osThreadId rfRxHandle;
osThreadId xbeeTxRxHandle;
osThreadId diagnosticHandle;
osMessageQId canTxQueueHandle;
osMessageQId canRxQueueHandle;
osMessageQId canSubQueueHandle;
osMessageQId sdioSubQueueHandle;
osMessageQId sdioLogQueueHandle;
osMessageQId xbeeInternalMailQueueHandle;
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
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
void StartCanGtkpTask(void const * argument);
void StartSdioGtkpTask(void const * argument);
void StartIwdgGtkpTask(void const * argument);
void StartBtRxTask(void const * argument);
void StartGnssRxTask(void const * argument);
void StartRfRxTask(void const * argument);
void StartXbeeTxRxTask(void const * argument);
void StartDiagnosticTask(void const * argument);

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
  MX_ADC1_Init();
  MX_TIM7_Init();
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
  /* definition and creation of canTxQueue */
  osMessageQDef(canTxQueue, 32, SCanFrame);
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);

  /* definition and creation of canRxQueue */
  osMessageQDef(canRxQueue, 32, SCanFrame);
  canRxQueueHandle = osMessageCreate(osMessageQ(canRxQueue), NULL);

  /* definition and creation of canSubQueue */
  osMessageQDef(canSubQueue, 32, uint32_t);
  canSubQueueHandle = osMessageCreate(osMessageQ(canSubQueue), NULL);

  /* definition and creation of sdioSubQueue */
  osMessageQDef(sdioSubQueue, 32, uint32_t);
  sdioSubQueueHandle = osMessageCreate(osMessageQ(sdioSubQueue), NULL);

  /* definition and creation of sdioLogQueue */
  osMessageQDef(sdioLogQueue, 16, const char*);
  sdioLogQueueHandle = osMessageCreate(osMessageQ(sdioLogQueue), NULL);

  /* definition and creation of xbeeInternalMailQueue */
  osMessageQDef(xbeeInternalMailQueue, 4, EXbeeInternalMail);
  xbeeInternalMailQueueHandle = osMessageCreate(osMessageQ(xbeeInternalMailQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of canGtkp */
  osThreadDef(canGtkp, StartCanGtkpTask, osPriorityNormal, 0, 128);
  canGtkpHandle = osThreadCreate(osThread(canGtkp), NULL);

  /* definition and creation of sdioGtkp */
  osThreadDef(sdioGtkp, StartSdioGtkpTask, osPriorityNormal, 0, 512);
  sdioGtkpHandle = osThreadCreate(osThread(sdioGtkp), NULL);

  /* definition and creation of iwdgGtkp */
  osThreadDef(iwdgGtkp, StartIwdgGtkpTask, osPriorityNormal, 0, 128);
  iwdgGtkpHandle = osThreadCreate(osThread(iwdgGtkp), NULL);

  /* definition and creation of btRx */
  osThreadDef(btRx, StartBtRxTask, osPriorityNormal, 0, 128);
  btRxHandle = osThreadCreate(osThread(btRx), NULL);

  /* definition and creation of gnssRx */
  osThreadDef(gnssRx, StartGnssRxTask, osPriorityNormal, 0, 128);
  gnssRxHandle = osThreadCreate(osThread(gnssRx), NULL);

  /* definition and creation of rfRx */
  osThreadDef(rfRx, StartRfRxTask, osPriorityNormal, 0, 128);
  rfRxHandle = osThreadCreate(osThread(rfRx), NULL);

  /* definition and creation of xbeeTxRx */
  osThreadDef(xbeeTxRx, StartXbeeTxRxTask, osPriorityNormal, 0, 128);
  xbeeTxRxHandle = osThreadCreate(osThread(xbeeTxRx), NULL);

  /* definition and creation of diagnostic */
  osThreadDef(diagnostic, StartDiagnosticTask, osPriorityNormal, 0, 128);
  diagnosticHandle = osThreadCreate(osThread(diagnostic), NULL);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1599;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart4.Init.BaudRate = 9600;
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
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
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
  HAL_GPIO_WritePin(RF_SPI1_CSN_GPIO_Port, RF_SPI1_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RF_PWR_UP_Pin|RF_TRX_CE_Pin|RF_TX_EN_Pin|GNSS_FORCE_ON_Pin
                          |GNSS_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : XBEE_STATUS_Pin XBEE_RSSI_Pin XBEE_RESET_Pin RF_AM_Pin */
  GPIO_InitStruct.Pin = XBEE_STATUS_Pin|XBEE_RSSI_Pin|XBEE_RESET_Pin|RF_AM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_SPI1_CSN_Pin */
  GPIO_InitStruct.Pin = RF_SPI1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SPI1_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_DR_Pin */
  GPIO_InitStruct.Pin = RF_DR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_DR_GPIO_Port, &GPIO_InitStruct);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(RF_DR_Pin == GPIO_Pin) {

		/* Notify the rfRx task */
		vTaskNotifyGiveFromISR(rfRxHandle, NULL);

	}

}

/**
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (DIAGNOSTIC_ADC_INSTANCE == hadc->Instance) {

		/* Notify the diagnostic task */
		vTaskNotifyGiveFromISR(diagnosticHandle, NULL);

	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCanGtkpTask */
/**
 * @brief Function implementing the canGtkp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanGtkpTask */
void StartCanGtkpTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/* Wait for SDIO gatekeeper to test if there is a valid telemetry subscription stored on the SD card */
	canGtkp_WaitSubscriptionFromSdioGtkp();

	/* Start the CAN module */
	(void) HAL_CAN_Start(&hcan1);

	/* Infinite loop */
	for (;;) {

		/* Check for outgoing messages */
		canGtkp_HandleOutbox();

		/* Check for incoming messages */
		canGtkp_HandleInbox();

		/* Check for new telemetry subscription */
		canGtkp_HandleNewSubscription();

		/* Report to watchdog */
		WATCHDOG_CHECKIN(NV_IWDGGTKP_CANGTKP);

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSdioGtkpTask */
/**
 * @brief Function implementing the sdioGtkp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdioGtkpTask */
void StartSdioGtkpTask(void const * argument)
{
  /* USER CODE BEGIN StartSdioGtkpTask */

	static FATFS fatFs; /* File system object structure */
	/* Try mounting the logical drive */
	if (FR_OK != f_mount(&fatFs, SDPath, 1)) {

		/* On f_mount failure, suspend the task indefinitely */
		vTaskSuspend(NULL);

	}

	/* Try loading the telemetry subscription from the SD card */
	uint32_t nv = sdioGtkp_LoadTelemetrySubscription();
	/* Notify sdioGtkp of the result */
	(void) xTaskNotify(sdioGtkpHandle, nv, eSetValueWithOverwrite);

	/* Infinite loop */
	for (;;) {

		/* Listen for incoming error messages */
		sdioGtkp_HandleLogger();

		/* Listen for new telemetry subscription */
		sdioGtkp_HandleNewSubscription();

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartSdioGtkpTask */
}

/* USER CODE BEGIN Header_StartIwdgGtkpTask */
/**
 * @brief  Function implementing the iwdgGtkp thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIwdgGtkpTask */
void StartIwdgGtkpTask(void const * argument)
{
  /* USER CODE BEGIN StartIwdgGtkpTask */

	/* Give the other tasks time to set up */
	vTaskDelay(pdMS_TO_TICKS(1000));

	/* Initialize the watchdog */
	(void) HAL_IWDG_Init(&hiwdg);

	/* Infinite loop */
	for (;;) {

		static uint32_t nv;
		/* Wait for notification */
		if (pdTRUE == xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY,
		CLEAR_NO_BITS_ON_EXIT, &nv,
		portMAX_DELAY)) {

			/* If all tasks checked in */
			if (nv
					== (NV_IWDGGTKP_BTRX | NV_IWDGGTKP_XBEETXRX
							| NV_IWDGGTKP_GNSSRX | NV_IWDGGTKP_RFRX
							| NV_IWDGGTKP_CANGTKP)) {

				/* Refresh the counter */
				(void) HAL_IWDG_Refresh(&hiwdg);
				/* Clear the notification value */
				CLEARNV();

			}

		}

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartIwdgGtkpTask */
}

/* USER CODE BEGIN Header_StartBtRxTask */
/**
 * @brief Function implementing the btRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBtRxTask */
void StartBtRxTask(void const * argument)
{
  /* USER CODE BEGIN StartBtRxTask */

	/* Start listening for incoming data */
	btRx_StartCircularBufferIdleDetectionRx();

	/* Infinite loop */
	for (;;) {

		/* Handle for the message */
		btRx_HandleMessage();

		/* Report to watchdog */
		WATCHDOG_CHECKIN(NV_IWDGGTKP_BTRX);

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartBtRxTask */
}

/* USER CODE BEGIN Header_StartGnssRxTask */
/**
 * @brief Function implementing the gnssRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGnssRxTask */
void StartGnssRxTask(void const * argument)
{
  /* USER CODE BEGIN StartGnssRxTask */

	/* Configure the device */
	gnssRx_DeviceConfig();

	/* Start listening for incoming data */
	gnssRx_StartCircularBufferIdleDetectionRx();

	/* Infinite loop */
	for (;;) {

		/* Handle for the message */
		gnssRx_HandleMessage();

		/* Report to watchdog */
		WATCHDOG_CHECKIN(NV_IWDGGTKP_GNSSRX);

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartGnssRxTask */
}

/* USER CODE BEGIN Header_StartRfRxTask */
/**
 * @brief Function implementing the rfRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRfRxTask */
void StartRfRxTask(void const * argument)
{
  /* USER CODE BEGIN StartRfRxTask */

	/* Configure the device */
	rfRx_DeviceConfig();

	/* Infinite loop */
	for (;;) {

		/* Listen for the message */
		rfRx_HandleMessage();

		/* Report to watchdog */
		WATCHDOG_CHECKIN(NV_IWDGGTKP_RFRX);

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartRfRxTask */
}

/* USER CODE BEGIN Header_StartXbeeTxRxTask */
/**
 * @brief Function implementing the xbeeTxRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeTxRxTask */
void StartXbeeTxRxTask(void const * argument)
{
  /* USER CODE BEGIN StartXbeeTxRxTask */

	/* Configure the device */
	xbeeTxRx_DeviceConfig();

	/* Start listening for incoming data */
	xbeeTxRx_StartCircularBufferIdleDetectionRx();

	/* Start the timer */
	HAL_TIM_Base_Start_IT(&TIM_1s_HANDLE);

	/* Infinite loop */
	for (;;) {

		/* Listen for internal communication */
		xbeeTxRx_HandleInternalMail();

		/* Poll the queue for CAN frames to send */
		xbeeTxRx_HandleOutgoingR3tpComms();

		/* Report to watchdog */
		WATCHDOG_CHECKIN(NV_IWDGGTKP_XBEETXRX);

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

	}

  /* USER CODE END StartXbeeTxRxTask */
}

/* USER CODE BEGIN Header_StartDiagnosticTask */
/**
 * @brief Function implementing the diagnostic thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDiagnosticTask */
void StartDiagnosticTask(void const * argument)
{
  /* USER CODE BEGIN StartDiagnosticTask */

	/* Infinite loop */
	for (;;) {

		/* Run diagnostics */
		diagnostic_RunDiagnostics();

		/* Sleep for one second */
		vTaskDelay(pdMS_TO_TICKS(1000));

	}

  /* USER CODE END StartDiagnosticTask */
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

	if (TIM_1s_INSTANCE == htim->Instance) {

		/* Notify the xbeeTxRx task */
		EXbeeInternalMail mail = EXbeeInternalMail_PeriodElapsed;
		xQueueSendFromISR(xbeeInternalMailQueueHandle, &mail, NULL);

	}

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
