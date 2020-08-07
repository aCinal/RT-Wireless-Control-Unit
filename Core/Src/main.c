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
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Intuitive names for peripheral instances */
#define BT_UART_INSTANCE		USART1
#define GNSS_UART_INSTANCE		USART3
#define XBEE_UART_INSTANCE		UART4
#define RF_SPI_INSTANCE			SPI1
#define TEMPSENSOR_ADC_INSTANCE	ADC1

/* Intuitive names for peripheral handles */
#define BT_UART_HANDLE			huart1
#define GNSS_UART_HANDLE		huart3
#define XBEE_UART_HANDLE		huart4
#define RF_SPI_HANDLE			hspi1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief Logs an error message to the SD card
 */
#define LOGERROR(message) do { \
	\
	/* Allocate the memory for the error message */ \
	char* errMsg = pvPortMalloc(WCU_LOGGER_TIMESTAMP_SIZE + strlen(message) + 1U); \
	/* Assert successful memory allocation */ \
	if(errMsg != NULL) { \
		\
		/* Write the timestamp to the memory block */ \
		(void) sprintf(errMsg, "%010lu ", HAL_GetTick()); \
		/* Write the message to the memory block */ \
		(void) sprintf(errMsg + WCU_LOGGER_TIMESTAMP_SIZE, message); \
		/* Push the pointer to the message to the logErrorQueue */ \
		if(pdPASS != xQueueSend(sdioLogQueueHandle, &errMsg, WCU_SDIOLOGQUEUE_XQUEUESEND_TIMEOUT)) { \
			/* Cleanup on failure to push to queue */ \
			vPortFree(errMsg);\
		} \
		\
	} \
	\
} while(0)

/**
 * @brief Pushes a CAN frame to the canTxQueue
 */
#define ADDTOCANTXQUEUE(pCanFrame, errMsg) do { \
	\
	/* Push the frame to the queue */ \
	if (pdPASS != xQueueSend(canTxQueueHandle, pCanFrame, WCU_CANTXQUEUE_XQUEUESEND_TIMEOUT)) { \
		\
		/* Log the error */ \
		LOGERROR(errMsg); \
		\
	} \
	\
} while(0)

/**
 * @brief Checks in with the watchdog thread
 */
#define WATCHDOG_CHECKIN(notificationValue) ((void)xTaskNotify((TaskHandle_t)iwdgGtkpHandle, notificationValue, eSetBits))

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
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId iwdgGtkpHandle;
osThreadId btRxHandle;
osThreadId xbeeTxHandle;
osThreadId xbeeRxHandle;
osThreadId gnssRxHandle;
osThreadId rfRxHandle;
osThreadId canGtkpHandle;
osThreadId sdioGtkpHandle;
osThreadId diagHandle;
osMessageQId canTxQueueHandle;
osMessageQId canRxQueueHandle;
osMessageQId sdioLogQueueHandle;
osMessageQId sdioSubQueueHandle;
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
void StartIwdgGtkpTask(void const * argument);
void StartBtRxTask(void const * argument);
void StartXbeeTxTask(void const * argument);
void StartXbeeRxTask(void const * argument);
void StartGnssRxTask(void const * argument);
void StartRfRxTask(void const * argument);
void StartCanGtkpTask(void const * argument);
void StartSdioGtkpTask(void const * argument);
void StartDiagTask(void const * argument);

/* USER CODE BEGIN PFP */

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeRx_WaitSubscriptionFromSdioGtkp(void);

/**
 * @brief Receives and handles the telemetry subscription via UART
 * @param buff UART Rx buffer of size R3TP_VER1_MAX_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveSubscription(uint8_t buff[]);

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count);

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void);

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

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void);

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(FIL *fp, const TCHAR *path);

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
  /* USER CODE BEGIN 2 */

	/* Start the CAN module */
	HAL_CAN_Start(&hcan1);

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
  osMessageQDef(canTxQueue, 32, CanFrameTypedef);
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);

  /* definition and creation of canRxQueue */
  osMessageQDef(canRxQueue, 16, CanFrameTypedef);
  canRxQueueHandle = osMessageCreate(osMessageQ(canRxQueue), NULL);

  /* definition and creation of sdioLogQueue */
  osMessageQDef(sdioLogQueue, 16, const char*);
  sdioLogQueueHandle = osMessageCreate(osMessageQ(sdioLogQueue), NULL);

  /* definition and creation of sdioSubQueue */
  osMessageQDef(sdioSubQueue, 32, uint32_t);
  sdioSubQueueHandle = osMessageCreate(osMessageQ(sdioSubQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of iwdgGtkp */
  osThreadDef(iwdgGtkp, StartIwdgGtkpTask, osPriorityNormal, 0, 128);
  iwdgGtkpHandle = osThreadCreate(osThread(iwdgGtkp), NULL);

  /* definition and creation of btRx */
  osThreadDef(btRx, StartBtRxTask, osPriorityNormal, 0, 128);
  btRxHandle = osThreadCreate(osThread(btRx), NULL);

  /* definition and creation of xbeeTx */
  osThreadDef(xbeeTx, StartXbeeTxTask, osPriorityNormal, 0, 128);
  xbeeTxHandle = osThreadCreate(osThread(xbeeTx), NULL);

  /* definition and creation of xbeeRx */
  osThreadDef(xbeeRx, StartXbeeRxTask, osPriorityNormal, 0, 128);
  xbeeRxHandle = osThreadCreate(osThread(xbeeRx), NULL);

  /* definition and creation of gnssRx */
  osThreadDef(gnssRx, StartGnssRxTask, osPriorityAboveNormal, 0, 128);
  gnssRxHandle = osThreadCreate(osThread(gnssRx), NULL);

  /* definition and creation of rfRx */
  osThreadDef(rfRx, StartRfRxTask, osPriorityNormal, 0, 128);
  rfRxHandle = osThreadCreate(osThread(rfRx), NULL);

  /* definition and creation of canGtkp */
  osThreadDef(canGtkp, StartCanGtkpTask, osPriorityHigh, 0, 128);
  canGtkpHandle = osThreadCreate(osThread(canGtkp), NULL);

  /* definition and creation of sdioGtkp */
  osThreadDef(sdioGtkp, StartSdioGtkpTask, osPriorityNormal, 0, 1024);
  sdioGtkpHandle = osThreadCreate(osThread(sdioGtkp), NULL);

  /* definition and creation of diag */
  osThreadDef(diag, StartDiagTask, osPriorityBelowNormal, 0, 128);
  diagHandle = osThreadCreate(osThread(diag), NULL);

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_RESET);

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
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (TEMPSENSOR_ADC_INSTANCE == hadc->Instance) {

		/* Resume the selfDiagnostic task */
		vTaskNotifyGiveFromISR(diagHandle, NULL);

	}

}

/**
 * @brief  Rx Transfer completed callback.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (RF_SPI_INSTANCE == hspi->Instance) {

		/* Resume the rfRx task */
		vTaskNotifyGiveFromISR(rfRxHandle, NULL);

	}

}

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

	if (XBEE_UART_INSTANCE == huart->Instance) {

		/* Resume the xbeeTx task */
		vTaskNotifyGiveFromISR(xbeeTxHandle, NULL);

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

		/* Notify the btRx task */
		vTaskNotifyGiveFromISR(btRxHandle, NULL);
		break;

	case (uint32_t) GNSS_UART_INSTANCE:

		/* Notify the gnssRx task */
		vTaskNotifyGiveFromISR(gnssRxHandle, NULL);
		break;

	case (uint32_t) XBEE_UART_INSTANCE:

		/* Notify the xbeeRx task */
		vTaskNotifyGiveFromISR(xbeeRxHandle, NULL);
		break;

	}

}

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid subscription stored on the SD card
 * @retval None
 */
void xbeeRx_WaitSubscriptionFromSdioGtkp(void) {

	uint32_t frameNum; /* Number of frames in a subscription */
	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	uint32_t notificationValue; /* Buffer for the notification value */

	/* Wait for sdioGtkp to notify the task if there is a valid subscription stored on the SD card */
	if (pdTRUE
			== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL, &notificationValue,
			WCU_XBEERX_XTASKNOTIFYWAIT_TIMEOUT)) {

		if (notificationValue <= 28UL) {

			/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
			frameNum = notificationValue;

			for (uint32_t i = 0; i < frameNum; i += 1UL) {

				if (pdPASS
						!= xQueueReceive(sdioSubQueueHandle, subscription + i,
						WCU_SDIOSUBQUEUE_XQUEUERECEIVE_TIMEOUT)) {

					/* Log the error and return */
					LOGERROR("xbeeRx failed to receive from sdioSubQueue\r\n");
					return;

				}

			}

			/* Set the CAN filters */
			setCanFilterList(&hcan1, subscription, frameNum);

		} else {

			/* Log the error */
			switch (notificationValue) {

			case WCU_XBEERX_NV_FOPENFAILED:

				LOGERROR("sdioGtkp failed to open the subscription file\r\n");
				break;

			case WCU_XBEERX_NV_FREADFAILED:

				LOGERROR(
						"sdioGtkp failed to read from the subscription file\r\n");
				break;

			case WCU_XBEERX_NV_INVALIDFRAMENUM:

				LOGERROR("Invalid FRAME NUM in the subscription file\r\n");
				break;

			case WCU_XBEERX_NV_XQUEUESENDFAILED:

				LOGERROR("sdioGtkp failed to send to sdioSubQueue\r\n");
				break;

			default:

				LOGERROR(
						"Invalid FRAME NUM in xbeeRx_WaitSubscriptionSdioGtkp\r\n");
				break;

			}

		}

	}

}

/**
 * @brief Receives and handles the telemetry subscription via UART
 * @param buff UART Rx buffer of size R3TP_VER1_MAX_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveSubscription(uint8_t buff[]) {
	/* Receive SEQ NUM, CHECKSUM and FRAME NUM */
	if (HAL_OK != HAL_UART_Receive(&XBEE_UART_HANDLE, buff + 1U, 7,
	WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive SEQ NUM, CHECKSUM and FRAME NUM in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Read the FRAME NUM field */
	uint32_t frameNum = READ32(buff[7], buff[6], buff[5], buff[4]);

	/* Assert the payload won't overflow the buffer */
	if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Log the error */
		LOGERROR("Invalid FRAME NUM in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1U,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {

			__NOP();

		}
		return;

	}

	/* Receive the payload */
	if (HAL_OK
			!= HAL_UART_Receive(&XBEE_UART_HANDLE,
					R3TP_VER1_PAYLOAD_BEGIN(buff), frameNum * 4U,
					WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive the payload in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Receive the frame align bytes (two) and END SEQ (also two bytes) */
	if (HAL_OK
			!= HAL_UART_Receive(&XBEE_UART_HANDLE,
					R3TP_VER1_EPILOGUE_BEGIN(buff, frameNum), 4U,
					WCU_XBEERX_UART_TIMEOUT)) {

		/* Log the error */
		LOGERROR(
				"Failed to receive END SEQ in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Validate the END SEQ field */
	if ((R3TP_END_SEQ_LOW_BYTE != buff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 2U])
			|| (R3TP_END_SEQ_HIGH_BYTE
					!= buff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 1U])) {

		/* Log the error */
		LOGERROR("Invalid END SEQ in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1U,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {
			__NOP();
		}
		return;

	}

	/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
	uint16_t readCrc = READ16(buff[3], buff[2]);

	/* Clear the CHECKSUM field */
	buff[2] = 0x00U;
	buff[3] = 0x00U;

	/* Get the CRC mutex */
	if (osOK != osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {

		/* Log the error */
		LOGERROR("crcMutex timeout in xbeeRx_UartReceiveSubscription\r\n");
		return;

	}

	/* Calculate the CRC */
	uint16_t calculatedCrc =
			TWOLOWBYTES(
					HAL_CRC_Calculate(&hcrc, (uint32_t*)buff, R3TP_VER1_MESSAGE_LENGTH(frameNum) / 4U));

	/* Release the CRC mutex */
	(void) osMutexRelease(crcMutexHandle);

	/* Validate the CRC */
	if (readCrc != calculatedCrc) {

		/* Log the error */
		LOGERROR("Invalid CRC in xbeeRx_UartReceiveSubscription\r\n");
		/* Assert the invalid message won't raise any more interrupts */
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, buff, 1,
		WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {

			__NOP();

		}
		return;

	}

	uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */
	/* Read the payload */
	for (uint32_t i = 0; i < frameNum; i += 1UL) {

		subscription[i] = READ32(
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 3U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 2U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 1U + 4U * i)),
				*(R3TP_VER1_PAYLOAD_BEGIN_OFFSET(buff, 4U * i)));

	}

	/* Forward the subscription to the sdioGtkp */
	(void) xbeeRx_SendSubscriptionToSdioGtkp(subscription, frameNum);

	/* Set the CAN filters */
	setCanFilterList(&hcan1, subscription, frameNum);

}

/**
 * @brief Receives and handles the driver warning R3TP frame
 * @param buff UART Rx buffer of size R3TP_VER2_FRAME_SIZE
 * @retval None
 */
void xbeeRx_UartReceiveDriverWarning(uint8_t buff[]) {

	/*
	 * TODO
	 */

}

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count) {

	BaseType_t ret = pdPASS; /* Buffer for xQueueSend return value */

	/* Write subscription to sdioSubscriptionQueue */
	for (uint32_t i = 0; i < count; i += 1UL) {

		ret = xQueueSend(sdioSubQueueHandle, ids + i,
				WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT);
		/* Send the frame to the queue */
		if (pdPASS != ret) {

			/* Log the error */
			LOGERROR("xbeeRx failed to send to sdioSubQueue\r\n");
			/* Cleanup */
			(void) xQueueReset(sdioSubQueueHandle);
			return ret;

		}

	}

	/* Notify sdioGatekeeper */
	(void) xTaskNotify(sdioGtkpHandle, count, eSetValueWithOverwrite);

	return ret;
}

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void) {

	/* Wait for the device to turn on and set up */
	vTaskDelay(WCU_GNSSRX_DEVICECONFIG_SETUP_DELAY);

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	const char PMTK_SET_POS_FIX[] = "$PMTK220,100*1F\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) PMTK_SET_POS_FIX,
					sizeof(PMTK_SET_POS_FIX),
					WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT)) {

		Error_Handler();

	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*2B\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE,
					(uint8_t*) PMTK_API_SET_GNSS_SEARCH_MODE,
					sizeof(PMTK_API_SET_GNSS_SEARCH_MODE),
					WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT)) {

		Error_Handler();

	}

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_POS;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

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

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS2(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 6;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_POS2;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

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

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS2 failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_STATUS(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_GPS_STATUS;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canFrame.Payload[0] = pData->SatellitesInViewGLONASS
			+ pData->SatellitesInViewGPS;
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

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_STATUS failed to send to canTxQueue\r\n");

}

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void) {

	/*
	 * TODO
	 */

}

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(FIL *fp, const TCHAR *path) {

	/* Try opening the file */
	if (FR_OK != f_open(fp, path, FA_READ | FA_OPEN_EXISTING)) {

		/* If failed to open the file */
		return WCU_XBEERX_NV_FOPENFAILED;

	}

	uint32_t frameNum; /* Buffer for the number of frames */
	uint32_t frameBuff; /* Buffer for a subscription frame */
	uint8_t temp[4]; /* Temporary buffer for four bytes to be read as a single 32-bit little endian value */
	UINT bytesRead; /* Buffer for the number of bytes read */

	/* Try reading the number of frames */
	if(FR_OK != f_read(fp, temp, 4, &bytesRead)) {

		/* If failed to read the number of frames */
		/* Cleanup */
		(void) f_close(fp);
		return WCU_XBEERX_NV_FREADFAILED;

	}

	/* Parse the number of frames */
	frameNum = READ32(temp[3], temp[2], temp[1], temp[0]);

	/* Validate the number of frames */
	if(frameNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Cleanup */
		(void) f_close(fp);
		return WCU_XBEERX_NV_INVALIDFRAMENUM;

	}

	/* Read the payload and push it to the queue */
	for (uint32_t i = 0; i < frameNum; i += 1UL) {

		/* Try reading the frame */
		if(FR_OK != f_read(fp, temp, 4, &bytesRead)) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			return WCU_XBEERX_NV_FREADFAILED;

		}

		/* Assert end of file was not reached */
		if (bytesRead < 4U) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			return WCU_XBEERX_NV_INVALIDFRAMENUM;

		}

		/* Parse the frame */
		frameBuff = READ32(temp[3], temp[2], temp[1], temp[0]);

		/* Send the frame to the queue */
		if (pdPASS
				!= xQueueSend(sdioSubQueueHandle, &frameBuff,
						WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT)) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			return WCU_XBEERX_NV_XQUEUESENDFAILED;

		}

	}

	/* Cleanup */
	(void) f_close(fp);
	/* Return the number of frames read */
	return frameNum;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartIwdgGtkpTask */
/**
 * @brief  Function implementing the iwdgGtkp thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIwdgGtkpTask */
void StartIwdgGtkpTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	static uint32_t notificationValue; /* Buffer to pass the notification value out of the xTaskNotifyWait function */

	/* Give the other tasks time to set up */
	vTaskDelay(WCU_IWDGGTKP_INIT_DELAY);

	/* Initialize the watchdog */
	(void) HAL_IWDG_Init(&hiwdg);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		/* Wait for notification */
		if (pdTRUE
				== xTaskNotifyWait(0x00000000UL, 0x00000000UL,
						&notificationValue,
						WCU_IWDGGTKP_XTASKNOTIFYWAIT_TIMEOUT)) {

			/* If all tasks checked in */
			if (notificationValue
					== (WCU_IWDGGTKP_NV_BTRX | WCU_IWDGGTKP_NV_XBEETX
							| WCU_IWDGGTKP_NV_GNSSRX | WCU_IWDGGTKP_NV_RFRX
							| WCU_IWDGGTKP_NV_CANGTKP)) {

				/* Refresh the counter */
				(void) HAL_IWDG_Refresh(&hiwdg);
				/* Clear the notification value */
				(void) xTaskNotifyWait(0xFFFFFFFFUL, 0xFFFFFFFFUL,
						&notificationValue, 0);

			}

		}

	}

  /* USER CODE END 5 */
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

	static CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	static uint8_t uartRxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Rx buffer */
	/* Listen for the message */
	(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
	R3TP_VER0_FRAME_SIZE);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_BTRX_ULTASKNOTIFYTAKE_TIMEOUT)) {

			/* Validate the VER and RES/SEQ field */
			if (R3TP_VER0_VER_RES_SEQ_BYTE != uartRxBuff[0]) {

				LOGERROR("Invalid VER/RES/SEQ in btRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
				R3TP_VER0_FRAME_SIZE);
				continue;

			}

			/* Validate the END SEQ field */
			if ((R3TP_END_SEQ_LOW_BYTE != uartRxBuff[R3TP_VER0_FRAME_SIZE - 2U])
					|| (R3TP_END_SEQ_HIGH_BYTE
							!= uartRxBuff[R3TP_VER0_FRAME_SIZE - 1U])) {

				LOGERROR("Invalid END SEQ in btRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
				R3TP_VER0_FRAME_SIZE);
				continue;

			}

			static uint16_t readCrc; /* Buffer for the transmitted CRC */
			/* Read the CHECKSUM field - note that the CRC is transmitted as little endian */
			readCrc = READ16(uartRxBuff[3], uartRxBuff[2]);

			/* Clear the CHECKSUM field */
			uartRxBuff[2] = 0x00U;
			uartRxBuff[3] = 0x00U;



			/* Get the CRC mutex */
			if (osOK != osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {

				/* Log the error */
				LOGERROR("crcMutex timeout in btRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
				R3TP_VER0_FRAME_SIZE);
				continue;

			}

			/* Calculate the CRC */
			uint16_t calculatedCrc =
					TWOLOWBYTES(
							HAL_CRC_Calculate(&hcrc, (uint32_t* )uartRxBuff, R3TP_VER0_FRAME_SIZE / 4U));

			/* Release the CRC mutex */
			(void) osMutexRelease(crcMutexHandle);

			/* Validate the CRC */
			if (readCrc != calculatedCrc) {

				LOGERROR("Invalid CRC in btRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
				R3TP_VER0_FRAME_SIZE);
				continue;

			}

			/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
			canFrame.Header.Tx.StdId = READ32(uartRxBuff[7], uartRxBuff[6],
					uartRxBuff[5], uartRxBuff[4]);
			/* Read the Data Length Code */
			canFrame.Header.Tx.DLC = (uint32_t) uartRxBuff[8];
			if (CAN_PAYLOAD_SIZE < canFrame.Header.Tx.DLC) {

				LOGERROR("Invalid DLC in btRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
				R3TP_VER0_FRAME_SIZE);
				continue;

			}

			/* Read the payload */
			for (uint8_t i = 0; i < canFrame.Header.Tx.DLC; i += 1U) {

				canFrame.Payload[i] = uartRxBuff[9U + i];

			}

			/* Transmit the frame */
			ADDTOCANTXQUEUE(&canFrame, "btRx failed to send to canTxQueue\r\n");

			/* Listen for the next message */
			(void) HAL_UART_Receive_DMA(&BT_UART_HANDLE, uartRxBuff,
			R3TP_VER0_FRAME_SIZE);

		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGGTKP_NV_BTRX);

	}

  /* USER CODE END StartBtRxTask */
}

/* USER CODE BEGIN Header_StartXbeeTxTask */
/**
 * @brief Function implementing the xbeeTx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeTxTask */
void StartXbeeTxTask(void const * argument)
{
  /* USER CODE BEGIN StartXbeeTxTask */

	/* Activate XBEE Pro by driving the XBEE_RESET pin high */
	HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		static CanFrameTypedef frameBuff; /* CAN frame buffer */
		/* Listen on the canRxQueue for messages to send */
		if (pdPASS == xQueueReceive(canRxQueueHandle, &frameBuff,
		WCU_CANRXQUEUE_XQUEUERECEIVE_TIMEOUT)) {
			/* Assert valid data direction */
			if (RX != frameBuff.DataDirection) {

				/* Log the error */
				LOGERROR("Invalid DataDirection in xbeeTx\r\n");
				continue;

			}

			static uint8_t uartTxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Tx buffer */
			/* Clear the buffer */
			(void) memset(uartTxBuff, 0x00U, R3TP_VER0_FRAME_SIZE);

			/* Set VER and RES/SEQ field */
			uartTxBuff[0] = R3TP_VER0_VER_RES_SEQ_BYTE;

			static uint8_t seqNum = 0U; /* Sequence number */
			/* Set the SEQ NUM field */
			uartTxBuff[1] = seqNum;
			/* Increment the sequence number */
			seqNum = (seqNum < 255U) ? seqNum + 1U : 0U;

			/* Set the END SEQ field */
			uartTxBuff[R3TP_VER0_FRAME_SIZE - 2U] =
			R3TP_END_SEQ_LOW_BYTE;
			uartTxBuff[R3TP_VER0_FRAME_SIZE - 1U] =
			R3TP_END_SEQ_HIGH_BYTE;

			/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
			uartTxBuff[4] = LSB(frameBuff.Header.Rx.StdId);
			uartTxBuff[5] = MSB16(frameBuff.Header.Rx.StdId);

			/* Set the DLC field */
			uartTxBuff[8] = (uint8_t) frameBuff.Header.Rx.DLC;

			/* Set the DATA field */
			for (uint8_t i = 0; i < frameBuff.Header.Rx.DLC; i += 1U) {

				uartTxBuff[9U + i] = frameBuff.Payload[i];

			}

			/* Get the CRC mutex */
			if (osOK != osMutexWait(crcMutexHandle, WCU_CRCMUTEX_TIMEOUT)) {

				/* Log the error */
				LOGERROR("crcMutex timeout in xbeeTx\r\n");
				continue;

			}

			/* Calculate the CRC */
			uint16_t calculatedCrc =
					TWOLOWBYTES(
							HAL_CRC_Calculate(&hcrc, (uint32_t*)uartTxBuff, R3TP_VER0_FRAME_SIZE / 4U));

			/* Release the CRC mutex */
			(void) osMutexRelease(crcMutexHandle);

			/* Set the CHECKSUM field - note that the CRC is transmitted as little endian */
			uartTxBuff[2] = LSB(calculatedCrc);
			uartTxBuff[3] = MSB16(calculatedCrc);

			/* Transmit the frame */
			(void) HAL_UART_Transmit_DMA(&XBEE_UART_HANDLE, uartTxBuff,
			R3TP_VER0_FRAME_SIZE);

			/* Wait for the transmission to end */
			if (0UL < ulTaskNotifyTake(pdTRUE,
			WCU_XBEETX_ULTASKNOTIFYTAKE_TIMEOUT)) {

				/* Log the error */
				LOGERROR(
						"xbeeTx failed to receive notification from TxCpltCallback\r\n");

			}

		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGGTKP_NV_XBEETX);

	}

  /* USER CODE END StartXbeeTxTask */
}

/* USER CODE BEGIN Header_StartXbeeRxTask */
/**
 * @brief Function implementing the xbeeRx thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeRxTask */
void StartXbeeRxTask(void const * argument)
{
  /* USER CODE BEGIN StartXbeeRxTask */

	/* Wait for SDIO gatekeeper to test if there is a valid subscription stored on the SD card */
	xbeeRx_WaitSubscriptionFromSdioGtkp();

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		static uint8_t uartRxBuff[R3TP_VER1_MAX_FRAME_SIZE]; /* UART Rx buffer */
		/* Listen for the subscription frame VER octet */
		(void) HAL_UART_Receive_DMA(&XBEE_UART_HANDLE, uartRxBuff, 1);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_XBEERX_ULTASKNOTIFYTAKE_TIMEOUT)) {

			/* Identify the protocol version */
			switch (uartRxBuff[0]) {
			case R3TP_VER1_VER_RES_SEQ_BYTE: /* New subscription */

				xbeeRx_UartReceiveSubscription(uartRxBuff);
				break;

			case R3TP_VER2_VER_RES_SEQ_BYTE: /* Warning for the driver */

				xbeeRx_UartReceiveDriverWarning(uartRxBuff);
				break;

			default:

				/* Log the error */
				LOGERROR("Invalid VER/RES/SEQ in xbeeRx\r\n");
				/* Assert the invalid message won't raise any more interrupts */
				while (HAL_OK
						== HAL_UART_Receive(&XBEE_UART_HANDLE, uartRxBuff, 1U,
						WCU_XBEERX_UART_CLEANUP_TIMEOUT)) {

					__NOP();

				}
				break;

			}

		}

	}

  /* USER CODE END StartXbeeRxTask */
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

	static uint8_t uartRxBuff[WCU_GNSSRX_UARTRXBUFF_SIZE]; /* UART Rx buffer */
	/* Listen for the message */
	(void) HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, uartRxBuff,
	WCU_GNSSRX_UARTRXBUFF_SIZE);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_GNSSRX_ULTASKNOTIFYTAKE_TIMEOUT)) {

			static GnssDataTypedef dataBuff; /* GNSS data buffer */
			/* Try parsing the message */
			switch (parseMessage(&dataBuff, (char*) uartRxBuff,
			WCU_GNSSRX_UARTRXBUFF_SIZE)) {

			case GNSS_DATA_READY: /* If the data is ready */

				/* Send the data to CAN */
				gnssRx_Send_GPS_POS(&dataBuff);
				gnssRx_Send_GPS_POS2(&dataBuff);
				gnssRx_Send_GPS_STATUS(&dataBuff);

				/* Clear the data buffer */
				(void) memset(&dataBuff, 0x00, sizeof(dataBuff));

				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, uartRxBuff,
				WCU_GNSSRX_UARTRXBUFF_SIZE);
				break;

			case GNSS_DATA_PENDING: /* If the data is not complete */

				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, uartRxBuff,
				WCU_GNSSRX_UARTRXBUFF_SIZE);
				break;

			case GNSS_DATA_ERROR: /* If the parser failed */

				/* Log the error */
				LOGERROR("parseMessage failed in gnssRx\r\n");
				/* Listen for the next message */
				(void) HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, uartRxBuff,
				WCU_GNSSRX_UARTRXBUFF_SIZE);
				break;

			}

		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGGTKP_NV_GNSSRX);

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
	uint8_t spiRxBuff[WCU_RFRX_SPIRXBUFF_SIZE]; /* SPI Rx buffer */

	/* Configure the device */
	rfRx_DeviceConfig();

	/* Listen for the message */
	(void) HAL_SPI_Receive_DMA(&RF_SPI_HANDLE, spiRxBuff,
	WCU_RFRX_SPIRXBUFF_SIZE);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		/* Wait for notify from ISR/message received callback */
		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_RFRX_ULTASKNOTIFYTAKE_TIMEOUT)) {

			/*
			 * TODO: Decode the message
			 */

		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGGTKP_NV_RFRX);

	}
  /* USER CODE END StartRfRxTask */
}

/* USER CODE BEGIN Header_StartCanGtkpTask */
/**
 * @brief Function implementing the canGtkp thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanGtkpTask */
void StartCanGtkpTask(void const * argument)
{
  /* USER CODE BEGIN StartCanGtkpTask */

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		static CanFrameTypedef frameBuff; /* CAN frame buffer */

		/* Check for outgoing messages */
		if (pdPASS == xQueueReceive(canTxQueueHandle, &frameBuff,
		WCU_CANTXQUEUE_XQUEUERECEIVE_TIMEOUT)) {

			/* Validate the DataDirection member */
			if (TX == frameBuff.DataDirection) {

				static uint32_t dummy; /* CAN Tx mailbox */

				/* Send the message */
				(void) HAL_CAN_AddTxMessage(&hcan1, &frameBuff.Header.Tx,
						frameBuff.Payload, &dummy);

			} else {

				/* Log the error */
				LOGERROR("Invalid DataDirection in canGtkp\r\n");

			}

		}

		/* Check for incoming messages */
		if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

			/* Receive the message */
			(void) HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,
					&frameBuff.Header.Rx, frameBuff.Payload);
			/* Set the DataDirection member in the CAN frame struct */
			frameBuff.DataDirection = RX;
			/* Send the frame to the telemetry queue */
			if (pdPASS
					!= xQueueSend(canRxQueueHandle, &frameBuff,
							WCU_CANRXQUEUE_XQUEUESEND_TIMEOUT)) {

				/* Log the error */
				LOGERROR("canGtkp failed to send to canRxQueue\r\n");

			}

		}

		/* Report to watchdog */
		WATCHDOG_CHECKIN(WCU_IWDGGTKP_NV_CANGTKP);

	}

  /* USER CODE END StartCanGtkpTask */
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

		/* On f_mount failure, suspend the task */
		vTaskSuspend(NULL);

	}

	static uint32_t notificationValue; /* Buffer for the notification value */
	static FIL subscriptionFile; /* Telemetry subscription file object structure */

	/* Try loading the telemetry subscription from the SD card */
	notificationValue = sdioGtkp_LoadTelemetrySubscription(&subscriptionFile,
	WCU_SDIOGTKP_SUBFILE_PATH);
	/* Notify xbeeRx of the result */
	(void) xTaskNotify(xbeeRxHandle, notificationValue, eSetValueWithOverwrite);

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DEFAULT_TASK_DELAY);

		static char *errorLogBuff; /* Buffer for the pointer to the error message */

		/* Listen for incoming error messages */
		if (pdPASS == xQueueReceive(sdioLogQueueHandle, &errorLogBuff,
		WCU_SDIOLOGQUEUE_XQUEUERECEIVE_TIMEOUT)) {

			static FIL errorLogFile; /* Error log file object structure */

			/* Try opening the file */
			if (FR_OK == f_open(&errorLogFile, WCU_SDIOGTKP_LOGFILE_PATH,
			FA_WRITE | FA_OPEN_APPEND)) {

				UINT bytesWritten; /* Buffer for the number of bytes written */
				/* Write the error message to the file */
				(void) f_write(&errorLogFile, errorLogBuff,
						strlen(errorLogBuff), &bytesWritten);
				/* Close the file */
				(void) f_close(&errorLogFile);
				/* Free the allocated memory */
				vPortFree(errorLogBuff);
				errorLogBuff = NULL;

			}

		}

		/* Listen for notifications from xbeeSubscribe */
		if (pdTRUE
				== xTaskNotifyWait(0x00000000UL, 0xFFFFFFFFUL,
						&notificationValue,
						WCU_SDIOGTKP_XTASKNOTIFYWAIT_TIMEOUT)) {

			/* Validate the notification */
			if (28UL < notificationValue) {

				/* Log the error and continue */
				LOGERROR("Invalid notification value in sdioGtkp\r\n");
				continue;

			}

			/* Try opening the file */
			if (FR_OK
					!= f_open(&subscriptionFile, WCU_SDIOGTKP_SUBFILE_PATH,
							FA_WRITE | FA_CREATE_ALWAYS)) {

				/* Log the error and continue */
				LOGERROR("sdioGtkp failed to open the subscription file\r\n");
				continue;

			}

			uint32_t frameBuff; /* Buffer for a subscription frame */
			uint8_t temp[4]; /* Temporary buffer to facilitate transmitting a 32-bit little endian value */
			UINT bytesWritten; /* Buffer for the number of bytes written */

			/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
			/* Print the number of frames to the SD card */
			temp[0] = LSB(notificationValue);
			temp[1] = LOWMID32(notificationValue);
			temp[2] = HIGHMID32(notificationValue);
			temp[3] = MSB32(notificationValue);
			(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

			/* Print the subscription to the file */
			for (uint32_t i = 0; i < notificationValue; i += 1UL) {

				if (pdPASS != xQueueReceive(sdioSubQueueHandle, &frameBuff,
				WCU_SDIOSUBQUEUE_XQUEUERECEIVE_TIMEOUT)) {

					/* Log the error and break */
					LOGERROR(
							"sdioGtkp failed to receive from sdioSubQueue\r\n");
					break;

				}

				/* Print the frame to the SD card */
				temp[0] = LSB(frameBuff);
				temp[1] = LOWMID32(frameBuff);
				temp[2] = HIGHMID32(frameBuff);
				temp[3] = MSB32(frameBuff);
				(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

			}

			/* Close the file */
			(void) f_close(&subscriptionFile);

		}

	}

  /* USER CODE END StartSdioGtkpTask */
}

/* USER CODE BEGIN Header_StartDiagTask */
/**
 * @brief Function implementing the diag thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDiagTask */
void StartDiagTask(void const * argument)
{
  /* USER CODE BEGIN StartDiagTask */

	static CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 4;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CANID_WCU_DIAG;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	/* Infinite loop */
	for (;;) {

		vTaskDelay(WCU_DIAG_TASK_DELAY);

		static uint16_t temperatureSensorAdcBuff; /* Buffer for the result of the temperature sensor ADC conversion */
		/* Start the ADC */
		(void) HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &temperatureSensorAdcBuff,
				1);

		if (0UL < ulTaskNotifyTake(pdTRUE,
		WCU_DIAG_ULTASKNOTIFYTAKE_TIMEOUT)) {

			typedef float float32_t; /* 32-bit floating-point type typedef */

			/* Temperature sensor characteristics */
			static const float32_t VDD = 3.3; /* Supply voltage */
			static const float32_t V25 = 0.76; /* Voltage at 25 degreeC */
			static const float32_t Avg_Slope = 2.5 / 1000.0; /* Average slope: 2.5 mV/degreeC */

			/* Calculate the sensed voltage */
			float32_t Vsense = temperatureSensorAdcBuff / 4095.0 * VDD;

			/* Calculate the MCU temperature based on the sensed voltage */
			float32_t floatTemperature = ((Vsense - V25) / Avg_Slope) + 25.0;

			static int16_t mcuTemperature; /* Buffer for the MCU temperature in degrees Celsius times 10 */
			/* Normalize the temperature to fit it in the CAN frame */
			mcuTemperature = (int16_t) lround(floatTemperature * 10.0);

			static uint16_t mcuUptime; /* Buffer for the MCU uptime in seconds */
			/* Calculate the MCU uptime in seconds */
			mcuUptime = (uint16_t) (HAL_GetTick() / 1000UL);

			/* Write the MCU temperature to the frame payload */
			canFrame.Payload[0] = MSB16(mcuTemperature);
			canFrame.Payload[1] = LSB(mcuTemperature);

			/* Write the MCU uptime to the frame payload */
			canFrame.Payload[2] = MSB16(mcuUptime);
			canFrame.Payload[3] = LSB(mcuUptime);

			/* Push CAN frame to queue */
			ADDTOCANTXQUEUE(&canFrame, "diag failed to send to canTxQueue\r\n");

		}
	}

  /* USER CODE END StartDiagTask */
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
