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

#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Structure facilitating communication with the CAN peripheral gatekeeper
 */
typedef struct {
	/**
	 * @brief Member specifying the data direction and, by extension, type of the Header member
	 */
	enum {
		TX, RX
	} DataDirection;

	/*
	 * @brief CAN header
	 */
	union {
		CAN_TxHeaderTypeDef Tx;
		CAN_RxHeaderTypeDef Rx;
	} Header;

	/**
	 * @brief CAN frame data
	 */
	uint8_t Payload[WCU_CAN_PAYLOAD_SIZE];
} CanFrameTypedef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Intuitive names for UART instances */
#define WCU_BT_UART USART1
#define WCU_GNSS_UART USART3
#define WCU_XBEE_UART UART4

#define CAN_FILTERBANKS_COUNT 28U /* Number of CAN filter banks */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @brief This macro converts two separate 8-bit values to a single 16-bit value
 */
#define READAS16BIT(highbyte, lowbyte) (uint16_t)(((uint8_t)highbyte << 8) | (uint8_t)lowbyte)

/**
 * @brief This macro converts four separate 8-bit values to a single 32-bit value
 */
#define READAS32BIT(highbyte, midhighbyte, midlowbyte, lowbyte) (uint32_t)(((uint8_t)highbyte << 24) | ((uint8_t)midhighbyte << 16) | ((uint8_t)midlowbyte << 8) | (uint8_t)lowbyte)

/**
 * @brief Retrieves two least significant bytes of a value as uint16_t
 */
#define GET16LEASTSIGNIFACTBITS(value) (uint16_t)(value & 0xFFFF)

/**
 * @brief Retrieves the least significant byte of a 16-bit value
 */
#define GETLSBOF16(value) (uint8_t)((uint16_t)value & 0xFF)

/**
 * @brief Retrieves the most significant byte of a 16-bit value
 */
#define GETMSBOF16(value) (uint8_t)(((uint16_t)value >> 8) & 0xFF)

/**
 * @brief Shifts the bits of an 11-bit CAN standard ID to align with the filter bank register field mapping
 */
#define ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(id) (uint32_t)(id << 5)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId watchdogHandle;
osThreadId btReceiveHandle;
osThreadId xbeeSendHandle;
osThreadId xbeeReceiveHandle;
osThreadId gnssReceiveHandle;
osThreadId rfReceiveHandle;
osThreadId canGatekeeperHandle;
osThreadId sdGatekeeperHandle;
osMessageQId reportToWatchdogQueueHandle;
osMessageQId canTransmitQueueHandle;
osMessageQId canSubbedFramesQueueHandle;
osMutexId crcMutexHandle;
/* USER CODE BEGIN PV */

/* Intuitive names for UART handles */
UART_HandleTypeDef *phuart_bt = &huart1;
UART_HandleTypeDef *phuart_gnss = &huart3;
UART_HandleTypeDef *phuart_xbee = &huart4;

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
void StartWatchdogTask(void const *argument);
void StartBtReceiveTask(void const *argument);
void StartXbeeSendTask(void const *argument);
void StartXbeeReceiveTask(void const *argument);
void StartGnssReceiveTask(void const *argument);
void StartRfReceiveTask(void const *argument);
void StartCanGatekeeperTask(void const *argument);
void StartSdGatekeeperTask(void const *argument);

/* USER CODE BEGIN PFP */

void setCanSubscriptionFilter(CAN_HandleTypeDef *hcan, uint32_t* ids, uint32_t count);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* definition and creation of reportToWatchdogQueue */
	osMessageQDef(reportToWatchdogQueue, 16, osThreadId);
	reportToWatchdogQueueHandle = osMessageCreate(
			osMessageQ(reportToWatchdogQueue), NULL);

	/* definition and creation of canTransmitQueue */
	osMessageQDef(canTransmitQueue, 16, CanFrameTypedef);
	canTransmitQueueHandle = osMessageCreate(osMessageQ(canTransmitQueue),
	NULL);

	/* definition and creation of canSubbedFramesQueue */
	osMessageQDef(canSubbedFramesQueue, 16, CanFrameTypedef);
	canSubbedFramesQueueHandle = osMessageCreate(
			osMessageQ(canSubbedFramesQueue), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of watchdog */
	osThreadDef(watchdog, StartWatchdogTask, osPriorityNormal, 0, 128);
	watchdogHandle = osThreadCreate(osThread(watchdog), NULL);

	/* definition and creation of btReceive */
	osThreadDef(btReceive, StartBtReceiveTask, osPriorityLow, 0, 128);
	btReceiveHandle = osThreadCreate(osThread(btReceive), NULL);

	/* definition and creation of xbeeSend */
	osThreadDef(xbeeSend, StartXbeeSendTask, osPriorityIdle, 0, 128);
	xbeeSendHandle = osThreadCreate(osThread(xbeeSend), NULL);

	/* definition and creation of xbeeReceive */
	osThreadDef(xbeeReceive, StartXbeeReceiveTask, osPriorityIdle, 0, 128);
	xbeeReceiveHandle = osThreadCreate(osThread(xbeeReceive), NULL);

	/* definition and creation of gnssReceive */
	osThreadDef(gnssReceive, StartGnssReceiveTask, osPriorityIdle, 0, 128);
	gnssReceiveHandle = osThreadCreate(osThread(gnssReceive), NULL);

	/* definition and creation of rfReceive */
	osThreadDef(rfReceive, StartRfReceiveTask, osPriorityIdle, 0, 128);
	rfReceiveHandle = osThreadCreate(osThread(rfReceive), NULL);

	/* definition and creation of canGatekeeper */
	osThreadDef(canGatekeeper, StartCanGatekeeperTask, osPriorityIdle, 0, 128);
	canGatekeeperHandle = osThreadCreate(osThread(canGatekeeper), NULL);

	/* definition and creation of sdGatekeeper */
	osThreadDef(sdGatekeeper, StartSdGatekeeperTask, osPriorityIdle, 0, 128);
	sdGatekeeperHandle = osThreadCreate(osThread(sdGatekeeper), NULL);

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

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
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
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
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
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
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
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
static void MX_SDIO_SD_Init(void) {

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
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

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
	HAL_GPIO_WritePin(GPIOB,
			RF_PWR_UP_Pin | RF_TRX_CE_Pin | RF_TX_EN_Pin | GNSS_FORCE_ON_Pin
					| GNSS_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : XBEE_RSSI_Pin RF_DR_Pin RF_AM_Pin */
	GPIO_InitStruct.Pin = XBEE_RSSI_Pin | RF_DR_Pin | RF_AM_Pin;
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
	GPIO_InitStruct.Pin = RF_CD_Pin | RF_uPCLK_Pin | GNSS_1PPS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : RF_PWR_UP_Pin RF_TRX_CE_Pin RF_TX_EN_Pin GNSS_FORCE_ON_Pin
	 GNSS_RESET_Pin */
	GPIO_InitStruct.Pin = RF_PWR_UP_Pin | RF_TRX_CE_Pin | RF_TX_EN_Pin
			| GNSS_FORCE_ON_Pin | GNSS_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static BaseType_t dummy; /* Buffer for pxHigherPriorityTaskWoken flag */
	switch ((uint32_t) huart->Instance) {
	case (uint32_t) WCU_BT_UART:
		/* Notify btReceive task */
		vTaskNotifyGiveFromISR((TaskHandle_t) btReceiveHandle, &dummy);
		break;
	case (uint32_t) WCU_GNSS_UART:
		/* Notify gnssReceive task */
		vTaskNotifyGiveFromISR((TaskHandle_t) gnssReceiveHandle, &dummy);
		break;
	}
}

/**
 * @brief Configures the CAN filter according the the telemetry subscription
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param ids Pointer to an array of 32-bit CAN ids to filter for
 * @param count Length of the ids array
 */
void setCanSubscriptionFilter(CAN_HandleTypeDef *hcan, uint32_t* ids, uint32_t count) {
	if(count > CAN_FILTERBANKS_COUNT * 4) {
		/*
		 * TODO:
		 * Log invalid frame count
		 */
		return;
	}

	/* Prepare the filter configuration structure */
	CAN_FilterTypeDef filterConfig;
	/* Select the CAN FIFO to filter */
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	/* Select the filter mode as IDLIST - CAN IDs will be stored in the filter bank registers */
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	/* Set the filter scale as 16 bit, since only the standard 11-bit CAN IDs are used - this allows four IDs per bank */
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	/* Clear the previous filter config */
	filterConfig.FilterIdHigh = 0x00000000U;
	filterConfig.FilterIdLow = 0x00000000U;
	filterConfig.FilterMaskIdHigh = 0x00000000U;
	filterConfig.FilterMaskIdLow = 0x00000000U;
	filterConfig.FilterActivation = DISABLE;
	for(uint32_t i = 0; i < CAN_FILTERBANKS_COUNT; i += 1) {
		filterConfig.FilterBank = i;
		HAL_CAN_ConfigFilter(hcan, &filterConfig);
	}

	/* Set the new filter */
	filterConfig.FilterActivation = ENABLE;
	for(uint32_t i = 0; i < count; i += 1) {
		switch(i % 4) {
		case 0:
			filterConfig.FilterIdHigh = ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
			break;
		case 1:
			filterConfig.FilterIdLow = ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
			break;
		case 2:
			filterConfig.FilterMaskIdHigh = ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
			break;
		case 3:
			filterConfig.FilterMaskIdLow = ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
			break;
		}

		/* If the filter bank is fully configured or there are no more IDs, call HAL_CAN_ConfigFilter */
		if((i % 4 == 3) || (i + 1 == count)) {
			/* Configure the filter */
			HAL_CAN_ConfigFilter(hcan, &filterConfig);
			/* On fully configured filter bank, proceed to the next one */
			filterConfig.FilterBank += 1;
			/* Clear the config structure ID members */
			filterConfig.FilterIdHigh = 0x00000000U;
			filterConfig.FilterIdLow = 0x00000000U;
			filterConfig.FilterMaskIdHigh = 0x00000000U;
			filterConfig.FilterMaskIdLow = 0x00000000U;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartWatchdogTask */
/**
 * @brief  Function implementing the watchdog thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartWatchdogTask */
void StartWatchdogTask(void const *argument) {
	/* USER CODE BEGIN 5 */
	/* Initialize the watchdog */
	HAL_IWDG_Init(&hiwdg);

	/* Define key-value pair array for testing threads activity */
	struct {
		osThreadId Id;
		bool Reported;
	} reportSheet[WCU_NUMBER_OF_WATCHED_THREADS ] = { { .Id = btReceiveHandle,
			.Reported = false }, { .Id = xbeeSendHandle, .Reported = false }, {
			.Id = gnssReceiveHandle, .Reported = false }, { .Id =
			rfReceiveHandle, .Reported = false }, { .Id = canGatekeeperHandle,
			.Reported = false } };

	/* Buffer for reading thread IDs from the queue */
	osThreadId buff;
	/* Flag raised when all watched threads have reported to the watchdog */
	bool allReported;

	/* Infinite loop */
	for (;;) {
		/* Test for all threads' activity */
		if (xQueueReceive(reportToWatchdogQueueHandle, &buff,
		WCU_REPORTTOWATCHDOG_QUEUE_RECEIVE_TIMEOUT) == pdTRUE) {
			/* Identify the thread that just reported and set the appropriate flag in the report sheet */
			for (uint8_t i = 0U; i < WCU_NUMBER_OF_WATCHED_THREADS ; i += 1U) {
				if (buff == reportSheet[i].Id) {
					reportSheet[i].Reported = true;
				}
			}

			/* Assume all threads reported to the watchdog */
			allReported = true;
			/* Test for threads that did not report */
			for (uint8_t i = 0U; i < WCU_NUMBER_OF_WATCHED_THREADS ; i += 1U) {
				if (reportSheet[i].Reported == false) {
					/* If a thread did not report to the watchdog, reset the flag and break */
					allReported = false;
					break;
				}
			}

			if (allReported == true) {
				/* If all threads have reported to the watchdog, refresh the timer */
				HAL_IWDG_Refresh(&hiwdg);

				/* Reset the report sheet */
				for (uint8_t i = 0U; i < WCU_NUMBER_OF_WATCHED_THREADS ; i +=
						1U) {
					reportSheet[i].Reported = false;
				}

				/* Reset the flag */
				allReported = false;
			}
		}
		osDelay(WCU_DEFAULT_TASK_DELAY);
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
void StartBtReceiveTask(void const *argument) {
	/* USER CODE BEGIN StartBtReceiveTask */
	static osThreadId localId = NULL; /* Local copy of the thread's ID */
	do {
		localId = osThreadGetId();
		/* Assert localId is valid */
	} while (localId == NULL);

	static CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */
	static uint8_t btUartRxBuff[WCU_BT_UART_RX_BUFF_SIZE]; /* UART Rx buffer */
	static uint16_t readCrc; /* Buffer for the transmitted CRC */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */

	HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
	WCU_BT_UART_RX_BUFF_SIZE);

	/* Infinite loop */
	for (;;) {
		/* Wait for message received callback */
		if (ulTaskNotifyTake(pdTRUE, WCU_BT_UART_RX_NOTIFY_TAKE_TIMEOUT)
				>= 1UL) {

			/* Validate the VER and RES/SEQ octet of the R3TP frame */
			if (btUartRxBuff[0] != R3TP_VER0_VER_RES_SEQ_BYTE) {
				/*
				 * TODO:
				 * Log invalid VER/RES/SEQ octet
				 */
				/* Listen for for the next message */
				HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
				WCU_BT_UART_RX_BUFF_SIZE);
				continue;
			}

			/* Validate the END SEQ octets of the R3TP frame */
			if (btUartRxBuff[R3TP_VER0_FRAME_SIZE - 2U] != R3TP_END_SEQ_LOW_BYTE
					|| btUartRxBuff[R3TP_VER0_FRAME_SIZE - 1U]
							!= R3TP_END_SEQ_HIGH_BYTE) {
				/*
				 * TODO:
				 * Log invalid END SEQ octet
				 */
				/* Listen for for the next message */
				HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
				WCU_BT_UART_RX_BUFF_SIZE);
				continue;
			}

			/* Read CRC - note that the CRC is transmitted as little endian */
			readCrc = READAS16BIT(btUartRxBuff[2], btUartRxBuff[1]);

			/* Clear the CHECKSUM field */
			memset(btUartRxBuff + 1U, 0x00, 2U);

			if (osMutexWait(crcMutexHandle, WCU_CRC_MUTEX_TIMEOUT) == osOK) {
				/* Calculate the CRC */
				calculatedCrc =
						GET16LEASTSIGNIFACTBITS(
								HAL_CRC_Calculate(&hcrc, (uint32_t*)btUartRxBuff, WCU_BT_UART_RX_BUFF_SIZE / 4));
				osMutexRelease(crcMutexHandle);
			} else {
				/*
				 * TODO:
				 * Log CRC mutex timeout
				 */
				/* Listen for for the next message */
				HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
				WCU_BT_UART_RX_BUFF_SIZE);
				continue;
			}

			if (readCrc != calculatedCrc) {
				/*
				 * TODO:
				 * Log invalid CRC
				 */
				/* Listen for for the next message */
				HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
				WCU_BT_UART_RX_BUFF_SIZE);
				continue;
			}

			/* Read the CAN ID - note that the CAN ID is transmitted as little endian */
			canFrame.Header.Tx.StdId = READAS32BIT(btUartRxBuff[6],
					btUartRxBuff[5], btUartRxBuff[4], btUartRxBuff[3]);
			/* Read the Data length code */
			canFrame.Header.Tx.DLC = (uint32_t) (
					btUartRxBuff[7] < WCU_CAN_PAYLOAD_SIZE ?
							btUartRxBuff[7] : WCU_CAN_PAYLOAD_SIZE);

			/* Read the payload */
			for (uint8_t i = 0; i < canFrame.Header.Tx.DLC; i += 1) {
				canFrame.Payload[i] = btUartRxBuff[8 + i];
			}

			/* Push CAN frame to queue */
			if (xQueueSend(canTransmitQueueHandle, &canFrame,
					WCU_CANTRANSMIT_QUEUE_SEND_TIMEOUT) != pdTRUE) {
				/*
				 * TODO:
				 * Log failed to push to queue
				 */
			}

			/* Listen for for the next message */
			HAL_UART_Receive_DMA(phuart_bt, btUartRxBuff,
			WCU_BT_UART_RX_BUFF_SIZE);

		}

		/* Report to watchdog */
		if (xQueueSend(reportToWatchdogQueueHandle, &localId,
				WCU_REPORTTOWATCHDOG_QUEUE_SEND_TIMEOUT) != pdTRUE) {
			/*
			 * TODO:
			 * Log failed to push to queue
			 */
		}

		osDelay(WCU_DEFAULT_TASK_DELAY);
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
void StartXbeeSendTask(void const *argument) {
	/* USER CODE BEGIN StartXbeeSendTask */
	static osThreadId localId = NULL; /* Local copy of the thread's ID */
	do {
		localId = osThreadGetId();
		/* Assert localId is valid */
	} while (localId == NULL);

	static CanFrameTypedef frameBuff; /* CAN frame buffer */
	static uint8_t xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE]; /* UART Tx buffer */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */
	static uint8_t seqNum = 0U; /* Sequence number */

	/* Activate XBEE Pro by driving the XBEE_RESET pin high */
	HAL_GPIO_WritePin(XBEE_RESET_GPIO_Port, XBEE_RESET_Pin, GPIO_PIN_SET);

	/* Infinite loop */
	for (;;) {
		if (xQueueReceive(canSubbedFramesQueueHandle, xbeeUartTxBuff,
		WCU_CANSUBBEDFRAMES_QUEUE_RECEIVE_TIMEOUT) == pdTRUE) {
			if (frameBuff.DataDirection == RX) {
				/* Clear the buffer */
				memset(xbeeUartTxBuff, 0x00, R3TP_VER0_FRAME_SIZE);

				/* Set VER and RES/SEQ field */
				xbeeUartTxBuff[0] = R3TP_VER0_VER_RES_SEQ_BYTE;

				/* Set the SEQ NUM field */
				xbeeUartTxBuff[1] = seqNum;
				/* Increment the sequence number */
				seqNum = (seqNum < 255U) ? seqNum + 1 : 0U;

				/* Set the END SEQ field */
				xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 2U] =
				R3TP_END_SEQ_LOW_BYTE;
				xbeeUartTxBuff[R3TP_VER0_FRAME_SIZE - 1U] =
				R3TP_END_SEQ_HIGH_BYTE;

				/* Set CAN ID field - note that the CAN ID is transmitted as little endian */
				xbeeUartTxBuff[4] = GETLSBOF16(frameBuff.Header.Rx.StdId);
				xbeeUartTxBuff[5] = GETMSBOF16(frameBuff.Header.Rx.StdId);

				/* Set the DLC field */
				xbeeUartTxBuff[8] = (uint8_t) frameBuff.Header.Rx.DLC;

				/* Set the DATA field */
				for (uint8_t i = 0; i < frameBuff.Header.Rx.DLC; i += 1) {
					xbeeUartTxBuff[9 + i] = frameBuff.Payload[i];
				}

				if (osMutexWait(crcMutexHandle, WCU_CRC_MUTEX_TIMEOUT)
						== osOK) {
					/* Calculate the CRC */
					calculatedCrc =
							GET16LEASTSIGNIFACTBITS(
									HAL_CRC_Calculate(&hcrc, (uint32_t*)xbeeUartTxBuff, R3TP_VER0_FRAME_SIZE / 4));
					osMutexRelease(crcMutexHandle);

					/* Set the CRC field - note that the CRC is transmitted as little endian */
					xbeeUartTxBuff[2] = GETLSBOF16(calculatedCrc);
					xbeeUartTxBuff[3] = GETMSBOF16(calculatedCrc);
				} else {
					/*
					 * TODO:
					 * Log CRC mutex timeout
					 */
					continue;
				}

				/* Transmit frame */
				HAL_UART_Transmit_DMA(phuart_xbee, xbeeUartTxBuff,
				R3TP_VER0_FRAME_SIZE);

			} else {
				/*
				 * TODO:
				 * Log invalid data direction
				 */
			}
		}

		/* Report to watchdog */
		if (xQueueSend(reportToWatchdogQueueHandle, &localId,
				WCU_REPORTTOWATCHDOG_QUEUE_SEND_TIMEOUT) != pdTRUE) {
			/*
			 * TODO:
			 * Log failed to push to queue
			 */
		}

		osDelay(WCU_DEFAULT_TASK_DELAY);
	}
	/* USER CODE END StartXbeeSendTask */
}

/* USER CODE BEGIN Header_StartXbeeReceiveTask */
/**
 * @brief Function implementing the xbeeReceive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartXbeeReceiveTask */
void StartXbeeReceiveTask(void const *argument) {
	/* USER CODE BEGIN StartXbeeReceiveTask */
	static CAN_FilterTypeDef canFilter = { .FilterBank = 0U,
			.FilterFIFOAssignment = CAN_FILTER_FIFO0, .FilterMode =
			CAN_FILTERMODE_IDLIST, .FilterScale =
			CAN_FILTERSCALE_16BIT, };

	static uint8_t xbeeUartRxBuff[R3TP_VER1_MAX_FRAME_SIZE]; /* UART Rx buffer */
	static uint32_t frameNum; /* Number of frames in a subscription */
	static uint16_t readCrc; /* Buffer for the transmitted CRC */
	static uint16_t calculatedCrc; /* Buffer for the calculated CRC */
	static uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM];

	/* Infinite loop */
	for (;;) {
		/* Listen for the subscription (VER1) frame */
		HAL_UART_Receive(phuart_xbee, xbeeUartRxBuff, 1,
		WCU_XBEE_UART_RX_TIMEOUT);
		if (xbeeUartRxBuff[0] == R3TP_VER1_VER_RES_SEQ_BYTE) {
			/* On valid version byte, receive SEQ NUM, CHECKSUM and FRAME NUM */
			HAL_UART_Receive(phuart_xbee, xbeeUartRxBuff + 1, 7,
			WCU_XBEE_UART_RX_TIMEOUT);

			/* Read the CHECKSUM */
			readCrc = READAS16BIT(xbeeUartRxBuff[3], xbeeUartRxBuff[2]);

			/* Clear the CHECKSUM field */
			memset(xbeeUartRxBuff + 2U, 0x00, 2U);

			/* Read the FRAME NUM */
			frameNum = READAS32BIT(xbeeUartRxBuff[7], xbeeUartRxBuff[6],
					xbeeUartRxBuff[5], xbeeUartRxBuff[4]);

			/* Assert the payload won't overflow the buffer */
			if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {
				/*
				 * TODO:
				 * Log invalid FRAME NUM
				 */
				continue;
			}

			/* Receive the payload */
			HAL_UART_Receive(phuart_xbee,
					R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff), frameNum * 4,
					WCU_XBEE_UART_RX_TIMEOUT);

			/* Receive the frame align bytes (two) and END SEQ (also two bytes) */
			HAL_UART_Receive(phuart_xbee,
					R3TP_VER1_EPILOGUE_BEGIN(xbeeUartRxBuff, frameNum), 4,
					WCU_XBEE_UART_RX_TIMEOUT);

			/* Validate the END SEQ */
			if (xbeeUartRxBuff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 2U]
					!= R3TP_END_SEQ_LOW_BYTE
					|| xbeeUartRxBuff[R3TP_VER1_MESSAGE_LENGTH(frameNum) - 1U]
							!= R3TP_END_SEQ_HIGH_BYTE) {
				/*
				 * TODO:
				 * Log invalid END SEQ octet
				 */
				continue;
			}

			if (osMutexWait(crcMutexHandle, WCU_CRC_MUTEX_TIMEOUT) == osOK) {
				/* Calculate the CRC */
				calculatedCrc =
						GET16LEASTSIGNIFACTBITS(
								HAL_CRC_Calculate(&hcrc, (uint32_t* )xbeeUartRxBuff,
										R3TP_VER1_MESSAGE_LENGTH(frameNum)/4));
				osMutexRelease(crcMutexHandle);
			} else {
				/*
				 * TODO:
				 * Log CRC mutex timeout
				 */
				continue;
			}

			/* Validate the CRC */
			if (readCrc != calculatedCrc) {
				/*
				 * TODO:
				 * Log invalid CRC
				 */
				continue;
			}

			/* Read the payload */
			for (uint32_t i = 0; i < frameNum; i += 1) {
				subscription[i] = READAS32BIT(
						*(R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff) + 3 + 4*i),
						*(R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff) + 2 + 4*i),
						*(R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff) + 1 + 4*i),
						*(R3TP_VER1_PAYLOAD_BEGIN(xbeeUartRxBuff) + 4 * i));
			}

			/*
			 * TODO:
			 * Write subscription to SD
			 */

			setCanSubscriptionFilter(&hcan1, subscription, frameNum);
		} else {
			/**
			 * TODO:
			 * Log invalid VER/RES/SEQ octet
			 */
		}
		osDelay(WCU_DEFAULT_TASK_DELAY);
	}
	/* USER CODE END StartXbeeReceiveTask */
}

/* USER CODE BEGIN Header_StartGnssReceiveTask */
/**
 * @brief Function implementing the gnssReceive thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGnssReceiveTask */
void StartGnssReceiveTask(void const *argument) {
	/* USER CODE BEGIN StartGnssReceiveTask */
	static osThreadId localId = NULL; /* Local copy of the thread's ID */
	do {
		localId = osThreadGetId();
		/* Assert localId is valid */
	} while (localId == NULL);

	/* Infinite loop */
	for (;;) {

		/* Report to watchdog */
		if (xQueueSend(reportToWatchdogQueueHandle, &localId,
				WCU_REPORTTOWATCHDOG_QUEUE_SEND_TIMEOUT) != pdTRUE) {
			/*
			 * TODO:
			 * Log failed to push to queue
			 */
		}

		osDelay(WCU_DEFAULT_TASK_DELAY);
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
void StartRfReceiveTask(void const *argument) {
	/* USER CODE BEGIN StartRfReceiveTask */
	static osThreadId localId = NULL; /* Local copy of the thread's ID */
	do {
		localId = osThreadGetId();
		/* Assert localId is valid */
	} while (localId == NULL);

	/* Infinite loop */
	for (;;) {

		/* Report to watchdog */
		if (xQueueSend(reportToWatchdogQueueHandle, &localId,
				WCU_REPORTTOWATCHDOG_QUEUE_SEND_TIMEOUT) != pdTRUE) {
			/*
			 * TODO:
			 * Log failed to push to queue
			 */
		}

		osDelay(WCU_DEFAULT_TASK_DELAY);
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
void StartCanGatekeeperTask(void const *argument) {
	/* USER CODE BEGIN StartCanGatekeeperTask */
	static CanFrameTypedef frameBuff; /* CAN frame buffer */
	static uint32_t dummy; /* CAN Tx mailbox */
	/* Infinite loop */
	for (;;) {
		/* Check for outgoing messages */
		if (xQueueReceive(canTransmitQueueHandle, &frameBuff,
		WCU_CANTRANSMIT_QUEUE_RECEIVE_TIMEOUT) == pdTRUE) {
			if (frameBuff.DataDirection == TX) {
				HAL_CAN_AddTxMessage(&hcan1, &frameBuff.Header.Tx,
						frameBuff.Payload, &dummy);
			} else {
				/*
				 * TODO:
				 * Log invalid data direction
				 */
			}
		}

		/* Check for incoming messages */
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0U) {
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &frameBuff.Header.Rx,
					frameBuff.Payload);
			frameBuff.DataDirection = RX;
			if (xQueueSend(canSubbedFramesQueueHandle, &frameBuff,
					WCU_CANSUBBEDFRAMES_QUEUE_SEND_TIMEOUT) != pdTRUE) {
				/*
				 * TODO:
				 * Log failed to push to queue
				 */
			}
		}

		osDelay(WCU_DEFAULT_TASK_DELAY);
	}
	/* USER CODE END StartCanGatekeeperTask */
}

/* USER CODE BEGIN Header_StartSdGatekeeperTask */
/**
 * @brief Function implementing the sdGatekeeper thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSdGatekeeperTask */
void StartSdGatekeeperTask(void const *argument) {
	/* USER CODE BEGIN StartSdGatekeeperTask */
	/* Infinite loop */
	for (;;) {
		osDelay(WCU_DEFAULT_TASK_DELAY);
	}
	/* USER CODE END StartSdGatekeeperTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
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
void Error_Handler(void) {
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
