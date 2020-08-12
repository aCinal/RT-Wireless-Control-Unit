/**
 * @author Adrian Cinal
 * @file wcu_basic.h
 * @brief Header file containing basic WCU definitions, macros and external variables declarations
 */

#ifndef __WCU_BASIC_H_
#define __WCU_BASIC_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"
#include "rt12e_libs_can.h"

#include <stdio.h>
#include <string.h>

/* External variables -------------------------------------------------------------------------- */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern CAN_HandleTypeDef hcan1;

extern CRC_HandleTypeDef hcrc;

extern IWDG_HandleTypeDef hiwdg;

extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;

extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_tim7_up;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern osThreadId canGtkpHandle;
extern osThreadId sdioGtkpHandle;
extern osThreadId iwdgGtkpHandle;
extern osThreadId xbeeTxHandle;
extern osThreadId xbeeRxHandle;
extern osThreadId xbeeDiagHandle;
extern osThreadId btRxHandle;
extern osThreadId gnssRxHandle;
extern osThreadId rfRxHandle;
extern osThreadId diagnosticHandle;
extern osMessageQId canTxQueueHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId sdioLogQueueHandle;
extern osMutexId crcMutexHandle;

/* Exported defines -------------------------------------------------------------------------- */
/**
 * @brief Intuitive names for peripheral instances
 */
#define XBEE_TIM_INSTANCE		TIM7
#define TEMPSENSOR_ADC_INSTANCE	ADC1
#define BT_UART_INSTANCE		USART1
#define GNSS_UART_INSTANCE		USART3
#define XBEE_UART_INSTANCE		UART4
#define RF_SPI_INSTANCE			SPI1

/**
 * @brief Intuitive names for peripheral handles
 */
#define BT_UART_HANDLE			huart1
#define GNSS_UART_HANDLE		huart3
#define XBEE_UART_HANDLE		huart4
#define RF_SPI_HANDLE			hspi1

/**
 * @brief Notification values
 * @note Name of each definition adheres to the format: WCU_NV_[taker]_[giver]_[description (optional)],
 *       where [taker] is the task receiving the notification, [giver] is the task/function sending the notification
 */
#define WCU_NV_IWDGGTKP_BTRX						(0x00000001UL)			/* btRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_XBEETX						(0x00000002UL)			/* xbeeTx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_GNSSRX						(0x00000004UL)			/* gnssRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_RFRX						(0x00000008UL)			/* rfRx task's unique notification value for checking in with the watchdog */
#define WCU_NV_IWDGGTKP_CANGTKP						(0x00000010UL)			/* canGtkp task's unique notification value for checking in with the watchdog */
#define WCU_NV_XBEERX_SDIOGTKP_FAIL					(29UL)					/* Value to notify xbeeRx that reading subscription from SD card failed */
#define WCU_NV_XBEERX_TIM_PERIOD_ELAPSED_CB			(0x00100000UL)			/* Value to notify xbeeRx from TIM */
#define WCU_NV_XBEERX_UART_RX_CPLT_CB				(0x00200000UL)			/* Value to notify xbeeRx from HAL_UART_RxCpltCallback */
#define WCU_NV_XBEERX_XBEETX_LISTEN_FOR_RSSI		(0x00400000UL)			/* Value to notify xbeeRx from xbeeTx to start listening for RSSI value */
#define WCU_NV_XBEERX_XBEETX_DEVICE_READY			(0x00800000UL)			/* Value to notify xbeeRx from xbeeTx that the device is configured and ready */
#define WCU_NV_XBEETX_XBEERX_CONFIRM_LISTENING		(0x00000001UL)			/* Value for xbeeRx to confirm listening for RSSI value */
#define WCU_NV_XBEETX_XBEERX_RSSI_RECEIVED			(0x00000002UL)			/* Value to notify xbeeTx from xbeeRx that RSSI value was received from the device */
#define WCU_NV_XBEETX_UART_TX_CPLT_CB				(0x00000004UL)			/* Value to notify xbeeTx from HAL_UART_TxCpltCallback */
#define WCU_NV_RFRX_SPI_RX_CPLT_CB					(0x00000001UL)			/* Value to notify rfRx from HAL_SPI_RxCpltCallback */

/**
 * @brief Task delays
 */
#define WCU_DEFAULT_TASK_DELAY						(1U)					/* Default task delay */
#define WCU_DIAGNOSTIC_TASK_DELAY					(pdMS_TO_TICKS(1000))	/* diagnostic task delay */
#define WCU_XBEEDIAG_TASK_DELAY						(pdMS_TO_TICKS(1000))	/* xbeeDiag task delay */
#define WCU_IWDGGTKP_INIT_DELAY						(1000U)					/* iwdgGtkp IWDG init delay */
#define WCU_GNSSRX_DEVICECONFIG_SETUP_DELAY			(1000U)					/* gnssRx_DeviceConfig device setup delay */

/**
 * @brief Timeouts
 */
#define WCU_DEFAULT_TIMEOUT							(portMAX_DELAY)			/* Default timeout */
#define WCU_IWDGGTKP_XTASKNOTIFYWAIT_TIMEOUT		(WCU_DEFAULT_TIMEOUT)	/* iwdgGtkp xTaskNotifyWait timeout */
#define WCU_CANTXQUEUE_XQUEUESEND_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* canTxQueue xQueueSend timeout */
#define WCU_CANTXQUEUE_XQUEUERECEIVE_TIMEOUT		(0U)					/* canTxQueue xQueueReceive timeout */
#define WCU_CANRXQUEUE_XQUEUESEND_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* canRxQueue xQueueSend timeout */
#define WCU_CANRXQUEUE_XQUEUERECEIVE_TIMEOUT		(0U)					/* canRxQueue xQueueReceive timeout */
#define WCU_SDIOLOGQUEUE_XQUEUESEND_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* sdioLogQueue xQueueSend timeout */
#define WCU_SDIOLOGQUEUE_XQUEUERECEIVE_TIMEOUT		(0U)					/* sdioLogQueue xQueueReceive timeout */
#define WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* sdioSubQueue xQueueSend timeout */
#define WCU_SDIOSUBQUEUE_XQUEUERECEIVE_TIMEOUT		(WCU_DEFAULT_TIMEOUT)	/* sdioSubQueue xQueueReceive timeout */
#define WCU_CRCMUTEX_TIMEOUT						(WCU_DEFAULT_TIMEOUT)	/* crcMutex acquire timeout */
#define WCU_BTRX_ULTASKNOTIFYTAKE_TIMEOUT			(0U)					/* btRx ulTaskNotifyTake timeout */
#define WCU_XBEETX_ULTASKNOTIFYTAKE_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* xbeeTx ulTaskNotifyTake timeout */
#define WCU_XBEETX_UART_TX_TIMEOUT					(1000U)					/* xbeeTx UART Tx timeout */
#define WCU_XBEERX_ULTASKNOTIFYTAKE_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* xbeeRx ulTaskNotifyTake timeout */
#define WCU_XBEERX_XTASKNOTIFYWAIT_TIMEOUT			pdMS_TO_TICKS(1000)		/* xbeeRx xTaskNotifyWait timeout */
#define WCU_SDIOGTKP_XTASKNOTIFYWAIT_TIMEOUT		(0U)					/* sdioGtkp xTaskNotifyWait timeout */
#define WCU_GNSSRX_ULTASKNOTIFYTAKE_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* gnssRx ulTaskNotifyTake timeout */
#define WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT		(WCU_DEFAULT_TIMEOUT)	/* gnssRx_DeviceConfig UART Tx timeout */
#define WCU_RFRX_ULTASKNOTIFYTAKE_TIMEOUT			(WCU_DEFAULT_TIMEOUT)	/* rfRx ulTaskNotifyTake timeout */
#define WCU_DIAGNOSTIC_ULTASKNOTIFYTAKE_TIMEOUT		(WCU_DEFAULT_TIMEOUT)	/* diagnostic ulTaskNotifyTake timeout */
#define WCU_XBEEDIAG_XTASKNOTIFYWAIT_TIMEOUT		(0U)					/* xbeeDiag xTaskNotifyWait timeout */
#define WCU_XBEERX_UART_TIMEOUT						(500U)					/* xbeeRx UART Rx timeout */
#define WCU_XBEERX_UART_CLEANUP_TIMEOUT				(10U)					/* xbeeRx UART Rx cleanup timeout */
#define WCU_RFRX_SPI_TX_TIMEOUT						(50U)					/* rfRx SPI Tx timeout */
#define WCU_RFRX_SPI_RX_TIMEOUT						(1000U)					/* rfRx SPI Rx timeout */

/**
 * @brief Data sizes
 */
#define WCU_LOGGER_TIMESTAMP_SIZE					(11U)					/* Length of the error log timestamp */
#define WCU_GNSSRX_UART_RX_BUFF_SIZE				(100U)					/* Size in bytes of the gnssRx UART Rx buffer */
#define WCU_RFRX_SPI_RX_BUFF_SIZE					(32U)					/* Size in bytes of the rfRx SPI Rx buffer */

/**
 * @brief File paths
 */
#define WCU_SDIOGTKP_LOGFILE_PATH					("ERRLOG.TXT")			/* Error log file path */
#define WCU_SDIOGTKP_SUBFILE_PATH					("SUBSCR")				/* Subscription file path */

/**
 * @brief CAN IDs
 */
#define WCU_CAN_ID_GPS_POS							(0x500UL)				/* CAN ID: _500_GPS_POS */
#define WCU_CAN_ID_GPS_POS2							(0x501UL)				/* CAN ID: _501_GPS_POS2 */
#define WCU_CAN_ID_GPS_STATUS						(0x502UL)				/* CAN ID: _502_GPS_STATUS */
#define WCU_CAN_ID_WCU_DIAG							(0x733UL)				/* CAN ID: _733_WCU_DIAG */
#define WCU_CAN_ID_TELEMETRY_DIAG					(0x712UL)				/* CAN ID: _712_TELEMETRY_DIAG */

/* Exported macros -------------------------------------------------------------------------- */
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

#endif /* __WCU_BASIC_H_ */
