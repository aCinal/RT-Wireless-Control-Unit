/**
 * @author Adrian Cinal
 * @file wcu_xbeetxrx_calls.h
 * @brief Header file providing prototypes of functions called by the xbeeTxRx task
 */

#ifndef __WCU_XBEETXRX_CALLS_H_
#define __WCU_XBEETXRX_CALLS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdbool.h>

extern CRC_HandleTypeDef hcrc;

extern UART_HandleTypeDef huart4;
#define XBEE_UART_HANDLE huart4
#define XBEE_UART_INSTANCE UART4

extern osThreadId canGtkpHandle;
extern osThreadId sdioGtkpHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId xbeeInternalMailQueueHandle;
extern osMutexId crcMutexHandle;

#define WCU_CAN_ID_TELEMETRY_DIAG		(uint32_t)(0x733UL)		/* CAN ID: _733_TELEMETRY_DIAG */

#define TELEMETRY_STATE_BIT				(uint8_t)(0x80U)		/* Telemetry_State bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_WARNING_BIT			(uint8_t)(0x40U)		/* Telemetry_Warning bit of the TELEMETRY_DIAG CAN frame */
#define TELEMETRY_PIT_BIT				(uint8_t)(0x20U)		/* Telemetry_Pit bit of the TELEMETRY_DIAG CAN frame */

#define XBEE_GT_DEFAULT					(uint16_t)(0x0CE4U)		/* XBEE Pro Guard Times default value */
#define XBEE_GT_DESIRED					(uint16_t)(0x000AU)		/* XBEE Pro Guard Times desired value */

/**
 * @brief Internal messages enumeration for internal communication between xbeeTxRx task and callbacks
 */
typedef enum EXbeeInternalMail {

	EXbeeInternalMail_PeriodElapsed = 0,

	EXbeeInternalMail_R3tpVer1HeaderReceived,

	EXbeeInternalMail_R3tpVer1MessageReceived,

	EXbeeInternalMail_R3tpVer2MessageReceived,

	EXbeeInternalMail_UnknownProtocol

} EXbeeInternalMail;

typedef struct SXbeeDiagnostics {

	bool greenWarningActive; /* Fleg set when the green warning is active */

	uint8_t greenWarningDuration; /* Buffer for the green warning's duration */

	bool redWarningActive; /* Flag set when the red warning is active */

	uint8_t redWarningDuration; /* Buffer for the red warning's duration */

	uint8_t rssi; /* Buffer for the RSSI value */

} SXbeeDiagnostics;

/**
 * @brief Ignores the rest of the incoming UART transmission
 */
#define IGNORE_REST_OF_THE_MESSAGE() do { \
		uint8_t buff; \
		while (HAL_OK == HAL_UART_Receive(&XBEE_UART_HANDLE, &buff, 1, \
			50)) { \
		\
		__NOP(); \
		\
	} \
} while(0)

extern void Error_Handler(void);

/**
 * @brief Configures the XBEE Pro device
 * @retval None
 */
void xbeeTxRx_DeviceConfig(void);

/**
 * @brief Handles internal messages
 * @param rxBuffTable UART Rx Buffer
 * @retval None
 */
void xbeeTxRx_HandleInternalMail(uint8_t rxBuffTable[]);

/**
 * @brief Handles transmitting telemetry data
 * @retval None
 */
void xbeeTxRx_HandleOutgoingR3tpComms(void);

/**
 * @brief Handles the new telemetry subscription
 * @param rxBuffTable UART Rx Buffer
 * @retval None
 */
void xbeeTxRx_HandleNewSubscription(uint8_t rxBuffTable[]);

/**
 * @brief Handles the driver warning
 * @param[in] rxBuffTable UART Rx Buffer
 * @param[out] diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */

void xbeeTxRx_HandleDriverWarning(uint8_t rxBuffTable[], SXbeeDiagnostics* diagnosticsPtr);

/**
 * @brief Polls the XBEE Pro device for the RSSI value of the last transmission received
 * @param[out] rssiPtr Address where the received RSSI value will be stored
 * @retval None
 */
void xbeeTxRx_PollForRssi(uint8_t* rssiPtr);

/**
 * @brief Sends the telemetry diagnostic frame to the CAN bus
 * @param diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
void xbeeTxRx_SendDiagnostics(SXbeeDiagnostics* diagnosticsPtr);

/**
 * @brief Decrements the warning duration counters and updates the warning active flags
 * @param diagnosticsPtr Pointer to the diagnostics structure
 * @retval None
 */
void xbeeTxRx_UpdateWarnings(SXbeeDiagnostics* diagnosticsPtr);

/**
 * @brief Forwards the telemetry subscription to the CAN gatekeeper for the appropriate filters to be set
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeTxRx_SendSubscriptionToCanGtkp(uint32_t ids[], size_t count);

/**
 * @brief Forwards the telemetry subscription to the SDIO gatekeeper to be stored on the SD card
 * @param ids Pointer to the subscription memory block
 * @param count Length of the ids array
 * @retval BaseType_t pdPASS if the subscription was successfully forwarded to the gatekeeper, errQUEUE_FULL otherwise
 */
BaseType_t xbeeTxRx_SendSubscriptionToSdioGtkp(uint32_t ids[], size_t count);

#endif /* __WCU_XBEETXRX_CALLS_H_ */
