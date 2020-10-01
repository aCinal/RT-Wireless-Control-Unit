/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.h
 * @brief Header file providing prototypes of functions called by the rfRx task
 */


#ifndef __WCU_RFRX_CALLS_H_
#define __WCU_RFRX_CALLS_H_

/**
 * @brief Error code return value enumeration
 */
typedef enum ERfRxRet {
	ERfRxRet_Ok = 0,
	ERfRxRet_Error
} ERfRxRet;

/**
 * @brief Internal messages enumeration for internal communication between rfRx task and callbacks
 */
typedef enum ERfRxInternalMail {
	ERfRxInternalMail_Exti = 0,
	ERfRxInternalMail_PeriodElapsed
} ERfRxInternalMail;

/**
 * @brief Configure the S2-LP device
 * @retval ERfRxRet Status
 */
ERfRxRet rfRx_DeviceConfig(void);

/**
 * @brief Handle internal messages
 * @retval ERfRxRet Status
 */
ERfRxRet rfRx_HandleInternalMail(void);

/**
 * @brief Callback on external interrupt
 * @retval None
 */
void rfRx_ExtiCallback(void);

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void rfRx_PeriodElapsedCallback(void);

#endif /* __WCU_RFRX_CALLS_H_ */
