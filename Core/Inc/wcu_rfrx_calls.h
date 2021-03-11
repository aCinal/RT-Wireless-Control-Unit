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
typedef enum ERfRxEvent {
	ERfRxEvent_Exti = 0,
	ERfRxEvent_PeriodElapsed
} ERfRxEvent;

/**
 * @brief Configure the S2-LP device
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxDeviceConfig(void);

/**
 * @brief Handle event messages
 * @retval ERfRxRet Status
 */
ERfRxRet RfRxHandleEvents(void);

/**
 * @brief Callback on external interrupt
 * @retval None
 */
void RfRxExtiCallback(void);

/**
 * @brief Callback on timer period elapsed
 * @retval None
 */
void RfRxPeriodElapsedCallback(void);

#endif /* __WCU_RFRX_CALLS_H_ */
