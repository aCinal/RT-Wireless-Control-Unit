/**
 * @author Adrian Cinal
 * @file wcu_cangtkp_calls.h
 * @brief Header file providing prototypes of functions called by the canGtkp task
 */

#ifndef __WCU_CANGTKP_CALLS_H_
#define __WCU_CANGTKP_CALLS_H_

/**
 * @brief Error code return value enumeration
 */
typedef enum EWcuCanGtkpRet {
	EWcuCanGtkpRet_Ok = 0,
	EWcuCanGtkpRet_Error
} EWcuCanGtkpRet;

/**
 * @brief Handle the CAN outgoing messages
 * @retval None
 */
void canGtkp_HandleOutbox(void);

/**
 * @brief Handle the CAN incoming messages
 * @retval EWcuCanGtkpRet Status
 */
EWcuCanGtkpRet canGtkp_HandleInbox(void);

/**
 * @brief Handle setting CAN filters according to the new telemetry subscription
 * @retval EWcuCanGtkpRet Status
 */
EWcuCanGtkpRet canGtkp_HandleNewSubscription(void);

#endif /* __WCU_CANGTKP_CALLS_H_ */
