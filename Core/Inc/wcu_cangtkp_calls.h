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
typedef enum ECanGtkpRet {
	ECanGtkpRet_Ok = 0,
	ECanGtkpRet_Error
} ECanGtkpRet;

/**
 * @brief Handle setting CAN filters according to the new telemetry subscription
 * @retval ECanGtkpRet Status
 */
ECanGtkpRet CanGtkpHandleNewSubscription(void);

/**
 * @brief Handle the CAN outgoing messages
 * @retval None
 */
void CanGtkpHandleOutbox(void);

/**
 * @brief Handle the CAN incoming messages
 * @retval ECanGtkpRet Status
 */
ECanGtkpRet CanGtkpHandleInbox(void);

#endif /* __WCU_CANGTKP_CALLS_H_ */
