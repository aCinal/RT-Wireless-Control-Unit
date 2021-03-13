/**
 * @author Adrian Cinal
 * @file wcu_bt.h
 * @brief Header file providing the external interface of the BT service
 */

#ifndef __WCU_BT_H_
#define __WCU_BT_H

/**
 * @brief BT service startup
 * @retval None
 */
void WcuBtStartup(void);

/**
 * @brief Handle the pending message
 * @retval None
 */
void WcuBtHandlePendingMessage(void);

#endif /* __WCU_BT_H_ */
