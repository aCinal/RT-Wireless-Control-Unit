/**
 * @author Adrian Cinal
 * @file wcu_gnss.h
 * @brief Header file providing the external interface of the GNSS service
 */

#ifndef __WCU_GNSS_H_
#define __WCU_GNSS_H_

/**
 * @brief GNSS service startup
 * @retval None
 */
void WcuGnssStartup(void);

/**
 * @brief Handle a pending GNSS message
 * @retval None
 */
void WcuGnssHandlePendingMessage(void);

#endif /* __WCU_GNSS_H_ */
