/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.h
 * @brief Header file providing prototypes of functions called by the gnssRx task
 */

#ifndef __WCU_GNSSRX_CALLS_H_
#define __WCU_GNSSRX_CALLS_H_

#include "quectel_l26_gnss_parser.h"

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

#endif /* __WCU_GNSSRX_CALLS_H_ */
