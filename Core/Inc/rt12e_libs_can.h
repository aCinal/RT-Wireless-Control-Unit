/**
 * @author Adrian Cinal
 * @file rt12e_libs_can.h
 * @brief Header file containing functions and defines facilitating the use of the CAN peripheral
 */

#ifndef __RT12E_LIBS_CAN_H_
#define __RT12E_LIBS_CAN_H_

#include "stm32f4xx_hal.h"

/* Exported defines ------------------------------------------------------------*/
#define CAN_FILTERBANKS_COUNT  ((uint32_t) 28)  /* Number of CAN filter banks */
#define CAN_PAYLOAD_SIZE       ((uint32_t) 8U)  /* CAN payload size in bytes */

/* Exported typedef ------------------------------------------------------------*/
/**
 * @brief Structure facilitating communication with the CAN peripheral gatekeeper
 */
typedef struct {
	CAN_TxHeaderTypeDef TxHeader;          /* Tx header */
	CAN_RxHeaderTypeDef RxHeader;          /* Rx header */
	uint8_t PayloadTbl[CAN_PAYLOAD_SIZE];  /* Payload */
} SCanFrame;

/* Exported macro ------------------------------------------------------------*/
/**
 * @brief Shifts the bits of an 11-bit CAN standard ID to align with the filter bank register field mapping
 */
#define AlignCanIdWithFilterFieldMapping(id) (((uint32_t)(id)) << 5)

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Configures the CAN filters according to the provided list of CAN IDs
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param idsTbl Pointer to an array of 32-bit CAN IDs to filter for
 * @param count Length of the idsTbl array
 */
void setCanFilterList(CAN_HandleTypeDef *hcan, uint32_t idsTbl[], uint32_t count);

/**
 * @brief Configures the CAN filters to block all incoming messages
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void setCanFilterBlockAll(CAN_HandleTypeDef *hcan);

/**
 * @brief Configures the CAN filters to allow all incoming messages to pass through
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void setCanFilterBlockNone(CAN_HandleTypeDef *hcan);

#endif /* __RT12E_LIBS_CAN_H_ */
