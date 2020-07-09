/**
 * @author Adrian Cinal
 * @file rte12_libs_can.h
 * @brief Header file containing functions and defines facilitating the use of the CAN peripheral
 */

#ifndef __RTE12_LIBS_CAN_H_
#define __RTE12_LIBS_CAN_H_

#include "main.h"

/* Exported typedef ------------------------------------------------------------*/
/**
 * @brief Structure facilitating communication with the CAN peripheral gatekeeper
 */
typedef struct {
	/**
	 * @brief Member specifying the data direction and, by extension, type of the Header member
	 */
	enum {
		TX, RX
	} DataDirection;

	/*
	 * @brief CAN header
	 */
	union {
		CAN_TxHeaderTypeDef Tx;
		CAN_RxHeaderTypeDef Rx;
	} Header;

	/**
	 * @brief CAN frame data
	 */
	uint8_t Payload[WCU_CAN_PAYLOAD_SIZE];
} CanFrameTypedef;

/* Exported defines ------------------------------------------------------------*/
#define CAN_FILTERBANKS_COUNT 28U /* Number of CAN filter banks */

/* Exported macro ------------------------------------------------------------*/
/**
 * @brief Shifts the bits of an 11-bit CAN standard ID to align with the filter bank register field mapping
 */
#define ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(id) (uint32_t)(id << 5)

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Configures the CAN filters according to the provided list of CAN IDs
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param ids Pointer to an array of 32-bit CAN IDs to filter for
 * @param count Length of the ids array
 */
void setCanFilterList(CAN_HandleTypeDef *hcan, uint32_t* ids, uint32_t count);

#endif /* __RTE12_LIBS_CAN_H_ */
