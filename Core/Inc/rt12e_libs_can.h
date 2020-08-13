/**
 * @author Adrian Cinal
 * @file rt12e_libs_can.h
 * @brief Header file containing functions and defines facilitating the use of the CAN peripheral
 */

#ifndef __RT12E_LIBS_CAN_H_
#define __RT12E_LIBS_CAN_H_

#include "stm32f4xx_hal.h"

/* Exported defines ------------------------------------------------------------*/
#define CAN_FILTERBANKS_COUNT	28UL	/* Number of CAN filter banks */
#define CAN_PAYLOAD_SIZE		8UL		/* CAN payload size in bytes */

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
	} EDataDirection;

	/*
	 * @brief CAN header
	 */
	union {
		CAN_TxHeaderTypeDef Tx;
		CAN_RxHeaderTypeDef Rx;
	} UHeader;

	/**
	 * @brief CAN frame data
	 */
	uint8_t PayloadTable[CAN_PAYLOAD_SIZE];
} SCanFrame;

/* Exported macro ------------------------------------------------------------*/
/**
 * @brief Shifts the bits of an 11-bit CAN standard ID to align with the filter bank register field mapping
 */
#define alignCanIdWithFilterFieldMapping(id) ((uint32_t)(id << 5))

/* Exported function prototypes -----------------------------------------------*/
/**
 * @brief Configures the CAN filters according to the provided list of CAN IDs
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param ids Pointer to an array of 32-bit CAN IDs to filter for
 * @param count Length of the ids array
 */
void setCanFilterList(CAN_HandleTypeDef *hcan, uint32_t ids[], uint32_t count);

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
