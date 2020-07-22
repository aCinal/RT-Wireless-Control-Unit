/**
 * @author Adrian Cinal
 * @file rt12e_libs_can.c
 * @brief Source file defining functions facilitating the use of the CAN peripheral
 */

#include <rt12e_libs_can.h>

/**
 * @brief Configures the CAN filters according to the provided list of CAN IDs
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param ids Pointer to an array of 32-bit CAN IDs to filter for
 * @param count Length of the ids array
 */
void setCanFilterList(CAN_HandleTypeDef *hcan, uint32_t *ids, uint32_t count) {
	/* Assert valid ids array length */
	if (count <= CAN_FILTERBANKS_COUNT * 4U) {
		/* Prepare the filter configuration structure */
		CAN_FilterTypeDef filterConfig;
		/* Assign the CAN FIFO to the filter */
		filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		/* Select the filter mode as IDLIST - CAN IDs will be stored in the filter bank registers */
		filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
		/* Set the filter scale as 16 bit, since only the standard 11-bit CAN IDs are used - this allows four IDs per bank */
		filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

		/* Clear the previous filter config */
		filterConfig.FilterIdHigh = 0x00000000U;
		filterConfig.FilterIdLow = 0x00000000U;
		filterConfig.FilterMaskIdHigh = 0x00000000U;
		filterConfig.FilterMaskIdLow = 0x00000000U;
		filterConfig.FilterActivation = CAN_FILTER_DISABLE;
		for (uint32_t i = 0U; i < CAN_FILTERBANKS_COUNT; i += 1U) {
			filterConfig.FilterBank = i;
			HAL_CAN_ConfigFilter(hcan, &filterConfig);
		}

		/* Set the new filter */
		filterConfig.FilterActivation = CAN_FILTER_ENABLE;
		for (uint32_t i = 0U; i < count; i += 1U) {
			switch (i % 4U) {
			case 0:
				filterConfig.FilterIdHigh =
						ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
				break;
			case 1:
				filterConfig.FilterIdLow =
						ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
				break;
			case 2:
				filterConfig.FilterMaskIdHigh =
						ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
				break;
			case 3:
				filterConfig.FilterMaskIdLow =
						ALIGN_CAN_ID_WITH_FILTER_FIELD_MAPPING(ids[i]);
				break;
			}

			/* If the filter bank is fully configured or there are no more IDs, call HAL_CAN_ConfigFilter */
			if ((i % 4U == 3U) || (i + 1U == count)) {
				/* Configure the filter */
				HAL_CAN_ConfigFilter(hcan, &filterConfig);
				/* On fully configured filter bank, proceed to the next one */
				filterConfig.FilterBank += 1U;
				/* Clear the config structure ID members */
				filterConfig.FilterIdHigh = 0x00000000U;
				filterConfig.FilterIdLow = 0x00000000U;
				filterConfig.FilterMaskIdHigh = 0x00000000U;
				filterConfig.FilterMaskIdLow = 0x00000000U;
			}
		}
	}
}

/**
 * @brief Configures the CAN filters to block all incoming messages
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void setCanFilterBlockAll(CAN_HandleTypeDef *hcan) {
	/* Prepare the filter configuration structure */
	CAN_FilterTypeDef filterConfig;
	filterConfig.FilterActivation = CAN_FILTER_DISABLE;

	/* Disable all CAN filters */
	for(uint32_t i = 0U; i < CAN_FILTERBANKS_COUNT; i += 1U) {
		filterConfig.FilterBank = i;
		HAL_CAN_ConfigFilter(hcan, &filterConfig);
	}
}

/**
 * @brief Configures the CAN filters to allow all incoming messages to pass through
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void setCanFilterBlockNone(CAN_HandleTypeDef *hcan) {
	/* Prepare the filter configuration structure */
	CAN_FilterTypeDef filterConfig;
	/* Only one filter bank needs to be configured - all messages will go through it */
	filterConfig.FilterBank = 0U;
	/* Assign the CAN FIFO to the filter */
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	/* Select the filter mode as IDMASK - a mask of 0x00000000U will cause all messages to get passed through to the FIFO */
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	/* Set the filter scale as 16 bit, since only the standard 11-bit CAN IDs are used */
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	/* Set the masks as 0x00000000U */
	filterConfig.FilterMaskIdHigh = 0x00000000U;
	filterConfig.FilterMaskIdLow = 0x00000000U;
	/* Enable the filter */
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;
	/* Configure the filter */
	HAL_CAN_ConfigFilter(hcan, &filterConfig);
}

