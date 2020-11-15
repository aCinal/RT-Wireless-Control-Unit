/**
 * @author Adrian Cinal
 * @file rt12e_libs_can.c
 * @brief Source file defining functions facilitating the use of the CAN peripheral
 */

#include "rt12e_libs_can.h"

/**
 * @brief Configures the CAN filters according to the provided list of CAN IDs
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param idsTbl Pointer to an array of 32-bit CAN IDs to filter for
 * @param count Length of the idsTbl array
 */
void SetCanFilterList(CAN_HandleTypeDef *hcan, uint32_t idsTbl[], uint32_t count) {

	/* Assert valid idsTbl array length */
	if (count <= CAN_FILTERBANKS_COUNT * 4UL) {

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
		for (uint32_t i = 0; i < CAN_FILTERBANKS_COUNT; i += 1UL) {

			filterConfig.FilterBank = i;
			HAL_CAN_ConfigFilter(hcan, &filterConfig);

		}

		/* Set the new filter */
		filterConfig.FilterActivation = CAN_FILTER_ENABLE;
		for (uint32_t i = 0; i < count; i += 1UL) {

#if !defined (CAN_SINGLE_FIFO)
			if((count / 2UL) == i) {

				/* Switch to the second FIFO */
				filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;

			}
#endif /* !defined (CAN_SINGLE_FIFO) */

			switch (i % 4UL) {

			case 0:

				filterConfig.FilterIdHigh =
						AlignCanIdWithFilterFieldMapping(idsTbl[i]);
				break;

			case 1:

				filterConfig.FilterIdLow =
						AlignCanIdWithFilterFieldMapping(idsTbl[i]);
				break;

			case 2:

				filterConfig.FilterMaskIdHigh =
						AlignCanIdWithFilterFieldMapping(idsTbl[i]);
				break;

			case 3:

				filterConfig.FilterMaskIdLow =
						AlignCanIdWithFilterFieldMapping(idsTbl[i]);
				break;

			default:

				break;

			}

			/* If the filter bank is fully configured or there are no more IDs, call HAL_CAN_ConfigFilter */
			if ((i % 4UL == 3UL) || (i + 1UL == count)) {

				/* Configure the filter */
				HAL_CAN_ConfigFilter(hcan, &filterConfig);
				/* On fully configured filter bank, proceed to the next one */
				filterConfig.FilterBank += 1UL;
				/* Clear the config structure ID members */
				filterConfig.FilterIdHigh = 0x00000000;
				filterConfig.FilterIdLow = 0x00000000;
				filterConfig.FilterMaskIdHigh = 0x00000000;
				filterConfig.FilterMaskIdLow = 0x00000000;

			}

		}

	}

}

/**
 * @brief Configures the CAN filters to block all incoming messages
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void SetCanFilterBlockAll(CAN_HandleTypeDef *hcan) {

	/* Prepare the filter configuration structure */
	CAN_FilterTypeDef filterConfig;
	filterConfig.FilterActivation = CAN_FILTER_DISABLE;

	/* Disable all CAN filters */
	for(uint32_t i = 0; i < CAN_FILTERBANKS_COUNT; i += 1UL) {

		filterConfig.FilterBank = i;
		HAL_CAN_ConfigFilter(hcan, &filterConfig);

	}

}

/**
 * @brief Configures the CAN filters to allow all incoming messages to pass through
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 */
void SetCanFilterBlockNone(CAN_HandleTypeDef *hcan) {

	/* Prepare the filter configuration structure */
	CAN_FilterTypeDef filterConfig;
	/* Only one filter bank needs to be configured - all messages will go through it */
	filterConfig.FilterBank = 0;
	/* Assign the CAN FIFO to the filter */
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	/* Select the filter mode as IDMASK - a mask of 0x00000000U will cause all messages to get passed through to the FIFO */
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	/* Set the filter scale as 16 bit, since only the standard 11-bit CAN IDs are used */
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	/* Set the masks as 0x00000000U */
	filterConfig.FilterMaskIdHigh = 0x00000000;
	filterConfig.FilterMaskIdLow = 0x00000000;
	/* Enable the filter */
	filterConfig.FilterActivation = CAN_FILTER_ENABLE;
	/* Configure the filter */
	HAL_CAN_ConfigFilter(hcan, &filterConfig);

}
