/**
 * @author Adrian Cinal
 * @file wcu_cangtkp_calls.c
 * @brief Source file defining functions called by the canGtkp task
 */

#include "wcu_cangtkp_calls.h"
#include "wcu_common.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_r3tp.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern osMessageQId canTxQueueHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;

/**
 * @brief Waits for SDIO gatekeeper to test if there is a valid telemetry subscription stored on the SD card
 * @retval None
 */
void canGtkp_WaitSubscriptionFromSdioGtkp(void) {

	uint32_t nv; /* Buffer to pass the notification value out of the xTaskNotifyWait function */

	/* Wait for sdioGtkp to notify the task if there is a valid subscription stored on the SD card */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, pdMS_TO_TICKS(1000))) {

		/* If notificationValue is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue, else - an error occured */
		if (nv <= 28UL) {

			uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM]; /* Buffer for telemetry subscription CAN IDs */

			for (uint32_t i = 0; i < nv; i += 1UL) {

				if (pdPASS
						!= xQueueReceive(canSubQueueHandle, subscription + i,
								0)) {

					/* Log the error and return */
					LogError("canGtkp failed to receive from canSubQueue\r\n");
					return;

				}

			}

			/* Set the CAN filters */
			setCanFilterList(&hcan1, subscription, nv);

		}

	}

}

/**
 * @brief Handles the CAN outgoing messages
 * @retval None
 */
void canGtkp_HandleOutbox(void) {

	SCanFrame frameBuff; /* CAN frame buffer */

	/* Check for outgoing messages */
	if (pdPASS == xQueueReceive(canTxQueueHandle, &frameBuff, 0)) {

		uint32_t dummy; /* Buffer for the CAN Tx mailbox used */

		/* Send the message */
		(void) HAL_CAN_AddTxMessage(&hcan1, &frameBuff.UHeader.Tx,
				frameBuff.PayloadTable, &dummy);

	}

}

/**
 * @brief Handles the CAN incoming messages
 * @retval None
 */
void canGtkp_HandleInbox(void) {

	SCanFrame frameBuff; /* CAN frame buffer */

	/* Check for incoming messages */
	if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

		/* Receive the message */
		(void) HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &frameBuff.UHeader.Rx,
				frameBuff.PayloadTable);
		/* Set the DataDirection member in the CAN frame struct */
		frameBuff.EDataDirection = RX;
		/* Send the frame to the telemetry queue */
		if (pdPASS != xQueueSend(canRxQueueHandle, &frameBuff, 0)) {

			/* Log the error */
			LogError("canGtkp failed to send to canRxQueue\r\n");

		}

	}

}

/**
 * @brief Handles setting CAN filters according to the new telemetry subscription
 * @retval None
 */
void canGtkp_HandleNewSubscription(void) {

	uint32_t nv; /* Buffer to pass the notification value out of the xTaskNotifyWait function */
	/* Check for new telemetry subscription notification */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, 0)) {

		uint32_t subscription[R3TP_VER1_MAX_FRAME_NUM];
		for (uint32_t i = 0; i < nv; i += 1UL) {

			if (pdPASS
					!= xQueueReceive(canSubQueueHandle, subscription + i, 0)) {

				/* Log the error and continue */
				LogError("canGtkp failed to receive from canSubQueue\r\n");
				continue;

			}

		}

		/* Set the filters */
		setCanFilterList(&hcan1, subscription, nv);

	}

}
