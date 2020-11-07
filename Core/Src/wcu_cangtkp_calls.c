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

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern osMessageQId canTxQueueHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;

static EWcuCanGtkpRet canGtkp_ForwardRxMessage(uint32_t rxFifo);

/**
 * @brief Handle setting CAN filters according to the new telemetry subscription
 * @retval EWcuCanGtkpRet Status
 */
EWcuCanGtkpRet canGtkp_HandleNewSubscription(void) {

	EWcuCanGtkpRet status = EWcuCanGtkpRet_Ok;

	uint32_t nv;
	/* Check for new telemetry subscription notification */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, 0)) {

		uint32_t subscrTbl[R3TP_VER1_MAX_FRAME_NUM];
		for (uint32_t i = 0; i < nv; i += 1UL) {

			if (pdPASS
					!= xQueueReceive(canSubQueueHandle, &(subscrTbl[i]), 0)) {

				LogPrint("canGtkp_HandleNewSubscription: Queue is empty");
				status = EWcuCanGtkpRet_Error;

			}

		}

		if (EWcuCanGtkpRet_Ok == status) {

			/* Set the filters */
			setCanFilterList(&hcan1, subscrTbl, nv);

		}

	}

	return status;

}

/**
 * @brief Handle the CAN outgoing messages
 * @retval None
 */
void canGtkp_HandleOutbox(void) {

	SCanFrame frBuf;
	/* Check for outgoing messages */
	if (pdPASS == xQueueReceive(canTxQueueHandle, &frBuf, WCU_COMMON_TIMEOUT)) {

		uint32_t dummy; /* Buffer for the CAN Tx mailbox used */

		/* Send the message */
		(void) HAL_CAN_AddTxMessage(&hcan1, &frBuf.TxHeader, frBuf.PayloadTbl,
				&dummy);

	}

}

/**
 * @brief Handle the CAN incoming messages
 * @retval EWcuCanGtkpRet Status
 */
EWcuCanGtkpRet canGtkp_HandleInbox(void) {

	EWcuCanGtkpRet status = EWcuCanGtkpRet_Ok;

	/* Check both FIFOs for incoming messages */
	if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

		(void) canGtkp_ForwardRxMessage(CAN_RX_FIFO0);

	}
#if !defined (CAN_SINGLE_FIFO)
	if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1)) {

		(void) canGtkp_ForwardRxMessage(CAN_RX_FIFO1);

	}
#endif /* !defined (CAN_SINGLE_FIFO) */

	return status;

}

/**
 * @brief Read a received message from the designated FIFO and forward it to the xbeeTxRx task
 * @param rxFifo RX FIFO number
 * @retval EWcuCanGtkpRet Status
 */
static EWcuCanGtkpRet canGtkp_ForwardRxMessage(uint32_t rxFifo) {

	EWcuCanGtkpRet status = EWcuCanGtkpRet_Ok;

	/* Turn on the LED */
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);

	SCanFrame frBuf;
	/* Receive the message */
	(void) HAL_CAN_GetRxMessage(&hcan1, rxFifo, &frBuf.RxHeader,
			frBuf.PayloadTbl);

	/* Send the frame to the telemetry queue */
	if (pdPASS != xQueueSend(canRxQueueHandle, &frBuf, 0)) {

		LogPrint("canGtkp_ForwardRxMessage: Queue is full");
		status = EWcuCanGtkpRet_Error;

	}

	/* Turn off the LED */
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);

	return status;

}
