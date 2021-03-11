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

#define MOCK_CAN_TRAFFIC  1                         /* Flag that enables emulating CAN bus traffic */
#if (MOCK_CAN_TRAFFIC)
#define FAKE_CAN_ID_LO_BOUND  ( (uint32_t) 0x001 )  /* Lower bound for fake CAN IDs */
#define FAKE_CAN_ID_HI_BOUND  ( (uint32_t) 0x733 )  /* Upper bound for fake CAN IDs */
#endif /* (MOCK_CAN_TRAFFIC) */

extern CAN_HandleTypeDef hcan1;
extern osMessageQId canTxQueueHandle;
extern osMessageQId canRxQueueHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;

static ECanGtkpRet CanGtkpForwardRxMessage(uint32_t rxFifo);
#if (MOCK_CAN_TRAFFIC)
static ECanGtkpRet CanGtkpMockCanTraffic(void);
#endif /* (MOCK_CAN_TRAFFIC) */

/**
 * @brief Handle setting CAN filters according to the new telemetry subscription
 * @retval ECanGtkpRet Status
 */
ECanGtkpRet CanGtkpHandleNewSubscription(void) {

	ECanGtkpRet status = ECanGtkpRet_Ok;

	uint32_t nv;
	/* Check for new telemetry subscription notification */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, 0)) {

		uint32_t subscrTbl[R3TP_VER1_MAX_FRAME_NUM];
		for (uint32_t i = 0; i < nv; i += 1UL) {

			if (pdPASS
					!= xQueueReceive(canSubQueueHandle, &(subscrTbl[i]), 0)) {

				LogError("CanGtkpHandleNewSubscription: Queue is empty");
				status = ECanGtkpRet_Error;

			}

		}

		if (ECanGtkpRet_Ok == status) {

			/* Set the filters */
			SetCanFilterList(&hcan1, subscrTbl, nv);
			LogInfo("CanGtkpHandleNewSubscription: New subscription in place");

		}

	}

	return status;

}

/**
 * @brief Handle the CAN outgoing messages
 * @retval None
 */
void CanGtkpHandleOutbox(void) {

	SCanFrame frBuf;
	/* Check for outgoing messages */
	if (pdPASS == xQueueReceive(canTxQueueHandle, &frBuf, WCU_COMMON_TIMEOUT)) {

		uint32_t dummy;

		/* Send the message */
		(void) HAL_CAN_AddTxMessage(&hcan1, &frBuf.TxHeader, frBuf.PayloadTbl,
				&dummy);

	}

}

/**
 * @brief Handle the CAN incoming messages
 * @retval ECanGtkpRet Status
 */
ECanGtkpRet CanGtkpHandleInbox(void) {

	ECanGtkpRet status = ECanGtkpRet_Ok;

#if (MOCK_CAN_TRAFFIC)

	status = CanGtkpMockCanTraffic();

#endif /* (MOCK_CAN_TRAFFIC) */

	/* Check both FIFOs for incoming messages */
	if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {

		status = CanGtkpForwardRxMessage(CAN_RX_FIFO0);
		LogDebug("CanGtkpHandleInbox: Message received in CAN_RX_FIFO0");

	}
#if !defined (CAN_SINGLE_FIFO)
	if (0UL < HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1)) {

		status = CanGtkpForwardRxMessage(CAN_RX_FIFO1);
		LogDebug("CanGtkpHandleInbox: Message received in CAN_RX_FIFO1");

	}
#endif /* !defined (CAN_SINGLE_FIFO) */

	return status;

}

/**
 * @brief Read a received message from the designated FIFO and forward it to the xbeeTxRx task
 * @param rxFifo RX FIFO number
 * @retval ECanGtkpRet Status
 */
static ECanGtkpRet CanGtkpForwardRxMessage(uint32_t rxFifo) {

	ECanGtkpRet status = ECanGtkpRet_Ok;

	/* Turn on the LED */
	SET_PIN(CAN_LED);

	SCanFrame frBuf;
	/* Receive the message */
	(void) HAL_CAN_GetRxMessage(&hcan1, rxFifo, &frBuf.RxHeader,
			frBuf.PayloadTbl);

	/* Send the frame to the telemetry queue */
	if (pdPASS != xQueueSend(canRxQueueHandle, &frBuf, 0)) {

		LogError("CanGtkpForwardRxMessage: Queue is full");
		status = ECanGtkpRet_Error;

	}

	/* Turn off the LED */
	RESET_PIN(CAN_LED);

	return status;

}

#if (MOCK_CAN_TRAFFIC)
/**
 * @brief Emulate CAN traffic
 * @retval ECanGtkpRet Status
 */
static ECanGtkpRet CanGtkpMockCanTraffic(void) {

	ECanGtkpRet status = ECanGtkpRet_Ok;

	/* Turn on the LED */
	SET_PIN(CAN_LED);

	SCanFrame frBuf;
	static uint32_t fakeStdId = FAKE_CAN_ID_LO_BOUND;
	/* Fake a random CAN ID */
	frBuf.RxHeader.StdId = fakeStdId;
	/* Increment the fake ID */
	fakeStdId =
			(fakeStdId < FAKE_CAN_ID_HI_BOUND) ?
					fakeStdId + 1UL : FAKE_CAN_ID_LO_BOUND;
	/* Generate a mock payload */
	frBuf.RxHeader.DLC = 8;
	for (uint32_t i = 0; i < frBuf.RxHeader.DLC; i += 1UL) {

		frBuf.PayloadTbl[i] = (uint8_t)((frBuf.RxHeader.StdId + i) & 0xFF);

	}

	/* Send the frame to the telemetry queue */
	if (pdPASS != xQueueSend(canRxQueueHandle, &frBuf, 0)) {

		LogError("CanGtkpMockCanTraffic: Queue is full");
		status = ECanGtkpRet_Error;

	}

	/* Turn off the LED */
	RESET_PIN(CAN_LED);

	return status;

}
#endif /* (MOCK_CAN_TRAFFIC) */
