/**
 * @author Adrian Cinal
 * @file wcu_events.c
 * @brief Source file implementing the event dispatcher functions
 */

#include "wcu_events.h"
#include "wcu_wrappers.h"

#include "wcu_sdio.h"
#include "wcu_logger.h"
#include "wcu_xbee.h"
#include "wcu_bt.h"
#include "wcu_can.h"
#include "wcu_gnss.h"
#include "rt12e_libs_uartringbuffer_tx.h"

#include "wcu_diagnostics.h"
#include "cmsis_os.h"

extern QueueHandle_t wcuEventQueueHandle;

static void WcuRunDispatcher(void);
static SWcuEvent WcuEventReceive(void);
static void WcuEventDispatch(SWcuEvent event);

/**
 * @brief  Function implementing the EventDispatcher thread.
 * @param  argument: Not used
 * @retval None
 */
void DispatcherEntryPoint(void const *argument) {

	(void) argument;

	/* Run startups */
	WcuSdioStartup();
	WcuCanStartup();
	WcuXbeeStartup();
	WcuBtStartup();
	WcuGnssStartup();
	WcuDiagnosticsStartup();

	/* Run the dispatcher */
	WcuRunDispatcher();
}

/**
 * @brief Create and send event
 * @param signal Event type
 * @param param Pointer to event parameters
 * @retval EWcuRet Status
 */
EWcuRet WcuEventSend(EWcuEventType signal, void *param) {

	EWcuRet status = EWcuRet_Ok;

	/* Create the event */
	SWcuEvent event;
	event.signal = signal;
	event.param = param;

	/* Push the event to the event queue */
	if (xPortIsInsideInterrupt()) {

		if (pdPASS != xQueueSendFromISR(wcuEventQueueHandle, &event, NULL)) {

			status = EWcuRet_Error;
		}

	} else {

		if (pdPASS != xQueueSend(wcuEventQueueHandle, &event, 0)) {

			status = EWcuRet_Error;
		}
	}

	return status;
}

/**
 * @brief Event dispatcher implementation (this function must never return)
 * @retval None
 */
static void WcuRunDispatcher(void) {

	for (;;) {

		/* Block on the event queue */
		SWcuEvent event = WcuEventReceive();
		/* Dispatch the received event */
		WcuEventDispatch(event);
	}
}

/**
 * @brief Block on the event queue for the incoming event
 * @retval SWcuEvent Received event
 */
static SWcuEvent WcuEventReceive(void) {

	SWcuEvent event;
	/* Block on the event queue indefinitely */
	(void) xQueueReceive(wcuEventQueueHandle, &event, portMAX_DELAY);
	return event;
}

/**
 * @brief Dispatch the received event to the relevant handler
 * @param event Received event
 * @retval None
 */
static void WcuEventDispatch(SWcuEvent event) {

	switch (event.signal) {

	case EWcuEventType_AdcConversionComplete:

		WcuDiagnosticsHandleAdcConversionComplete();
		break;

	case EWcuEventType_BtRxMessagePending:

		WcuBtHandlePendingMessage();
		break;

	case EWcuEventType_CanRxMessagePending:

	{
		uint32_t fifo = *(uint32_t*) event.param;
		WcuCanHandlePendingMessage(fifo);
	}
		break;

	case EWcuEventType_DiagnosticsTimerExpired:

		WcuDiagnosticsHandleTimerExpired();
		break;

	case EWcuEventType_GnssRxMessagePending:

		WcuGnssHandlePendingMessage();
		break;

	case EWcuEventType_LogEntryPending:

	{
		char *logEntry = (char*) event.param;
		WcuLoggerCommitEntry(logEntry);
	}
		break;

	case EWcuEventType_UartTxMessagePending:

	{
		SUartTxRb *rb = (SUartTxRb*)event.param;
		if (EUartTxRbRet_Again == UartTxRbSend(rb)) {

			/* If the UART is busy, enqueue the event again */
			WcuEventSend(event.signal, event.param);
		}
	}
		break;

	case EWcuEventType_WatchdogTimerExpired:

		WcuReloadWatchdogCounter();
		break;

	case EWcuEventType_XbeeRxMessagePending:

		WcuXbeeHandlePendingRxMessage();
		break;

	case EWcuEventType_XbeeStatusTimerExpired:

		WcuXbeeHandleTimerExpired();
		break;

	default:

		WcuLogError("WcuEventDispatch: Unknown event received");
		break;
	}
}
