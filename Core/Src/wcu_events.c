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
#include "wcu_watchdog.h"
#include "wcu_diagnostics.h"
#include "wcu_timers.h"
#include "cmsis_os.h"
#include "rt12e_libs_tx_ringbuffer.h"

extern QueueHandle_t wcuEventQueueHandle;

static void WcuRunDispatcher(void);
static SWcuEvent WcuEventReceive(void);
static void WcuEventDispatch(const SWcuEvent *event);
static void WcuEventDispatchTimerEvent(const SWcuEvent *event);

/**
 * @brief  Function implementing the EventDispatcher thread.
 * @param  argument: Not used
 * @retval None
 */
void WcuEventDispatcherEntryPoint(void const *argument) {

	(void) argument;

	/* Run startups */
	WcuWatchdogStartup();
	WcuLoggerStartup();
	WcuSdioStartup();
	WcuXbeeStartup();
	WcuBtStartup();
	WcuGnssStartup();
	WcuDiagnosticsStartup();
	WcuCanStartup();

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

			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(EventQueueStarvations);
			status = EWcuRet_Error;
		}

	} else {

		if (pdPASS != xQueueSend(wcuEventQueueHandle, &event, 0)) {

			WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(EventQueueStarvations);
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

	WcuLogInfo("Entering the event loop...");

	for (;;) {

		/* Block on the event queue */
		SWcuEvent event = WcuEventReceive();
		/* Dispatch the received event */
		WcuEventDispatch(&event);
		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(EventsDispatched);
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
static void WcuEventDispatch(const SWcuEvent *event) {

	switch (event->signal) {

	case EWcuEventType_AdcConversionComplete:

		WcuDiagnosticsHandleAdcConversionComplete();
		break;

	case EWcuEventType_BtRxMessagePending:

		WcuBtHandlePendingMessage();
		break;

	case EWcuEventType_CanError:

		WcuCanHandleBusError();
		break;

	case EWcuEventType_CanRxMessagePending:

		WcuCanHandlePendingMessage();
		break;

	case EWcuEventType_GnssRxMessagePending:

		WcuGnssHandlePendingMessage();
		break;

	case EWcuEventType_LogEntriesPending:

		WcuLoggerFlushRingBuffer();
		break;

	case EWcuEventType_TimerExpired:

		WcuEventDispatchTimerEvent(event);
		break;

	case EWcuEventType_UartTxMessagePending:

	{
		STxRb *rb = (STxRb*) event->param;
		if (ETxRbRet_Busy == TxRbFlush(rb)) {

			/* If the UART is busy, enqueue the event again */
			WcuEventSend(event->signal, event->param);
		}
	}
		break;

	case EWcuEventType_XbeeRxMessagePending:

		WcuXbeeHandlePendingRxMessage();
		break;

	default:

		WcuLogError("WcuEventDispatch: Unknown event received");
		break;
	}
}

/**
 * @brief Dispatch the received timer event to the relevant handler
 * @param event Timer event
 * @retval None
 */
static void WcuEventDispatchTimerEvent(const SWcuEvent *event) {

	if (WCU_XBEE_STATUS_TIMER == WCU_EVENT_TIMER_INSTANCE(event->param)) {

		WcuXbeeHandleTimerExpired();

	} else if (WCU_WATCHDOG_RELOAD_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->param)) {

		WcuWatchdogReload();

	} else if (WCU_DIAGNOSTICS_SELFCHECK_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->param)) {

		WcuDiagnosticsStartSelfCheck();

	} else if (WCU_DIAGNOSTICS_SNAPSHOT_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->param)) {

		WcuDiagnosticsLogDatabaseSnapshot();
	}
}
