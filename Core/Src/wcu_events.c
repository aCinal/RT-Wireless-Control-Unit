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

#include "wcu_diagnostics.h"
#include "cmsis_os.h"

extern QueueHandle_t wcuEventQueueHandle;

static void WcuRunDispatcher(void);
static SWcuEvent WcuEventReceive(void);
static void WcuEventDispatch(SWcuEvent event);

/**
 * @brief  Function implementing the dispatcherThread thread.
 * @param  argument: Not used
 * @retval None
 */
void DispatcherThreadEntryPoint(void const *argument) {

	(void) argument;

	/* Send the init event before entering the event loop */
	WcuEventSend(EWcuEventSignal_Init, NULL);

	/* Run the dispatcher */
	WcuRunDispatcher();
}

/**
 * @brief Create and send event
 * @param signal Event signal (event ID)
 * @param param Pointer to event parameters
 * @retval EWcuRet Status
 */
EWcuRet WcuEventSend(EWcuEventSignal signal, void *param) {

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

	case EWcuEventSignal_Init:

		WcuSdioStartup();
		WcuCanStartup();
		WcuXbeeStartup();
		WcuGnssStartup();
		WcuBtStartup();
		WcuDiagnosticsStartup();
		break;

	case EWcuEventSignal_WatchdogWakeup:

		WcuReloadWatchdogCounter();
		break;

	case EWcuEventSignal_PendingLogEntry:

	{
		char *logEntry = (char*) event.param;
		WcuLoggerCommitEntry(logEntry);
	}
		break;

	case EWcuEventSignal_CanPendingMessage:

	{
		uint32_t fifo = *(uint32_t*) event.param;
		WcuCanHandlePendingMessage(fifo);
	}
		break;

	case EWcuEventSignal_BtPendingMessage:

		WcuBtHandlePendingMessage();
		break;

	case EWcuEventSignal_GnssPendingMessage:

		WcuGnssHandlePendingMessage();
		break;

	case EWcuEventSignal_XbeePendingMessage:

		WcuXbeeHandlePendingMessage();
		break;

	case EWcuEventSignal_XbeeStatusTimerExpired:

		WcuXbeeHandleTimerExpired();
		break;

	case EWcuEventSignal_DiagnosticsTimerExpired:

		WcuDiagnosticsHandleTimerExpired();
		break;

	case EWcuEventSignal_AdcConversionComplete:

		WcuDiagnosticsHandleAdcConversionComplete();
		break;

	default:

		WcuLogError("WcuEventDispatch: Unknown event received");
		break;
	}
}
