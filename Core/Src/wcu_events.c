/**
 * @author Adrian Cinal
 * @file wcu_events.c
 * @brief Source file implementing the event dispatcher functions
 */

#include "wcu_utils.h"
#include "wcu_mem.h"
#include "wcu_events.h"
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
#include "rt12e_libs_generic.h"

extern QueueHandle_t wcuEventQueueHandle;

static inline void WcuRunDispatcher(void) __attribute__((noreturn));
static inline SWcuEvent WcuEventReceive(void);
static inline void WcuEventDispatch(const SWcuEvent *event);
static inline void WcuEventDispatchTimerEvent(const SWcuEvent *event);

/**
 * @brief  Function implementing the EventDispatcher thread.
 * @param  argument: Not used
 * @retval None
 */
void WcuEventDispatcherEntryPoint(void const *argument) {

	(void) argument;

	/* Disable context switches */
	vTaskSuspendAll();

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
 * @param type Event type
 * @param paramPtr Pointer parameter
 * @param paramUint Integer parameter
 * @retval EWcuRet Status
 */
EWcuRet WcuEventSend(EWcuEventType type, void *paramPtr, uint32_t paramUint) {

	EWcuRet status = EWcuRet_Ok;

	/* Create the event */
	SWcuEvent event;
	event.type = type;
	event.paramPtr = paramPtr;
	event.paramUint = paramUint;

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
static inline void WcuRunDispatcher(void) {

	WcuLogInfo("%s(): Entering the event loop...", __FUNCTION__);

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
static inline SWcuEvent WcuEventReceive(void) {

	SWcuEvent event;

	/* Spin on the event queue */
	while (pdPASS != xQueueReceive(wcuEventQueueHandle, &event, 0)) {
		;
	}

	return event;
}

/**
 * @brief Dispatch the received event to the relevant handler
 * @param event Received event
 * @retval None
 */
static inline void WcuEventDispatch(const SWcuEvent *event) {

	switch (event->type) {

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

	case EWcuEventType_DeferredMemoryUnref:

		WcuMemFree(event->paramPtr);
		WCU_DIAGNOSTICS_DATABASE_INCREMENT_STAT(DeferredUnrefCount);
		break;

	case EWcuEventType_GnssRxMessagePending:

		WcuGnssHandlePendingMessage();
		break;

	case EWcuEventType_LogEntriesPending:

		WcuLoggerFlushRingbuffer();
		break;

	case EWcuEventType_TimerExpired:

		WcuEventDispatchTimerEvent(event);
		break;

	case EWcuEventType_UartTxMessagePending:

	{
		STxRb *rb = (STxRb*) event->paramPtr;
		if (ETxRbRet_Busy == TxRbFlush(rb)) {

			/* If the UART is busy, enqueue the event again */
			WcuEventSend(event->type, event->paramPtr, 0);
		}
	}
		break;

	case EWcuEventType_XbeeAcknowledgePending:

		/* Retrieve the R3TP message ID and sequence number from the event parameter */
		WcuXbeeHandlePendingAcknowledge(_getbyte(event->paramUint, 0),
				_getbyte(event->paramUint, 1));
		break;

	case EWcuEventType_XbeeRxMessagePending:

		WcuXbeeHandlePendingRxMessage();
		break;

	default:

		WcuLogError("%s(): Received event of unsupported type: %d", __FUNCTION__, event->type);
		break;
	}
}

/**
 * @brief Dispatch the received timer event to the relevant handler
 * @param event Timer event
 * @retval None
 */
static inline void WcuEventDispatchTimerEvent(const SWcuEvent *event) {

	if (WCU_XBEE_STATUS_TIMER == WCU_EVENT_TIMER_INSTANCE(event->paramPtr)) {

		WcuXbeeHandleTimerExpired();

	} else if (WCU_WATCHDOG_RELOAD_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->paramPtr)) {

		WcuWatchdogReload();

	} else if (WCU_DIAGNOSTICS_SELFCHECK_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->paramPtr)) {

		WcuDiagnosticsStartSelfCheck();

	} else if (WCU_DIAGNOSTICS_SNAPSHOT_TIMER
			== WCU_EVENT_TIMER_INSTANCE(event->paramPtr)) {

		WcuDiagnosticsLogDatabaseSnapshot();
	}
}
