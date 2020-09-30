/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.c
 * @brief Source file defining functions called by the sdioGtkp task
 */

#include "wcu_sdiogtkp_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"

#include "cmsis_os.h"
#include "fatfs.h"
#include <string.h>
#include <stddef.h>

extern osThreadId canGtkpHandle;
extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId sdioLogQueueHandle;

#define SDIOGTKP_LOGFILE_PATH  ("ERRLOG.TXT")  /* Error log file path */
#define SDIOGTKP_SUBFILE_PATH  ("SUBSCR")      /* Subscription file path */

/**
 * @brief Tries loading the telemetry subscription from the SD card and forwarding it to CAN gatekeeper
 * @retval ESdioGtkpRet Status
 */
ESdioGtkpRet sdioGtkp_LoadTelemetrySubscription(void) {

	ESdioGtkpRet status = ESdioGtkpRet_Ok;

	FIL subscriptionFile;
	/* Try opening the file */
	if (FR_OK != f_open(&subscriptionFile, SDIOGTKP_SUBFILE_PATH,
	FA_READ | FA_OPEN_EXISTING)) {

		LogPrint("sdioGtkp failed to open the subscription file");
		status = ESdioGtkpRet_Error;

	}

	uint32_t frNum;
	uint32_t frBuf;
	uint8_t temp[4];
	UINT bytesRead;

	if(ESdioGtkpRet_Ok == status) {

		/* Try reading the number of frames */
		if (FR_OK != f_read(&subscriptionFile, temp, 4, &bytesRead)) {

			/* Cleanup */
			(void) f_close(&subscriptionFile);

			LogPrint("sdioGtkp failed to read from the subscription file");
			status = ESdioGtkpRet_Error;

		}

	}

	if(ESdioGtkpRet_Ok == status) {

		/* Parse the number of frames */
		frNum = _reinterpret32bits(temp[3], temp[2], temp[1], temp[0]);

		/* Validate the number of frames */
		if (frNum > R3TP_VER1_MAX_FRAME_NUM) {

			LogPrint("Invalid FRAME NUM in the subscription file");
			status = ESdioGtkpRet_Error;

		}

	}

	if(ESdioGtkpRet_Ok == status) {

		/* Read the payload and push it to the queue */
		for (uint32_t i = 0; i < frNum; i += 1UL) {

			/* Try reading the frame */
			if (FR_OK != f_read(&subscriptionFile, temp, 4, &bytesRead)) {

				/* Cleanup */
				(void) xQueueReset(canSubQueueHandle);

				LogPrint("sdioGtkp failed to read from the subscription file");
				status = ESdioGtkpRet_Error;
				break;

			}

			/* Assert end of file was not reached */
			if (bytesRead < 4U) {

				/* Cleanup */
				(void) xQueueReset(canSubQueueHandle);

				LogPrint("Invalid FRAME NUM in the subscription file");
				status = ESdioGtkpRet_Error;
				break;

			}

			/* Parse the frame */
			frBuf = _reinterpret32bits(temp[3], temp[2], temp[1], temp[0]);

			/* Send the frame to the queue */
			if (pdPASS != xQueueSend(canSubQueueHandle, &frBuf, 0)) {

				/* Cleanup */
				(void) xQueueReset(canSubQueueHandle);

				LogPrint("sdioGtkp failed to send to canSubQueue");
				status = ESdioGtkpRet_Error;
				break;

			}

		}

	}

	if(ESdioGtkpRet_Ok == status) {

		/* Notify CAN gatekeeper of the pending subscription */
		(void) xTaskNotify(canGtkpHandle, frNum, eSetValueWithOverwrite);

	}

	/* Cleanup */
	(void) f_close(&subscriptionFile);

	return status;

}

/**
 * @brief Handles the error logger
 * @retval None
 */
void sdioGtkp_HandleLogger(void) {

	char *errMsgPtr;
	/* Receive an error message */
	if (pdPASS == xQueueReceive(sdioLogQueueHandle, &errMsgPtr, WCU_COMMON_TIMEOUT)) {

		FIL errorLogFile;
		/* Try opening the log file */
		if (FR_OK == f_open(&errorLogFile, SDIOGTKP_LOGFILE_PATH,
		FA_WRITE | FA_OPEN_APPEND)) {

			UINT bytesWritten;
			/* Write the error message to the file */
			(void) f_write(&errorLogFile, errMsgPtr,
					strlen(errMsgPtr), &bytesWritten);
			/* Close the file */
			(void) f_close(&errorLogFile);
			/* Free the allocated memory */
			vPortFree(errMsgPtr);
			errMsgPtr = NULL;

		}

	}

}

/**
 * @brief Handles saving the new telemetry subscription to the SD card
 * @retval ESdioGtkpRet Status
 */
ESdioGtkpRet sdioGtkp_HandleNewSubscription(void) {

	ESdioGtkpRet status = ESdioGtkpRet_Ok;

	uint32_t nv;
	/* Listen for notifications from xbeeSubscribe */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, 0)) {

		/* Validate the notification */
		if (28UL < nv) {

			LogPrint("Invalid notification value in sdioGtkp");
			status = ESdioGtkpRet_Error;

		}

		FIL subscriptionFile;

		if(ESdioGtkpRet_Ok == status) {

			/* Try opening the file */
			if (FR_OK != f_open(&subscriptionFile, SDIOGTKP_SUBFILE_PATH,
			FA_WRITE | FA_CREATE_ALWAYS)) {

				LogPrint("sdioGtkp failed to open the subscription file");
				status = ESdioGtkpRet_Error;

			}

		}

		if(ESdioGtkpRet_Ok == status) {

			/* Print the number of frames to the SD card */
			uint8_t temp[4];
			UINT bytesWritten;
			temp[0] = _bits0_7(nv);
			temp[1] = _bits8_15(nv);
			temp[2] = _bits16_23(nv);
			temp[3] = _bits24_31(nv);
			(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

			uint32_t frBuf;
			/* Print the subscription to the file */
			for (uint32_t i = 0; i < nv; i += 1UL) {

				if (pdPASS != xQueueReceive(sdioSubQueueHandle, &frBuf, 0)) {

					LogPrint("sdioGtkp failed to receive from sdioSubQueue");
					status = ESdioGtkpRet_Error;
					break;

				}

				/* Print the frame to the SD card */
				temp[0] = _bits0_7(frBuf);
				temp[1] = _bits8_15(frBuf);
				temp[2] = _bits16_23(frBuf);
				temp[3] = _bits24_31(frBuf);
				(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

			}

		}

		/* Cleanup */
		(void) f_close(&subscriptionFile);

	}

	return status;

}
