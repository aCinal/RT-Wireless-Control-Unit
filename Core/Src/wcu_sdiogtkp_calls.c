/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.c
 * @brief Source file defining functions called by the sdioGtkp task
 */

#include "wcu_sdiogtkp_calls.h"
#include "wcu_common.h"
#include "fatfs.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_r3tp.h"
#include "cmsis_os.h"

extern osMessageQId canSubQueueHandle;
extern osMessageQId sdioSubQueueHandle;
extern osMessageQId sdioLogQueueHandle;

#define SDIOGTKP_LOGFILE_PATH					("ERRLOG.TXT")			/* Error log file path */
#define SDIOGTKP_SUBFILE_PATH					("SUBSCR")				/* Subscription file path */
#define NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL		(29UL)					/* Value to notify canGtkp that reading subscription from SD card failed */

/**
 * @brief Tries loading the telemetry subscription from the SD card and pushing it to an appropriate queue
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(void) {

	FIL subscriptionFile; /* Telemetry subscription file object structure */

	/* Try opening the file */
	if (FR_OK
			!= f_open(&subscriptionFile, SDIOGTKP_SUBFILE_PATH,
					FA_READ | FA_OPEN_EXISTING)) {

		/* If failed to open the file */
		/* Log the error and return */
		LogError("sdioGtkp failed to open the subscription file\r\n");
		return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

	}

	uint32_t frNum; /* Buffer for the number of frames */
	uint32_t frBuf; /* Buffer for a subscription frame */
	uint8_t temp[4]; /* Temporary buffer for four bytes to be read as a single 32-bit little endian value */
	UINT bytesRead; /* Buffer for the number of bytes read */

	/* Try reading the number of frames */
	if (FR_OK != f_read(&subscriptionFile, temp, 4, &bytesRead)) {

		/* If failed to read the number of frames */
		/* Cleanup */
		(void) f_close(&subscriptionFile);
		/* Log the error and return */
		LogError("sdioGtkp failed to read from the subscription file\r\n");
		return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

	}

	/* Parse the number of frames */
	frNum = _join32bits(temp[3], temp[2], temp[1], temp[0]);

	/* Validate the number of frames */
	if (frNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Cleanup */
		(void) f_close(&subscriptionFile);
		/* Log the error and return */
		LogError("Invalid FRAME NUM in the subscription file\r\n");
		return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

	}

	/* Read the payload and push it to the queue */
	for (uint32_t i = 0; i < frNum; i += 1UL) {

		/* Try reading the frame */
		if (FR_OK != f_read(&subscriptionFile, temp, 4, &bytesRead)) {

			/* Cleanup */
			(void) f_close(&subscriptionFile);
			(void) xQueueReset(canSubQueueHandle);
			/* Log the error and return */
			LogError("sdioGtkp failed to read from the subscription file\r\n");
			return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

		}

		/* Assert end of file was not reached */
		if (bytesRead < 4U) {

			/* Cleanup */
			(void) f_close(&subscriptionFile);
			(void) xQueueReset(canSubQueueHandle);
			/* Log the error and return */
			LogError("Invalid FRAME NUM in the subscription file\r\n");
			return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

		}

		/* Parse the frame */
		frBuf = _join32bits(temp[3], temp[2], temp[1], temp[0]);

		/* Send the frame to the queue */
		if (pdPASS != xQueueSend(canSubQueueHandle, &frBuf, 0)) {

			/* Cleanup */
			(void) f_close(&subscriptionFile);
			(void) xQueueReset(canSubQueueHandle);
			/* Log the error and return */
			LogError("sdioGtkp failed to send to canSubQueue\r\n");
			return NV_CANGTKP_SDIOGTKP_SUB_READ_FAIL;

		}

	}

	/* Cleanup */
	(void) f_close(&subscriptionFile);
	/* Return the number of frames read */
	return frNum;

}

/**
 * @brief Handles the error logger
 * @retval None
 */
void sdioGtkp_HandleLogger(void) {

	char *errorMessagePtr; /* Buffer for the pointer to the error message */

	if (pdPASS == xQueueReceive(sdioLogQueueHandle, &errorMessagePtr, 0)) {

		FIL errorLogFile; /* Error log file object structure */

		/* Try opening the file */
		if (FR_OK == f_open(&errorLogFile, SDIOGTKP_LOGFILE_PATH,
		FA_WRITE | FA_OPEN_APPEND)) {

			UINT bytesWritten; /* Buffer for the number of bytes written */
			/* Write the error message to the file */
			(void) f_write(&errorLogFile, errorMessagePtr, strlen(errorMessagePtr),
					&bytesWritten);
			/* Close the file */
			(void) f_close(&errorLogFile);
			/* Free the allocated memory */
			vPortFree(errorMessagePtr);
			errorMessagePtr = NULL;

		}

	}

}

/**
 * @brief Handles saving the new telemetry subscription to the SD card
 */
void sdioGtkp_HandleNewSubscription(void) {

	uint32_t nv; /* Buffer to pass the notification value out of the xTaskNotifyWait function */

	/* Listen for notifications from xbeeSubscribe */
	if (pdTRUE
			== xTaskNotifyWait(CLEAR_NO_BITS_ON_ENTRY, CLEAR_ALL_BITS_ON_EXIT,
					&nv, 0)) {

		/* Validate the notification */
		if (28UL < nv) {

			/* Log the error and continue */
			LogError("Invalid notification value in sdioGtkp\r\n");
			return;

		}

		FIL subscriptionFile; /* Telemetry subscription file object structure */

		/* Try opening the file */
		if (FR_OK != f_open(&subscriptionFile, SDIOGTKP_SUBFILE_PATH,
		FA_WRITE | FA_CREATE_ALWAYS)) {

			/* Log the error and continue */
			LogError("sdioGtkp failed to open the subscription file\r\n");
			return;

		}

		uint32_t frBuf; /* Buffer for a subscription frame */
		uint8_t temp[4]; /* Temporary buffer to facilitate transmitting a 32-bit little endian value */
		UINT bytesWritten; /* Buffer for the number of bytes written */

		/* If nv is less than or equal to 28, it is to be interpreted as the number of frames waiting in the queue */
		/* Print the number of frames to the SD card */
		temp[0] = _bits0_7(nv);
		temp[1] = _bits8_15(nv);
		temp[2] = _bits16_23(nv);
		temp[3] = _bits24_31(nv);
		(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

		/* Print the subscription to the file */
		for (uint32_t i = 0; i < nv; i += 1UL) {

			if (pdPASS != xQueueReceive(sdioSubQueueHandle, &frBuf, 0)) {

				/* Log the error and break */
				LogError("sdioGtkp failed to receive from sdioSubQueue\r\n");
				break;

			}

			/* Print the frame to the SD card */
			temp[0] = _bits0_7(frBuf);
			temp[1] = _bits8_15(frBuf);
			temp[2] = _bits16_23(frBuf);
			temp[3] = _bits24_31(frBuf);
			(void) f_write(&subscriptionFile, temp, 4U, &bytesWritten);

		}

		/* Close the file */
		(void) f_close(&subscriptionFile);

	}

}
