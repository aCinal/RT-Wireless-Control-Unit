/**
 * @author Adrian Cinal
 * @file wcu_sdiogtkp_calls.c
 * @brief Source file defining functions called by the sdioGtkp task
 */

#include "wcu_sdiogtkp_calls.h"
#include "wcu_basic.h"

/**
 * @brief Tries loading the telemetry subscription from the SD card
 * @param fp Pointer to the blank file object
 * @param path Pointer to the file name
 * @retval uint32_t Number of frames loaded from SD card or error code if over 28UL
 */
uint32_t sdioGtkp_LoadTelemetrySubscription(FIL *fp, const TCHAR *path) {

	/* Try opening the file */
	if (FR_OK != f_open(fp, path, FA_READ | FA_OPEN_EXISTING)) {

		/* If failed to open the file */
		/* Log the error and return */
		LOGERROR("sdioGtkp failed to open the subscription file\r\n");
		return WCU_NV_XBEERX_SDIOGTKP_FAIL;

	}

	uint32_t frameNum; /* Buffer for the number of frames */
	uint32_t frameBuff; /* Buffer for a subscription frame */
	uint8_t temp[4]; /* Temporary buffer for four bytes to be read as a single 32-bit little endian value */
	UINT bytesRead; /* Buffer for the number of bytes read */

	/* Try reading the number of frames */
	if (FR_OK != f_read(fp, temp, 4, &bytesRead)) {

		/* If failed to read the number of frames */
		/* Cleanup */
		(void) f_close(fp);
		/* Log the error and return */
		LOGERROR(
			"sdioGtkp failed to read from the subscription file\r\n");
		return WCU_NV_XBEERX_SDIOGTKP_FAIL;

	}

	/* Parse the number of frames */
	frameNum = _join32bits(temp[3], temp[2], temp[1], temp[0]);

	/* Validate the number of frames */
	if (frameNum > R3TP_VER1_MAX_FRAME_NUM) {

		/* Cleanup */
		(void) f_close(fp);
		/* Log the error and return */
		LOGERROR("Invalid FRAME NUM in the subscription file\r\n");
		return WCU_NV_XBEERX_SDIOGTKP_FAIL;

	}

	/* Read the payload and push it to the queue */
	for (uint32_t i = 0; i < frameNum; i += 1UL) {

		/* Try reading the frame */
		if (FR_OK != f_read(fp, temp, 4, &bytesRead)) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			/* Log the error and return */
			LOGERROR(
				"sdioGtkp failed to read from the subscription file\r\n");
			return WCU_NV_XBEERX_SDIOGTKP_FAIL;

		}

		/* Assert end of file was not reached */
		if (bytesRead < 4U) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			/* Log the error and return */
			LOGERROR("Invalid FRAME NUM in the subscription file\r\n");
			return WCU_NV_XBEERX_SDIOGTKP_FAIL;

		}

		/* Parse the frame */
		frameBuff = _join32bits(temp[3], temp[2], temp[1], temp[0]);

		/* Send the frame to the queue */
		if (pdPASS
				!= xQueueSend(sdioSubQueueHandle, &frameBuff,
						WCU_SDIOSUBQUEUE_XQUEUESEND_TIMEOUT)) {

			/* Cleanup */
			(void) f_close(fp);
			(void) xQueueReset(sdioSubQueueHandle);
			/* Log the error and return */
			LOGERROR("sdioGtkp failed to send to sdioSubQueue\r\n");
			return WCU_NV_XBEERX_SDIOGTKP_FAIL;

		}

	}

	/* Cleanup */
	(void) f_close(fp);
	/* Return the number of frames read */
	return frameNum;

}
