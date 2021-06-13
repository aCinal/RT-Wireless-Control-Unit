/**
 * @author Adrian Cinal
 * @file wcu_sdio.c
 * @brief Source file implementing the SDIO service
 */

#include "wcu_sdio.h"
#include "wcu_defs.h"
#include "wcu_logger.h"
#include "fatfs.h"
#include "main.h"

static FATFS g_Filesystem;
bool g_WcuSdioReady = false;

/**
 * @brief SDIO service
 * @retval None
 */
void WcuSdioStartup(void) {

	/* Mount the FAT filesystem */
	if (FR_OK == f_mount(&g_Filesystem, SDPath, 1)) {

		/* Set the global flag */
		g_WcuSdioReady = true;
		/* Turn on the LED */
		SET_PIN(LED);
	}
}

/**
 * @brief Open a FATFS file
 * @param fd Pointer to pass the FATFS file descriptor out of the function
 * @param path Path to the file
 * @param mode Mode in which the file will be opened
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileOpen(FIL *fd, const char *path, BYTE mode) {

	EWcuRet status = EWcuRet_Ok;

	if (!g_WcuSdioReady) {

		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		if (FR_OK != f_open(fd, path, mode)) {

			status = EWcuRet_Error;
		}
	}

	return status;
}

/**
 * @brief Close a FATFS file
 * @param fd FATFS file descriptor
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileClose(FIL *fd) {

	EWcuRet status = EWcuRet_Ok;

	if (!g_WcuSdioReady) {

		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		if (FR_OK != f_close(fd)) {

			status = EWcuRet_Error;
		}
	}

	return status;
}

/**
 * @brief Read data from a file
 * @param fd FATFS file descriptor
 * @param buffer Data buffer
 * @param len Length of the data to be read
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileRead(FIL *fd, uint8_t *buffer, uint32_t len) {

	EWcuRet status = EWcuRet_Ok;
	UINT bytesRead = 0;

	if (!g_WcuSdioReady) {

		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		if (FR_OK != f_read(fd, buffer, len, &bytesRead)
				|| (bytesRead != len)) {

			status = EWcuRet_Error;
		}
	}

	return status;
}

/**
 * @brief Write data to a file
 * @param fd FATFS file descriptor
 * @param data Data buffer
 * @param len Length of the data to be written
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileWrite(FIL *fd, uint8_t *data, uint32_t len) {

	EWcuRet status = EWcuRet_Ok;
	UINT bytesWritten = 0;

	if (!g_WcuSdioReady) {

		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		if (FR_OK != f_write(fd, data, len, &bytesWritten)
				|| (bytesWritten != len)) {

			status = EWcuRet_Error;
		}
	}

	return status;
}
