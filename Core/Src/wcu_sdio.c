/**
 * @author Adrian Cinal
 * @file wcu_sdio.c
 * @brief Source file implementing the SDIO service
 */

#include "wcu_sdio.h"
#include "wcu_defs.h"
#include "wcu_logger.h"
#include "fatfs.h"

static FATFS g_Filesystem;
bool g_WcuSdioReady = false;
#if !WCU_REDIRECT_LOGS_TO_SERIAL_PORT
bool g_WcuLoggerReady = false;
FIL g_WcuLogfileFd;
#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */

/**
 * @brief SDIO service
 * @retval None
 */
void WcuSdioStartup(void) {

	/* Mount the FAT filesystem */
	if (FR_OK == f_mount(&g_Filesystem, SDPath, 1)) {

		g_WcuSdioReady = true;
	}

#if !WCU_REDIRECT_LOGS_TO_SERIAL_PORT
	if (g_WcuSdioReady) {

		/* Open the logfile for writing in append mode */
		if (EWcuRet_Ok == WcuSdioFileOpen(&g_WcuLogfileFd, WCU_LOG_PATH,
		FA_WRITE | FA_OPEN_APPEND)) {

			g_WcuLoggerReady = true;
		}
	}
#endif /* !WCU_REDIRECT_LOGS_TO_SERIAL_PORT */
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

	if (FR_OK != f_open(fd, path, mode)) {

		status = EWcuRet_Error;
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

	if (FR_OK != f_close(fd)) {

		status = EWcuRet_Error;
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

	if (FR_OK != f_read(fd, buffer, len, &bytesRead)
			|| (bytesRead != len)) {

		status = EWcuRet_Error;
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

	if (FR_OK != f_write(fd, data, len, &bytesWritten)
			|| (bytesWritten != len)) {

		status = EWcuRet_Error;
	}

	return status;
}
