/**
 * @author Adrian Cinal
 * @file wcu_sdio.h
 * @brief Header file providing the external interface of the SDIO service
 */

#ifndef __WCU_SDIO_H_
#define __WCU_SDIO_H_

#include "wcu_defs.h"
#include "fatfs.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported defines -------------------------------------------------------------------------- */
#define WCU_LOG_PATH                     ("runtime.log")
#define WCU_TELEMETRY_SUBSCRIPTION_PATH  ("subscription.txt")

/* Exported variables -------------------------------------------------------------------------- */
extern FIL g_WcuLogfileFd;
extern bool g_WcuSdioReady;
extern bool g_WcuLoggerReady;

/* Public functions prototypes -------------------------------------------------------------------------- */
/**
 * @brief SDIO service
 * @retval None
 */
void WcuSdioStartup(void);

/**
 * @brief Open a FATFS file
 * @param fd Pointer to pass the FATFS file descriptor out of the function
 * @param path Path to the file
 * @param mode Mode in which the file will be opened
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileOpen(FIL *fd, const char *path, BYTE mode);

/**
 * @brief Close a FATFS file
 * @param fd FATFS file descriptor
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileClose(FIL *fd);

/**
 * @brief Read data from a file
 * @param fd FATFS file descriptor
 * @param buffer Data buffer
 * @param len Length of the data to be read
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileRead(FIL *fd, uint8_t *buffer, uint32_t len);

/**
 * @brief Write data to a file
 * @param fd FATFS file descriptor
 * @param data Data buffer
 * @param len Length of the data to be written
 * @retval EWcuRet Status
 */
EWcuRet WcuSdioFileWrite(FIL* fd, uint8_t *data, uint32_t len);

#endif /* __WCU_SDIO_COMMON_H_ */
