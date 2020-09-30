/**
 * @file xbeepro_config.h
 * @author Adrian Cinal
 * @brief XBee-Pro LLD/API configuration file
 */

#ifndef __XBEEPRO_CONFIG_H
#define __XBEEPRO_CONFIG_H

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart4;
#define XBEEPRO_UART_HANDLE    (huart4)
#define XBEEPRO_UART_INSTANCE  (UART4)

#endif /* __XBEEPRO_CONFIG_H */
