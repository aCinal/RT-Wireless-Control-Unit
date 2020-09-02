/**
 * @file nrf905lldconfig.h
 * @author Adrian Cinal
 * @brief nRF905 low-level drivers configuration and label mapping
 */

#ifndef __NRF905LLDCONFIG_H_
#define __NRF905LLDCONFIG_H_

#include "main.h"

extern SPI_HandleTypeDef		hspi1;
#define NRF_SPI_HANDLE			hspi1

#define NRF_MISO_GPIO_Port		RF_SPI1_MISO_GPIO_Port
#define NRF_MISO_Pin			RF_SPI1_MISO_Pin
#define NRF_MOSI_GPIO_Port		RF_SPI1_MOSI_GPIO_Port
#define NRF_MOSI_Pin			RF_SPI1_MOSI_Pin
#define NRF_SCK_GPIO_Port		RF_SPI1_SCK_GPIO_Port
#define NRF_SCK_Pin				RF_SPI1_SCK_Pin
#define NRF_CSN_GPIO_Port		RF_SPI1_CSN_GPIO_Port
#define NRF_CSN_Pin				RF_SPI1_CSN_Pin
#define NRF_DR_GPIO_Port		RF_DR_GPIO_Port
#define NRF_DR_Pin				RF_DR_Pin
#define NRF_AM_GPIO_Port		RF_AM_GPIO_Port
#define NRF_AM_Pin				RF_AM_Pin
#define NRF_uPLCK_GPIO_Port		RF_uPLCK_GPIO_Port
#define NRF_uPLCK_Pin			RF_uPLCK_Pin
#define NRF_TX_EN_GPIO_Port		RF_TX_EN_GPIO_Port
#define NRF_TX_EN_Pin			RF_TX_EN_Pin
#define NRF_TRX_CE_GPIO_Port	RF_TRX_CE_GPIO_Port
#define NRF_TRX_CE_Pin			RF_TRX_CE_Pin
#define NRF_PWR_UP_GPIO_Port	RF_PWR_UP_GPIO_Port
#define NRF_PWR_UP_Pin			RF_PWR_UP_Pin
#define NRF_CD_GPIO_Port		RF_CD_GPIO_Port
#define NRF_CD_Pin				RF_CD_Pin

#endif /* __NRF905LLDCONFIG_H_ */
