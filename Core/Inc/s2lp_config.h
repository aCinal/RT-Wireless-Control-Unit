/**
 * @file s2lp_config.h
 * @author Adrian Cinal
 * @brief S2-LP LLD/API configuration file
 */

#ifndef __S2LP_CONFIG_H_
#define __S2LP_CONFIG_H_

#include "main.h"

extern SPI_HandleTypeDef hspi1;
#define S2LP_SPI_HANDLE (hspi1)

#define PORT(label)  (label##_GPIO_Port)
#define PIN(label)   (label##_Pin)

/*
 * TODO: Change the following defines to map to actual GPIO ports and pins
 */
#define S2LP_GPIO_0_GPIO_Port  (PORT(RF_SPI1_SCK))
#define S2LP_GPIO_0_Pin        (PIN(RF_SPI1_SCK))
#define S2LP_GPIO_1_GPIO_Port  (PORT(RF_SPI1_SCK))
#define S2LP_GPIO_1_Pin        (PIN(RF_SPI1_SCK))
#define S2LP_GPIO_2_GPIO_Port  (PORT(RF_SPI1_SCK))
#define S2LP_GPIO_2_Pin        (PIN(RF_SPI1_SCK))
#define S2LP_GPIO_3_GPIO_Port  (PORT(RF_SPI1_SCK))
#define S2LP_GPIO_3_Pin        (PIN(RF_SPI1_SCK))
#define S2LP_CSn_GPIO_Port     (PORT(RF_SPI1_SCK))
#define S2LP_CSn_Pin           (PIN(RF_SPI1_SCK))
#define S2LP_SCLK_GPIO_Port    (PORT(RF_SPI1_SCK))
#define S2LP_SCLK_Pin          (PIN(RF_SPI1_SCK))
#define S2LP_SDI_GPIO_Port     (PORT(RF_SPI1_SCK))
#define S2LP_SDI_Pin           (PIN(RF_SPI1_SCK))
#define S2LP_SDO_GPIO_Port     (PORT(RF_SPI1_SCK))
#define S2LP_SDO_Pin           (PIN(RF_SPI1_SCK))
#define S2LP_SDN_GPIO_Port     (PORT(RF_SPI1_SCK))
#define S2LP_SDN_Pin           (PIN(RF_SPI1_SCK))

#endif /* __S2LP_CONFIG_H_ */
