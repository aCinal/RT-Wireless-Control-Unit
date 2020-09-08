/**
 * @author Adrian Cinal
 * @file wcu_rfrx_calls.c
 * @brief Source file defining functions called by the rfRx task
 */

#include "wcu_rfrx_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_generic.h"
#include "nrf905lld.h"

#include "main.h"
#include "cmsis_os.h"
#include <string.h>

#define TPMS_MESSAGE_PAYLOAD_WIDTH          ((uint32_t) 8)           /* TPMS message size */
#define TPMS_MESSAGE_ADDRESS_WIDTH          ((uint32_t) 4)           /* TPMS address size */

#define TPMS_ID_FL                          ((uint32_t) 0x00000000)  /* TBD experimentally */
#define TPMS_ID_FR                          ((uint32_t) 0x00000000)  /* TBD experimentally */
#define TPMS_ID_RL                          ((uint32_t) 0x00000000)  /* TBD experimentally */
#define TPMS_ID_RR                          ((uint32_t) 0x00000000)  /* TBD experimentally */
#define TPMS_PRES_SCAL_FAC                  ((float32_t) 0.25)       /* TPMS pressure scaling factor */
#define TPMS_PRES_OFFSET                    ((float32_t) (-7.))      /* TPMS pressure offset */

#define _bits0_6(x)                         ((uint8_t) (x & 0x7F))
#define _bit7(x)                            ((uint8_t) ((x >> 7) & 0x01))

/* Macros for scrambling the TPMS data from the bits of the message */
#define TPMS_SCRMBL_PRES_NONINVERTED(buff)  ((uint8_t) ((_bits0_6(buff[4]) << 1) | _bit7(buff[5])))
#define TPMS_SCRMBL_TEMP(buff)              ((uint8_t) ((_bits0_6(buff[5]) << 1) | _bit7(buff[6])))

/**
 * @brief TPMS data structure
 */
typedef struct STpmsData {

	uint32_t Id; /* TPMS ID */

	uint8_t PressurePsi; /* Pressure in PSI */
	uint8_t TemperatureC; /* Temperature in degrees Celsius */

} STpmsData;

/**
 * @brief TPMS data decoder return value
 */
typedef enum ETpmsDecoderRet {
	ETpmsDecoderRet_Ok = 0,
	ETpmsDecoderRet_InvalidCrc,
	ETpmsDecoderRet_PressureCheckError
} ETpmsDecoderRet;

static ETpmsDecoderRet rfRx_DecodeMessage(uint8_t buff[], STpmsData *tpmsData);

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void) {

	Nrf905Lld_SetRxPayloadWidth(TPMS_MESSAGE_PAYLOAD_WIDTH);
	/*
	 * TODO: Determine the address and its width
	 */
	Nrf905Lld_SetRxAddressWidth(TPMS_MESSAGE_ADDRESS_WIDTH);
	Nrf905Lld_SetDeviceIdentity(NULL, 0);
	Nrf905Lld_EnableCrcCheck(true);
	Nrf905Lld_SetCrcMode(ENrf905LldCrcMode_Crc8);

	Nrf905Lld_ModeSelectRx();
	Nrf905Lld_TransmissionEnable();
	Nrf905Lld_PwrUp();

}

/**
 * @brief Listens for and handles the RF message
 * @retval None
 */
void rfRx_HandleMessage(void) {

	static uint8_t rxBufTbl[TPMS_MESSAGE_PAYLOAD_WIDTH];
	/* Wait for notification from EXTI ISR */
	if (0UL < ulTaskNotifyTake(pdTRUE, 0)) {

		/* Read the payload */
		Nrf905Lld_TransmissionDisable();
		Nrf905Lld_ReadRxPayload(rxBufTbl, TPMS_MESSAGE_PAYLOAD_WIDTH);
		Nrf905Lld_TransmissionEnable();

		STpmsData tpmsData;
		/* Clear the data buffer */
		memset(&tpmsData, 0x00, sizeof(tpmsData));
		/* Parse the message */
		if (ETpmsDecoderRet_Ok != rfRx_DecodeMessage(rxBufTbl, &tpmsData)) {

			LogPrint("TPMS decoder error\r\n");
			return;

		}

		/* Handle the CAN frame */
		/*
		 * TODO: Assert all four sensors transmitted their readings
		 */

	}

}

/**
 * @brief Decodes the TPMS data message
 * @param[in] buff Message buffer of size RFRX_SPI_RX_BUFSIZE
 * @param[out] tpmsData TPMS data structure
 * @retval ETpmsDecoderRet Status
 */
static ETpmsDecoderRet rfRx_DecodeMessage(uint8_t buff[], STpmsData *tpmsData) {

	ETpmsDecoderRet ret = ETpmsDecoderRet_Ok; /* Return value */

	/* Pressure check */
	uint8_t pressureNonInverted = TPMS_SCRMBL_PRES_NONINVERTED(buff);
	uint8_t pressureInverted = buff[7];
	if (pressureNonInverted == ~pressureInverted) {

		/* Read the TPMS ID */
		tpmsData->Id = _reinterpret32bits(buff[0], buff[1], buff[2], buff[3]);
		tpmsData->PressurePsi = TPMS_PRES_SCAL_FAC
				* pressureNonInverted+ TPMS_PRES_OFFSET;
		tpmsData->TemperatureC = TPMS_SCRMBL_TEMP(buff);

	} else {

		ret = ETpmsDecoderRet_PressureCheckError;

	}

	return ret;

}
