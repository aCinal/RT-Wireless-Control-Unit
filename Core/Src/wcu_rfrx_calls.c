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

#define TPMS_MESSAGE_PAYLOAD_WIDTH            ((uint32_t) 8)           /* TPMS message size */
#define TPMS_MESSAGE_ADDRESS_WIDTH            ((uint32_t) 4)           /* TPMS address size */

#define TPMS_ID_FL                            ((uint32_t) 0x00000000)  /* TBD experimentally */
#define TPMS_ID_FR                            ((uint32_t) 0x00000001)  /* TBD experimentally */
#define TPMS_ID_RL                            ((uint32_t) 0x00000002)  /* TBD experimentally */
#define TPMS_ID_RR                            ((uint32_t) 0x00000003)  /* TBD experimentally */
#define TPMS_PRES_SCAL_FAC                    ((float32_t) 0.25)       /* TPMS pressure scaling factor */
#define TPMS_PRES_OFFSET                      ((float32_t) (-7.))      /* TPMS pressure offset */
#define TPMS_FL_RECEIVED                      ((uint8_t) 0x01)         /* TPMS front-left message received */
#define TPMS_FR_RECEIVED                      ((uint8_t) 0x02)         /* TPMS front-right message received */
#define TPMS_RL_RECEIVED                      ((uint8_t) 0x04)         /* TPMS rear-left message received */
#define TPMS_RR_RECEIVED                      ((uint8_t) 0x08)         /* TPMS rear-right message received */

#define _bits0_6(x)                           ((uint8_t) (x & 0x7F))
#define _bit7(x)                              ((uint8_t) ((x >> 7) & 0x01))

/* Macros for scrambling the TPMS data from the bits of the message */
#define TPMS_SCRMBL_PRES_NONINVERTED(bufTbl)  ((uint8_t) ((_bits0_6(bufTbl[4]) << 1) | _bit7(bufTbl[5])))
#define TPMS_SCRMBL_TEMP(bufTbl)              ((uint8_t) ((_bits0_6(bufTbl[5]) << 1) | _bit7(bufTbl[6])))

/* Typedef for storing flags corresponding to received TPMS packets */
typedef uint8_t TTpmsPacketRxFlags;

/**
 * @brief TPMS data packet structure
 */
typedef struct STpmsDataPacket {

	uint32_t Id; /* TPMS ID */

	uint8_t PressurePsi; /* Pressure in PSI */
	uint8_t TemperatureC; /* Temperature in degrees Celsius */

} STpmsDataPacket;

/**
 * @brief TPMS data decoder return value enumeration
 */
typedef enum ETpmsDecoderRet {
	ETpmsDecoderRet_Ok = 0,
	ETpmsDecoderRet_InvalidCrc,
	ETpmsDecoderRet_PressureCheckError
} ETpmsDecoderRet;

/**
 * @brief TPMS data status enumeration
 */
typedef enum ETpmsDataStatus {
	ETpmsDataStatus_Ready = 0, ETpmsDataStatus_Pending
} ETpmsDataStatus;

static ETpmsDecoderRet rfRx_DecodeMessage(STpmsDataPacket *tpmsDataPcktPtr,
		uint8_t bufTbl[]);
static ETpmsDataStatus rfRx_AddDataToCanFrame(SCanFrame *canFrPtr,
		STpmsDataPacket tpmsDataPckt);

/**
 * @brief Configures the nRF905 device
 * @retval None
 */
void rfRx_DeviceConfig(void) {

	Nrf905Lld_RestoreDefaultConfig();
	Nrf905Lld_SetRxPayloadWidth(TPMS_MESSAGE_PAYLOAD_WIDTH);
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

		STpmsDataPacket tpmsDataPckt;
		/* Clear the data buffer */
		(void) memset(&tpmsDataPckt, 0, sizeof(tpmsDataPckt));
		/* Parse the message */
		if (ETpmsDecoderRet_Ok != rfRx_DecodeMessage(&tpmsDataPckt, rxBufTbl)) {

			LogPrint("TPMS decoder error\r\n");
			return;

		}

		static SCanFrame canFrame;
		/* Add data to CAN frame */
		if (ETpmsDataStatus_Ready
				== rfRx_AddDataToCanFrame(&canFrame, tpmsDataPckt)) {

			/* Send data to CAN bus */
			AddToCanTxQueue(&canFrame, "rfRx failed to send to canTxQueue\r\n");

			/* Clear the frame buffer */
			(void) memset(&canFrame, 0, sizeof(canFrame));

		}

	}

}

/**
 * @brief Decodes the TPMS data message
 * @param[out] tpmsDataPcktPtr TPMS data structure
 * @param[in] bufTbl Message buffer of size RFRX_SPI_RX_BUFSIZE
 * @retval ETpmsDecoderRet Status
 */
static ETpmsDecoderRet rfRx_DecodeMessage(STpmsDataPacket *tpmsDataPcktPtr,
		uint8_t bufTbl[]) {

	ETpmsDecoderRet ret = ETpmsDecoderRet_Ok; /* Return value */

	/* Pressure check */
	uint8_t pressureNonInverted = TPMS_SCRMBL_PRES_NONINVERTED(bufTbl);
	uint8_t pressureInverted = bufTbl[7];
	if (pressureNonInverted == ~pressureInverted) {

		/* Read the TPMS ID */
		tpmsDataPcktPtr->Id = _reinterpret32bits(bufTbl[0], bufTbl[1],
				bufTbl[2], bufTbl[3]);
		tpmsDataPcktPtr->PressurePsi = ((TPMS_PRES_SCAL_FAC * pressureNonInverted)
				+ TPMS_PRES_OFFSET);
		tpmsDataPcktPtr->TemperatureC = TPMS_SCRMBL_TEMP(bufTbl);

	} else {

		ret = ETpmsDecoderRet_PressureCheckError;

	}

	return ret;

}

/**
 * @brief Adds TPMS data to a CAN frame
 * @param[in,out] canFrPtr CAN frame structure
 * @param[in] tpmsDataPckt TPMS data packet structure
 * @retval ETpmsDataStatus Data status
 */
static ETpmsDataStatus rfRx_AddDataToCanFrame(SCanFrame *canFrPtr,
		STpmsDataPacket tpmsDataPckt) {

	static TTpmsPacketRxFlags packetsReceived = 0;
	ETpmsDataStatus ret = ETpmsDataStatus_Pending;

	/* Identify the sensor */
	switch (tpmsDataPckt.Id) {

	case TPMS_ID_FL:

		canFrPtr->PayloadTbl[0] = tpmsDataPckt.PressurePsi;
		canFrPtr->PayloadTbl[4] = tpmsDataPckt.TemperatureC;
		packetsReceived = TPMS_FL_RECEIVED;
		break;

	case TPMS_ID_FR:

		canFrPtr->PayloadTbl[1] = tpmsDataPckt.PressurePsi;
		canFrPtr->PayloadTbl[5] = tpmsDataPckt.TemperatureC;
		packetsReceived = TPMS_FR_RECEIVED;
		break;

	case TPMS_ID_RL:

		canFrPtr->PayloadTbl[2] = tpmsDataPckt.PressurePsi;
		canFrPtr->PayloadTbl[6] = tpmsDataPckt.TemperatureC;
		packetsReceived = TPMS_RL_RECEIVED;
		break;

	case TPMS_ID_RR:

		canFrPtr->PayloadTbl[3] = tpmsDataPckt.PressurePsi;
		canFrPtr->PayloadTbl[7] = tpmsDataPckt.TemperatureC;
		packetsReceived = TPMS_RR_RECEIVED;
		break;

	default:

		LogPrint("Unknown TPMS ID received\r\n");

	}

	/* Test if all packets have been received */
	if((TPMS_FL_RECEIVED | TPMS_FR_RECEIVED | TPMS_RL_RECEIVED | TPMS_RR_RECEIVED) == packetsReceived) {

		/* Clear the flags */
		packetsReceived = 0;
		ret = ETpmsDataStatus_Ready;

	}

	return ret;

}
