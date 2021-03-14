/**
 * @author Adrian Cinal
 * @file wcu_gnss.c
 * @brief Source file implementing the GNSS service
 */

#include "wcu_gnss.h"
#include "wcu_defs.h"
#include "wcu_can.h"
#include "wcu_logger.h"
#include "wcu_events.h"
#include "rt12e_libs_uartringbuffer.h"
#include "rt12e_libs_generic.h"
#include "rt12e_libs_can.h"
#include "l26_api.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include <math.h>

#define WCU_GNSS_RING_BUFFER_SIZE   ( (uint32_t) 200 )    /* UART ring buffer size */
#define WCU_GNSS_PARSE_BUFFER_SIZE  ( (uint32_t) 50 )     /* NMEA parser's internal buffer's size */
#define WCU_CAN_ID_GPS_POS          ( (uint32_t) 0x500 )  /* CAN ID: _500_GPS_POS */
#define WCU_CAN_ID_GPS_POS2         ( (uint32_t) 0x501 )  /* CAN ID: _501_GPS_POS2 */
#define WCU_CAN_ID_GPS_STATUS       ( (uint32_t) 0x502 )  /* CAN ID: _502_GPS_STATUS */

extern UART_HandleTypeDef huart3;

SUartRb g_WcuGnssRingBuffer;

static EWcuRet WcuGnssRingBufferInit(void);
static EWcuRet WcuGnssDeviceConfig(void);
static EWcuRet WcuGnssSendCommand(char* command);
static EWcuRet WcuGnssHandleNmeaMessage(void);
static EWcuRet WcuGnssSendGpsPos(SL26ApiGnssData *data);
static EWcuRet WcuGnssSendGpsPos2(SL26ApiGnssData *data);
static EWcuRet WcuGnssSendGpsStatus(SL26ApiGnssData *data);
static void WcuGnssRxCallback(void);
static int32_t WcuGnssNormalizeCoordinate(float64_t coordinate,
		uint32_t direction);
static uint16_t WcuGnssNormalizeSpeed(float32_t speed);
static uint16_t WcuGnssNormalizeDirection(float32_t direction);
static uint16_t WcuGnssNormalizeAltitude(float32_t altitude);
static uint32_t WcuGnssNormalizeTime(float64_t time);

/**
 * @brief GNSS service startup
 * @retval None
 */
void WcuGnssStartup(void) {

	/* Initialize the ring buffer */
	if (EWcuRet_Ok == WcuGnssRingBufferInit()) {

		WcuLogInfo("WcuGnssStartup: GNSS ring buffer initialized");

	} else {

		WcuLogError("WcuGnssStartup: GNSS ring buffer initialization failed");
	}

	/* Device config */
	(void) WcuGnssDeviceConfig();
}

/**
 * @brief Handle a pending GNSS message
 * @retval None
 */
void WcuGnssHandlePendingMessage(void) {

	(void) WcuGnssHandleNmeaMessage();
}

/**
 * @brief Initialize the GNSS ring buffer
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssRingBufferInit(void) {

	EWcuRet status = EWcuRet_Ok;

	static uint8_t ringbuffer[WCU_GNSS_RING_BUFFER_SIZE];


	/* Configure the ring buffer structure */
	(void) UartRbInit(&g_WcuGnssRingBuffer, &huart3, ringbuffer, sizeof(ringbuffer), &WcuGnssRxCallback);

	/* Start listening */
	if (EUartRbRet_Ok != UartRbStart(&g_WcuGnssRingBuffer)) {

		status = EWcuRet_Error;
	}

	return status;
}

/**
 * @brief Configure the Quectel L26 device
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssDeviceConfig(void) {

	EWcuRet status = EWcuRet_Ok;

	/* Set the appropriate pin levels */
	RESET_PIN(GNSS_FORCE_ON);
	SET_PIN(GNSS_RESET);

	char PMTK_SET_POS_FIX[] = "$PMTK220,100*00\r\n";
	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	if (EWcuRet_Error == WcuGnssSendCommand(PMTK_SET_POS_FIX)) {

		status = EWcuRet_Error;
	}

	char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*00\r\n";
	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	if (EWcuRet_Error == WcuGnssSendCommand(PMTK_API_SET_GNSS_SEARCH_MODE)) {

		status = EWcuRet_Error;
	}

	return status;
}


/**
 * @brief Send a command to the Quectel L26 device
 * @param command Command string
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssSendCommand(char* command) {

	EWcuRet status = EWcuRet_Ok;

	/* Print the checksum to the command string */
	L26ApiAddNmeaChecksum(command);

	if (HAL_OK
			!= HAL_UART_Transmit(&huart3, (uint8_t*) command,
					strlen(command), 50)) {

		status = EWcuRet_Ok;
	}

	return status;
}

/**
 * @brief Handle the pending NMEA message
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssHandleNmeaMessage(void) {

	EWcuRet status = EWcuRet_Ok;

	uint8_t buffer[WCU_GNSS_PARSE_BUFFER_SIZE];
	size_t bytesRead = 0;
	/* Read the message from the buffer */
	if (EUartRbRet_Ok != UartRbRead(&g_WcuGnssRingBuffer, buffer, sizeof(buffer), &bytesRead)) {

		WcuLogError("WcuGnssHandleNmeaMessage: Ring buffer read failed");
		status = EWcuRet_Error;
	}

	if (EWcuRet_Ok == status) {

		static SL26ApiGnssData parsedData;

		/* Try parsing the message */
		switch (L26ApiParseMessage(&parsedData, (char*) buffer,
		sizeof(buffer))) {

		case EL26ApiDataStatus_Ready:

			/* Send the data to CAN */
			WcuGnssSendGpsPos(&parsedData);
			WcuGnssSendGpsPos2(&parsedData);
			WcuGnssSendGpsStatus(&parsedData);

			/* Clear the data buffer */
			(void) memset(&parsedData, 0, sizeof(parsedData));
			break;

		case EL26ApiDataStatus_NotReady:

			/* If the data is not complete, continue listening */
			break;

		case EL26ApiDataStatus_Error:

			WcuLogError("WcuGnssHandleNmeaMessage: Parser failed");
			status = EWcuRet_Error;
			break;

		default:

			break;
		}
	}

	return status;
}

/**
 * @brief Send _GPS_POS CAN frame
 * @param data Parsed GNSS data
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssSendGpsPos(SL26ApiGnssData *data) {

	SCanFrame canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = 8;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = WCU_CAN_ID_GPS_POS;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the longitude to the frame payload */
	int32_t longitude = WcuGnssNormalizeCoordinate(data->Longitude,
			data->LonDir);
	canMessage.PayloadTbl[0] = _getbyte(longitude, 3);
	canMessage.PayloadTbl[1] = _getbyte(longitude, 2);
	canMessage.PayloadTbl[2] = _getbyte(longitude, 1);
	canMessage.PayloadTbl[3] = _getbyte(longitude, 0);

	/* Write the latitude to the frame payload */
	int32_t latitude = WcuGnssNormalizeCoordinate(data->Latitude,
			data->LatDir);
	canMessage.PayloadTbl[4] = _getbyte(latitude, 3);
	canMessage.PayloadTbl[5] = _getbyte(latitude, 2);
	canMessage.PayloadTbl[6] = _getbyte(latitude, 1);
	canMessage.PayloadTbl[7] = _getbyte(latitude, 0);

	/* Transmit the frame */
	return WcuCanMessageSend(&canMessage);
}

/**
 * @brief Send _GPS_POS2 CAN frame
 * @param data Parsed GNSS data
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssSendGpsPos2(SL26ApiGnssData *data) {

	SCanFrame canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = 6;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = WCU_CAN_ID_GPS_POS2;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the speed to the frame payload */
	uint16_t speed = WcuGnssNormalizeSpeed(data->Speed);
	canMessage.PayloadTbl[0] = _getbyte(speed, 1);
	canMessage.PayloadTbl[1] = _getbyte(speed, 0);

	/* Write the direction to the frame payload */
	uint16_t direction = WcuGnssNormalizeDirection(data->Cog);
	canMessage.PayloadTbl[2] = _getbyte(direction, 1);
	canMessage.PayloadTbl[3] = _getbyte(direction, 0);

	/* Write the altitude to the frame payload */
	uint16_t altitude = WcuGnssNormalizeAltitude(data->Altitude);
	canMessage.PayloadTbl[4] = _getbyte(altitude, 1);
	canMessage.PayloadTbl[5] = _getbyte(altitude, 0);

	/* Transmit the frame */
	return WcuCanMessageSend(&canMessage);
}

/**
 * @brief Send _GPS_POS STATUS frame
 * @param data Parsed GNSS data
 * @retval EWcuRet Status
 */
static EWcuRet WcuGnssSendGpsStatus(SL26ApiGnssData *data) {

	SCanFrame canMessage;
	/* Configure the CAN Tx header */
	canMessage.TxHeader.DLC = 8;
	canMessage.TxHeader.IDE = CAN_ID_STD;
	canMessage.TxHeader.RTR = CAN_RTR_DATA;
	canMessage.TxHeader.StdId = WCU_CAN_ID_GPS_STATUS;
	canMessage.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canMessage.PayloadTbl[0] = data->SatellitesInViewGlonass
			+ data->SatellitesInViewGps;
	/* Clear two most significant bits */
	canMessage.PayloadTbl[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canMessage.PayloadTbl[0] |= (uint8_t) data->FixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canMessage.PayloadTbl[1] = data->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = WcuGnssNormalizeTime(data->Time);
	canMessage.PayloadTbl[2] = _getbyte(time, 3);
	canMessage.PayloadTbl[3] = _getbyte(time, 2);
	canMessage.PayloadTbl[4] = _getbyte(time, 1);
	canMessage.PayloadTbl[5] = _getbyte(time, 0);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canMessage.PayloadTbl[5] |= 0xF & _getbyte(data->Date, 3);
	canMessage.PayloadTbl[6] = 0xFF & (data->Date >> 20U);
	canMessage.PayloadTbl[7] = 0xFF & (data->Date >> 12U);

	/* Transmit the frame */
	return WcuCanMessageSend(&canMessage);
}

/**
 * @brief Message received callback
 * @retval None
 */
static void WcuGnssRxCallback(void) {

	(void) WcuEventSend(EWcuEventSignal_GnssPendingMessage, NULL);
}


/**
 * @brief Normalize the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
static int32_t WcuGnssNormalizeCoordinate(float64_t coordinate,
		uint32_t direction) {

	/* Separate degrees from the minutes */
	float64_t degrees = floor(coordinate / 100.0);
	/* Get minutes */
	float64_t minutes = coordinate - (degrees * 100.0);
	/* Add minutes as decimal fraction to the degrees */
	float64_t floatResult = degrees + (minutes / 60.0);

	/* Make the result negative if the direction is SOUTH or WEST */
	if ((ELatDir_LatitudeSouth == direction)
			|| (ELonDir_LongitudeWest == direction)) {

		floatResult *= -1.0;

	}

	/* Round the result and multiply by 1000000.0 to get a 32-bit unsigned integer */
	int32_t intResult = llround(floatResult * 1000000.0);

	return intResult;
}

/**
 * @brief Normalize the speed as kilometers per hour multiplied by 10
 * @param speed Speed over ground in kilometers per hour
 * @retval uint16_t Speed normalized as kilometers per hour multiplied by 10
 */
static uint16_t WcuGnssNormalizeSpeed(float32_t speed) {

	return lround(speed * 10.0);
}

/**
 * @brief Normalize the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
static uint16_t WcuGnssNormalizeDirection(float32_t direction) {

	return lround(direction * 10.0);
}

/**
 * @brief Normalize the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
static uint16_t WcuGnssNormalizeAltitude(float32_t altitude) {

	return lround(altitude * 10.0);
}

/**
 * @brief Normalize the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
static uint32_t WcuGnssNormalizeTime(float64_t time) {

	return llround(time * 1000.0);
}
