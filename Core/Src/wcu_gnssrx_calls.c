/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.c
 * @brief Source file defining functions called by the gnssRx task
 */

#include <l26_api.h>
#include "wcu_gnssrx_calls.h"

#include "wcu_common.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define CAN_ID_GPS_POS        ( (uint32_t) 0x500 )  /* CAN ID: _500_GPS_POS */
#define CAN_ID_GPS_POS2       ( (uint32_t) 0x501 )  /* CAN ID: _501_GPS_POS2 */
#define CAN_ID_GPS_STATUS     ( (uint32_t) 0x502 )  /* CAN ID: _502_GPS_STATUS */
#define GNSSRX_READ_BUF_SIZE  ( (uint32_t) 50 )     /* Read buffer size */
#define GNSSRX_RING_BUF_SIZE  ( (uint32_t) 200 )    /* UART ring buffer size */
#define GNSS_UART_HANDLE      (huart3)              /* UART handle alias */
#define GNSS_UART_INSTANCE    (USART3)              /* UART instance alias */

extern UART_HandleTypeDef GNSS_UART_HANDLE;
extern osThreadId gnssRxHandle;
SUartRingBuf gGnssRxRingBuffer;

static EGnssRxRet GnssRxSendCommand(char *message);
static void GnssRxRingBufferIdleCallback(void);
static void GnssRxTransmitGpsPos(SL26ApiGnssData *gnssDataPtr);
static void GnssRxTransmitGpsPos2(SL26ApiGnssData *gnssDataPtr);
static void GnssRxTransmitGpsStatus(SL26ApiGnssData *gnssDataPtr);
static int32_t GnssRxNormalizeCoordinate(float64_t coordinate,
		uint32_t direction);
static uint16_t GnssRxNormalizeSpeed(float32_t speed);
static uint16_t GnssRxNormalizeDirection(float32_t direction);
static uint16_t GnssRxNormalizeAltitude(float32_t altitude);
static uint32_t GnssRxNormalizeTime(float64_t time);

/**
 * @brief Configure the Quectel L26 device
 * @retval EGnssRxRet Status
 */
EGnssRxRet GnssRxDeviceConfig(void) {

	EGnssRxRet status = EGnssRxRet_Ok;

	/* Set the appropriate pin levels */
	RESET_PIN(GNSS_FORCE_ON);
	SET_PIN(GNSS_RESET);

	/* Wait for the device to turn on and set up */
	vTaskDelay(pdMS_TO_TICKS(1000));

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	if (EGnssRxRet_Error == GnssRxSendCommand("$PMTK220,100*00\r\n")) {

		status = EGnssRxRet_Error;

	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	if (EGnssRxRet_Error == GnssRxSendCommand("$PMTK353,1,1,0,0,0*00\r\n")) {

		status = EGnssRxRet_Error;

	}

	return status;

}

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet GnssRxStartRingBufferIdleDetectionRx(void) {

	static uint8_t ringBufTbl[GNSSRX_RING_BUF_SIZE];

	/* Configure the ring buffer structure */
	gGnssRxRingBuffer.BufferPtr = ringBufTbl;
	gGnssRxRingBuffer.BufferSize = GNSSRX_RING_BUF_SIZE;
	gGnssRxRingBuffer.Callback = &GnssRxRingBufferIdleCallback;
	gGnssRxRingBuffer.PeriphHandlePtr = &GNSS_UART_HANDLE;

	/* Start listening */
	return UartRingBufStart(&gGnssRxRingBuffer);

}

/**
 * @brief Listen for and handles the GNSS message
 * @retval EGnssRxRet Status
 */
EGnssRxRet GnssRxHandleCom(void) {

	EGnssRxRet status = EGnssRxRet_Ok;

	/* Wait for notification from idle line detection callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, WCU_COMMON_TIMEOUT)) {

		static uint8_t rxBufTbl[GNSSRX_READ_BUF_SIZE];
		/* Read the data from the ring buffer */
		UartRingBufRead(&gGnssRxRingBuffer, rxBufTbl, GNSSRX_READ_BUF_SIZE);

		static SL26ApiGnssData dataBuf;
		/* Try parsing the message */
		switch (L26ApiParseMessage(&dataBuf, (char*) rxBufTbl,
		GNSSRX_READ_BUF_SIZE)) {

		case EL26ApiDataStatus_Ready:

			/* Send the data to CAN */
			GnssRxTransmitGpsPos(&dataBuf);
			GnssRxTransmitGpsPos2(&dataBuf);
			GnssRxTransmitGpsStatus(&dataBuf);

			/* Clear the data buffer */
			(void) memset(&dataBuf, 0, sizeof(dataBuf));
			break;

		case EL26ApiDataStatus_Pending:

			/* If the data is not complete, continue listening */
			break;

		case EL26ApiDataStatus_Error:

			LogError("GnssRxHandleCom: Parser failed");
			status = EGnssRxRet_Error;
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Send a command to the Quectel L26-DR device
 * @param message Message string
 * @retval EGnssRxRet Status
 */
static EGnssRxRet GnssRxSendCommand(char *message) {

	EGnssRxRet status = EGnssRxRet_Ok;

	/* Print the checksum to the message string */
	L26ApiAddNmeaChecksum(message);

	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) message,
					strlen(message), WCU_COMMON_TIMEOUT)) {

		status = EGnssRxRet_Error;

	}

	return status;

}

/**
 * @brief Callback on idle line detection in the ring buffer implementation
 * @retval None
 */
static void GnssRxRingBufferIdleCallback(void) {

	/* Notify the task */
	vTaskNotifyGiveFromISR(gnssRxHandle, NULL);

}

/**
 * @brief Send _GPS_POS CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void GnssRxTransmitGpsPos(SL26ApiGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 8;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_POS;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the longitude to the frame payload */
	int32_t longitude = GnssRxNormalizeCoordinate(gnssDataPtr->Longitude,
			gnssDataPtr->LonDir);
	canFrame.PayloadTbl[0] = _getbyte(longitude, 3);
	canFrame.PayloadTbl[1] = _getbyte(longitude, 2);
	canFrame.PayloadTbl[2] = _getbyte(longitude, 1);
	canFrame.PayloadTbl[3] = _getbyte(longitude, 0);

	/* Write the latitude to the frame payload */
	int32_t latitude = GnssRxNormalizeCoordinate(gnssDataPtr->Latitude,
			gnssDataPtr->LatDir);
	canFrame.PayloadTbl[4] = _getbyte(latitude, 3);
	canFrame.PayloadTbl[5] = _getbyte(latitude, 2);
	canFrame.PayloadTbl[6] = _getbyte(latitude, 1);
	canFrame.PayloadTbl[7] = _getbyte(latitude, 0);

	/* Transmit the frame */
	SendToCan(&canFrame);
	LogDebug("GnssRxTransmitGpsPos: Frame sent");

}

/**
 * @brief Send _GPS_POS2 CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void GnssRxTransmitGpsPos2(SL26ApiGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 6;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_POS2;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the speed to the frame payload */
	uint16_t speed = GnssRxNormalizeSpeed(gnssDataPtr->Speed);
	canFrame.PayloadTbl[0] = _getbyte(speed, 1);
	canFrame.PayloadTbl[1] = _getbyte(speed, 0);

	/* Write the direction to the frame payload */
	uint16_t direction = GnssRxNormalizeDirection(gnssDataPtr->Cog);
	canFrame.PayloadTbl[2] = _getbyte(direction, 1);
	canFrame.PayloadTbl[3] = _getbyte(direction, 0);

	/* Write the altitude to the frame payload */
	uint16_t altitude = GnssRxNormalizeAltitude(gnssDataPtr->Altitude);
	canFrame.PayloadTbl[4] = _getbyte(altitude, 1);
	canFrame.PayloadTbl[5] = _getbyte(altitude, 0);

	/* Transmit the frame */
	SendToCan(&canFrame);
	LogDebug("GnssRxTransmitGpsPos2: Frame sent");

}

/**
 * @brief Send _GPS_POS CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void GnssRxTransmitGpsStatus(SL26ApiGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 8;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_STATUS;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canFrame.PayloadTbl[0] = gnssDataPtr->SatellitesInViewGlonass
			+ gnssDataPtr->SatellitesInViewGps;
	/* Clear two most significant bits */
	canFrame.PayloadTbl[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canFrame.PayloadTbl[0] |= (uint8_t) gnssDataPtr->FixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canFrame.PayloadTbl[1] = gnssDataPtr->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = GnssRxNormalizeTime(gnssDataPtr->Time);
	canFrame.PayloadTbl[2] = _getbyte(time, 3);
	canFrame.PayloadTbl[3] = _getbyte(time, 2);
	canFrame.PayloadTbl[4] = _getbyte(time, 1);
	canFrame.PayloadTbl[5] = _getbyte(time, 0);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canFrame.PayloadTbl[5] |= 0xF & _getbyte(gnssDataPtr->Date, 3);
	canFrame.PayloadTbl[6] = 0xFF & (gnssDataPtr->Date >> 20U);
	canFrame.PayloadTbl[7] = 0xFF & (gnssDataPtr->Date >> 12U);

	/* Transmit the frame */
	SendToCan(&canFrame);
	LogDebug("GnssRxTransmitGpsStatus: Frame sent");

}

/**
 * @brief Normalize the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
static int32_t GnssRxNormalizeCoordinate(float64_t coordinate,
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
static uint16_t GnssRxNormalizeSpeed(float32_t speed) {

	return lround(speed * 10.0);

}

/**
 * @brief Normalize the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
static uint16_t GnssRxNormalizeDirection(float32_t direction) {

	return lround(direction * 10.0);

}

/**
 * @brief Normalize the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
static uint16_t GnssRxNormalizeAltitude(float32_t altitude) {

	return lround(altitude * 10.0);

}

/**
 * @brief Normalize the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
static uint32_t GnssRxNormalizeTime(float64_t time) {

	return llround(time * 1000.0);

}
