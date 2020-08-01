/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.c
 * @brief Source file defining functions for parsing a Quectel L26 NMEA message
 */

#include <quectel_l26_gnss_parser.h>
#include <math.h>

/**
 * @brief Tests if all NMEA sentences were received
 * @param pData Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool isDataComplete(GnssDataTypedef *pData) {
	/* Test if all flags are set */
	return ((NMEA_RMC_RECEIVED | NMEA_GPVTG_RECEIVED | NMEA_GPGGA_RECEIVED
			| NMEA_GSA_RECEIVED | NMEA_GSV_RECEIVED | NMEA_GLL_RECEIVED
			| NMEA_GPTXT_RECEIVED) == pData->SentencesReceived);
}

/**
 * @brief Tries parsing the Quectel L26 message
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pMessage Pointer to the message
 * @param[in] length Length of the message
 */
GnssDataStatusTypedef parseMessage(GnssDataTypedef* pDataBuff, const char* pMessage, size_t length) {
	static struct {
		char Buff[NMEA_PARSER_BUFFER_SIZE];
		size_t SentenceLength;
	} messageBuffer;

	/*
	 * TODO: Parse the message
	 */
	(void)messageBuffer;

	return (isDataComplete(pDataBuff) ? GNSS_DATA_READY : GNSS_DATA_PENDING);
}

/**
 * @brief Parses an NMEA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pSentence Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef parseNmeaSentence(GnssDataTypedef* pDataBuff, const char* pSentence, size_t length) {
	/*
	 * TODO: Parse NMEA sentence
	 */
	return NMEA_ERROR_NONE;
}

/**
 * @brief Normalizes the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
int32_t normalizeCoordinate(float64_t coordinate, uint32_t direction) {
	/* Separate degrees from the minutes */
	float64_t degrees = floor(coordinate / 100.0);
	/* Get minutes */
	float64_t minutes = coordinate - (degrees * 100.0);
	/* Add minutes as decimal fraction to the degrees */
	float64_t floatResult = degrees + (minutes / 60.0);
	/* Make the result negative if the direction is SOUTH or WEST */
	if ((GNSS_LATITUDE_SOUTH == direction)
			|| (GNSS_LONGITUDE_WEST == direction)) {
		floatResult *= -1.0;
	}
	/* Round the result and multiply by 1000000.0 to get a 32-bit unsigned integer */
	int32_t intResult = llround(floatResult * 1000000.0);
	return intResult;
}

/**
 * @brief Normalizes the speed as kilometers per hour multiplied by 10
 * @param speed Speed over ground in kilometers per hour
 * @retval uint16_t Speed normalized as kilometers per hour multiplied by 10
 */
uint16_t normalizeSpeed(float32_t speed) {
	return lround(speed * 10.0);
}

/**
 * @brief Normalizes the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
uint16_t normalizeDirection(float32_t direction) {
	return lround(direction * 10.0);
}

/**
 * @brief Normalizes the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
uint16_t normalizeAltitude(float32_t altitude) {
	return lround(altitude * 10.0);
}

/**
 * @brief Normalizes the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
uint32_t normalizeTime(float64_t time) {
	return llround(time * 1000.0);
}
