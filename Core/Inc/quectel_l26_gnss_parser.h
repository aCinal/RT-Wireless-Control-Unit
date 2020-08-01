/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.h
 * @brief Header file containing functions and typedefs for parsing a Quectel L26 NMEA message
 */

#ifndef __QUECTEL_L26_GNSS_PARSER_H_
#define __QUECTEL_L26_GNSS_PARSER_H_

#include "main.h"
#include <stdbool.h>

/* Exported defines ------------------------------------------------------------*/
#define NMEA_PARSER_BUFFER_SIZE		512				/* Parser buffer size */

#define NMEA_RMC_RECEIVED			0x01U			/* --RMC NMEA sentence received flag */
#define NMEA_GPVTG_RECEIVED			0x02U			/* GPVTG NMEA sentence received flag */
#define NMEA_GPGGA_RECEIVED			0x04U			/* GPGGA NMEA sentence received flag */
#define NMEA_GSA_RECEIVED			0x08U			/* --GSA NMEA sentence received flag */
#define NMEA_GSV_RECEIVED			0x10U			/* --GSV NMEA sentence received flag */
#define NMEA_GLL_RECEIVED			0x20U			/* --GLL NMEA sentence received flag */
#define NMEA_GPTXT_RECEIVED			0x40U			/* GPTXT NMEA sentence received flag */

/* Exported typedefs ------------------------------------------------------------*/
typedef float float32_t; /* 32-bit floating point variable typedef */
typedef double float64_t; /* 64-bit floating point variable typedef */
typedef uint8_t NMEASentencesRxFlagsTypedef; /* Typedef for storing flags corresponding to received NMEA sentences */

/**
 * @brief GNSS data structure
 */
typedef struct {
	float64_t Latitude; /* Latitude in format 'ddmm.mmmm' (degree and minutes) */
	enum {
		GNSS_LATITUDE_NORTH = 0x00000000UL, GNSS_LATITUDE_SOUTH
	} LatDir; /* Latitude direction */

	float64_t Longitude; /* Longitude in format 'dddmm.mmmm' (degree and minutes) */
	enum {
		GNSS_LONGITUDE_EAST = 0x00000000UL, GNSS_LONGITUDE_WEST
	} LonDir; /* Longitude direction */

	float32_t Altitude; /* Altitude in meters */

	float32_t Speed; /* Speed over ground in kilometers per hour */
	float32_t Direction; /* Direction in degrees */

	uint32_t Date; /* Date formatted as ddmmyy */
	float64_t Time; /* Time formatted as hhmmss.sss */

	uint8_t SatellitesInUse; /* Number of satellites in use */
	uint8_t SatellitesVisibleGPS; /* Number of GPS satellites visible */
	uint8_t SatellitesVisibleGLONASS; /* Number of GLONASS satellites visible */
	enum {
		GNSS_GSA_NoFix = 1U, GNSS_GSA_2DFix, GNSS_GSA_3DFix
	} FixStatus; /* --GSA sentence fix status */

	NMEASentencesRxFlagsTypedef SentencesReceived; /* Sentences received flags */

} GnssDataTypedef;

/**
 * @brief NMEA parser status typedef
 */
typedef enum {
	NMEA_ERROR_NONE = 0U,
	NMEA_ERROR_BAD_LENGTH,
	NMEA_ERROR_BAD_START_SEQ,
	NMEA_ERROR_BAD_END_SEQ,
	NMEA_ERROR_BAD_CHECKSUM
} NmeaParserStatusTypedef;

/**
 * @brief GNSS data status typedef
 */
typedef enum {
	GNSS_DATA_READY = 0U,
	GNSS_DATA_PENDING,
	GNSS_DATA_ERROR
} GnssDataStatusTypedef;

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief Tests if all NMEA sentences were received
 * @param pData Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool isDataComplete(GnssDataTypedef* pData);

/**
 * @brief Tries parsing the Quectel L26 message
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pMessage Pointer to the message
 * @param[in] length Length of the message
 */
GnssDataStatusTypedef parseMessage(GnssDataTypedef* pDataBuff, const char* pMessage, size_t length);

/**
 * @brief Parses an NMEA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pSentence Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef parseNmeaSentence(GnssDataTypedef* pDataBuff, const char* pSentence, size_t length);

/**
 * @brief Normalizes the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
int32_t normalizeCoordinate(float64_t coordinate, uint32_t direction);

/**
 * @brief Normalizes the speed as kilometers per hour multiplied by 10
 * @param speed Speed over ground in kilometers per hour
 * @retval uint16_t Speed normalized as kilometers per hour multiplied by 10
 */
uint16_t normalizeSpeed(float32_t speed);

/**
 * @brief Normalizes the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
uint16_t normalizeDirection(float32_t direction);

/**
 * @brief Normalizes the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
uint16_t normalizeAltitude(float32_t altitude);

/**
 * @brief Normalizes the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
uint32_t normalizeTime(float64_t time);

#endif /* __QUECTEL_L26_GNSS_PARSER_H_ */
