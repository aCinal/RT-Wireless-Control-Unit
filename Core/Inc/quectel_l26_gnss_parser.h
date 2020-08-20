/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.h
 * @brief Header file containing function prototypes and defines for parsing a Quectel L26 NMEA message
 */

#ifndef __QUECTEL_L26_GNSS_PARSER_H_
#define __QUECTEL_L26_GNSS_PARSER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* Exported defines ------------------------------------------------------------*/
#define NMEA_PARSER_SENTENCE_BUFFER_SIZE	(uint32_t)(64UL)	/* Message parser sentence buffer size */
#define NMEA_PARSER_DATAFIELD_BUFFER_SIZE	(uint32_t)(11UL)	/* Sentence parser data field buffer size */

#define NMEA_RMC_RECEIVED					(uint8_t)(0x01U)	/* --RMC NMEA sentence received flag */
#define NMEA_VTG_RECEIVED					(uint8_t)(0x02U)	/* GPVTG NMEA sentence received flag */
#define NMEA_GGA_RECEIVED					(uint8_t)(0x04U)	/* GPGGA NMEA sentence received flag */
#define NMEA_GSA_RECEIVED					(uint8_t)(0x08U)	/* --GSA NMEA sentence received flag */
#define NMEA_GPGSV_RECEIVED					(uint8_t)(0x10U)	/* GPGSV NMEA sentence received flag */
#define NMEA_GLGSV_RECEIVED					(uint8_t)(0x20U)	/* GLGSV NMEA sentence received flag */
#define NMEA_GLL_RECEIVED					(uint8_t)(0x40U)	/* --GLL NMEA sentence received flag */
#define NMEA_TXT_RECEIVED					(uint8_t)(0x80U)	/* GPTXT NMEA sentence received flag */

/* Exported macros ------------------------------------------------------------*/
/**
 * @brief Returns the address at which the payload of an NMEA sentence begins in reference to the start
 */
#define NMEA_PAYLOAD_BEGIN(start) (char*)((char*)start + 7UL)

/**
 * @brief Returns the length of an NMEA sentence payload
 */
#define NMEA_PAYLOAD_LENGTH(sentenceLength) (size_t)(sentenceLength - 11UL)

/* Exported typedefs ------------------------------------------------------------*/
typedef float float32_t; /* 32-bit floating point variable typedef */
typedef double float64_t; /* 64-bit floating point variable typedef */
typedef uint8_t TNmeaSentencesRxFlags; /* Typedef for storing flags corresponding to received NMEA sentences */

/**
 * @brief GNSS data structure
 */
typedef struct SGnssData {
	float64_t Latitude; /* Latitude in format 'ddmm.mmmm' (degree and minutes) */
	enum {
		ELatDir_LatitudeNorth = 0x00000000UL, ELatDir_LatitudeSouth
	} ELatDir; /* Latitude direction */

	float64_t Longitude; /* Longitude in format 'dddmm.mmmm' (degree and minutes) */
	enum {
		ELonDir_LongitudeEast = 0x00000000UL, ELonDir_LongitudeWest
	} ELonDir; /* Longitude direction */

	float32_t Altitude; /* Altitude in meters */

	float32_t Speed; /* Speed over ground in kilometers per hour */
	float32_t COG; /* Course over ground in degrees */

	uint32_t Date; /* Date formatted as ddmmyy */
	float64_t Time; /* Time formatted as hhmmss.sss */

	uint8_t SatellitesInUse; /* Number of satellites in use */
	uint8_t SatellitesInViewGLONASS; /* Number of GLONASS satellites in view */
	uint8_t SatellitesInViewGPS; /* Number of GPS satellites in view */
	enum {
		EFixStatus_NoFix = 1U, EFixStatus_2DFix, EFixStatus_3DFix
	} EFixStatus; /* --GSA sentence fix status */

	TNmeaSentencesRxFlags SentencesReceived; /* Sentences received flags */

} SGnssData;

/**
 * @brief NMEA parser status typedef
 */
typedef enum ENmeaParserStatus {
	ENmeaParserStatus_OK = 0,
	ENmeaParserStatus_InvalidFormat,
	ENmeaParserStatus_InvalidChecksum,
	ENmeaParserStatus_InvalidId,
	ENmeaParserStatus_InvalidData,
} ENmeaParserStatus;

/**
 * @brief GNSS data status typedef
 */
typedef enum EGnssDataStatus {
	EGnssDataStatus_Ready = 0,
	EGnssDataStatus_Pending,
	EGnssDataStatus_Error
} EGnssDataStatus;

/**
 * @brief --GSV sentence talker ID typedef
 */
typedef enum EGsvTalkerIdTypedef {
	EGsvTalkerId_GP = 0,
	EGsvTalkerId_GL
} EGsvTalkerId;

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief Tests if all NMEA sentences were received
 * @param pData Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool isDataComplete(SGnssData *pData);

/**
 * @brief Tries parsing the Quectel L26 message
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pMessage Pointer to the message
 * @param[in] length Length of the message
 */
EGnssDataStatus parseMessage(SGnssData *pDataBuff,
		const char *pMessage, size_t length);

/**
 * @brief Parses an NMEA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pSentence Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval ENmeaParserStatus Error code
 */
ENmeaParserStatus parseNmeaSentence(SGnssData *pDataBuff,
		const char *pSentence, size_t length);

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
