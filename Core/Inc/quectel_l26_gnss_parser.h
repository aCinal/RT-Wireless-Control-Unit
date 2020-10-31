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
#define NMEA_RMC_RECEIVED    ( (uint8_t) 0x01U )  /* --RMC NMEA sentence received flag */
#define NMEA_VTG_RECEIVED    ( (uint8_t) 0x02U )  /* GPVTG NMEA sentence received flag */
#define NMEA_GGA_RECEIVED    ( (uint8_t) 0x04U )  /* GPGGA NMEA sentence received flag */
#define NMEA_GSA_RECEIVED    ( (uint8_t) 0x08U )  /* --GSA NMEA sentence received flag */
#define NMEA_GPGSV_RECEIVED  ( (uint8_t) 0x10U )  /* GPGSV NMEA sentence received flag */
#define NMEA_GLGSV_RECEIVED  ( (uint8_t) 0x20U )  /* GLGSV NMEA sentence received flag */
#define NMEA_GLL_RECEIVED    ( (uint8_t) 0x40U )  /* --GLL NMEA sentence received flag */
#define NMEA_TXT_RECEIVED    ( (uint8_t) 0x80U )  /* GPTXT NMEA sentence received flag */

/* Exported macros ------------------------------------------------------------*/
/**
 * @brief Returns the address at which the payload of an NMEA sentence begins in reference to the start
 */
#define NMEA_PAYLOAD_BEGIN(start)  ( (char*)&( ( (char*)(start) )[7]) )

/**
 * @brief Returns the length of an NMEA sentence payload
 */
#define NMEA_PAYLOAD_LENGTH(sentenceLength) ( (size_t)( (sentenceLength) - 11UL ) )

/* Exported typedefs ------------------------------------------------------------*/
typedef float float32_t;                /* 32-bit floating point variable typedef */
typedef double float64_t;               /* 64-bit floating point variable typedef */
typedef uint8_t TNmeaSentencesRxFlags;  /* Typedef for storing flags corresponding to received NMEA sentences */

/**
 * @brief GNSS data structure
 */
typedef struct SGnssData {
	float64_t Latitude; /* Latitude in format 'ddmm.mmmm' (degree and minutes) */
	enum ELatDir {
		ELatDir_LatitudeNorth = 0x00000000UL, ELatDir_LatitudeSouth
	} LatDir; /* Latitude direction */

	float64_t Longitude; /* Longitude in format 'dddmm.mmmm' (degree and minutes) */
	enum ELonDir {
		ELonDir_LongitudeEast = 0x00000000UL, ELonDir_LongitudeWest
	} LonDir; /* Longitude direction */

	float32_t Altitude; /* Altitude in meters */

	float32_t Speed; /* Speed over ground in kilometers per hour */
	float32_t COG; /* Course over ground in degrees */

	uint32_t Date; /* Date formatted as ddmmyy */
	float64_t Time; /* Time formatted as hhmmss.sss */

	uint8_t SatellitesInUse; /* Number of satellites in use */
	uint8_t SatellitesInViewGLONASS; /* Number of GLONASS satellites in view */
	uint8_t SatellitesInViewGPS; /* Number of GPS satellites in view */
	enum EFixStatus {
		EFixStatus_NoFix = 1U, EFixStatus_2DFix, EFixStatus_3DFix
	} FixStatus; /* --GSA sentence fix status */

	TNmeaSentencesRxFlags SentencesReceived; /* Sentences received flags */

} SGnssData;

/**
 * @brief NMEA parser status typedef
 */
typedef enum ENmeaParserRet {
	ENmeaParserRet_Ok = 0,
	ENmeaParserRet_InvalidFormat,
	ENmeaParserRet_InvalidChecksum,
	ENmeaParserRet_InvalidId,
	ENmeaParserRet_InvalidData,
} ENmeaParserRet;

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
 * @brief Test if all NMEA sentences were received
 * @param dataPtr Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool IsDataComplete(SGnssData *dataPtr);

/**
 * @brief Try parsing the Quectel L26 message
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] messagePtr Pointer to the message
 * @param[in] length Length of the message
 */
EGnssDataStatus ParseMessage(SGnssData *dataBufPtr,
		const char *messagePtr, size_t length);

/**
 * @brief Parse an NMEA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] sentencePtr Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval ENmeaParserRet Status
 */
ENmeaParserRet ParseNmeaSentence(SGnssData *dataBufPtr,
		const char *sentencePtr, size_t length);

/**
 * @brief Normalize the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
int32_t NormalizeCoordinate(float64_t coordinate, uint32_t direction);

/**
 * @brief Normalize the speed as kilometers per hour multiplied by 10
 * @param speed Speed over ground in kilometers per hour
 * @retval uint16_t Speed normalized as kilometers per hour multiplied by 10
 */
uint16_t NormalizeSpeed(float32_t speed);

/**
 * @brief Normalize the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
uint16_t NormalizeDirection(float32_t direction);

/**
 * @brief Normalize the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
uint16_t NormalizeAltitude(float32_t altitude);

/**
 * @brief Normalize the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
uint32_t NormalizeTime(float64_t time);

#endif /* __QUECTEL_L26_GNSS_PARSER_H_ */
