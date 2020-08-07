/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.h
 * @brief Header file containing functions and typedefs for parsing a Quectel L26 NMEA message
 */

#ifndef __QUECTEL_L26_GNSS_PARSER_H_
#define __QUECTEL_L26_GNSS_PARSER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/* Exported defines ------------------------------------------------------------*/
#define NMEA_PARSER_SENTENCE_BUFFER_SIZE	512				/* Message parser sentence buffer size */
#define NMEA_PARSER_DATAFIELD_BUFFER_SIZE	11				/* Sentence parser data field buffer size */

#define NMEA_RMC_RECEIVED					0x01U			/* --RMC NMEA sentence received flag */
#define NMEA_VTG_RECEIVED					0x02U			/* GPVTG NMEA sentence received flag */
#define NMEA_GGA_RECEIVED					0x04U			/* GPGGA NMEA sentence received flag */
#define NMEA_GSA_RECEIVED					0x08U			/* --GSA NMEA sentence received flag */
#define NMEA_GPGSV_RECEIVED					0x10U			/* GPGSV NMEA sentence received flag */
#define NMEA_GLGSV_RECEIVED					0x20U			/* GLGSV NMEA sentence received flag */
#define NMEA_GLL_RECEIVED					0x40U			/* --GLL NMEA sentence received flag */
#define NMEA_TXT_RECEIVED					0x80U			/* GPTXT NMEA sentence received flag */

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
	float32_t COG; /* Course over ground in degrees */

	uint32_t Date; /* Date formatted as ddmmyy */
	float64_t Time; /* Time formatted as hhmmss.sss */

	uint8_t SatellitesInUse; /* Number of satellites in use */
	uint8_t SatellitesInViewGLONASS; /* Number of GLONASS satellites in view */
	uint8_t SatellitesInViewGPS; /* Number of GPS satellites in view */
	enum {
		GNSS_FixStatus_NoFix = 1U, GNSS_FixStatus_2DFix, GNSS_FixStatus_3DFix
	} FixStatus; /* --GSA sentence fix status */

	NMEASentencesRxFlagsTypedef SentencesReceived; /* Sentences received flags */

} GnssDataTypedef;

/**
 * @brief NMEA parser status typedef
 */
typedef enum {
	NMEA_ERROR_NONE = 0U,
	NMEA_ERROR_INVALID_LENGTH,
	NMEA_ERROR_INVALID_FORMAT,
	NMEA_ERROR_INVALID_CHECKSUM,
	NMEA_ERROR_INVALID_ID,
	NMEA_ERROR_INVALID_DATA
} NmeaParserStatusTypedef;

/**
 * @brief GNSS data status typedef
 */
typedef enum {
	GNSS_DATA_READY = 0U, GNSS_DATA_PENDING, GNSS_DATA_ERROR
} GnssDataStatusTypedef;\

/**
 * @brief --GSV sentence talker ID typedef
 */
typedef enum {
	GSV_TALKER_ID_GP = 0U,
	GSV_TALKER_ID_GL
} GsvTalkerIdTypedef;

/* Exported function prototypes -----------------------------------------------*/

/**
 * @brief Tests if all NMEA sentences were received
 * @param pData Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool isDataComplete(GnssDataTypedef *pData);

/**
 * @brief Tries parsing the Quectel L26 message
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pMessage Pointer to the message
 * @param[in] length Length of the message
 */
GnssDataStatusTypedef parseMessage(GnssDataTypedef *pDataBuff,
		const char *pMessage, size_t length);

/**
 * @brief Parses an NMEA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pSentence Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef parseNmeaSentence(GnssDataTypedef *pDataBuff,
		const char *pSentence, size_t length);

/**
 * @brief Parses the payload of an NMEA --RMC sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseRmcPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

/**
 * @brief Parses the payload of an NMEA GPVTG sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseVtgPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

/**
 * @brief Parses the payload of an NMEA GPGGA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGgaPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

/**
 * @brief Parses the payload of an NMEA --GSA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGsaPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

/**
 * @brief Parses the payload of an NMEA --GSV sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @param[in] talkerId The --GSV sentence talker ID
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGsvPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length, GsvTalkerIdTypedef talkerId);

/**
 * @brief Parses the payload of an NMEA --GLL sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGllPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

/**
 * @brief Parses the payload of an NMEA GPTXT sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseTxtPayload(GnssDataTypedef *pDataBuff, const char *pPayload, size_t length);

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
