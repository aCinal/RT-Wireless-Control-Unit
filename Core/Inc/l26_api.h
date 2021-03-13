/**
 * @file l26dr_api.h
 * @author Adrian Cinal
 * @brief L26-DR API header file
 */

#ifndef __L26_API_H_
#define __L26_API_H_

/*---------------------------------------------- Includes ----------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

/*---------------------------------------------- Typedefs ----------------------------------------------*/

typedef float float32_t;                /* 32-bit floating point variable typedef */
typedef double float64_t;               /* 64-bit floating point variable typedef */
typedef uint8_t TNmeaSentencesRxFlags;  /* Typedef for storing flags corresponding to received NMEA sentences */

/**
 * @brief GNSS data structure
 */
typedef struct SL26ApiGnssData {
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
	float32_t Cog; /* Course over ground in degrees */

	uint32_t Date; /* Date formatted as ddmmyy */
	float64_t Time; /* Time formatted as hhmmss.sss */

	uint8_t SatellitesInUse; /* Number of satellites in use */
	uint8_t SatellitesInViewGlonass; /* Number of GLONASS satellites in view */
	uint8_t SatellitesInViewGps; /* Number of GPS satellites in view */
	enum EFixStatus {
		EFixStatus_NoFix = 1U, EFixStatus_2dFix, EFixStatus_3dFix
	} FixStatus; /* --GSA sentence fix status */

	TNmeaSentencesRxFlags SentencesReceived; /* Sentences received flags */

} SL26ApiGnssData;

/**
 * @brief GNSS data status typedef
 */
typedef enum EL26ApiDataStatus {
	EL26ApiDataStatus_Ready = 0,
	EL26ApiDataStatus_NotReady,
	EL26ApiDataStatus_Error
} EL26ApiDataStatus;

/*---------------------------------------------- Defines ----------------------------------------------*/

#define L26API_NMEA_RMC_RECEIVED    ( (uint8_t) 0x01U )  /* --RMC NMEA sentence received flag */
#define L26API_NMEA_VTG_RECEIVED    ( (uint8_t) 0x02U )  /* GPVTG NMEA sentence received flag */
#define L26API_NMEA_GGA_RECEIVED    ( (uint8_t) 0x04U )  /* GPGGA NMEA sentence received flag */
#define L26API_NMEA_GSA_RECEIVED    ( (uint8_t) 0x08U )  /* --GSA NMEA sentence received flag */
#define L26API_NMEA_GPGSV_RECEIVED  ( (uint8_t) 0x10U )  /* GPGSV NMEA sentence received flag */
#define L26API_NMEA_GLGSV_RECEIVED  ( (uint8_t) 0x20U )  /* GLGSV NMEA sentence received flag */
#define L26API_NMEA_GLL_RECEIVED    ( (uint8_t) 0x40U )  /* --GLL NMEA sentence received flag */
#define L26API_NMEA_TXT_RECEIVED    ( (uint8_t) 0x80U )  /* GPTXT NMEA sentence received flag */

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Add an NMEA checksum to the sentence string
 * @param message Message string
 * @retval None
 */
void L26ApiAddNmeaChecksum(char* message);

/**
 * @brief Try parsing the Quectel L26-DR message
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] messagePtr Pointer to the message
 * @param[in] length Length of the message
 */
EL26ApiDataStatus L26ApiParseMessage(SL26ApiGnssData *dataBufPtr,
		const char *messagePtr, size_t length);

#endif /* __L26_API_H_ */
