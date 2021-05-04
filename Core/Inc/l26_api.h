/**
 * @file l26_api.h
 * @author Adrian Cinal
 * @brief Header file exposing the Quectel L26 API
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
typedef uint8_t TL26SentencesReceived;  /* Typedef for storing flags corresponding to received NMEA sentences */

/**
 * @brief GNSS data structure
 */
typedef struct SL26GnssData {
	float64_t Latitude; /* Latitude in degrees */

	float64_t Longitude; /* Longitude in degrees */

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

	TL26SentencesReceived SentencesReceived; /* Sentences received flags */

} SL26GnssData;

/**
 * @brief GNSS data status typedef
 */
typedef enum EL26DataStatus {
	EL26DataStatus_Ready = 0,
	EL26DataStatus_NotReady,
	EL26DataStatus_Error
} EL26DataStatus;

/*---------------------------------------------- Defines ----------------------------------------------*/

#define L26_NMEA_RMC_RECEIVED    ( (uint8_t) 0x01U )  /* --RMC NMEA sentence received flag */
#define L26_NMEA_VTG_RECEIVED    ( (uint8_t) 0x02U )  /* GPVTG NMEA sentence received flag */
#define L26_NMEA_GGA_RECEIVED    ( (uint8_t) 0x04U )  /* GPGGA NMEA sentence received flag */
#define L26_NMEA_GSA_RECEIVED    ( (uint8_t) 0x08U )  /* --GSA NMEA sentence received flag */
#define L26_NMEA_GLGSV_RECEIVED  ( (uint8_t) 0x10U )  /* GLGSV NMEA sentence received flag */
#define L26_NMEA_GPGSV_RECEIVED  ( (uint8_t) 0x20U )  /* GPGSV NMEA sentence received flag */
#define L26_NMEA_GPGLL_RECEIVED  ( (uint8_t) 0x40U )  /* GPGLL NMEA sentence received flag */

/*---------------------------------------------- Function prototypes ----------------------------------------------*/

/**
 * @brief Add an NMEA checksum to the sentence string
 * @param message Message string
 * @retval None
 */
void L26AddNmeaChecksum(char* message);

/**
 * @brief Try parsing buffered Quectel L26 messages
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param buffer Pointer to the block where the received messages are buffered
 * @param length Length of the buffered data
 * @retval EL26DataStatus Data completeness status
 */
EL26DataStatus L26ParseBufferedMessages(SL26GnssData* data,
	char* buffer, size_t length);

#endif /* __L26_API_H_ */
