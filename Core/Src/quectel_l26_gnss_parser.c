/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.c
 * @brief Source file defining functions for parsing a Quectel L26 NMEA message
 */

#include "quectel_l26_gnss_parser.h"
#include <string.h>
#include <math.h>

/**
 * @brief Tell the compiler that a variable is purposefully ignored, i.e. not used
 */
#define UNUSED(x)  ( (void) x )

#define NMEA_PARSER_SENTENCE_BUFSIZE   ((uint32_t) 64UL)  /* Message parser sentence buffer size */
#define NMEA_PARSER_DATAFIELD_BUFSIZE  ((uint32_t) 11UL)  /* Sentence parser data field buffer size */

static ENmeaParserRet NmeaParseRmcPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static ENmeaParserRet NmeaParseVtgPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static ENmeaParserRet NmeaParseGgaPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static ENmeaParserRet NmeaParseGsaPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static ENmeaParserRet NmeaParseGsvPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length, EGsvTalkerId talkerId);
static ENmeaParserRet NmeaParseGllPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static ENmeaParserRet NmeaParseTxtPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length);

/**
 * @brief Test if all NMEA sentences were received
 * @param dataPtr Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool IsDataComplete(SGnssData *dataPtr) {

	/* Test if all flags are set */
	return ((NMEA_RMC_RECEIVED | NMEA_VTG_RECEIVED | NMEA_GGA_RECEIVED
			| NMEA_GSA_RECEIVED | NMEA_GPGSV_RECEIVED | NMEA_GLGSV_RECEIVED
			| NMEA_GLL_RECEIVED | NMEA_TXT_RECEIVED) == dataPtr->SentencesReceived);

}

/**
 * @brief Try parsing the Quectel L26 message
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] messagePtr Pointer to the message
 * @param[in] length Length of the message
 */
EGnssDataStatus ParseMessage(SGnssData *dataBufPtr, const char *messagePtr,
		size_t length) {

	static char sentenceBuffer[NMEA_PARSER_SENTENCE_BUFSIZE];
	static size_t sentenceLength = 0;

	/* Go through the entire message */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if the start character has been found */
		if (0UL < sentenceLength) {

			/* Increment the length counter */
			sentenceLength += 1UL;
			/* Save the next character of the message */
			sentenceBuffer[sentenceLength - 1UL] = messagePtr[i];
			/* Test for buffer overflow */
			if (NMEA_PARSER_SENTENCE_BUFSIZE <= sentenceLength) {

				/* Reset the counter */
				sentenceLength = 0;
				return EGnssDataStatus_Error;

			}

		} else {

			/* Search for the start character */
			if ('$' == messagePtr[i]) {

				/* Increment the length counter */
				sentenceLength += 1UL;
				/* Save the start character */
				sentenceBuffer[sentenceLength - 1UL] = messagePtr[i];

			}

		}

		/* If the length exceeds the minimum sentence length necessary to validate the format */
		if (5U <= sentenceLength) {

			/* Test if end sequence has been found */
			if ( ('\r' == sentenceBuffer[sentenceLength - 2UL])
					&& ('\n' == sentenceBuffer[sentenceLength - 1UL])
					&& ('*' == sentenceBuffer[sentenceLength - 5UL]) ) {

				/* Parse the received sentence */
				if (ENmeaParserRet_Ok
						!= ParseNmeaSentence(dataBufPtr, sentenceBuffer,
								sentenceLength)) {

					/* Reset the counter */
					sentenceLength = 0;
					return EGnssDataStatus_Error;

				}

				/* Reset the counter */
				sentenceLength = 0;

			}

		}

	}

	return (IsDataComplete(dataBufPtr) ?
			EGnssDataStatus_Ready : EGnssDataStatus_Pending);

}

/**
 * @brief Parse an NMEA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] sentencePtr Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval ENmeaParserRet Status
 */
ENmeaParserRet ParseNmeaSentence(SGnssData *dataBufPtr, const char *sentencePtr,
		size_t length) {

	/* Retrieve the message ID */
	char messageId[6];
	strncpy(messageId, sentencePtr + 1U, 5);
	messageId[5] = '\0';

	/* Read the checksum */
	char checksumString[3] = { sentencePtr[length - 4UL], sentencePtr[length - 3UL],
			'\0' };
	uint8_t readChecksum = (uint8_t) strtoul(checksumString, NULL, 16);

	/* Calculate the checksum - note that the checksum is calculated by exclusive OR of all characters between '$' and '*' */
	uint8_t calculatedChecksum = 0;
	for (size_t i = 1; i < (length - 5UL); i += 1UL) {

		calculatedChecksum ^= sentencePtr[i];

	}

	/* Validate the checksum */
	if (readChecksum != calculatedChecksum) {

		return ENmeaParserRet_InvalidChecksum;

	}

	/* Identify the sentence by the message ID */
	if ( ( 0 == strcmp(messageId, "GPRMC") )
			|| ( 0 == strcmp(messageId, "GNRMC") ) ) {

		/* Parse the payload */
		return NmeaParseRmcPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( 0 == strcmp(messageId, "GPVTG") ) {

		/* Parse the payload */
		return NmeaParseVtgPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( 0 == strcmp(messageId, "GPGGA") ) {

		/* Parse the payload */
		return NmeaParseGgaPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( ( 0 == strcmp(messageId, "GPGSA") )
			|| ( 0 == strcmp(messageId, "GNGSA") ) ) {

		/* Parse the payload */
		return NmeaParseGsaPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GLGSV")) {

		/* Parse the payload */
		return NmeaParseGsvPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length), EGsvTalkerId_GL);

	}

	if (0 == strcmp(messageId, "GPGSV")) {

		/* Parse the payload */
		return NmeaParseGsvPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length), EGsvTalkerId_GP);

	}

	if ((0 == strcmp(messageId, "GPGLL"))
			|| (0 == strcmp(messageId, "GNGLL"))) {

		/* Parse the payload */
		return NmeaParseGllPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GPTXT")) {

		/* Parse the payload */
		return NmeaParseTxtPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	return ENmeaParserRet_InvalidId;
}

/**
 * @brief Parse the payload of an NMEA --RMC sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseRmcPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFSIZE];
	size_t bufferIndex = 0;
	uint8_t dataFieldNumber = 0;

	/* Go through the entire sentence */
	for (size_t i = 0; i < length; i += 1UL) {
		/* Test if not a comma, i.e. data fields separator */
		if (',' != pPayload[i]) {

			/* Save the data field character */
			dataFieldBuffer[bufferIndex] = pPayload[i];
			/* Increment the buffer index */
			bufferIndex += 1UL;
			/* Test for buffer overflow */
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFSIZE) {
				return ENmeaParserRet_InvalidFormat;
			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* UTC Time */

				dataBufPtr->Time = strtod(dataFieldBuffer, NULL);
				break;

			case 1: /* Data Valid */

				if ('A' != dataFieldBuffer[0]) {

					return ENmeaParserRet_InvalidData;

				}
				break;

			case 2: /* Latitude */

				dataBufPtr->Latitude = strtod(dataFieldBuffer, NULL);
				break;

			case 3: /* N/S */

				switch (dataFieldBuffer[0]) {

				case 'N':

					dataBufPtr->LatDir = ELatDir_LatitudeNorth;
					break;

				case 'S':

					dataBufPtr->LatDir = ELatDir_LatitudeSouth;
					break;

				default:

					return ENmeaParserRet_InvalidData;

				}
				break;

			case 4: /* Longitude */

				dataBufPtr->Longitude = strtod(dataFieldBuffer, NULL);
				break;

			case 5: /* E/W */

				switch (dataFieldBuffer[0]) {

				case 'E':

					dataBufPtr->LonDir = ELonDir_LongitudeEast;
					break;

				case 'W':

					dataBufPtr->LonDir = ELonDir_LongitudeWest;
					break;

				default:

					return ENmeaParserRet_InvalidData;

				}
				break;

			case 8: /* Date */

				dataBufPtr->Date = strtol(dataFieldBuffer, NULL, 10);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFSIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}
	}

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_RMC_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPVTG sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseVtgPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFSIZE]; /* Buffer for the data field */
	size_t bufferIndex = 0; /* Buffer index */
	uint8_t dataFieldNumber = 0; /* Number of data field currently being parsed */

	/* Go through the entire sentence */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if not a comma, i.e. data fields separator */
		if (',' != pPayload[i]) {

			/* Save the data field character */
			dataFieldBuffer[bufferIndex] = pPayload[i];
			/* Increment the buffer index */
			bufferIndex += 1UL;
			/* Test for buffer overflow */
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFSIZE) {

				return ENmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* Course over ground in degrees */

				dataBufPtr->COG = strtof(dataFieldBuffer, NULL);
				break;

			case 6: /* Speed over ground in km/h */

				dataBufPtr->Speed = strtof(dataFieldBuffer, NULL);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFSIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_VTG_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPGGA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseGgaPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFSIZE]; /* Buffer for the data field */
	size_t bufferIndex = 0; /* Buffer index */
	uint8_t dataFieldNumber = 0; /* Number of data field currently being parsed */

	/* Go through the entire sentence */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if not a comma, i.e. data fields separator */
		if (',' != pPayload[i]) {

			/* Save the data field character */
			dataFieldBuffer[bufferIndex] = pPayload[i];
			/* Increment the buffer index */
			bufferIndex += 1UL;
			/* Test for buffer overflow */
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFSIZE) {

				return ENmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 6: /* Number of satellites being used (0-12) */

				dataBufPtr->SatellitesInUse = (uint8_t) strtol(dataFieldBuffer,
				NULL, 10);
				break;

			case 8: /* Altitude in meters according to WGS84 ellipsoid */

				dataBufPtr->Altitude = strtof(dataFieldBuffer, NULL);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFSIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_GGA_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GSA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseGsaPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFSIZE]; /* Buffer for the data field */
	size_t bufferIndex = 0; /* Buffer index */
	uint8_t dataFieldNumber = 0; /* Number of data field currently being parsed */

	/* Go through the entire sentence */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if not a comma, i.e. data fields separator */
		if (',' != pPayload[i]) {

			/* Save the data field character */
			dataFieldBuffer[bufferIndex] = pPayload[i];
			/* Increment the buffer index */
			bufferIndex += 1UL;
			/* Test for buffer overflow */
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFSIZE) {

				return ENmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 1: /* Fix Status */

				switch (dataFieldBuffer[0]) {

				case '1':

					dataBufPtr->FixStatus = EFixStatus_NoFix;
					break;

				case '2':

					dataBufPtr->FixStatus = EFixStatus_2DFix;
					break;

				case '3':

					dataBufPtr->FixStatus = EFixStatus_3DFix;
					break;

				default:

					return ENmeaParserRet_InvalidData;

				}
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFSIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_GSA_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GSV sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @param[in] talkerId The --GSV sentence talker ID
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseGsvPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length, EGsvTalkerId talkerId) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFSIZE]; /* Buffer for the data field */
	size_t bufferIndex = 0; /* Buffer index */
	uint8_t dataFieldNumber = 0; /* Number of data field currently being parsed */

	/* Go through the entire sentence */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if not a comma, i.e. data fields separator */
		if (',' != pPayload[i]) {

			/* Save the data field character */
			dataFieldBuffer[bufferIndex] = pPayload[i];
			/* Increment the buffer index */
			bufferIndex += 1UL;
			/* Test for buffer overflow */
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFSIZE) {

				return ENmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 2: /* Total satellites in view */

				switch (talkerId) {

				case EGsvTalkerId_GL:

					dataBufPtr->SatellitesInViewGLONASS = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				case EGsvTalkerId_GP:

					dataBufPtr->SatellitesInViewGPS = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				default:

					return ENmeaParserRet_InvalidData;

				}
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFSIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	switch (talkerId) {

	case EGsvTalkerId_GL:

		dataBufPtr->SentencesReceived |= NMEA_GLGSV_RECEIVED;
		break;

	case EGsvTalkerId_GP:

		dataBufPtr->SentencesReceived |= NMEA_GPGSV_RECEIVED;
		break;

	default:

		return ENmeaParserRet_InvalidData;

	}

	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GLL sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseGllPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(dataBufPtr);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_GLL_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPTXT sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval ENmeaParserRet Status
 */
static ENmeaParserRet NmeaParseTxtPayload(SGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(dataBufPtr);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	dataBufPtr->SentencesReceived |= NMEA_TXT_RECEIVED;
	return ENmeaParserRet_Ok;

}

/**
 * @brief Normalize the coordinate (longitude/latitude) as degrees multiplied by 1,000,000
 * @param coordinate Coordinate in format 'dddmm.mmmm' (degree and minutes)
 * @param direction Direction flag (GNSS_LATITUDE_NORTH, GNSS_LATITUDE_SOUTH, GNSS_LATITUDE_EAST, GNSS_LATITUDE_WEST)
 * @retval int32_t Coordinate normalized as degrees multiplied by 1,000,000
 */
int32_t NormalizeCoordinate(float64_t coordinate, uint32_t direction) {

	/* Separate degrees from the minutes */
	float64_t degrees = floor(coordinate / 100.0);
	/* Get minutes */
	float64_t minutes = coordinate - (degrees * 100.0);
	/* Add minutes as decimal fraction to the degrees */
	float64_t floatResult = degrees + (minutes / 60.0);

	/* Make the result negative if the direction is SOUTH or WEST */
	if ( (ELatDir_LatitudeSouth == direction)
			|| (ELonDir_LongitudeWest == direction) ) {

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
uint16_t NormalizeSpeed(float32_t speed) {

	return lround(speed * 10.0);

}

/**
 * @brief Normalize the direction as degrees multiplied by 10
 * @param direction Direction in degrees
 * @retval uint16_t Direction normalized as degrees multiplied by 10
 */
uint16_t NormalizeDirection(float32_t direction) {

	return lround(direction * 10.0);

}

/**
 * @brief Normalize the altitude as meters multiplied by 10
 * @param altitude Altitude in meters
 * @retval uint16_t Altitude normalized as meters multiplied by 10
 */
uint16_t NormalizeAltitude(float32_t altitude) {

	return lround(altitude * 10.0);

}

/**
 * @brief Normalize the time to the format hhmmsssss
 * @param time Time in format hhmmss.sss
 * @retval uint32_t Time normalized to the format hhmmsssss
 */
uint32_t NormalizeTime(float64_t time) {

	return llround(time * 1000.0);

}
