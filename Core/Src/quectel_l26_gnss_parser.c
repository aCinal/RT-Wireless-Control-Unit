/**
 * @author Adrian Cinal
 * @file quectel_l26_gnss_parser.c
 * @brief Source file defining functions for parsing a Quectel L26 NMEA message
 */

#include <quectel_l26_gnss_parser.h>
#include <string.h>
#include <math.h>

/**
 * @brief Tell the compiler that a variable is purposefully ignored, i.e. not used
 */
#define UNUSED(x) ((void)x)

/**
 * @brief Tests if all NMEA sentences were received
 * @param pData Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
bool isDataComplete(GnssDataTypedef *pData) {

	/* Test if all flags are set */
	return ((NMEA_RMC_RECEIVED | NMEA_VTG_RECEIVED | NMEA_GGA_RECEIVED
			| NMEA_GSA_RECEIVED | NMEA_GPGSV_RECEIVED | NMEA_GLGSV_RECEIVED | NMEA_GLL_RECEIVED | NMEA_TXT_RECEIVED)
			== pData->SentencesReceived);

}

/**
 * @brief Tries parsing the Quectel L26 message
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pMessage Pointer to the message
 * @param[in] length Length of the message
 */
GnssDataStatusTypedef parseMessage(GnssDataTypedef *pDataBuff,
		const char *pMessage, size_t length) {

	static char SentenceBuffer[NMEA_PARSER_SENTENCE_BUFFER_SIZE]; /* Sentence buffer */
	static size_t SentenceLength = 0; /* Sentence length */

	/* Go through the entire message */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if the start character has been found */
		if (0UL < SentenceLength) {

			/* Increment the length counter */
			SentenceLength += 1UL;
			/* Save the next character of the message */
			SentenceBuffer[SentenceLength - 1UL] = pMessage[i];
			/* Test for buffer overflow */
			if (NMEA_PARSER_SENTENCE_BUFFER_SIZE <= SentenceLength) {
				/* Reset the counter */
				SentenceLength = 0;
				return GNSS_DATA_ERROR;
			}

		} else {

			/* Search for the start character */
			if ('$' == pMessage[i]) {
				/* Increment the length counter */
				SentenceLength += 1UL;
				/* Save the start character */
				SentenceBuffer[SentenceLength - 1UL] = pMessage[i];
			}

		}

		/* If the length exceeds the minimum sentence length necessary to validate the format */
		if (5U <= SentenceLength) {

			/* Test if end sequence has been found */
			if (('\r' == SentenceBuffer[SentenceLength - 2UL])
					&& ('\n' == SentenceBuffer[SentenceLength - 1UL])
					&& ('*' == SentenceBuffer[SentenceLength - 5UL])) {

				/* Parse the received sentence */
				if (NMEA_ERROR_NONE
						!= parseNmeaSentence(pDataBuff, SentenceBuffer,
								SentenceLength)) {

					/* Reset the counter */
					SentenceLength = 0;
					return GNSS_DATA_ERROR;

				}
				/* Reset the counter */
				SentenceLength = 0;

			}

		}

	}

	return (isDataComplete(pDataBuff) ? GNSS_DATA_READY : GNSS_DATA_PENDING);

}

/**
 * @brief Parses an NMEA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pSentence Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef parseNmeaSentence(GnssDataTypedef *pDataBuff,
		const char *pSentence, size_t length) {

	/* Retrieve the message ID */
	char messageId[6];
	strncpy(messageId, pSentence + 1U, 5);
	messageId[5] = '\0';

	/* Read the checksum */
	char checksumString[3] = { pSentence[length - 4UL], pSentence[length - 3UL],
			'\0' };
	uint8_t readChecksum = (uint8_t) strtoul(checksumString, NULL, 16);

	/* Calculate the checksum - note that the checksum is calculated by exclusive OR of all characters between '$' and '*' */
	uint8_t calculatedChecksum = 0;
	for (size_t i = 1; i < (length - 5UL); i += 1UL) {

		calculatedChecksum ^= pSentence[i];

	}

	/* Validate the checksum */
	if (readChecksum != calculatedChecksum) {

		return NMEA_ERROR_INVALID_CHECKSUM;

	}

	/* Identify the sentence by the message ID */
	if ((0 == strcmp(messageId, "GPRMC"))
			|| (0 == strcmp(messageId, "GNRMC"))) {

		/* Parse the payload */
		return _NmeaParseRmcPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GPVTG")) {

		/* Parse the payload */
		return _NmeaParseVtgPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GPGGA")) {

		/* Parse the payload */
		return _NmeaParseGgaPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ((0 == strcmp(messageId, "GPGSA"))
			|| (0 == strcmp(messageId, "GNGSA"))) {

		/* Parse the payload */
		return _NmeaParseGsaPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GLGSV")) {

		/* Parse the payload */
		return _NmeaParseGsvPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length), GSV_TALKER_ID_GL);

	}

	if (0 == strcmp(messageId, "GPGSV")) {

		/* Parse the payload */
		return _NmeaParseGsvPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length), GSV_TALKER_ID_GP);

	}

	if ((0 == strcmp(messageId, "GPGLL"))
			|| (0 == strcmp(messageId, "GNGLL"))) {

		/* Parse the payload */
		return _NmeaParseGllPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GPTXT")) {

		/* Parse the payload */
		return _NmeaParseTxtPayload(pDataBuff, NMEA_PAYLOAD_BEGIN(pSentence),
				NMEA_PAYLOAD_LENGTH(length));

	}

	return NMEA_ERROR_INVALID_ID;
}

/**
 * @brief Parses the payload of an NMEA --RMC sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseRmcPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {
	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFFER_SIZE]; /* Buffer for the data field */
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
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFFER_SIZE) {
				return NMEA_ERROR_INVALID_FORMAT;
			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* UTC Time */

				pDataBuff->Time = strtod(dataFieldBuffer, NULL);
				break;

			case 1: /* Data Valid */

				if ('A' != dataFieldBuffer[0]) {

					return NMEA_ERROR_INVALID_DATA;

				}
				break;

			case 2: /* Latitude */

				pDataBuff->Latitude = strtod(dataFieldBuffer, NULL);
				break;

			case 3: /* N/S */

				switch (dataFieldBuffer[0]) {

				case 'N':

					pDataBuff->LatDir = GNSS_LATITUDE_NORTH;
					break;

				case 'S':

					pDataBuff->LatDir = GNSS_LATITUDE_SOUTH;
					break;

				default:

					return NMEA_ERROR_INVALID_DATA;

				}
				break;

			case 4: /* Longitude */

				pDataBuff->Longitude = strtod(dataFieldBuffer, NULL);
				break;

			case 5: /* E/W */

				switch (dataFieldBuffer[0]) {

				case 'E':

					pDataBuff->LonDir = GNSS_LONGITUDE_EAST;
					break;

				case 'W':

					pDataBuff->LonDir = GNSS_LONGITUDE_WEST;
					break;

				default:

					return NMEA_ERROR_INVALID_DATA;

				}
				break;

			case 8: /* Date */

				pDataBuff->Date = strtol(dataFieldBuffer, NULL, 10);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
					NMEA_PARSER_DATAFIELD_BUFFER_SIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}
	}

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_RMC_RECEIVED;
	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA GPVTG sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseVtgPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {
	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFFER_SIZE]; /* Buffer for the data field */
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
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFFER_SIZE) {

				return NMEA_ERROR_INVALID_FORMAT;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* Course over ground in degrees */

				pDataBuff->COG = strtof(dataFieldBuffer, NULL);
				break;

			case 6: /* Speed over ground in km/h */

				pDataBuff->Speed = strtof(dataFieldBuffer, NULL);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFFER_SIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_VTG_RECEIVED;
	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA GPGGA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGgaPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFFER_SIZE]; /* Buffer for the data field */
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
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFFER_SIZE) {

				return NMEA_ERROR_INVALID_FORMAT;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 6: /* Number of satellites being used (0-12) */

				pDataBuff->SatellitesInUse = (uint8_t) strtol(dataFieldBuffer,
						NULL, 10);
				break;

			case 8: /* Altitude in meters according to WGS84 ellipsoid */

				pDataBuff->Altitude = strtof(dataFieldBuffer, NULL);
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFFER_SIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_GGA_RECEIVED;
	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA --GSA sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGsaPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFFER_SIZE]; /* Buffer for the data field */
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
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFFER_SIZE) {

				return NMEA_ERROR_INVALID_FORMAT;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 1: /* Fix Status */

				switch (dataFieldBuffer[0]) {

				case '1':

					pDataBuff->FixStatus = GNSS_FixStatus_NoFix;
					break;

				case '2':

					pDataBuff->FixStatus = GNSS_FixStatus_2DFix;
					break;

				case '3':

					pDataBuff->FixStatus = GNSS_FixStatus_3DFix;
					break;

				default:

					return NMEA_ERROR_INVALID_DATA;

				}
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFFER_SIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_GSA_RECEIVED;
	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA --GSV sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @param[in] talkerId The --GSV sentence talker ID
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGsvPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length, GsvTalkerIdTypedef talkerId) {

	char dataFieldBuffer[NMEA_PARSER_DATAFIELD_BUFFER_SIZE]; /* Buffer for the data field */
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
			if (bufferIndex >= NMEA_PARSER_DATAFIELD_BUFFER_SIZE) {

				return NMEA_ERROR_INVALID_FORMAT;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 2: /* Total satellites in view */

				switch (talkerId) {

				case GSV_TALKER_ID_GL:

					pDataBuff->SatellitesInViewGLONASS = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				case GSV_TALKER_ID_GP:

					pDataBuff->SatellitesInViewGPS = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				default:

					return NMEA_ERROR_INVALID_DATA;

				}
				break;

			default: /* Ignore all other fields */

				break;

			}

			/* Clear the buffer */
			(void) memset(dataFieldBuffer, 0x00,
			NMEA_PARSER_DATAFIELD_BUFFER_SIZE);
			/* Reset the index counter */
			bufferIndex = 0;
			/* Increment the number of the data field */
			dataFieldNumber += 1U;

		}

	}

	/* Set the flag */
	switch (talkerId) {

	case GSV_TALKER_ID_GL:

		pDataBuff->SentencesReceived |= NMEA_GLGSV_RECEIVED;
		break;

	case GSV_TALKER_ID_GP:

		pDataBuff->SentencesReceived |= NMEA_GPGSV_RECEIVED;
		break;

	default:

		return NMEA_ERROR_INVALID_DATA;

	}

	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA --GLL sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseGllPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(pDataBuff);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_GLL_RECEIVED;
	return NMEA_ERROR_NONE;

}

/**
 * @brief Parses the payload of an NMEA GPTXT sentence
 * @param[out] pDataBuff Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval NmeaParserStatusTypedef Error code
 */
NmeaParserStatusTypedef _NmeaParseTxtPayload(GnssDataTypedef *pDataBuff,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(pDataBuff);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	pDataBuff->SentencesReceived |= NMEA_TXT_RECEIVED;
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
