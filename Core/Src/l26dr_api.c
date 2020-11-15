/**
 * @file l26dr_api.c
 * @author Adrian Cinal
 * @brief L26-DR API source file
 */

#include "l26dr_api.h"

#include <stdio.h>
#include <string.h>

#define NMEA_PAYLOAD_BEGIN(start)  ( (char*)&( ( (char*)(start) )[7]) )              /* Get the address of the start of the payload */
#define NMEA_PAYLOAD_LENGTH(sentenceLength) ( (size_t)( (sentenceLength) - 11UL ) )  /* Get payload length */
#define NMEA_PARSER_SENTENCE_BUFSIZE   ( (uint32_t) 64UL )                           /* Message parser sentence buffer size */
#define NMEA_PARSER_DATAFIELD_BUFSIZE  ( (uint32_t) 11UL )                           /* Sentence parser data field buffer size */

#define UNUSED(x)  ( (void) x )  /* Tell the compiler that a variable is purposefully ignored, i.e. not used */

/**
 * @brief NMEA parser status typedef
 */
typedef enum EL26DrApiNmeaParserRet {
	EL26DrApiNmeaParserRet_Ok = 0,
	EL26DrApiNmeaParserRet_InvalidFormat,
	EL26DrApiNmeaParserRet_InvalidChecksum,
	EL26DrApiNmeaParserRet_InvalidId,
	EL26DrApiNmeaParserRet_InvalidData,
} EL26DrApiNmeaParserRet;

/**
 * @brief --GSV sentence talker ID typedef
 */
typedef enum EL26DrApiGsvTalkerId {
	EL26DrApiGsvTalkerId_GP = 0,
	EL26DrApiGsvTalkerId_GL
} EL26DrApiGsvTalkerId;

static uint8_t L26DrApiGetNmeaChecksum(const char* message, size_t length);
static EL26DrApiNmeaParserRet L26DrApiParseNmeaSentence(SL26DrApiGnssData *dataBufPtr,
		const char *sentencePtr, size_t length);
static bool L26DrApiIsDataComplete(SL26DrApiGnssData *dataPtr);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseRmcPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseVtgPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGgaPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGsaPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGsvPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length, EL26DrApiGsvTalkerId talkerId);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGllPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);
static EL26DrApiNmeaParserRet L26DrApiNmeaParseTxtPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length);

/**
 * @brief Add an NMEA checksum to the sentence string
 * @param message Message string
 * @retval None
 */
void L26DrApiAddNmeaChecksum(char* message) {

	size_t length = strlen(message);
	/* Calculate the checksum */
	uint8_t checksum = L26DrApiGetNmeaChecksum(message, length);
	/* Print the checksum to the string */
	sprintf(&(message[length - 4UL]), "%02X", checksum);
	/* Restore the <CR> character overwritten by sprintf */
	message[length - 2UL] = '\r';

}

/**
 * @brief Try parsing the Quectel L26-DR message
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] messagePtr Pointer to the message
 * @param[in] length Length of the message
 */
EL26DrApiDataStatus L26DrApiParseMessage(SL26DrApiGnssData *dataBufPtr, const char *messagePtr,
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
				return EL26DrApiDataStatus_Error;

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
				if (EL26DrApiNmeaParserRet_Ok
						!= L26DrApiParseNmeaSentence(dataBufPtr, sentenceBuffer,
								sentenceLength)) {

					/* Reset the counter */
					sentenceLength = 0;
					return EL26DrApiDataStatus_Error;

				}

				/* Reset the counter */
				sentenceLength = 0;

			}

		}

	}

	return (L26DrApiIsDataComplete(dataBufPtr) ?
			EL26DrApiDataStatus_Ready : EL26DrApiDataStatus_Pending);

}

/**
  * @brief Calculate the checksum of an NMEA message
  * @param message Message string
  * @param length Length of the message (without '\0' terminator)
  * @retval uint8_t Calculated checksum
  */
static uint8_t L26DrApiGetNmeaChecksum(const char* message, size_t length) {

	/* Calculate the checksum - note that the checksum is calculated by exclusive OR of all characters between '$' and '*' */
	uint8_t ret = 0;
	for (size_t i = 1; i < (length - 5UL); i += 1UL) {

		ret ^= message[i];

	}

	return ret;

}

/**
 * @brief Parse an NMEA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] sentencePtr Pointer to the NMEA sentence
 * @param[in] length Length of the sentence
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiParseNmeaSentence(SL26DrApiGnssData *dataBufPtr, const char *sentencePtr,
		size_t length) {

	/* Retrieve the message ID */
	char messageId[6];
	strncpy(messageId, sentencePtr + 1U, 5);
	messageId[5] = '\0';

	/* Read the checksum */
	char checksumString[3] = { sentencePtr[length - 4UL], sentencePtr[length - 3UL],
			'\0' };
	uint8_t readChecksum = (uint8_t) strtoul(checksumString, NULL, 16);

	/* Calculate the checksum */
	uint8_t calculatedChecksum = L26DrApiGetNmeaChecksum(sentencePtr, length);

	/* Validate the checksum */
	if (readChecksum != calculatedChecksum) {

		return EL26DrApiNmeaParserRet_InvalidChecksum;

	}

	/* Identify the sentence by the message ID */
	if ( ( 0 == strcmp(messageId, "GPRMC") )
			|| ( 0 == strcmp(messageId, "GNRMC") ) ) {

		/* Parse the payload */
		return L26DrApiNmeaParseRmcPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( 0 == strcmp(messageId, "GPVTG") ) {

		/* Parse the payload */
		return L26DrApiNmeaParseVtgPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( 0 == strcmp(messageId, "GPGGA") ) {

		/* Parse the payload */
		return L26DrApiNmeaParseGgaPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if ( ( 0 == strcmp(messageId, "GPGSA") )
			|| ( 0 == strcmp(messageId, "GNGSA") ) ) {

		/* Parse the payload */
		return L26DrApiNmeaParseGsaPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GLGSV")) {

		/* Parse the payload */
		return L26DrApiNmeaParseGsvPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length), EL26DrApiGsvTalkerId_GL);

	}

	if (0 == strcmp(messageId, "GPGSV")) {

		/* Parse the payload */
		return L26DrApiNmeaParseGsvPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length), EL26DrApiGsvTalkerId_GP);

	}

	if ((0 == strcmp(messageId, "GPGLL"))
			|| (0 == strcmp(messageId, "GNGLL"))) {

		/* Parse the payload */
		return L26DrApiNmeaParseGllPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	if (0 == strcmp(messageId, "GPTXT")) {

		/* Parse the payload */
		return L26DrApiNmeaParseTxtPayload(dataBufPtr, NMEA_PAYLOAD_BEGIN(sentencePtr),
				NMEA_PAYLOAD_LENGTH(length));

	}

	return EL26DrApiNmeaParserRet_InvalidId;

}

/**
 * @brief Test if all NMEA sentences were received
 * @param dataPtr Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
static bool L26DrApiIsDataComplete(SL26DrApiGnssData *dataPtr) {

	/* Test if all flags are set */
	return ((L26DRAPI_NMEA_RMC_RECEIVED | L26DRAPI_NMEA_VTG_RECEIVED | L26DRAPI_NMEA_GGA_RECEIVED
			| L26DRAPI_NMEA_GSA_RECEIVED | L26DRAPI_NMEA_GPGSV_RECEIVED | L26DRAPI_NMEA_GLGSV_RECEIVED
			| L26DRAPI_NMEA_GLL_RECEIVED | L26DRAPI_NMEA_TXT_RECEIVED) == dataPtr->SentencesReceived);

}

/**
 * @brief Parse the payload of an NMEA --RMC sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseRmcPayload(SL26DrApiGnssData *dataBufPtr,
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
				return EL26DrApiNmeaParserRet_InvalidFormat;
			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* UTC Time */

				dataBufPtr->Time = strtod(dataFieldBuffer, NULL);
				break;

			case 1: /* Data Valid */

				if ('A' != dataFieldBuffer[0]) {

					return EL26DrApiNmeaParserRet_InvalidData;

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

					return EL26DrApiNmeaParserRet_InvalidData;

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

					return EL26DrApiNmeaParserRet_InvalidData;

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
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_RMC_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPVTG sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseVtgPayload(SL26DrApiGnssData *dataBufPtr,
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

				return EL26DrApiNmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 0: /* Course over ground in degrees */

				dataBufPtr->Cog = strtof(dataFieldBuffer, NULL);
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
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_VTG_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPGGA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGgaPayload(SL26DrApiGnssData *dataBufPtr,
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

				return EL26DrApiNmeaParserRet_InvalidFormat;

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
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_GGA_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GSA sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGsaPayload(SL26DrApiGnssData *dataBufPtr,
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

				return EL26DrApiNmeaParserRet_InvalidFormat;

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

					dataBufPtr->FixStatus = EFixStatus_2dFix;
					break;

				case '3':

					dataBufPtr->FixStatus = EFixStatus_3dFix;
					break;

				default:

					return EL26DrApiNmeaParserRet_InvalidData;

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
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_GSA_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GSV sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @param[in] talkerId The --GSV sentence talker ID
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGsvPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length, EL26DrApiGsvTalkerId talkerId) {

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

				return EL26DrApiNmeaParserRet_InvalidFormat;

			}

		} else {

			/* On comma, i.e. data field complete - parse the field */
			switch (dataFieldNumber) {

			case 2: /* Total satellites in view */

				switch (talkerId) {

				case EL26DrApiGsvTalkerId_GL:

					dataBufPtr->SatellitesInViewGlonass = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				case EL26DrApiGsvTalkerId_GP:

					dataBufPtr->SatellitesInViewGps = (uint8_t) strtol(
							dataFieldBuffer,
							NULL, 10);
					break;

				default:

					return EL26DrApiNmeaParserRet_InvalidData;

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

	case EL26DrApiGsvTalkerId_GL:

		dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_GLGSV_RECEIVED;
		break;

	case EL26DrApiGsvTalkerId_GP:

		dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_GPGSV_RECEIVED;
		break;

	default:

		return EL26DrApiNmeaParserRet_InvalidData;

	}

	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA --GLL sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseGllPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(dataBufPtr);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_GLL_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}

/**
 * @brief Parse the payload of an NMEA GPTXT sentence
 * @param[out] dataBufPtr Pointer to the GNSS data structure where the parsed data will be stored
 * @param[in] pPayload Pointer to the sentence payload
 * @param[in] length Length of the payload
 * @retval EL26DrApiNmeaParserRet Status
 */
static EL26DrApiNmeaParserRet L26DrApiNmeaParseTxtPayload(SL26DrApiGnssData *dataBufPtr,
		const char *pPayload, size_t length) {

	/* Message not used */
	UNUSED(dataBufPtr);
	UNUSED(pPayload);
	UNUSED(length);

	/* Set the flag */
	dataBufPtr->SentencesReceived |= L26DRAPI_NMEA_TXT_RECEIVED;
	return EL26DrApiNmeaParserRet_Ok;

}
