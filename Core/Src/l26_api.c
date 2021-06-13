/**
 * @file l26_api.c
 * @author Adrian Cinal
 * @brief Source file implementing the Quectel L26 API
 */

#include "l26_api.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

#define NMEA_PAYLOAD_BEGIN(sentence)         ( (char*)&( ( (char*)(sentence) )[7]) )      /* Get the address of the start of the payload */
#define NMEA_PAYLOAD_LENGTH(sentence)        ( (size_t)( ( strlen(sentence) ) - 12UL ) )  /* Get payload length */
#define NMEA_PARSER_MAX_SENTENCE_LENGTH      ( (uint32_t) 128UL )                         /* Message parser sentence buffer size */

 /* Test if the minimum length necessary to validate the end sequence has been reached, and if so validate the end sequence */
#define IS_PARSABLE_NMEA_FORMAT(buf, len)  ( ( (len) >= 5UL ) ? ( ('\r' == (buf)[(len) - 2UL]) && \
                                               ('\n' == (buf)[(len) - 1UL]) && \
											   ('*' == (buf)[(len) - 5UL]) ) : 0 )

/* Discriminate messages based on the message ID */
#define MATCH_MESSAGE_ID(sentence, msgId)    ( &( (sentence)[1] ) == strstr( &( (sentence)[1] ), (msgId) ) )
#define IS_PMTK_SENTENCE(sentence)           ( MATCH_MESSAGE_ID((sentence), "PMTK") )
#define IS_NMEA_RMC_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPRMC") || MATCH_MESSAGE_ID((sentence), "GNRMC") )
#define IS_NMEA_VTG_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPVTG") )
#define IS_NMEA_GGA_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPGGA") )
#define IS_NMEA_GSA_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPGSA") || MATCH_MESSAGE_ID((sentence), "GNGSA") )
#define IS_NMEA_GLGSV_SENTENCE(sentence)     ( MATCH_MESSAGE_ID((sentence), "GLGSV") )
#define IS_NMEA_GPGSV_SENTENCE(sentence)     ( MATCH_MESSAGE_ID((sentence), "GPGSV") )
#define IS_NMEA_GLL_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPGLL") )
#define IS_NMEA_TXT_SENTENCE(sentence)       ( MATCH_MESSAGE_ID((sentence), "GPTXT") )

#define UNUSED(x)  ( (void) x )
#define FOURBITSTOHEXCHAR(x) \
	( (char)( ( ( (x) & (0x0FU) ) < 0x0AU ) ? ( '0' + ( (x) & 0x0FU ) ) : ( 'A' + ( (x) & 0x0FU ) - 0x0AU ) ) )

/**
 * @brief NMEA parser status typedef
 */
typedef enum EL26ParserRet {
	EL26ParserRet_Ok = 0,
	EL26ParserRet_UnsupportedSentence,
	EL26ParserRet_TokenTooLong,
	EL26ParserRet_InvalidChecksum,
	EL26ParserRet_UnexpectedToken,
	EL26ParserRet_UnexpectedValue,
	EL26ParserRet_DataParsableButExplicitlyInvalid
} EL26ParserRet;

typedef EL26ParserRet(*TL26TokenParser)(SL26GnssData*, const char*, uint32_t);

static uint8_t L26CalculateNmeaChecksum(const char* message);
static uint8_t L26ParseNmeaChecksum(const char* message);
static EL26ParserRet L26ParseSentence(SL26GnssData* data,
	char* sentence);
static bool L26IsDataComplete(SL26GnssData* dataPtr);

static EL26ParserRet L26HandlePmtkSentence(const char* message);
static EL26ParserRet L26HandleNmeaTxtSentence(const char* message);

static EL26ParserRet L26ParseNmeaPayload(SL26GnssData* data, char* payload, TL26TokenParser tokenParser, TL26SentencesReceived flagToSetOnSuccess);

static EL26ParserRet L26NmeaParseRmcTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseVtgTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseGgaTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseGsaTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseGlgsvTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseGpgsvTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);
static EL26ParserRet L26NmeaParseGllTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex);


static float64_t dddmm_mmmm_to_degrees(float64_t dddmm_mmmm);

/**
 * @brief Add an NMEA checksum to the sentence string
 * @param message Message string
 * @retval None
 */
void L26AddNmeaChecksum(char* message) {

	size_t length = strlen(message);
	/* Calculate the checksum */
	uint8_t checksum = L26CalculateNmeaChecksum(message);
	/* Print the checksum to the string */
	message[length - 4UL] = FOURBITSTOHEXCHAR((checksum >> 4) & 0x0F);
	message[length - 3UL] = FOURBITSTOHEXCHAR(checksum & 0x0F);
}

/**
 * @brief Try parsing buffered Quectel L26 messages
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param buffer Pointer to the block where the received messages are buffered
 * @param length Length of the buffered data
 * @retval EL26DataStatus Data completeness status
 */
EL26DataStatus L26ParseBufferedMessages(SL26GnssData* data,
	char* buffer, size_t length) {

	static char sentenceBuffer[NMEA_PARSER_MAX_SENTENCE_LENGTH];
	static size_t sentenceLength = 0;

	/* Go through the entire message */
	for (size_t i = 0; i < length; i += 1UL) {

		/* Test if the start character has already been found */
		if (0UL < sentenceLength) {

			/* Assert no buffer overflow */
			if (NMEA_PARSER_MAX_SENTENCE_LENGTH == sentenceLength) {

				/* Reset the counter and raise an error */
				sentenceLength = 0;
				return EL26DataStatus_Error;
			}

			/* Save the next character of the message */
			sentenceBuffer[sentenceLength] = buffer[i];
			/* Increment the length counter */
			sentenceLength += 1UL;

		} else {

			/* If no start character has been found thus far, search for it */
			if ('$' == buffer[i]) {

				/* Save the start character */
				sentenceBuffer[sentenceLength] = buffer[i];
				/* Increment the length counter */
				sentenceLength += 1UL;
			}
		}

		if (IS_PARSABLE_NMEA_FORMAT(sentenceBuffer, sentenceLength)) {

			/* Properly terminate the sentence string */
			sentenceBuffer[sentenceLength] = '\0';
			/* Parse the received sentence */
			if (EL26ParserRet_Ok
				!= L26ParseSentence(data, sentenceBuffer)) {

				/* Reset the counter */
				sentenceLength = 0;
				return EL26DataStatus_Error;
			}

			/* Reset the counter */
			sentenceLength = 0;
		}
	}

	return (L26IsDataComplete(data) ?
		EL26DataStatus_Ready : EL26DataStatus_NotReady);
}

/**
 * @brief Calculate the checksum of an NMEA message
 * @param message Message string
 * @retval uint8_t Calculated checksum
 */
static uint8_t L26CalculateNmeaChecksum(const char* message) {

	size_t length = strlen(message);
	/* Calculate the checksum by exclusive OR of all characters between '$' and '*' */
	uint8_t checksum = 0;
	for (size_t i = 1; i < (length - 5UL); i += 1UL) {

		checksum ^= message[i];
	}

	return checksum;
}

/**
 * @brief Read the checksum from an NMEA message
 * @param message A null-terminated NMEA message
 * @retval uint8_t Parsed checksum
 */
static uint8_t L26ParseNmeaChecksum(const char* message) {

	size_t length = strlen(message);
	/* Create a null-terminated string out of the checksum fields */
	char checksumString[3] = { message[length - 4UL], message[length
			- 3UL], '\0' };
	/* Use stdlib to parse the checksum */
	return (uint8_t)strtoul(checksumString, NULL, 16);
}

/**
 * @brief Parse an NMEA/PMTK sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param sentence A null-terminated NMEA/PMTK sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26ParseSentence(SL26GnssData* data,
	char* sentence) {

	/* Parse the checksum */
	uint8_t receivedChecksum = L26ParseNmeaChecksum(sentence);
	/* Calculate the checksum */
	uint8_t calculatedChecksum = L26CalculateNmeaChecksum(sentence);

	/* Validate the checksum */
	if (receivedChecksum != calculatedChecksum) {

		return EL26ParserRet_InvalidChecksum;
	}



	/* PMTK and NMEA GPTXT sentences are handled separately instead of being parsed for GNSS data */
	if (IS_PMTK_SENTENCE(sentence)) {

		/* Handle a PMTK sentence */
		return L26HandlePmtkSentence(sentence);
	}

	if (IS_NMEA_TXT_SENTENCE(sentence)) {

		/* Handle an NMEA GPTXT sentence */
		return L26HandleNmeaTxtSentence(sentence);
	}



	TL26TokenParser tokenParser = NULL;
	TL26SentencesReceived flagToSetOnParsingSuccess = 0;

	/* Select the correct data fields parser */
	if (IS_NMEA_RMC_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseRmcTokens;
		flagToSetOnParsingSuccess = L26_NMEA_RMC_RECEIVED;
	}

	if (IS_NMEA_VTG_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseVtgTokens;
		flagToSetOnParsingSuccess = L26_NMEA_VTG_RECEIVED;
	}

	if (IS_NMEA_GGA_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseGgaTokens;
		flagToSetOnParsingSuccess = L26_NMEA_GGA_RECEIVED;
	}

	if (IS_NMEA_GSA_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseGsaTokens;
		flagToSetOnParsingSuccess = L26_NMEA_GSA_RECEIVED;
	}

	if (IS_NMEA_GLGSV_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseGlgsvTokens;
		flagToSetOnParsingSuccess = L26_NMEA_GLGSV_RECEIVED;
	}

	if (IS_NMEA_GPGSV_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseGpgsvTokens;
		flagToSetOnParsingSuccess = L26_NMEA_GPGSV_RECEIVED;
	}

	if (IS_NMEA_GLL_SENTENCE(sentence)) {

		tokenParser = L26NmeaParseGllTokens;
		flagToSetOnParsingSuccess = L26_NMEA_GPGLL_RECEIVED;
	}

	/* If relevant data field parser was found (i.e. the sentence was recognized and is supported), parse the payload */
	if (NULL != tokenParser) {

		/* Null-terminate the payload string */
		NMEA_PAYLOAD_BEGIN(sentence)[NMEA_PAYLOAD_LENGTH(sentence)] = '\0';
		return L26ParseNmeaPayload(data, NMEA_PAYLOAD_BEGIN(sentence), tokenParser, flagToSetOnParsingSuccess);
	}

	return EL26ParserRet_UnsupportedSentence;
}

/**
 * @brief Test if all NMEA sentences were received
 * @param dataPtr Pointer to the GNSS data structure
 * @retval bool True if all sentences were received and the data is complete, false otherwise
 */
static bool L26IsDataComplete(SL26GnssData* dataPtr) {

	/* Test if all flags are set */
	return ((L26_NMEA_RMC_RECEIVED | L26_NMEA_VTG_RECEIVED
		| L26_NMEA_GGA_RECEIVED | L26_NMEA_GSA_RECEIVED
		| L26_NMEA_GLGSV_RECEIVED | L26_NMEA_GPGSV_RECEIVED)
		== dataPtr->SentencesReceived);
}

/**
 * @brief Handle a PMTK sentence
 * @param message A null-terminated PMTK sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26HandlePmtkSentence(const char* message) {

	/* Ignore the message */
	UNUSED(message);

	return EL26ParserRet_Ok;
}

/**
 * @brief Handle an NMEA GPTXT sentence
 * @param message A null-terminated NMEA GPTXT sentence
 * @retval EL26ParserRet Status
  */
static EL26ParserRet L26HandleNmeaTxtSentence(const char* message) {

	/* Ignore the message */
	UNUSED(message);

	return EL26ParserRet_Ok;
}

/**
  * @brief Parse an NMEA sentence payload
  * @param data Pointer to the GNSS data structure where the parsed data will be stored
  * @param payload A null-terminated NMEA payload
  * @param tokenParser Function used to parse the data fields/tokens
  * @param flagToSetOnSuccess Flag indicating that a sentence has been received that is to be set in the data structure on success
  */
static EL26ParserRet L26ParseNmeaPayload(SL26GnssData* data, char* payload, TL26TokenParser tokenParser, TL26SentencesReceived flagToSetOnSuccess) {

	EL26ParserRet status = EL26ParserRet_Ok;
	uint32_t i;
	char* in;
	char* tok;
	char *ctx;

	/* Tokenize the sentence and parse each token */
	for (i = 0, in = payload; NULL != (tok = strtok_r(in, ",", &ctx)); i += 1UL) {

		/* Parse the data field */
		status = tokenParser(data, tok, i);
		if (EL26ParserRet_Ok != status) {

			break;
		}

		in = NULL;
	}

	if (EL26ParserRet_Ok == status) {

		/* On success, set the appropriate flag */
		data->SentencesReceived |= flagToSetOnSuccess;
	}

	if (EL26ParserRet_DataParsableButExplicitlyInvalid == status) {

		/* If the 'V' field (data invalid) was set in the RMC payload, report parser success without setting the sentence received flag */
		status = EL26ParserRet_Ok;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA --RMC sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseRmcTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 0: /* UTC Time */

		data->Time = strtod(token, NULL);
		break;

	case 1: /* Data Valid */

		switch (token[0]) {

		case 'A':

			break;

		case 'V':

			status = EL26ParserRet_DataParsableButExplicitlyInvalid;
			break;

		default:

			status = EL26ParserRet_UnexpectedValue;
			break;
		}
		break;

	case 2: /* Latitude */

		data->Latitude = dddmm_mmmm_to_degrees(strtod(token, NULL));
		break;

	case 3: /* N/S */

		switch (token[0]) {

		case 'N':

			break;

		case 'S':

			data->Latitude *= -1.0;
			break;

		default:

			status = EL26ParserRet_UnexpectedValue;
			break;
		}

	case 4: /* Longitude */

		data->Longitude = dddmm_mmmm_to_degrees(strtod(token, NULL));
		break;

	case 5: /* E/W */

		switch (token[0]) {

		case 'E':

			break;

		case 'W':

			data->Longitude *= -1.0;
			break;

		default:

			status = EL26ParserRet_UnexpectedValue;
			break;
		}

	case 8: /* Date in format 'ddmmyy' */

		data->Date = strtol(token, NULL, 10);
		break;;

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA --VTG sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseVtgTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 0: /* Course over ground in degrees */

		data->Cog = strtof(token, NULL);
		break;

	case 5: /* Speed over ground in km/h */

		/* Note that speed is actually the sixth (zero-based) token in the VTG payload,
		 * but the Quectel L26 device does not output course over ground (magnetic), instead
		 * outputting two commas (token delimiters) next to each other. strtok_r ignores an empty
		 * token thus making speed over groudn in km/h the fifth token. */
		data->Speed = strtof(token, NULL);
		break;

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA --GGA sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseGgaTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 6: /* Number of satellites being used (0-12) */

		data->SatellitesInUse = (uint8_t)strtol(token,
			NULL, 10);
		break;

	case 8: /* Altitude in meters according to WGS84 ellipsoid */

		data->Altitude = strtof(token, NULL);
		break;

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA --GSA sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseGsaTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 1: /* Fix Status */

		switch (token[0]) {

		case '1':

			data->FixStatus = EFixStatus_NoFix;
			break;

		case '2':

			data->FixStatus = EFixStatus_2dFix;
			break;

		case '3':

			data->FixStatus = EFixStatus_3dFix;
			break;

		default:

			status = EL26ParserRet_UnexpectedValue;
		}
		break;

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA GLGSV sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseGlgsvTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 2: /* Total satellites in view */

		data->SatellitesInViewGlonass = (uint8_t)strtol(token, NULL, 10);

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA GPGSV sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseGpgsvTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	EL26ParserRet status = EL26ParserRet_Ok;

	switch (tokenIndex) {

	case 2: /* Total satellites in view */

		data->SatellitesInViewGps = (uint8_t)strtol(token, NULL, 10);

	default: /* Ignore all other fields */

		break;
	}

	return status;
}

/**
 * @brief Parse a single token of an NMEA GPGLL sentence
 * @param data Pointer to the GNSS data structure where the parsed data will be stored
 * @param token A null-terminated token string
 * @param tokenIndex Index of the token in the sentence
 * @retval EL26ParserRet Status
 */
static EL26ParserRet L26NmeaParseGllTokens(SL26GnssData* data,
	const char* token, uint32_t tokenIndex) {

	/* Ignore all fields */
	return EL26ParserRet_Ok;
}

/**
  * @brief Convert geographic coordinates from the 'dddmm.mmmm' format to floating-point degrees value
  * @param dddmm_mmmm Coordinate in the 'dddmm.mmmm' format
  * @retval float64_t Coordinate in degrees
  */
static float64_t dddmm_mmmm_to_degrees(float64_t dddmm_mmmm) {

	/* Separate the degrees from the minutes */
	float64_t degrees = floor(dddmm_mmmm / 100.0);
	/* Get the minutes */
	float64_t minutes = dddmm_mmmm - (degrees * 100.0);
	/* Add minutes as decimal fraction to the degrees */
	float64_t ret = degrees + (minutes / 60.0);

	return ret;
}
