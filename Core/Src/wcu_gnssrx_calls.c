/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.c
 * @brief Source file defining functions called by the gnssRx task
 */

#include "wcu_gnssrx_calls.h"
#include "wcu_basic.h"

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void) {

	/* Wait for the device to turn on and set up */
	vTaskDelay(WCU_GNSSRX_DEVICECONFIG_SETUP_DELAY);

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	const char PMTK_SET_POS_FIX[] = "$PMTK220,100*1F\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) PMTK_SET_POS_FIX,
					sizeof(PMTK_SET_POS_FIX),
					WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT)) {

		Error_Handler();

	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*2B\r";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE,
					(uint8_t*) PMTK_API_SET_GNSS_SEARCH_MODE,
					sizeof(PMTK_API_SET_GNSS_SEARCH_MODE),
					WCU_GNSSRX_DEVICECONFIG_UART_TIMEOUT)) {

		Error_Handler();

	}

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CAN_ID_GPS_POS;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	/* Write the longitude to the frame payload */
	int32_t longitude = normalizeCoordinate(pData->Longitude, pData->LonDir);
	canFrame.Payload[0] = _bits24_31(longitude);
	canFrame.Payload[1] = _bits16_23(longitude);
	canFrame.Payload[2] = _bits8_15(longitude);
	canFrame.Payload[3] = _bits0_7(longitude);

	/* Write the latitude to the frame payload */
	int32_t latitude = normalizeCoordinate(pData->Latitude, pData->LatDir);
	canFrame.Payload[4] = _bits24_31(latitude);
	canFrame.Payload[5] = _bits16_23(longitude);
	canFrame.Payload[6] = _bits8_15(longitude);
	canFrame.Payload[7] = _bits0_7(longitude);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS2(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 6;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CAN_ID_GPS_POS2;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	/* Write the speed to the frame payload */
	uint16_t speed = normalizeSpeed(pData->Speed);
	canFrame.Payload[0] = _bits8_15(speed);
	canFrame.Payload[1] = _bits0_7(speed);

	/* Write the direction to the frame payload */
	uint16_t direction = normalizeDirection(pData->COG);
	canFrame.Payload[2] = _bits8_15(direction);
	canFrame.Payload[3] = _bits0_7(direction);

	/* Write the altitude to the frame payload */
	uint16_t altitude = normalizeAltitude(pData->Altitude);
	canFrame.Payload[4] = _bits8_15(altitude);
	canFrame.Payload[5] = _bits0_7(altitude);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS2 failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_STATUS(GnssDataTypedef *pData) {

	CanFrameTypedef canFrame = { .DataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.Header.Tx.DLC = 8;
	canFrame.Header.Tx.IDE = CAN_ID_STD;
	canFrame.Header.Tx.RTR = CAN_RTR_DATA;
	canFrame.Header.Tx.StdId = WCU_CAN_ID_GPS_STATUS;
	canFrame.Header.Tx.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canFrame.Payload[0] = pData->SatellitesInViewGLONASS
			+ pData->SatellitesInViewGPS;
	/* Clear two most significant bits */
	canFrame.Payload[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canFrame.Payload[0] |= (uint8_t) pData->FixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canFrame.Payload[1] = pData->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = normalizeTime(pData->Time);
	canFrame.Payload[2] = _bits24_31(time);
	canFrame.Payload[3] = _bits16_23(time);
	canFrame.Payload[4] = _bits8_15(time);
	canFrame.Payload[5] = _bits0_7(time);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canFrame.Payload[5] |= 0xF & _bits24_31(pData->Date);
	canFrame.Payload[6] = 0xFF & (pData->Date >> 20U);
	canFrame.Payload[7] = 0xFF & (pData->Date >> 12U);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_STATUS failed to send to canTxQueue\r\n");

}
