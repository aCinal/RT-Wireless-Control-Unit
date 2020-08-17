/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.c
 * @brief Source file defining functions called by the gnssRx task
 */

#include "wcu_gnssrx_calls.h"
#include "wcu_base.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"

/**
 * @brief Configures the Quectel L26 device
 * @retval None
 */
void gnssRx_DeviceConfig(void) {

	/* Wait for the device to turn on and set up */
	vTaskDelay(pdMS_TO_TICKS(1000));

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	const char PMTK_SET_POS_FIX[] = "$PMTK220,100*1F\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) PMTK_SET_POS_FIX,
					sizeof(PMTK_SET_POS_FIX), 1000)) {

		Error_Handler();

	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*2B\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE,
					(uint8_t*) PMTK_API_SET_GNSS_SEARCH_MODE,
					sizeof(PMTK_API_SET_GNSS_SEARCH_MODE), 1000)) {

		Error_Handler();

	}

}

/**
 * @brief Listens for and handles the GNSS message
 * @retval None
 */
void gnssRx_HandleMessage(void) {

	static uint8_t rxBuffTable[WCU_GNSSRX_UART_RX_BUFF_SIZE]; /* UART Rx buffer */
	/* Listen for the message */
	(void) HAL_UART_Receive_DMA(&GNSS_UART_HANDLE, rxBuffTable,
	WCU_GNSSRX_UART_RX_BUFF_SIZE);

	/* Wait for notification from ISR/message received callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, 0)) {

		static SGnssData dataBuff; /* GNSS data buffer */
		/* Try parsing the message */
		switch (parseMessage(&dataBuff, (char*) rxBuffTable,
		WCU_GNSSRX_UART_RX_BUFF_SIZE)) {

		case EGnssDataStatus_Ready: /* If the data is ready */

			/* Send the data to CAN */
			gnssRx_Send_GPS_POS(&dataBuff);
			gnssRx_Send_GPS_POS2(&dataBuff);
			gnssRx_Send_GPS_STATUS(&dataBuff);

			/* Clear the data buffer */
			(void) memset(&dataBuff, 0x00, sizeof(dataBuff));
			break;

		case EGnssDataStatus_Pending: /* If the data is not complete */

			break;

		case EGnssDataStatus_Error: /* If the parser failed */

			/* Log the error */
			LOGERROR("parseMessage failed in gnssRx\r\n");
			break;

		default:

			break;

		}

	}

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS(SGnssData *pData) {

	SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.UHeader.Tx.DLC = 8;
	canFrame.UHeader.Tx.IDE = CAN_ID_STD;
	canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
	canFrame.UHeader.Tx.StdId = WCU_CAN_ID_GPS_POS;
	canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

	/* Write the longitude to the frame payload */
	int32_t longitude = normalizeCoordinate(pData->Longitude, pData->ELonDir);
	canFrame.PayloadTable[0] = _bits24_31(longitude);
	canFrame.PayloadTable[1] = _bits16_23(longitude);
	canFrame.PayloadTable[2] = _bits8_15(longitude);
	canFrame.PayloadTable[3] = _bits0_7(longitude);

	/* Write the latitude to the frame payload */
	int32_t latitude = normalizeCoordinate(pData->Latitude, pData->ELatDir);
	canFrame.PayloadTable[4] = _bits24_31(latitude);
	canFrame.PayloadTable[5] = _bits16_23(longitude);
	canFrame.PayloadTable[6] = _bits8_15(longitude);
	canFrame.PayloadTable[7] = _bits0_7(longitude);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_POS2(SGnssData *pData) {

	SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.UHeader.Tx.DLC = 6;
	canFrame.UHeader.Tx.IDE = CAN_ID_STD;
	canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
	canFrame.UHeader.Tx.StdId = WCU_CAN_ID_GPS_POS2;
	canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

	/* Write the speed to the frame payload */
	uint16_t speed = normalizeSpeed(pData->Speed);
	canFrame.PayloadTable[0] = _bits8_15(speed);
	canFrame.PayloadTable[1] = _bits0_7(speed);

	/* Write the direction to the frame payload */
	uint16_t direction = normalizeDirection(pData->COG);
	canFrame.PayloadTable[2] = _bits8_15(direction);
	canFrame.PayloadTable[3] = _bits0_7(direction);

	/* Write the altitude to the frame payload */
	uint16_t altitude = normalizeAltitude(pData->Altitude);
	canFrame.PayloadTable[4] = _bits8_15(altitude);
	canFrame.PayloadTable[5] = _bits0_7(altitude);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_POS2 failed to send to canTxQueue\r\n");

}

/**
 * @brief Sends _GPS_POS CAN frame
 * @param pData Pointer to the GNSS data structure
 * @retval None
 */
void gnssRx_Send_GPS_STATUS(SGnssData *pData) {

	SCanFrame canFrame = { .EDataDirection = TX }; /* CAN frame structure */

	/* Configure the CAN Tx header */
	canFrame.UHeader.Tx.DLC = 8;
	canFrame.UHeader.Tx.IDE = CAN_ID_STD;
	canFrame.UHeader.Tx.RTR = CAN_RTR_DATA;
	canFrame.UHeader.Tx.StdId = WCU_CAN_ID_GPS_STATUS;
	canFrame.UHeader.Tx.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canFrame.PayloadTable[0] = pData->SatellitesInViewGLONASS
			+ pData->SatellitesInViewGPS;
	/* Clear two most significant bits */
	canFrame.PayloadTable[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canFrame.PayloadTable[0] |= (uint8_t) pData->EFixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canFrame.PayloadTable[1] = pData->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = normalizeTime(pData->Time);
	canFrame.PayloadTable[2] = _bits24_31(time);
	canFrame.PayloadTable[3] = _bits16_23(time);
	canFrame.PayloadTable[4] = _bits8_15(time);
	canFrame.PayloadTable[5] = _bits0_7(time);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canFrame.PayloadTable[5] |= 0xF & _bits24_31(pData->Date);
	canFrame.PayloadTable[6] = 0xFF & (pData->Date >> 20U);
	canFrame.PayloadTable[7] = 0xFF & (pData->Date >> 12U);

	/* Transmit the frame */
	ADDTOCANTXQUEUE(&canFrame,
			"gnssRx_Send_GPS_STATUS failed to send to canTxQueue\r\n");

}