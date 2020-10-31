/**
 * @author Adrian Cinal
 * @file wcu_gnssrx_calls.c
 * @brief Source file defining functions called by the gnssRx task
 */

#include "wcu_gnssrx_calls.h"

#include "wcu_common.h"
#include "quectel_l26_gnss_parser.h"
#include "rt12e_libs_can.h"
#include "rt12e_libs_generic.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#define CAN_ID_GPS_POS        ( (uint32_t) 0x500 )  /* CAN ID: _500_GPS_POS */
#define CAN_ID_GPS_POS2       ( (uint32_t) 0x501 )  /* CAN ID: _501_GPS_POS2 */
#define CAN_ID_GPS_STATUS     ( (uint32_t) 0x502 )  /* CAN ID: _502_GPS_STATUS */
#define GNSSRX_READ_BUF_SIZE  ( (uint32_t) 50 )     /* Read buffer size */
#define GNSSRX_RING_BUF_SIZE  ( (uint32_t) 200 )    /* UART ring buffer size */
#define GNSS_UART_HANDLE      (huart3)              /* UART handle alias */
#define GNSS_UART_INSTANCE    (USART3)              /* UART instance alias */

extern UART_HandleTypeDef GNSS_UART_HANDLE;
extern osThreadId gnssRxHandle;
SUartRingBuf gGnssRxRingBuffer;

static void gnssRx_RingBufferIdleCallback(void);
static void gnssRx_Send_GPS_POS(SGnssData *gnssDataPtr);
static void gnssRx_Send_GPS_POS2(SGnssData *gnssDataPtr);
static void gnssRx_Send_GPS_STATUS(SGnssData *gnssDataPtr);

/**
 * @brief Configure the Quectel L26 device
 * @retval EGnssRxRet Status
 */
EGnssRxRet gnssRx_DeviceConfig(void) {

	EGnssRxRet status = EGnssRxRet_Ok;

	/* Send packet 220 PMTK_SET_POS_FIX - set position fix interval to 100 ms */
	const char PMTK_SET_POS_FIX[] = "$PMTK220,100*1F\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE, (uint8_t*) PMTK_SET_POS_FIX,
					sizeof(PMTK_SET_POS_FIX), 1000)) {

		LogPrint("Failed to send PMTK_SET_POS_FIX string in gnssRx");
		status = EGnssRxRet_Error;

	}

	/* Send packet 353 PMTK_API_SET_GNSS_SEARCH_MODE - configure the receiver to start searching GPS and GLONASS satellites */
	const char PMTK_API_SET_GNSS_SEARCH_MODE[] = "$PMTK353,1,1,0,0,0*2B\r\n";
	if (HAL_OK
			!= HAL_UART_Transmit(&GNSS_UART_HANDLE,
					(uint8_t*) PMTK_API_SET_GNSS_SEARCH_MODE,
					sizeof(PMTK_API_SET_GNSS_SEARCH_MODE), 1000)) {

		LogPrint(
				"Failed to send PMTK_API_SET_GNSS_SEARCH_MODE string in gnssRx");
		status = EGnssRxRet_Error;

	}

	return status;

}

/**
 * @brief Start listening for incoming UART transmissions
 * @retval EUartRingBufRet Status
 */
EUartRingBufRet gnssRx_StartRingBufferIdleDetectionRx(void) {

	static uint8_t cirBufTbl[GNSSRX_RING_BUF_SIZE ];

	/* Configure the ring buffer structure */
	gGnssRxRingBuffer.BufferPtr = cirBufTbl;
	gGnssRxRingBuffer.BufferSize = GNSSRX_RING_BUF_SIZE;
	gGnssRxRingBuffer.Callback = &gnssRx_RingBufferIdleCallback;
	gGnssRxRingBuffer.PeriphHandlePtr = &GNSS_UART_HANDLE;

	/* Start listening */
	return UartRingBuf_Start(&gGnssRxRingBuffer);

}

/**
 * @brief Listen for and handles the GNSS message
 * @retval EGnssRxRet Status
 */
EGnssRxRet gnssRx_HandleCom(void) {

	EGnssRxRet status = EGnssRxRet_Ok;

	/* Wait for notification from idle line detection callback */
	if (0UL < ulTaskNotifyTake(pdTRUE, WCU_COMMON_TIMEOUT)) {

		static uint8_t rxBufTbl[GNSSRX_READ_BUF_SIZE];
		/* Read the data from the ring buffer */
		UartRingBuf_Read(&gGnssRxRingBuffer, rxBufTbl, GNSSRX_READ_BUF_SIZE);

		static SGnssData dataBuf;
		/* Try parsing the message */
		switch (ParseMessage(&dataBuf, (char*) rxBufTbl,
		GNSSRX_READ_BUF_SIZE)) {

		case EGnssDataStatus_Ready: /* If the data is ready */

			/* Send the data to CAN */
			gnssRx_Send_GPS_POS(&dataBuf);
			gnssRx_Send_GPS_POS2(&dataBuf);
			gnssRx_Send_GPS_STATUS(&dataBuf);

			/* Clear the data buffer */
			(void) memset(&dataBuf, 0, sizeof(dataBuf));
			break;

		case EGnssDataStatus_Pending: /* If the data is not complete */

			break;

		case EGnssDataStatus_Error: /* If the parser failed */

			LogPrint("parseMessage failed (gnssRx)");
			status = EGnssRxRet_Error;
			break;

		default:

			break;

		}

	}

	return status;

}

/**
 * @brief Callback on idle line detection in the ring buffer implementation
 * @retval None
 */
static void gnssRx_RingBufferIdleCallback(void) {

	/* Notify the task */
	vTaskNotifyGiveFromISR(gnssRxHandle, NULL);

}

/**
 * @brief Send _GPS_POS CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void gnssRx_Send_GPS_POS(SGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 8;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_POS;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the longitude to the frame payload */
	int32_t longitude = NormalizeCoordinate(gnssDataPtr->Longitude,
			gnssDataPtr->LonDir);
	canFrame.PayloadTbl[0] = _bits24_31(longitude);
	canFrame.PayloadTbl[1] = _bits16_23(longitude);
	canFrame.PayloadTbl[2] = _bits8_15(longitude);
	canFrame.PayloadTbl[3] = _bits0_7(longitude);

	/* Write the latitude to the frame payload */
	int32_t latitude = NormalizeCoordinate(gnssDataPtr->Latitude,
			gnssDataPtr->LatDir);
	canFrame.PayloadTbl[4] = _bits24_31(latitude);
	canFrame.PayloadTbl[5] = _bits16_23(longitude);
	canFrame.PayloadTbl[6] = _bits8_15(longitude);
	canFrame.PayloadTbl[7] = _bits0_7(longitude);

	/* Transmit the frame */
	AddToCanTxQueue(&canFrame, "AddToCanTxQueue failed (gnssRx_Send_GPS_POS)");

}

/**
 * @brief Send _GPS_POS CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void gnssRx_Send_GPS_POS2(SGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 6;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_POS2;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the speed to the frame payload */
	uint16_t speed = NormalizeSpeed(gnssDataPtr->Speed);
	canFrame.PayloadTbl[0] = _bits8_15(speed);
	canFrame.PayloadTbl[1] = _bits0_7(speed);

	/* Write the direction to the frame payload */
	uint16_t direction = NormalizeDirection(gnssDataPtr->COG);
	canFrame.PayloadTbl[2] = _bits8_15(direction);
	canFrame.PayloadTbl[3] = _bits0_7(direction);

	/* Write the altitude to the frame payload */
	uint16_t altitude = NormalizeAltitude(gnssDataPtr->Altitude);
	canFrame.PayloadTbl[4] = _bits8_15(altitude);
	canFrame.PayloadTbl[5] = _bits0_7(altitude);

	/* Transmit the frame */
	AddToCanTxQueue(&canFrame,
			"AddToCanTxQueue failed (gnssRx_Send_GPS_POS2)");

}

/**
 * @brief Send _GPS_POS CAN frame
 * @param gnssDataPtr Pointer to the GNSS data structure
 * @retval None
 */
static void gnssRx_Send_GPS_STATUS(SGnssData *gnssDataPtr) {

	SCanFrame canFrame;
	/* Configure the CAN Tx header */
	canFrame.TxHeader.DLC = 8;
	canFrame.TxHeader.IDE = CAN_ID_STD;
	canFrame.TxHeader.RTR = CAN_RTR_DATA;
	canFrame.TxHeader.StdId = CAN_ID_GPS_STATUS;
	canFrame.TxHeader.TransmitGlobalTime = DISABLE;

	/* Write the satellites visible count to the frame payload */
	canFrame.PayloadTbl[0] = gnssDataPtr->SatellitesInViewGLONASS
			+ gnssDataPtr->SatellitesInViewGPS;
	/* Clear two most significant bits */
	canFrame.PayloadTbl[0] &= 0b00111111;
	/* Use two most significant bits of the first byte to store fix status flags */
	canFrame.PayloadTbl[0] |= (uint8_t) gnssDataPtr->FixStatus << 6U;

	/* Write the satellites in use count to the frame payload */
	canFrame.PayloadTbl[1] = gnssDataPtr->SatellitesInUse;

	/* Write the time to the frame payload */
	uint32_t time = NormalizeTime(gnssDataPtr->Time);
	canFrame.PayloadTbl[2] = _bits24_31(time);
	canFrame.PayloadTbl[3] = _bits16_23(time);
	canFrame.PayloadTbl[4] = _bits8_15(time);
	canFrame.PayloadTbl[5] = _bits0_7(time);

	/* Pack the date in the frame payload by overwriting four least significant bits of the time */
	canFrame.PayloadTbl[5] |= 0xF & _bits24_31(gnssDataPtr->Date);
	canFrame.PayloadTbl[6] = 0xFF & (gnssDataPtr->Date >> 20U);
	canFrame.PayloadTbl[7] = 0xFF & (gnssDataPtr->Date >> 12U);

	/* Transmit the frame */
	AddToCanTxQueue(&canFrame,
			"AddToCanTxQueue failed (gnssRx_Send_GPS_STATUS)");

}
