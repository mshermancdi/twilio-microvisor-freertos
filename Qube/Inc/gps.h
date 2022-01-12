/**********************************************************************************************


	gps.h



*************************************************************************************************/

#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include "debug_utils.h"
#include "NMEA.h"
//#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "app_threadx.h"
#include "stm32u585xx.h"

typedef enum _E_GPS_ERR {
	GPS_OK,
	GPS_NOT_OK
} E_GPS_ERR;

typedef enum _E_GPS_DEV_TYPE {
	NO_GPS,
	STM_TESEO_LIV3F,	// default 
	QUECTEL,
	UBLOX,
}E_GPS_DEV_TYPE;
typedef struct _S_GPS_DEVICE 			S_GPS_DEVICE;
typedef struct _S_GPS_DEVICE_DATA S_GPS_DEVICE_DATA;

typedef struct _S_GPS_DEVICE {
	TX_MUTEX 					*pMutex;
	E_GPS_DEV_TYPE 		deviceType;
	uint8_t						IsInitialized;
	S_NMEAParser 			*pNmeaParser;
	S_GPS_DEVICE_DATA *pData;
	TIM_TypeDef				*ppsTimerSource;
	uint8_t (*ParseBuffer)(uint8_t *pBuff, uint16_t dataLen);	
} S_GPS_DEVICE;

E_GPS_ERR GPS_Initialize(S_GPS_DEVICE *pDevice, TX_MUTEX *pGPSMutex);
void GPS_Summary(uint8_t *pBuffer, uint16_t bufLen);
void GPS_TimeUpdate(void);
#endif	/* __GPS_H */
