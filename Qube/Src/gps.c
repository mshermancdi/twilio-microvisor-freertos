/**
  ******************************************************************************
  * File Name          : gps.c
  * Description        : Code for GPS Interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 CDI.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
	
#include "gps.h"
#include "NMEA.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/**
 * @brief MISRA compliant typedef for float
 */
typedef float float32_t;

/**
 * @brief MISRA compliant typedef for double
 */
typedef double float64_t;

/*
 * Constant for strtol base param
 */
#define BASE 10

/**
 * @brief Constant that indicates the maximum number of sat per GSV message.
 */
#define GSV_MSG_SATS 4

/**
 * @brief Constant that indicates the maximum number of satellites.
 */  
#define MAX_SAT_NUM 12

/**
 * @brief Constant that indicates the maximum lenght of a string.
 */  
#define MAX_STR_LEN 32

#define GPGGA_CMD	"GPGGA"
#define GNGSA_CMD	"GNGSA"
#define	GLGSV_CMD	"GLGSV"
#define	GPGSV_CMD	"GPGSV"
#define	GPVTG_CMD	"GPVTG"
#define	GPRMC_CMD	"GPRMC"


/**
 * @brief Enumeration structure that contains the typologies of Gps fixing process got from a NMEA string
 */
enum {
  INVALID = 0,          /**< Invalid Fix status */
  VALID = 1,            /**< Valid Fix status */
  DGPS_FIX = 2,         /**< DGPS Fix status */
  PPS_FIX = 3,          /**< PPS Fix status */
  REAL_TIME = 4,        /**< Real Time Fix status */
  FLOAT_REAL_TIME = 5,  /**< Float Real Time Fix status */
  ESTIMATED = 6,        /**< Estimated Fix status */
  MANUAL_MODE = 7,      /**< Manual Mode Fix status */
  SIMULATION_MODE = 8   /**< Simulation Mode Fix status */
};

/**
 * @brief Data structure that contains the coordinates information
 */
typedef struct {
  float64_t lat;   /**< Latitude */
  float64_t lon;   /**< Longitude */
  float64_t alt;   /**< Altitude */
  uint8_t ns;      /**< Nord / Sud latitude type */
  uint8_t ew;      /**< East / West longitude type */
  uint8_t mis;     /**< Altitude unit misure */
} Coords_t;

/**
 * @brief Data structure that contains the Gps geoids information
 */
typedef struct {
  int16_t height;  /**< Geoid height */
  uint8_t mis;     /**< Geoid height misure unit */
} Geoid_Info_t;

/**
 * @brief Data structure that contains the UTC information
 */
typedef struct {
  int32_t utc;  /**< UTC Info */
  int16_t hh;   /**< Hours */
  int16_t mm;   /**< Minutes */
  int16_t ss;   /**< Seconds */
} UTC_Info_t;

/**
 * @brief Data structure that contains the Date information
 */
typedef struct {
  int32_t date;  /**< UTC Info */
  int16_t dd;   /**< Day */
  int16_t mm;   /**< Month */
  int16_t yy;   /**< Year */
} Date_Info_t;



/**
 * @brief Data structure that contains all of the information about the GSA satellites 
 */
typedef struct {
  uint8_t constellation[MAX_STR_LEN]; /**< Constellation enabled: GPGSA (GPS), GLGSA (GLONASS), GAGSA (GALILEO), BDGSA (BEIDOU), GNGSA (more than one) */ 
  uint8_t operating_mode;             /**< Operating Mode: 'M' = Manual, 'A' = Auto (2D/3D) */
  int16_t current_mode;               /**< Current Mode: 1. no fix available, 2. 2D, 3. 3D */
  int32_t sat_prn[MAX_SAT_NUM];       /**< Satellites list used in position fix (max N 12) */
  float32_t pdop;	                  /**< Position Dilution of Precision, max: 99.0 */
  float32_t hdop;                     /**< Horizontal Dilution of Precision, max: 99.0 */
  float32_t vdop;                     /**< Vertical Dilution of Precision, max: 99.0 */
  uint32_t checksum;                  /**< Checksum of the message bytes */
} GSA_Info_t;
  
/**
 * @brief Data structure that contains the GSV information
 */
typedef struct {
  int16_t prn;   /**< PRN */
  int16_t elev;  /**< Elevation of satellite in degree, 0 ... 90 */
  int16_t azim;  /**< Azimuth of satellite in degree, ref. "North, " 0 ... 359 */
  int16_t cn0;   /**< Carrier to noise ratio for satellite in dB, 0 ... 99 */
} GSV_SAT_Info_t;

/**
 * @brief Data structure that contains all of the information about the GSV satellites 
 */
typedef struct {
  uint8_t constellation[MAX_STR_LEN];    /**< Constellation enabled: GPGSV (GPS), GLGSV (GLONASS), GAGSV (GALILEO), BDGSV (BEIDOU), QZGSV (QZSS), GNGSV (more than one) */ 
  int16_t amount;                        /**< Total amount of GSV messages, max. 3 */
  int16_t number;                        /**< Continued GSV number of this message */
  int16_t tot_sats;                      /**< Total Number of Satellites in view, max. 12 */
  int16_t current_sats;
  GSV_SAT_Info_t gsv_sat_i[MAX_SAT_NUM]; /**< Satellite info  */
  uint32_t checksum;	                 /**< Checksum of the message bytes */
} GSV_Info_t;

/**
 * @brief Data structure that contains all of the information about the GPS position 
 */
typedef struct {
  UTC_Info_t utc;         /**< UTC Time */
  Coords_t xyz;	          /**< Coords data member */
  float32_t acc;          /**< GPS Accuracy */
  int16_t sats;	          /**< Number of satellities acquired */
  uint8_t valid;          /**< GPS Signal fix quality */
  Geoid_Info_t geoid;	  	/**< Geoids data info member */
  int16_t update;         /**< Update time from the last acquired GPS Info */
  uint32_t checksum;      /**< Checksum of the message bytes */
} GPGGA_Info_t;

/**
 * @brief Data structure that contains all of the information about the track and ground speed 
 */
typedef struct {
  uint8_t constellation[MAX_STR_LEN];   /**< Constellation enabled: GPGSV (GPS), GLGSV (GLONASS), GAGSV (GALILEO), BDGSV (BEIDOU), QZGSV (QZSS), GNGSV (more than one) */ 
	float32_t trackTrue;									/**< Track in reference to "true" earth poles */
	float32_t trackMag;										/**< Track in reference to "magnetic" earth poles */
	float32_t groundSpdKnots;							/**< Speed over Ground in knots */	
	float32_t groundSpdKmphr;								/**< Speed over Ground in kilometers per hour */	
	uint8_t	mode;													/**< Mode: A = Autonomous, D = Differential, E = Estimated */
	uint32_t checksum;      							/**< Checksum of the message bytes */
} VTG_Info_t;

typedef struct {	
	Date_Info_t date;	
} RMC_Info_t;

typedef struct _S_GPS_DEVICE_DATA
{
	GPGGA_Info_t 	GPGGA_Info;
	GSV_Info_t 		GSV_Info;
	GSA_Info_t		GNGSA_Info;
	VTG_Info_t		VTG_Info;
	RMC_Info_t		RMC_Info;
} S_GPS_DEVICE_DATA;

/* Module Level Global Variables */
S_GPS_DEVICE	*pGPSDevice = NULL;
S_NMEAParser 	GPS_nmeaParser;
S_GPS_DEVICE_DATA GPS_data;

void ProcessGGA(void *pDevice);
void ProcessGSA(void *pDevice);
void ProcessGSV(void *pDevice);
void ProcessVTG(void *pDevice);
void ProcessRMC(void *pDevice);

static void scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC);
static void scan_date(uint8_t *pDateStr, Date_Info_t *pDate);
static void NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo);
static void PrintDateTime(uint8_t *pBuff, uint16_t len);

#define NUM_GGA_PARAMS	12
#define NUM_GSA_PARAMS	17
#define NUM_GSV_PARAMS	20
#define NUM_VTG_PARAMS	9
#define NUM_RMC_PARAMS	12

/**
 * @brief Constant that indicates the maximum lenght of NMEA message field.
 */
#define MAX_MSG_LEN 48/* 32 */ /* was 19 */

/* Private variables ---------------------------------------------------------*/
static uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
static uint32_t GetTimeMs(S_GPS_DEVICE *pGPS);

NMEA_PROC_CMD_TABLE gpsNmeaProcCmdTable[] = {
	{GPGGA_CMD, ProcessGGA},
	{GNGSA_CMD, ProcessGSA},
	{GLGSV_CMD, ProcessGSV},
	{GPGSV_CMD, ProcessGSV},
	{GPVTG_CMD,	ProcessVTG},
	{GPRMC_CMD,	ProcessRMC},	
};

#define NUM_GPS_NMEA_PROC_CMD	sizeof(gpsNmeaProcCmdTable)/sizeof(gpsNmeaProcCmdTable[0])

uint8_t GPS_ProcessCommand(void)
{	
	volatile uint32_t len = 0;
	
	if (!pGPSDevice)
		return 1;
	
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pGPSDevice;
	
	for (uint16_t i=0; i<pGPS->pNmeaParser->m_numTableCmds; i++)	
	{
		len = strlen((const char*)pGPS->pNmeaParser->m_Command);
		if (!strncmp((char *)pGPS->pNmeaParser->m_Command,pGPS->pNmeaParser->pCmdTable[i].cmd,len))
		{
//			strcpy((char *)pGPS->lastCommandRcvd, (char *)pGPS->pNmeaParser->m_Command);
			pGPS->pNmeaParser->pCmdTable[i].fun(pGPS);
			break;
		}
	}
	return 0;
}

uint8_t GPS_ParseBuffer(uint8_t *pBuff, uint16_t dataLen)
{
	if (!pGPSDevice)
		return 1;
	
	ParseBuffer(pGPSDevice->pNmeaParser, pBuff, dataLen);
	return 0;
}

E_GPS_DEV_TYPE GetGPSDeviceType(void)
{
	// Use pin strapping to determine device type when available
	return STM_TESEO_LIV3F;
}

E_GPS_ERR GPS_Initialize(S_GPS_DEVICE *pDevice, TX_MUTEX *pGPSMutex)
{
	if (!pDevice || !pGPSMutex)
		return GPS_NOT_OK;
	
	pGPSDevice = pDevice;	// Save global pointer
	
	if (pDevice->IsInitialized)	// One-time initialization
		return GPS_OK;
		
	memset(pDevice, 0, sizeof(S_GPS_DEVICE));
	pDevice->pMutex = pGPSMutex;
	pDevice->deviceType = GetGPSDeviceType();
	pDevice->pNmeaParser = &GPS_nmeaParser;
	pDevice->pNmeaParser->pCmdTable = gpsNmeaProcCmdTable;
	pDevice->pNmeaParser->m_numTableCmds = NUM_GPS_NMEA_PROC_CMD;
	pDevice->pNmeaParser->ProcessCommand = GPS_ProcessCommand;
	pDevice->ParseBuffer = GPS_ParseBuffer;
	pDevice->pData = &GPS_data;
	pDevice->ppsTimerSource = TIM3;
	pDevice->IsInitialized = 1;
	return GPS_OK;
}

void GPS_Summary(uint8_t *pBuffer, uint16_t bufLen)
{
	if (!pBuffer)
		return;
	
	uint16_t lat_deg = pGPSDevice->pData->GPGGA_Info.xyz.lat/100;
	float32_t lat_ms = pGPSDevice->pData->GPGGA_Info.xyz.lat - (float32_t)lat_deg * 100.0;
	uint16_t lon_deg = pGPSDevice->pData->GPGGA_Info.xyz.lon/100;
	float32_t lon_ms = pGPSDevice->pData->GPGGA_Info.xyz.lon - (float32_t)lon_deg * 100.0;
	
	
	uint8_t fix_q[][MAX_MSG_LEN]= {
		"Invalid Fix status",					// INVALID
		"Valid Fix status",						// VALID
		"DGPS Fix status",						// DGPS_FIX
		"PPS Fix status",							// PPS_FIX
		"Real Time Fix status",				// REAL_TIME
		"Float Real Time Fix status",	// FLOAT_REAL_TIME
		"Estimated Fix status",				// ESTIMATED
		"Manual Mode Fix status",			// MANUAL_MODE	
		"Simulation Mode Fix status"	// SIMULATION_MODE	
	};
	
	uint8_t gsamode[][MAX_MSG_LEN] = {
		"",
		"Fix not available",
		"2D",
		"3D"
	};
	
	// Get the mutex 
	tx_mutex_get(pGPSDevice->pMutex, TX_WAIT_FOREVER);
	
	uint8_t buff[40] = {0};
	uint8_t gsv_sat_info[400] = {0};
	uint8_t gps_act_sats[100] = {0};
	uint8_t dtbuff[30] = {0};

	for (uint8_t i=0; i<pGPSDevice->pData->GSV_Info.tot_sats; i++)
	{
		snprintf((char*)buff, sizeof(buff), "%2d) ID: %2d El: %2d Az: %3d SNR: %2d\n",
			i+1, pGPSDevice->pData->GSV_Info.gsv_sat_i[i].prn, pGPSDevice->pData->GSV_Info.gsv_sat_i[i].elev, pGPSDevice->pData->GSV_Info.gsv_sat_i[i].azim, pGPSDevice->pData->GSV_Info.gsv_sat_i[i].cn0);
		strncat((char*)gsv_sat_info, (const char*)buff, strlen((char*)buff));
		memset(buff, 0, sizeof(buff));
	}

	sprintf((char*)gps_act_sats, "%s ", "GPS Act Sats ");
	for (uint8_t i=0; i<MAX_SAT_NUM; i++)
	{
		if (pGPSDevice->pData->GNGSA_Info.sat_prn[i] > 0)
		{
			snprintf((char*)buff, sizeof(buff), "%d: %ld ", i+1, pGPSDevice->pData->GNGSA_Info.sat_prn[i]);
			strncat((char*)gps_act_sats, (const char*)buff, strlen((char*)buff));
		}
		memset(buff, 0, sizeof(buff));
	}
	strcat((char*)gps_act_sats, "\n");

	PrintDateTime(dtbuff, sizeof(dtbuff));

	snprintf((char*)pBuffer, bufLen,
	"----------------------------------------------------\n" \
	"\t\t\tGPS Info\n" \
	"----------------------------------------------------\n" \
	"GPS FIXED DATA\n" \
	"Fix Quality: %s\n" \
	"Date/Time: %s\n" \
	"Lat/Lon: %dd %09.6f' %c %dd %09.6f' %c\n" \
	"Number Satellites: %2d\n" \
	"HDOP: %3.1f\n" \
	"Altitude: %3.1f %c\n" \
	"Height of geoid above WGS84 ellipsoid: %d %c\n" \
	"\n" \
	"GNSS DOP AND ACTIVE SATELLITES\n" \
	"Mode (Man/Auto): %c, %s\n" \
	"%s" \
	"PDOP: %3.1f\n" \
	"HDOP: %3.1f\n" \
	"VDOP: %3.1f\n" \
	"\n" \
	"GNSS SATELLITES IN VIEW\n" \
	"Sats in View: %d\n" \
	"%s" \
	"\n" \
	"COURSE OVER GROUND AND GROUND SPEED\n" \
	"%5.1f,T\tTrue track made good\n" \
	"%5.1f,M\tMag track made good\n" \
	"%5.1f,N\tGround speed, knots\n" \
	"%5.1f,N\tGround speed, km/hr \n" \
	"Mode (Auto, Diff, Est): %c\n" \
	"\n",
	fix_q[pGPSDevice->pData->GPGGA_Info.valid],
	dtbuff,
	lat_deg,lat_ms,pGPSDevice->pData->GPGGA_Info.xyz.ns,
	lon_deg,lon_ms,pGPSDevice->pData->GPGGA_Info.xyz.ew,
	pGPSDevice->pData->GPGGA_Info.sats,
	pGPSDevice->pData->GPGGA_Info.acc,
	pGPSDevice->pData->GPGGA_Info.xyz.alt, pGPSDevice->pData->GPGGA_Info.xyz.mis,
	pGPSDevice->pData->GPGGA_Info.geoid.height, pGPSDevice->pData->GPGGA_Info.geoid.mis,
	pGPSDevice->pData->GNGSA_Info.operating_mode, gsamode[pGPSDevice->pData->GNGSA_Info.current_mode],
	gps_act_sats,
	pGPSDevice->pData->GNGSA_Info.pdop,
	pGPSDevice->pData->GNGSA_Info.hdop,
	pGPSDevice->pData->GNGSA_Info.vdop,
	pGPSDevice->pData->GSV_Info.tot_sats,
	gsv_sat_info,
	pGPSDevice->pData->VTG_Info.trackTrue,
	pGPSDevice->pData->VTG_Info.trackMag,
	pGPSDevice->pData->VTG_Info.groundSpdKnots,
	pGPSDevice->pData->VTG_Info.groundSpdKmphr,
	pGPSDevice->pData->VTG_Info.mode
	);
	
	// Release the mutex
	tx_mutex_put(pGPSDevice->pMutex);
}

/***************************************************************************

	NMEA Sentence Processing Functions

****************************************************************************/
	

void ProcessGGA(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	if (!pDevice)
		return;
		
	// Cast to internal device type
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pDevice;

	/* clear the app[] buffer */ 
	for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		(void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
	}

	for (int8_t i=0; i<NUM_GGA_PARAMS; i++)
	{
		if(GetField(pGPS->pNmeaParser->m_Data, pField, i, MAXFIELD))			
			memcpy(app[i], pField, strlen((char*)pField));
	}
		
	// Get the mutex 
	tx_mutex_get(pGPS->pMutex, TX_WAIT_FOREVER);
	
		scan_utc(app[0], &(pGPS->pData->GPGGA_Info.utc));
		pGPS->pData->GPGGA_Info.xyz.lat = strtod((char*)app[1], NULL);
		pGPS->pData->GPGGA_Info.xyz.ns = *((uint8_t*)app[2]);
		pGPS->pData->GPGGA_Info.xyz.lon = strtod((char*)app[3], NULL);
		pGPS->pData->GPGGA_Info.xyz.ew = *((uint8_t*)app[4]);
		int32_t valid = strtol((char *)app[5], NULL, BASE);
		if((valid <= 8) && (valid >= 0))
		{
			pGPS->pData->GPGGA_Info.valid = (uint8_t)valid;
		}
		pGPS->pData->GPGGA_Info.sats = strtol((char*)app[6], NULL, BASE);
		pGPS->pData->GPGGA_Info.acc = strtof((char*)app[7], NULL);
		pGPS->pData->GPGGA_Info.xyz.alt = strtof((char*)app[8], NULL);
		pGPS->pData->GPGGA_Info.xyz.mis = *((uint8_t*)app[9]);
		pGPS->pData->GPGGA_Info.geoid.height = strtol((char*)app[10], NULL, BASE);
		pGPS->pData->GPGGA_Info.geoid.mis = *((uint8_t*)app[11]);			
		pGPS->pData->GPGGA_Info.checksum = pGPS->pNmeaParser->m_RxChecksum;
		
	// Release the mutex
	tx_mutex_put(pGPS->pMutex);
}

void ProcessGSA(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	if (!pDevice)
		return;
	
	// Cast to internal device type
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pDevice;
			
	/* clear the app[] buffer */ 
	for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		(void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
	}

	for (int8_t i=0; i<NUM_GSA_PARAMS; i++)
	{
		if(GetField(pGPS->pNmeaParser->m_Data, pField, i, MAXFIELD))			
			memcpy(app[i], pField, strlen((char*)pField));
	}
	
	// Get the mutex 
	tx_mutex_get(pGPS->pMutex, TX_WAIT_FOREVER);
	
		strncpy((char*)&pGPS->pData->GNGSA_Info.constellation, (char*)pGPS->pNmeaParser->m_Command, MAX_STR_LEN);
	
		pGPS->pData->GNGSA_Info.operating_mode = *((uint8_t*)app[0]);
		pGPS->pData->GNGSA_Info.current_mode = strtol((char*)app[1], NULL, BASE);
		
		int32_t *sat_prn = pGPS->pData->GNGSA_Info.sat_prn;
		for (int8_t i = 0; i < MAX_SAT_NUM; i++)
		{
			*(&sat_prn[i]) = strtol((char*)app[2+i], NULL, BASE);					
		}
		pGPS->pData->GNGSA_Info.pdop = strtof((char*)app[14], NULL);
		pGPS->pData->GNGSA_Info.hdop = strtof((char*)app[15], NULL);
		pGPS->pData->GNGSA_Info.vdop = strtof((char*)app[16], NULL);		
		pGPS->pData->GNGSA_Info.checksum = pGPS->pNmeaParser->m_RxChecksum;
	
	// Release the mutex
	tx_mutex_put(pGPS->pMutex);
}

void ProcessGSV(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	int32_t gsv_idx = 0;
	int8_t app_idx;
	
	if (!pDevice)
		return;
	
	// Cast to internal device type
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pDevice;
					
	/* clear the app[] buffer */ 
	for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		(void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
	}

	for (int8_t i=0; i<NUM_GSV_PARAMS; i++)
	{
		if(GetField(pGPS->pNmeaParser->m_Data, pField, i, MAXFIELD))			
			memcpy(app[i], pField, strlen((char*)pField));
	}
	
	// Get the mutex 
	tx_mutex_get(pGPS->pMutex, TX_WAIT_FOREVER);
	
		if ( strtol((char*)app[1], NULL, BASE) == 1)	// Reset message structure on first message of group
			NMEA_ResetGSVMsg(&pGPS->pData->GSV_Info);
	
		strncpy((char*)&pGPS->pData->GSV_Info.constellation, (char*)pGPS->pNmeaParser->m_Command, MAX_STR_LEN);
	
		pGPS->pData->GSV_Info.amount = strtol((char*)app[0], NULL, BASE);
		pGPS->pData->GSV_Info.number = strtol((char*)app[1], NULL, BASE);			
		pGPS->pData->GSV_Info.tot_sats = strtol((char*)app[2], NULL, BASE);			
		
		app_idx = 4;
		gsv_idx = pGPS->pData->GSV_Info.current_sats;
		
		for (int8_t i = 1; i<=GSV_MSG_SATS; i++)
		{
			pGPS->pData->GSV_Info.gsv_sat_i[gsv_idx].prn = 	strtol((char *)app[app_idx*i-1], 	NULL, BASE);
			pGPS->pData->GSV_Info.gsv_sat_i[gsv_idx].elev = strtol((char *)app[app_idx*i], 		NULL, BASE);
			pGPS->pData->GSV_Info.gsv_sat_i[gsv_idx].azim = strtol((char *)app[app_idx*i+1], 	NULL, BASE);
			pGPS->pData->GSV_Info.gsv_sat_i[gsv_idx].cn0 = 	strtol((char *)app[app_idx*i+2], 	NULL, BASE);
			
			if (pGPS->pData->GSV_Info.gsv_sat_i[gsv_idx].prn != 0)
			{
				pGPS->pData->GSV_Info.current_sats++;
			}
			gsv_idx++;
		}
		
		pGPS->pData->GSV_Info.checksum = pGPS->pNmeaParser->m_RxChecksum;
	
	// Release the mutex
	tx_mutex_put(pGPS->pMutex);
}

void ProcessVTG(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	if (!pDevice)
		return;
	
	// Cast to internal device type
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pDevice;

	/* clear the app[] buffer */ 
	for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		(void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
	}

	for (int8_t i=0; i<NUM_VTG_PARAMS; i++)
	{
		if(GetField(pGPS->pNmeaParser->m_Data, pField, i, MAXFIELD))			
			memcpy(app[i], pField, strlen((char*)pField));
	}
		
	// Get the mutex 
	tx_mutex_get(pGPS->pMutex, TX_WAIT_FOREVER);
	
		strncpy((char*)&pGPS->pData->VTG_Info.constellation, (char*)pGPS->pNmeaParser->m_Command, MAX_STR_LEN);
		pGPS->pData->VTG_Info.trackTrue = strtof((char*)app[0], NULL);
		pGPS->pData->VTG_Info.trackMag = strtof((char*)app[2], NULL);
		pGPS->pData->VTG_Info.groundSpdKnots = strtof((char*)app[4], NULL);
		pGPS->pData->VTG_Info.groundSpdKmphr = strtof((char*)app[6], NULL);
		pGPS->pData->VTG_Info.mode = *((uint8_t*)app[8]);	
		pGPS->pData->VTG_Info.checksum = pGPS->pNmeaParser->m_RxChecksum;
		
	// Release the mutex
	tx_mutex_put(pGPS->pMutex);
}

void ProcessRMC(void *pDevice)
{	
	uint8_t pField[MAXFIELD];
	
	if (!pDevice)
		return;
	
	// Cast to internal device type
	S_GPS_DEVICE *pGPS = (S_GPS_DEVICE*)pDevice;

	/* clear the app[] buffer */ 
	for (int8_t i = 0; i < MAX_MSG_LEN; i++)
	{
		(void)memset(app[i], 0, (size_t)MAX_MSG_LEN);
	}

	for (int8_t i=0; i<NUM_RMC_PARAMS; i++)
	{
		if(GetField(pGPS->pNmeaParser->m_Data, pField, i, MAXFIELD))			
			memcpy(app[i], pField, strlen((char*)pField));
	}
		
	// Get the mutex 
	tx_mutex_get(pGPS->pMutex, TX_WAIT_FOREVER);
	
		scan_date(app[8], &(pGPS->pData->RMC_Info.date));
				
	// Release the mutex
	tx_mutex_put(pGPS->pMutex);	
}

/***************************************************************************
	Utilities
****************************************************************************/
/*
 * Function that scans a string with UTC Info_t and fills all fields of a
 * UTC_Info_t struct
 */
static void scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC)
{
  pUTC->utc = strtol((char *)pUTCStr,NULL,10);
  
  pUTC->hh = (pUTC->utc / 10000);
  pUTC->mm = (pUTC->utc - (pUTC->hh * 10000)) / 100;
  pUTC->ss = pUTC->utc - ((pUTC->hh * 10000) + (pUTC->mm * 100));
  
  return;
}

/*
 * Function that scans a string with UTC Info_t and fills all fields of a
 * UTC_Info_t struct
 */
static void scan_date(uint8_t *pDateStr, Date_Info_t *pDate)
{
  pDate->date = strtol((char *)pDateStr,NULL,10);
  
  pDate->dd = (pDate->date / 10000);
  pDate->mm = (pDate->date - (pDate->dd * 10000)) / 100;
  pDate->yy = pDate->date - ((pDate->dd * 10000) + (pDate->mm * 100));
  
  return;
}

/*
 *  Helper function to reset GSV fields
 */
static void NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo)
{
  (void)memset(pGSVInfo->constellation, 0, (size_t)MAX_STR_LEN);
  pGSVInfo->amount = 0;
  pGSVInfo->number = 0;
  pGSVInfo->current_sats = 0;
  pGSVInfo->tot_sats = 0;
  for (int8_t i = 0; i < MAX_SAT_NUM; i++)
  {
    (void)memset(&pGSVInfo->gsv_sat_i[i], 0, sizeof(GSV_SAT_Info_t));
  }
}

/*
 *  Get millisecond time from timer 
 */
static uint32_t GetTimeMs(S_GPS_DEVICE *pGPS)
{
	if (pGPS)
		return pGPS->ppsTimerSource->CNT;
	else
		return 0;	
}

static void PrintDateTime(uint8_t *pBuff, uint16_t len)
{
	if (!pBuff)
		return;
	
//	snprintf((char*) pBuff, len, "%2d/%2d/%2d %02d:%02d:%02d.%03ld Z",
	snprintf((char*) pBuff, len, "%2d/%2d/%2d %02d:%02d:%02d",
	pGPSDevice->pData->RMC_Info.date.mm,
	pGPSDevice->pData->RMC_Info.date.dd,
	pGPSDevice->pData->RMC_Info.date.yy,	
	pGPSDevice->pData->GPGGA_Info.utc.hh,
	pGPSDevice->pData->GPGGA_Info.utc.mm, 
	pGPSDevice->pData->GPGGA_Info.utc.ss
//	,
//	GetTimeMs(pGPSDevice)
	);
}

/*
 *  Update time after PPS 
 */
void GPS_TimeUpdate(void)
{
	if (!pGPSDevice)
		return;
	
	// Get the mutex 
	tx_mutex_get(pGPSDevice->pMutex, TX_WAIT_FOREVER);
		pGPSDevice->pData->GPGGA_Info.utc.ss++;
		
		// Handle the rollover
		if (pGPSDevice->pData->GPGGA_Info.utc.ss > 59)
		{
			pGPSDevice->pData->GPGGA_Info.utc.ss = 0;
			pGPSDevice->pData->GPGGA_Info.utc.mm++;
			if (pGPSDevice->pData->GPGGA_Info.utc.mm > 59)
			{
				pGPSDevice->pData->GPGGA_Info.utc.mm = 0;
				pGPSDevice->pData->GPGGA_Info.utc.hh++;
				if (pGPSDevice->pData->GPGGA_Info.utc.hh > 23)
				{
					pGPSDevice->pData->GPGGA_Info.utc.hh = 0;
					pGPSDevice->pData->RMC_Info.date.dd++;	
					// Just increment the day in the event it crosses over the time boundary.
					// If it happens to be an invalid day (like Sep 31 or Feb 29 in non-leap year) the 
					// next second will be correct, but at minimum should not go backwards in time.
				}
			}	
		}
	// Release the mutex
	tx_mutex_put(pGPSDevice->pMutex);
}
