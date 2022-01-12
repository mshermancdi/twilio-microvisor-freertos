/**
  ******************************************************************************
  * File Name          : swarm.c
  * Description        : Code for Swarm Interface
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 CDI.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
	
#include "../Inc/swarm.h"

#include "NMEA.h"
#include <stdio.h>
#include <string.h>	
#include <stdlib.h>
#include "debug_menu_strings_en.h"
#include "main.h"
#include "debug_utils.h"
#include "app_threadx.h" 


#define CONFIG_SETTING_CMD	"CS"
#define DATETIME_STATUS_CMD	"DT"
#define FIRMWARE_VER_CMD		"FV"
#define GPS_JAM_SPOOF_CMD		"GJ"
#define GEO_INFO_CMD				"GN"
#define GPIO_CTL_CMD				"GP"
#define GPS_FIX_CMD					"GS"
#define MSGS_RX_CMD					"MM"
#define MSGS_TX_CMD					"MT"
#define POWEROFF_CMD				"PO"
#define POWERSTATUS_CMD			"PW"
#define RX_DATA_CMD					"RD"
#define RESTART_CMD					"RS"
#define RX_TEST_CMD					"RT"
#define SLEEP_MODE_CMD			"SL"
#define TILE_STATUS_CMD			"TILE"
#define M138_STATUS_CMD			"M138"
#define TX_DATA_CMD					"TD"
		 
#define RSSI_THRESHOLD	-93
		 
enum E_REQ_STATE {
	COUNT_REQ,
	DELETE_REQ
};
		 
extern uint8_t MESSAGES[NUM_E_MESSAGES][MAX_DEBUG_MENU_STRLEN];
#ifdef AZURE_RTOS_THREADX
	extern TX_EVENT_FLAGS_GROUP SwarmDataReadyFlagsGroup;
	extern TX_EVENT_FLAGS_GROUP LEDEventFlagsGroup;
#else
	extern osEventFlagsId_t SwarmDataReadyFlagsHandle;
	extern osThreadId_t ledTaskHandle;
#endif 

//uint8_t GetField(uint8_t* pData, uint8_t* pField, int nFieldNum, int nMaxFieldLen);
uint8_t SWARM_ParseBuffer(uint8_t *pBuff, uint16_t dataLen);
E_SWARM_STATE SWARM_Startup(uint8_t *pBuff);
uint8_t SWARM_ProcessCommand(void);
E_SWARM_ERR SWARM_GetResponse(uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
void SWARM_PackageTXMessage(uint8_t *pOut, uint8_t *pIn);
uint8_t nmeaChecksum(const char *sz, uint8_t len);

/*	Console Responses */
void GetConfigResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetDateTimeResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetFirmwareVersionResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetGPSJamResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetGeoInfoResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetGPIOResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetGPSFixResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetPowerOffResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetPowerStatusResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetRestartResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetRxDataResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetRxTestResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetStatusResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetSleepResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetTxDataResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetTxMessagesResponse(uint8_t *pResponseBuf, uint16_t bufLen);
void GetRxMessagesResponse(uint8_t *pResponseBuf, uint16_t bufLen);



/*	JSON Formatted Responses */
void SWARM_GetPositionTsJSON(uint8_t *pResponseBuf, uint8_t bufLen);


void ProcessRT(void *pDevice);
void ProcessGN(void *pDevice);
void ProcessGJ(void *pDevice);
void ProcessDT(void *pDevice);
void ProcessFV(void *pDevice);
void ProcessCS(void *pDevice);
void ProcessGP(void *pDevice);
void ProcessGS(void *pDevice);
void ProcessPO(void *pDevice);
void ProcessPW(void *pDevice);
void ProcessRS(void *pDevice);
void ProcessSL(void *pDevice);
void ProcessMM(void *pDevice);
void ProcessMT(void *pDevice);
void ProcessRD(void *pDevice);
void ProcessSWARM_DEVICE(void *pDevice);
void ProcessTD(void *pDevice);


NMEA_PROC_CMD_TABLE swarmNmeaProcCmdTable[] = {
	{RX_TEST_CMD, 				ProcessRT},
	{GEO_INFO_CMD, 				ProcessGN},
	{GPS_JAM_SPOOF_CMD, 	ProcessGJ},
	{DATETIME_STATUS_CMD, ProcessDT},
	{FIRMWARE_VER_CMD, 		ProcessFV},
	{CONFIG_SETTING_CMD, 	ProcessCS},
	{GPIO_CTL_CMD, 				ProcessGP},	
	{GPS_FIX_CMD, 				ProcessGS},	
	{POWEROFF_CMD, 				ProcessPO},	
	{POWERSTATUS_CMD, 		ProcessPW},
	{RESTART_CMD, 				ProcessRS},
	{SLEEP_MODE_CMD, 			ProcessSL},
	{MSGS_RX_CMD, 				ProcessMM},
	{MSGS_TX_CMD, 				ProcessMT},
	{RX_DATA_CMD, 				ProcessRD},
	{TILE_STATUS_CMD, 		ProcessSWARM_DEVICE},
	{M138_STATUS_CMD, 		ProcessSWARM_DEVICE},
	{TX_DATA_CMD, 				ProcessTD},
};


//(1) - These two variations allow the user application to use GPIO1 as a general purpose output.
//(2) - If either of these modes are selected, the pin will indicate if the Tile has received one or
//more unread messages and is holding them for the client. If multiple messages are pending for
//the client, the pin will maintain the state until all messages have been read.
//(3) - GPIO will change states no faster than 500�s prior to the Tile beginning to transmit and
//500�s after the Tile has finished transmitting.
//(4) - If either of these modes are selected, the pin will be set to the selected state after the client
//has issued the $SL command. The pin will return to the awake state only if the sleep mode is
//terminated by the S or T parameter being reached, or activity is detected on the serial RX line.


typedef struct _S_RT_CMD_DATA {
	uint32_t		rxCount;
	uint32_t		satRxCount;
	int16_t 		bg_rssi;
	int16_t 		rssi_sat;
	int16_t			snr;
	int16_t			fdev;
	uint16_t		OKCount;
	uint16_t		ERRCount;
	uint32_t		Rate;
	char				utc_time[20];
	char				sat_id[10];
} S_RT_CMD_DATA;

typedef struct _S_GN_CMD_DATA {
	uint32_t		rxCount;
	uint16_t		OKCount;
	uint16_t		ERRCount;
	uint32_t		Rate;
	float 			latitude;
	float				longitude;
	float				altitude;
	float				course;
	float				speed;
} S_GN_CMD_DATA;

typedef struct _S_GJ_CMD_DATA {
	uint32_t					rxCount;
	uint8_t						jamming_level;
	uint16_t					OKCount;
	uint16_t					ERRCount;
	uint32_t					Rate;	
	enum SPOOF_STATE 	spoof_state;
} S_GJ_CMD_DATA;

typedef struct _S_DT_CMD_DATA {
	uint32_t	rxCount;
	uint16_t	year;
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
	uint8_t		minute;
	uint8_t		seconds;
	uint16_t	OKCount;
	uint16_t	ERRCount;
	uint32_t	Rate;	
	char			ERR_CAUSE[16];
	char			flag;
} S_DT_CMD_DATA;

typedef struct _S_FV_CMD_DATA {
	uint32_t	rxCount;
	char 			version_str[30];
	uint16_t		ERRCount;
} S_FV_CMD_DATA;

typedef struct _S_CS_CMD_DATA {	
	uint32_t	rxCount;
	uint16_t		ERRCount;
	char			dev_id[10];
	char			dev_name[10];		
} S_CS_CMD_DATA;

typedef struct _S_GP_CMD_DATA {
	uint32_t						rxCount;
	enum SWARM_GPIO_MODE mode;
	uint16_t							OKCount;
	uint16_t							ERRCount;
} S_GP_CMD_DATA;

typedef struct _S_GS_CMD_DATA {
	uint32_t				rxCount;
	uint16_t				hdop;
	uint16_t				vdop;
	uint16_t				gnss_sats;
	uint16_t				unused;
	enum FIX_TYPE		fixType;
	uint16_t				OKCount;
	uint16_t				ERRCount;
	uint32_t				Rate;	
} S_GS_CMD_DATA;

typedef struct _S_PO_CMD_DATA {	
	uint32_t	rxCount;
	int16_t		OKCount;
	int16_t		ERRCount;
} S_PO_CMD_DATA;

typedef struct _S_PW_CMD_DATA {	
	uint32_t	rxCount;
	float			unused1;
	float			unused2;
	float			unused3;
	float			unused4;
	float			temp;
	uint16_t		OKCount;
	uint16_t		ERRCount;
	uint32_t		Rate;	
} S_PW_CMD_DATA;

typedef struct _S_RS_CMD_DATA {	
	uint32_t	rxCount;
	uint16_t		OKCount;
	uint16_t		ERRCount;
} S_RS_CMD_DATA;

typedef struct _S_SL_CMD_DATA {	
	uint32_t	rxCount;
	uint16_t		OKCount;	
	uint16_t		ERRCount;
	uint16_t		WAKECount;
	char			WAKE_CAUSE[32];
	char			ERR_CAUSE[16];
} S_SL_CMD_DATA;

typedef struct _S_MM_CMD_DATA {
	uint32_t	rxCount;
	uint64_t	msgID;
	uint64_t	markedMsgID;
	uint64_t	deletedMsgID;
	uint16_t	appID;
	uint32_t	epoch_sec;
	uint16_t	msgCount;	
	uint16_t	OKCount;	
	uint16_t	ERRCount;	
	char			ERR_CAUSE[16];	
	char			raw_data[MAXFIELD];
	char			msg_data[MAXFIELD];
	char			conv_time[80];
} S_MM_CMD_DATA;

typedef struct _S_MT_CMD_DATA {
	uint32_t		rxCount;
	uint64_t		msgID;	
	uint32_t		epoch_sec;
	uint16_t		msgCount;	
	uint16_t		OKCount;	
	uint16_t		ERRCount;	
	uint16_t		DELETEDCount;	
	enum E_REQ_STATE req_state;
	char				ERR_CAUSE[16];	
	char				raw_data[MAXFIELD];
	char				msg_data[MAXFIELD];
	char				conv_time[80];
} S_MT_CMD_DATA;

typedef struct _S_RD_CMD_DATA {
	uint32_t	rxCount;
	uint16_t	appID;
	int16_t		rssi_sat;
	int16_t		snr;
	int16_t		fdev;
	char			raw_data[MAXFIELD];
	char			msg_data[MAXFIELD];
} S_RD_CMD_DATA;

typedef struct _S_SWARM_DEV_CMD_DATA {
	uint32_t	rxCount;
	uint16_t	BOOTCount;
	uint16_t	DATETIMECount;
	uint16_t	POSITIONCount;
	char			BOOT_CAUSE[16];	
	char			boot_text[MAXFIELD/2];
	char			debug_text[MAXFIELD/2];
	char			error_text[MAXFIELD/2];
} S_SWARM_DEV_CMD_DATA;

typedef struct _S_TD_CMD_DATA {
	uint32_t	rxCount;
	int16_t		rssi_sat;
	int16_t		snr;
	int16_t		fdev;
	uint64_t	msgID;	
	uint16_t	OKCount;
	uint16_t	ERRCount;	
	uint16_t	SENTCount;
	char			ERR_CAUSE[20];	
} S_TD_CMD_DATA;

typedef struct _S_LOG_SENT {
	uint64_t msgID;
	uint8_t datetime[20];	
} S_LOG_SENT;

typedef struct _S_SWARM_METADATA {
	uint32_t 	totalRxCount;
	uint32_t	totalErrCount;
	S_LOG_SENT logSent[20];
	uint8_t logSentIndex;
}S_SWARM_METADATA;

typedef struct _S_SWARM_DEVICE_DATA
{	
	S_RT_CMD_DATA			d_rt;
	S_GN_CMD_DATA			d_gn;
	S_GJ_CMD_DATA			d_gj;
	S_DT_CMD_DATA			d_dt;
	S_FV_CMD_DATA			d_fv;
	S_CS_CMD_DATA			d_cs;
	S_GP_CMD_DATA			d_gp;
	S_GS_CMD_DATA			d_gs;
	S_PO_CMD_DATA			d_po;
	S_PW_CMD_DATA			d_pw;
	S_RS_CMD_DATA			d_rs;
	S_SL_CMD_DATA			d_sl;
	S_MM_CMD_DATA			d_mm;
	S_MT_CMD_DATA			d_mt;	
	S_RD_CMD_DATA			d_rd;
	S_SWARM_DEV_CMD_DATA	d_swarm_device;
	S_TD_CMD_DATA			d_td;
	S_SWARM_METADATA		d_meta;
} S_SWARM_DEVICE_DATA;

/* Module Level Global Variables */
S_SWARM_DEVICE 			*pSwarmDevice = NULL;
S_NMEAParser 				SWARM_nmeaParser;
S_SWARM_DEVICE_DATA 	SWARM_data;

#define NUM_SWARM_NMEA_PROC_CMD	sizeof(swarmNmeaProcCmdTable)/sizeof(swarmNmeaProcCmdTable[0])
				
E_SWARM_ERR templateFunction(uint8_t tbl_index, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
E_SWARM_ERR SWARM_GetCommands(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
E_SWARM_ERR RxMsgs(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
E_SWARM_ERR TxMsgs(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
//E_SWARM_ERR Sleep(uint8_t idx, uint8_t *pResponseBuf, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *), uint32_t *waitEventFlags, uint32_t *multiplicity);
//E_SWARM_ERR TxData(uint8_t idx, uint8_t *pResponseBuf, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *), uint32_t *waitEventFlags, uint32_t *multiplicity);
E_SWARM_ERR GetMetadata(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
E_SWARM_ERR SleepFunction(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
void txTemplateFunction(uint8_t tbl_index, char *arg, char *pSwarmTxBuf);

typedef struct _S_SWARM_DBG_CMD_LIST {
	E_SWARM_CMDS id;
	E_SWARM_MODE mode;
	char name[20];
	E_SWARM_ERR (*fun)(uint8_t fnIdx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
} S_SWARM_DBG_CMD_LIST ;

S_SWARM_DBG_CMD_LIST swarm_dbg_cmd_table[] = {
		{GET_CONFIG,		STARTUP_MODE,	"getconfig", 		templateFunction},
		{DATETIME,			STARTUP_MODE,	"datetime",			templateFunction},
		{VERSION,				STARTUP_MODE, "version", 			templateFunction},
		{GPS_JAM_SPOOF,	RUNTIME_MODE, "gpsjam",				templateFunction},
		{GEO_INFO,			STARTUP_MODE, "geospatial",		templateFunction},
		{GPIO_CTL,			RUNTIME_MODE,	"gpio",					templateFunction},
		{GPS_FIX,				STARTUP_MODE, "gpsfix",				templateFunction},
		{POWER_OFF,			RUNTIME_MODE,	"poweroff",			templateFunction},
		{POWER_STATUS,	RUNTIME_MODE,	"powerstatus",	templateFunction},
		{RX_DATA,				RUNTIME_MODE,	"rxdata",				templateFunction},
		{RESTART,				RUNTIME_MODE,	"restart",			templateFunction},
		{RX_TEST,				STARTUP_MODE,	"rxtest",				templateFunction},
		{SWARM_STATUS,		RUNTIME_MODE,	"status",				templateFunction},
		{TX_DATA,				RUNTIME_MODE,	"send",					templateFunction},	
		{RX_MSGS,				RUNTIME_MODE,	"rxmessages",		RxMsgs},
		{TX_MSGS,				RUNTIME_MODE,	"txmessages",		TxMsgs},
		{SLEEP,					RUNTIME_MODE,	"sleep",				SleepFunction},
		{SWARM_NULL_CMD,	NONE_MODE,		"-------------",NULL},
		{SWARM_HELP,			RUNTIME_MODE,	"?",						SWARM_GetCommands},
		{SWARM_HELP,			RUNTIME_MODE,	"help",					SWARM_GetCommands},
		{SWARM_META,			RUNTIME_MODE,	"summary",			GetMetadata},
};

#define NUM_SWARM_DBG_CMDS sizeof(swarm_dbg_cmd_table)/sizeof(swarm_dbg_cmd_table[0])
	
enum E_SWARM_DIRECTION {
	DIR_NONE,
	TX_RX,
	RX_ONLY,
	TX_ONLY
};

typedef struct _S_SWARM_TX_CMD_T {
	uint32_t eventFlag;
	uint32_t direction;
	char cmd_str[10];
	uint32_t hasParams;
	char def_param[10];
	uint32_t usageMsgIdx;
	uint32_t getMsgIdx;
	void (*cb)(uint8_t*, uint16_t);
} S_SWARM_TX_CMD_T;



S_SWARM_TX_CMD_T swarmTxCmdTable[] = {
 {1 << GET_CONFIG, 		TX_RX, 		CONFIG_SETTING_CMD, 	0, "", 0, 										GET_CONFIG_MSG, 					GetConfigResponse},						//GetConfig
 {1 << DATETIME, 			TX_RX, 		DATETIME_STATUS_CMD, 	1, "@", DATETIME_USAGE_MSG, 	GET_DATETIME_MSG, 				GetDateTimeResponse},					//DateTime
 {1 << VERSION, 			TX_RX, 		FIRMWARE_VER_CMD, 		0, "", 0, 										GET_FIRMWAREVERSION_MSG, 	GetFirmwareVersionResponse},	//Version
 {1 << GPS_JAM_SPOOF, TX_RX, 		GPS_JAM_SPOOF_CMD, 		1, "@", GPSJAM_USAGE_MSG, 		GET_GPSJAM_MSG, 					GetGPSJamResponse},						//GPS_Jam
 {1 << GEO_INFO, 			TX_RX, 		GEO_INFO_CMD, 				1, "@", GEO_USAGE_MSG, 				GET_GEO_MSG, 							GetGeoInfoResponse},					//Geospatial
 {1 << GPIO_CTL, 			TX_RX, 		GPIO_CTL_CMD, 				1, "?", GPIO_USAGE_MSG, 			GET_GPIO_MSG, 						GetGPIOResponse},							//GPIO_CTL
 {1 << GPS_FIX, 			TX_RX, 		GPS_FIX_CMD, 					1, "@", GPSFIX_USAGE_MSG, 		GET_GPSFIX_MSG, 					GetGPSFixResponse},						//GPS_Fix
 {1 << POWER_OFF, 		TX_RX, 		POWEROFF_CMD, 				0, "", 0, 										GET_POWEROFF_MSG, 				GetPowerOffResponse},					//POWER_OFF 
 {1 << POWER_STATUS, 	TX_RX, 		POWERSTATUS_CMD, 			1, "@", PWRSTS_USAGE_MSG, 		GET_POWERSTATUS_MSG, 			GetPowerStatusResponse},			//POWER_STATUS
 {1 << RX_DATA, 			RX_ONLY,	RX_DATA_CMD, 					0, "", 0, 										GET_RXDATA_MSG, 					GetRxDataResponse},						//RX_DATA
 {1 << RESTART, 			TX_RX, 		RESTART_CMD, 					1, "", 0, 										GET_RESTART_MSG, 					GetRestartResponse},					//RESTART
 {1 << RX_TEST, 			TX_RX, 		RX_TEST_CMD, 					1, "@", RXTEST_USAGE_MSG, 		GET_RXTEST_MSG, 					GetRxTestResponse},						//RX_TEST
 {1 << SWARM_STATUS, 	RX_ONLY, 	TILE_STATUS_CMD, 			0, "", 0, 										GET_STATUS_MSG, 					GetStatusResponse},						//TILE
 {1 << TX_DATA, 			TX_RX, 		TX_DATA_CMD, 					1, "", TXDATA_USAGE_MSG, 			GET_TXDATA_MSG, 					GetTxDataResponse},						//RX_TEST
 {1 << SWARM_STATUS, 	RX_ONLY, 	M138_STATUS_CMD, 			0, "", 0, 										GET_STATUS_MSG, 					GetStatusResponse},						//M138
};


void txTemplateFunction(uint8_t tbl_index, char *arg, char *pSwarmTxBuf)
{
	uint8_t len = 0;
	if (pSwarmTxBuf)
	{		
		pSwarmTxBuf[0] = NMEA_DELIM_CHAR;
		strncpy((char *)&pSwarmTxBuf[1], swarmTxCmdTable[tbl_index].cmd_str, strlen((const char *)swarmTxCmdTable[tbl_index].cmd_str));
		
		if (swarmTxCmdTable[tbl_index].hasParams)
		{
			strcat((char *)pSwarmTxBuf, " ");
			if (!strcmp((const char*)arg, ""))
			{
				strncat((char *)pSwarmTxBuf, swarmTxCmdTable[tbl_index].def_param, strlen((const char *)swarmTxCmdTable[tbl_index].def_param));
			}
			else
			{
				strncat((char *)pSwarmTxBuf, arg, strlen((const char *)arg));
			}
		}	
		
		char csBuf[3] = {'\0'};			
		len = strlen((const char*)pSwarmTxBuf);
		DecToHex(csBuf, nmeaChecksum((const char*)pSwarmTxBuf,len));
		sprintf(&pSwarmTxBuf[len], "*%s\n", csBuf);
	}
}
		
E_SWARM_ERR templateFunction(uint8_t tbl_index, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{		
	if (waitEventFlags && multiplicity)
	{
		*waitEventFlags = swarmTxCmdTable[tbl_index].eventFlag;
		*multiplicity = 1;
	}
	
	if (pResponseBuf)
	{
		if (swarmTxCmdTable[tbl_index].hasParams)
		{
			if (args.count == 3 && !strcmp((const char*)args.arg[2], "help"))
			{
				strncpy((char*)pResponseBuf, (const char*)MESSAGES[swarmTxCmdTable[tbl_index].usageMsgIdx], max(strlen((const char*)MESSAGES[swarmTxCmdTable[tbl_index].usageMsgIdx])-1, bufLen));
				*cbFunc = NULL;
				return SWARM_OK;
			}
		}	
	}
		
	if (swarmTxCmdTable[tbl_index].direction == TX_RX)
	{
		txTemplateFunction(tbl_index, (char *)args.arg[2], pSwarmTxBuf);
	}
	
	if (pResponseBuf)
	{
		strncpy((char*)pResponseBuf, (const char*)MESSAGES[swarmTxCmdTable[tbl_index].getMsgIdx], max(strlen((const char*)MESSAGES[swarmTxCmdTable[tbl_index].getMsgIdx])-1, bufLen));
	}
	
	if (cbFunc)
	{
		*cbFunc = swarmTxCmdTable[tbl_index].cb;
	}
	
	return SWARM_OK;
}


	
E_SWARM_ERR SWARM_GetResponse(uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{
	if (!pSwarmDevice || !pSwarmDevice->IsInitialized)
		return SWARM_NOT_OK;
	
	for (uint8_t i=0; i<NUM_SWARM_DBG_CMDS; i++)
	{		
		if (!strncmp((const char*) args.arg[1], swarm_dbg_cmd_table[i].name, strlen((const char*)swarm_dbg_cmd_table[i].name)))
		{
			if (swarm_dbg_cmd_table[i].fun)
			{
				return swarm_dbg_cmd_table[i].fun(swarm_dbg_cmd_table[i].id, pResponseBuf, bufLen, args, pSwarmTxBuf, cbFunc, waitEventFlags, multiplicity);
			}
			return SWARM_NOT_OK;
		}
	}
	return SWARM_NOT_OK;
}
	
E_SWARM_ERR SWARM_GetCommands(uint8_t fnIdx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{			
	char cmd[32] = {'\0'};
	*cbFunc = NULL;	// No callback function required
	
	if (!pResponseBuf)
		return SWARM_NOT_OK;
		
	char *pBuf = (char *)pResponseBuf;		
	sprintf(pBuf, "************************\n" \
								"Swarm Commands:\n" \
								"************************\n" \
	);		
	
	for (uint8_t i=0; i<NUM_SWARM_DBG_CMDS; i++)
	{
		if (swarm_dbg_cmd_table[i].id != SWARM_NULL_CMD)
		{
			sprintf(cmd, "swarm %s", (const char*)(swarm_dbg_cmd_table[i].name));
		}
		else
		{
			sprintf(cmd, "%s", (const char*)(swarm_dbg_cmd_table[i].name));
		}
		strncat(pBuf, (const char*)cmd, strlen(cmd));
		strcat(pBuf, "\n");
	}	
	
	return SWARM_OK;
}

E_SWARM_ERR GetMetadata(uint8_t fnIdx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{			
	*cbFunc = NULL;	// No callback function required
	S_SWARM_DEVICE_DATA *pD = NULL;
	
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
	if (pSwarmDevice)
	{
		pD = pSwarmDevice->pData;
		pD->d_meta.totalRxCount = pD->d_cs.rxCount;
		pD->d_meta.totalRxCount += pD->d_dt.rxCount;
		pD->d_meta.totalRxCount += pD->d_fv.rxCount;
		pD->d_meta.totalRxCount += pD->d_gj.rxCount;
		pD->d_meta.totalRxCount += pD->d_gn.rxCount;
		pD->d_meta.totalRxCount += pD->d_gp.rxCount;
		pD->d_meta.totalRxCount += pD->d_gs.rxCount;
		pD->d_meta.totalRxCount += pD->d_mm.rxCount;
		pD->d_meta.totalRxCount += pD->d_mt.rxCount;
		pD->d_meta.totalRxCount += pD->d_po.rxCount;
		pD->d_meta.totalRxCount += pD->d_pw.rxCount;
		pD->d_meta.totalRxCount += pD->d_rd.rxCount;
		pD->d_meta.totalRxCount += pD->d_rs.rxCount;
		pD->d_meta.totalRxCount += pD->d_rt.rxCount;
		pD->d_meta.totalRxCount += pD->d_sl.rxCount;
		pD->d_meta.totalRxCount += pD->d_td.rxCount;
		pD->d_meta.totalRxCount += pD->d_swarm_device.rxCount;
		
		pD->d_meta.totalErrCount = pD->d_cs.ERRCount;
		pD->d_meta.totalErrCount += pD->d_dt.ERRCount;
		pD->d_meta.totalErrCount += pD->d_fv.ERRCount;
		pD->d_meta.totalErrCount += pD->d_gj.ERRCount;
		pD->d_meta.totalErrCount += pD->d_gn.ERRCount;
		pD->d_meta.totalErrCount += pD->d_gp.ERRCount;
		pD->d_meta.totalErrCount += pD->d_gs.ERRCount;
		pD->d_meta.totalErrCount += pD->d_mm.ERRCount;
		pD->d_meta.totalErrCount += pD->d_mt.ERRCount;
		pD->d_meta.totalErrCount += pD->d_po.ERRCount;
		pD->d_meta.totalErrCount += pD->d_pw.ERRCount;
//		pD->d_meta.totalErrCount += pD->d_rd.ERRCount; -- No error count recorded
		pD->d_meta.totalErrCount += pD->d_rs.ERRCount;
		pD->d_meta.totalErrCount += pD->d_rt.ERRCount;
		pD->d_meta.totalErrCount += pD->d_sl.ERRCount;
		pD->d_meta.totalErrCount += pD->d_td.ERRCount;
//		pD->d_meta.totalErrCount += pD->d_swarm_device.ERRCount; -- No error count recorded

	}
	
	char *pBuf = (char *)pResponseBuf;		
	snprintf(pBuf, bufLen,
	"^^^^^^^^^^^^^^^^^^^^^^^^\n" \
	"   Swarm Summary\n" \
	"^^^^^^^^^^^^^^^^^^^^^^^^\n" \
	"Total Messages Received: %ld\n" \
	"Total Satellite Messages: %ld\n" \
	"Last Command Received: %s\n" \
	"Total Errors: %ld\n" \
	"Device ID: %s\n" \
	"Device Name: %s\n" \
	"Firmware Version: %s\n" \
	"Background RSSI: %d\n" \
	"GPS Lat: %f, Lon: %f, Alt: %f, Crs: %f, Spd: %f\n" \
	"Date/Time: %02d/%02d/%04d %02d:%02d:%02d\n" \
	"\n",
									
	pD->d_meta.totalRxCount,
	pD->d_rt.satRxCount,
	pSwarmDevice->lastCommandRcvd,
	pD->d_meta.totalErrCount,
	pD->d_cs.dev_id,
	pD->d_cs.dev_name,
	pD->d_fv.version_str,
	pD->d_rt.bg_rssi,
	pD->d_gn.latitude, pD->d_gn.longitude, pD->d_gn.altitude, pD->d_gn.course, pD->d_gn.speed,
	pD->d_dt.month, pD->d_dt.day, pD->d_dt.year, 
	pD->d_dt.hour, pD->d_dt.minute, pD->d_dt.seconds
	);		
	
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
	
	return SWARM_OK;
}

E_SWARM_ERR SleepFunction(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{
	uint8_t cmd[30] = {'\0'};
	char *pCmd = (char*)cmd;

	*waitEventFlags = 1 << SLEEP;
		
	if (args.count < 3 || args.count > 4)
	{
		strncpy((char*)pResponseBuf, (const char*)MESSAGES[SLEEP_USAGE_MSG], max(strlen((const char*)MESSAGES[SLEEP_USAGE_MSG])-1, bufLen));		
		*cbFunc = NULL;
		return SWARM_OK;
	}

	*pCmd = NMEA_DELIM_CHAR; pCmd += 2;
	strcpy(pCmd, SLEEP_MODE_CMD); pCmd += strlen((const char *)SLEEP_MODE_CMD);
	
	if (args.count == 3)	// Could be seconds or time 
	{
		if (strchr((const char*)args.arg[2], ':'))	// arg contains colon, use time
		{			
			strcat(pCmd, " U="); pCmd += strlen(" U=");
		}
		else	// No colon, use seconds
		{
			strcat(pCmd, " S="); pCmd += strlen(" S=");
		}
		strcat(pCmd, (char*)args.arg[2]);
		pCmd += strlen((const char*)args.arg[2]);
	}
	else // Assume this is date and time
	{
		strcat(pCmd, " U="); pCmd += strlen(" U=");
		strcat(pCmd, (char*)args.arg[2]); pCmd += strlen((const char*)args.arg[2]);
		strcat(pCmd, " "); pCmd += 1;
		strcat(pCmd, (char*)args.arg[3]); pCmd += strlen((const char*)args.arg[3]);
	}
			
	char csBuf[3] = {'\0'};	
	DecToHex(csBuf, nmeaChecksum((const char*)cmd,strlen((const char*)cmd)));	
	sprintf(pSwarmTxBuf, "%s*%s\n", cmd,csBuf);
	strncpy((char*)pResponseBuf, (const char*)MESSAGES[GET_SLEEP_MSG], max(strlen((const char*)MESSAGES[GET_SLEEP_MSG])-1,bufLen));		
	
	*cbFunc = GetSleepResponse;
	
	return SWARM_OK;
}

E_SWARM_ERR RxMsgs(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{
	*waitEventFlags = 1 << RX_MSGS;
	uint8_t msgArg[40] = {'\0'};			
	
	*multiplicity = 1;
		
	if (args.count < 3)
	{
		strncpy((char*)pResponseBuf, (const char*)MESSAGES[RXMSGS_USAGE_MSG], max(strlen((const char*)MESSAGES[RXMSGS_USAGE_MSG])-1, bufLen));		
		*cbFunc = NULL;
		return SWARM_OK;
	}

	if (!strcmp((const char*)args.arg[2], "count"))
	{
		sprintf((char*)msgArg, "C=");
//		if (!strcmp((const char*)args.arg[3], "unread"))
//			strcat((char*)msgArg, "U");
//		else if (!strcmp((const char*)args.arg[3], "all"))
			strcat((char*)msgArg, "*");
//		else 
//			return SWARM_NOT_OK;
	}
	else if (!strcmp((const char*)args.arg[2], "delete"))
	{
		sprintf((char*)msgArg, "D=");
//		if (!strcmp((const char*)args.arg[3], "read"))
//			strcat((char*)msgArg, "R");
//		else if (!strcmp((const char*)args.arg[3], "all"))
//			strcat((char*)msgArg, "*");
//		else 
			strcat((char*)msgArg, (const char*)args.arg[3]);
	}
	else if (!strcmp((const char*)args.arg[2], "read"))
	{
		sprintf((char*)msgArg, "R=");
		if (!strcmp((const char*)args.arg[3], "oldest"))
			strcat((char*)msgArg, "O");
		else if (!strcmp((const char*)args.arg[3], "newest"))
			strcat((char*)msgArg, "N");
		else 
			strcat((char*)msgArg, (const char*)args.arg[3]);
	}
	else
		return SWARM_NOT_OK;

	uint8_t cmd[30] = {'\0'};
	char *pCmd = (char*)cmd;
	*pCmd = NMEA_DELIM_CHAR; pCmd += 2;
	strcpy(pCmd, MSGS_RX_CMD); pCmd += strlen((const char *)MSGS_RX_CMD);
	
	strcpy(pCmd, " "); pCmd++;
	strcpy(pCmd, (char*)msgArg);
			
	char csBuf[3] = {'\0'};	
	DecToHex(csBuf, nmeaChecksum((const char*)cmd,strlen((const char*)cmd)));	
	sprintf(pSwarmTxBuf, "%s*%s\n", cmd,csBuf);
	strncpy((char*)pResponseBuf, (const char*)MESSAGES[GET_RXMSGS_MSG], max(strlen((const char*)MESSAGES[GET_TXMSGS_MSG])-1, bufLen));		
	
	*cbFunc = GetRxMessagesResponse;
	
	return SWARM_OK;
}

E_SWARM_ERR TxMsgs(uint8_t idx, uint8_t *pResponseBuf, uint16_t bufLen, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity)
{
	*waitEventFlags = 1 << TX_MSGS;
	uint8_t msgArg[40] = {'\0'};			
	
	*multiplicity = 1;
		
	if (args.count < 3)
	{
		strncpy((char*)pResponseBuf, (const char*)MESSAGES[TXMSGS_USAGE_MSG], max(strlen((const char*)MESSAGES[TXMSGS_USAGE_MSG])-1, bufLen));		
		*cbFunc = NULL;
		return SWARM_OK;
	}
	
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
		
	// Default to count request type
	pSwarmDevice->pData->d_mt.req_state = COUNT_REQ;

	if (!strcmp((const char*)args.arg[2], "count"))	
		sprintf((char*)msgArg, "C=U");
	else if (!strcmp((const char*)args.arg[2], "list"))
	{
		sprintf((char*)msgArg, "L=U");
		*multiplicity = max(1, pSwarmDevice->pData->d_mt.msgCount);
	}
	else if (!strcmp((const char*)args.arg[2], "delete"))
	{
		sprintf((char*)msgArg, "D=");			
		
		if (!strcmp((const char*)args.arg[3], "all"))
		{
			strcat((char*)msgArg, "U");
			pSwarmDevice->pData->d_mt.req_state = DELETE_REQ;
		}
		else
		{
			strcat((char*)msgArg, (const char*)args.arg[3]);
		}
	}
	else if (!strcmp((const char*)args.arg[2], "sent"))
	{
		char buf[80] = "\0";
		sprintf((char*)pResponseBuf, "%d msgs sent, %d pending\n", pSwarmDevice->pData->d_td.SENTCount, pSwarmDevice->pData->d_mt.msgCount);
		
		if (pSwarmDevice->pData->d_td.SENTCount > 0)
		{
			for (uint8_t i = 0; i < pSwarmDevice->pData->d_meta.logSentIndex; i++)
			{
				sprintf(buf, "%llu: %s\n", pSwarmDevice->pData->d_meta.logSent[i].msgID, pSwarmDevice->pData->d_meta.logSent[i].datetime);
				strcat((char*)pResponseBuf, buf);
			}
		}
		*cbFunc = NULL;
		return SWARM_OK;
	}
	else
		return SWARM_NOT_OK;

	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
	
	uint8_t cmd[30] = {'\0'};
	char *pCmd = (char*)cmd;
	*pCmd = NMEA_DELIM_CHAR; pCmd += 2;
	strcpy(pCmd, MSGS_TX_CMD); pCmd += strlen((const char *)MSGS_TX_CMD);
	
	strcpy(pCmd, " "); pCmd ++;
	strcpy(pCmd, (char*)msgArg);
			
	char csBuf[3] = {'\0'};	
	DecToHex(csBuf, nmeaChecksum((const char*)cmd,strlen((const char*)cmd)));	
	sprintf(pSwarmTxBuf, "%s*%s\n", cmd,csBuf);
	strncpy((char*)pResponseBuf, (const char*)MESSAGES[GET_TXMSGS_MSG], max(strlen((const char*)MESSAGES[GET_TXMSGS_MSG])-1, bufLen));		
	
	*cbFunc = GetTxMessagesResponse;
	
	return SWARM_OK;
}

//E_SWARM_ERR RxData(uint8_t idx, uint8_t *pResponseBuf, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *), uint32_t *waitEventFlags, uint32_t *multiplicity)
//{
//	E_SWARM_ERR retVal = SWARM_OK;
//	
//	sprintf(pSwarmTxBuf, "RxData\n");
//	sprintf((char*)pResponseBuf, "Receive Data...\n");
//	
//	return retVal;
//}

//E_SWARM_ERR TxData(uint8_t idx, uint8_t *pResponseBuf, ARG_LIST args, char *pSwarmTxBuf, void (**cbFunc)(uint8_t *), uint32_t *waitEventFlags, uint32_t *multiplicity)
//{
//	E_SWARM_ERR retVal = SWARM_OK;
//	
//	sprintf(pSwarmTxBuf, "TxData\n");
//	sprintf((char*)pResponseBuf, "Transmitting data...\n");
//	
//	return retVal;
//}
	
//$RT - Receive Test
//Set or query the rate for $RT unsolicited report messages for device power state. Also can
//retrieve the most current $RT message.

/***************************************************************************

	NMEA Sentence Processing Functions

****************************************************************************/

void ProcessRT(void *pDevice)
{
	uint8_t pField[MAXFIELD];
		
	// Cast to internal device type
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
		
	// Get the mutex 
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_rt.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_rt.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{
		pSwarm->pData->d_rt.ERRCount++;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, leaving background RSSI or rate
		if (pSwarm->pNmeaParser->m_Data[0] == 'R')
		{
			if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
			{				
				// Background RSSI (dBm)
				pSwarm->pData->d_rt.bg_rssi = atoi((const char*)pField+strlen("RSSI="));
			}
		}
		else
		{
			// Rate of request (s)
			pSwarm->pData->d_rt.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
		}
	}
	else 
	{		
		// Process most recent $RT message from satellite					
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Satellite RSSI
			pSwarm->pData->d_rt.rssi_sat = atoi((const char*)pField+strlen("RSSI="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// SNR (dB)
			pSwarm->pData->d_rt.snr = atoi((const char*)pField+strlen("SNR="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// Frequency deviation (Hz)
			pSwarm->pData->d_rt.fdev = atoi((const char*)pField+strlen("FDEV="));
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{
			// Compare last time with new time, if different then we received a message
			if((pSwarm->pData->d_rt.utc_time[0] != 0) && (strcmp(pSwarm->pData->d_rt.utc_time, (const char*)pField+strlen("TS="))))
			{
				// Count messages from satellite separately
				pSwarm->pData->d_rt.satRxCount++;
//				tx_event_flags_set(&LEDEventFlagsGroup,1 << LED_SAT_MSG_RX,TX_OR);
			}
			
			// Date/Time (UTC)
			strncpy(pSwarm->pData->d_rt.utc_time, (const char*)pField+strlen("TS="), sizeof(pSwarm->pData->d_rt.utc_time));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
		{
			// Device ID of satellite (hex)
			strncpy(pSwarm->pData->d_rt.sat_id, (const char*)pField+strlen("DI="), sizeof(pSwarm->pData->d_rt.sat_id));
		}
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	if (pSwarm->pData->d_rt.bg_rssi <= (int16_t)RSSI_THRESHOLD) // More negative is better
	{
	#ifdef AZURE_RTOS_THREADX
//		tx_event_flags_set(&LEDEventFlagsGroup,1 << LED_RSSI_GOOD,TX_OR);
	#else	
		osThreadFlagsSet(ledTaskHandle, 1 << SWARM_BG_RSSI_GOOD);
	#endif
	}
	else
	{		
	#ifdef AZURE_RTOS_THREADX
//		tx_event_flags_set(&LEDEventFlagsGroup,1 << LED_RSSI_BAD,TX_OR);
	#else	
		osThreadFlagsSet(ledTaskHandle, 1 << SWARM_BG_RSSI_BAD);
	#endif		
	}
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << RX_TEST, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << RX_TEST);
	#endif
}

//$GN - Geospatial information
//Set or query the rate for $GN unsolicited report messages for date and time. Also can retrieve
//the most current $GN message.

void ProcessGN(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex 
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_gn.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_gn.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_gn.ERRCount++;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be rate		
		// Rate of request (s)
		pSwarm->pData->d_gn.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{
		// Process most recent $GN message from satellite					
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Latitude
			pSwarm->pData->d_gn.latitude = atof((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Longitude
			pSwarm->pData->d_gn.longitude = atof((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// Altitude
			pSwarm->pData->d_gn.altitude = atof((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{
			// Course
			pSwarm->pData->d_gn.course = atof((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
		{
			// Speed
			pSwarm->pData->d_gn.speed = atof((const char*)pField);
		}
	}
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX		
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << GEO_INFO, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << GEO_INFO);
	#endif
}


//$GJ - GPS Jamming/Spoofing Indication
//Set or query the rate for $GJ unsolicited report messages for jamming and spoofing indicators.
//Also can retrieve the most current $GJ message.

void ProcessGJ(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_gj.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_gj.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_gj.ERRCount++;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be rate		
		// Rate of request (s)
		pSwarm->pData->d_gj.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{
		// Process most recent $GN message from satellite					
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Spoofing State
			pSwarm->pData->d_gj.spoof_state = atoi((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Jamming Level
			pSwarm->pData->d_gj.jamming_level = atoi((const char*)pField);
		}		
	}
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << GPS_JAM_SPOOF, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << GPS_JAM_SPOOF);
	#endif
}


//$DT - Date/Time
//Set or query the rate for $DT unsolicited report messages for date and time. Also can retrieve
//the most current $DT message. See unsolicited message description for $DT message format.

void ProcessDT(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	char pBuff[10];

	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_dt.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_dt.OKCount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERR,", sizeof("ERR")))
	{		
		memset(pSwarm->pData->d_dt.ERR_CAUSE, 0, sizeof(pSwarm->pData->d_dt.ERR_CAUSE));
		pSwarm->pData->d_dt.ERRCount++;
		strncpy(pSwarm->pData->d_dt.ERR_CAUSE, (const char*)pSwarm->pNmeaParser->m_Data+sizeof("ERR"), strlen((const char*)pSwarm->pNmeaParser->m_Data)-sizeof("ERR"));
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be rate		
		// Rate of request (s)
		pSwarm->pData->d_dt.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{
		// Process most recent $DT message from satellite					
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Year
			pBuff[0] = pField[0];
			pBuff[1] = pField[1];
			pBuff[2] = pField[2];			
			pBuff[3] = pField[3];
			pBuff[4] = '\0';
			pSwarm->pData->d_dt.year = atoi((const char*)pBuff);
			
			// Month
			pBuff[0] = pField[4];
			pBuff[1] = pField[5];
			pBuff[2] = '\0';
			pSwarm->pData->d_dt.month = atoi((const char*)pBuff);
			
			// Day
			pBuff[0] = pField[6];
			pBuff[1] = pField[7];
			pBuff[2] = '\0';
			pSwarm->pData->d_dt.day = atoi((const char*)pBuff);
						
			// Hour
			pBuff[0] = pField[8];
			pBuff[1] = pField[9];
			pBuff[2] = '\0';
			pSwarm->pData->d_dt.hour = atoi((const char*)pBuff);
						
			// Minutes
			pBuff[0] = pField[10];
			pBuff[1] = pField[11];
			pBuff[2] = '\0';
			pSwarm->pData->d_dt.minute = atoi((const char*)pBuff);
			
			// Seconds
			pBuff[0] = pField[12];
			pBuff[1] = pField[13];
			pBuff[2] = '\0';
			pSwarm->pData->d_dt.seconds = atoi((const char*)pBuff);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Flag
			pSwarm->pData->d_dt.flag = *(char*)pField;
		}		
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << DATETIME, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << DATETIME);
	#endif
}


//$FV - Firmware Version
//Returns the current device firmware version. 

void ProcessFV(void *pDevice)
{
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_fv.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_fv.ERRCount++;
	}
	else 
	{
		strncpy((char*)pSwarm->pData->d_fv.version_str, (const char*)pSwarm->pNmeaParser->m_Data, strlen((const char*)pSwarm->pNmeaParser->m_Data));
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << VERSION, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << VERSION);
	#endif
}

//$CS - Configuration Settings
//Retrieve and display the configuration settings for the Swarm device ID. These settings are
//determined by Swarm for identifying and communicating with each individual device. Since
//there are no variable parameters, the correct checksum has been added. 

void ProcessCS(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_cs.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_cs.ERRCount++;
	}
	else 
	{			
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Device ID
			strncpy(pSwarm->pData->d_cs.dev_id, (const char*)pField+strlen("DI="), sizeof(pSwarm->pData->d_cs.dev_id));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Device Name (dB)
			strncpy(pSwarm->pData->d_cs.dev_name, (const char*)pField+strlen("DN="), sizeof(pSwarm->pData->d_cs.dev_name));
		}		
	}	
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << GET_CONFIG, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << GET_CONFIG);
	#endif
}


//$GP - GPIO1 Control
//This command allows control of the GPIO1 pin to allow indications or control the operation of
//the Tile.

void ProcessGP(void *pDevice)
{
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_gp.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_gp.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_gp.ERRCount++;
	}
	else 
	{
		pSwarm->pData->d_gp.mode = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	
	// Release the mutex	
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << GPIO_CTL, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << GPIO_CTL);
	#endif
}


//$GS - GPS Fix Quality
//Set or query the rate for $GS unsolicited report messages for date and time. Also can retrieve
//the most current $GS message.

void ProcessGS(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	char pBuff[3];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_gs.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_gs.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_gs.ERRCount++;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be rate		
		// Rate of request (s)
		pSwarm->pData->d_gs.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{			
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Horizontal dilution of precision (0..9999) (integer = actual hdop * 100)
			pSwarm->pData->d_gs.hdop = atoi((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Vertical dilution of precision (0..9999) (integer = actual vdop * 100)
			pSwarm->pData->d_gs.vdop = atoi((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// Number of GNSS satellites used in solution (integer)
			pSwarm->pData->d_gs.gnss_sats = atoi((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{
			// Always reads as 0, unused
			pSwarm->pData->d_gs.unused = atoi((const char*)pField);
		}	
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
		{
			pBuff[0] = pField[0];
			pBuff[1] = pField[1];
			pBuff[2] = '\0';
					
			if (!strncmp(pBuff, "NF", sizeof("NF")))
				pSwarm->pData->d_gs.fixType = NF;
			else if (!strncmp(pBuff, "DR", sizeof("DR")))
				pSwarm->pData->d_gs.fixType = DR;
			else if (!strncmp(pBuff, "G2", sizeof("G2")))
				pSwarm->pData->d_gs.fixType = G2;
			else if (!strncmp(pBuff, "G3", sizeof("G3")))
				pSwarm->pData->d_gs.fixType = G3;
			else if (!strncmp(pBuff, "D2", sizeof("D2")))
				pSwarm->pData->d_gs.fixType = D2;
			else if (!strncmp(pBuff, "D3", sizeof("D3")))
				pSwarm->pData->d_gs.fixType = D3;
			else if (!strncmp(pBuff, "RK", sizeof("RK")))
				pSwarm->pData->d_gs.fixType = RK;
			else if (!strncmp(pBuff, "TT", sizeof("TT")))
				pSwarm->pData->d_gs.fixType = TT;
		}		
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << GPS_FIX, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << GPS_FIX);
	#endif
}

//$PO - Power Off
//Power off the device. If fully supported, after issuing $PO*1f the customer should command any
//Tile power supplies to disconnect. If power is not disconnected, the Tile enters a low power
//mode until power is completely removed and restored. Since there are no variable parameters,
//the correct checksum has been added.

void ProcessPO(void *pDevice)
{
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_po.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_po.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_po.ERRCount++;
	}
	
	// Release the mutex	
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << POWER_OFF, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << POWER_OFF);
	#endif
}

//$PW - Power Status
//Set or query the rate for $PW unsolicited report messages for device power state. Also can
//retrieve the most current $PW message.

void ProcessPW(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);

	pSwarm->pData->d_pw.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_pw.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_pw.ERRCount++;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be rate		
		// Rate of request (s)
		pSwarm->pData->d_pw.Rate = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{			
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// Unused 1
			pSwarm->pData->d_pw.unused1 = atof((const char*)pField);
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// Unused 2
			pSwarm->pData->d_pw.unused2 = atof((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// Unused 3
			pSwarm->pData->d_pw.unused3 = atof((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{
			// Unused 4
			pSwarm->pData->d_pw.unused4 = atof((const char*)pField);
		}	
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
		{
			// Temp
			pSwarm->pData->d_pw.temp = atof((const char*)pField);
		}		
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << POWER_STATUS, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << POWER_STATUS);
	#endif
}


//$RS - Restart Device
//Perform a software cold restart of the device. Since there are no variable parameters, the
//correct checksum has been added.

void ProcessRS(void *pDevice)
{
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_rs.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_rs.OKCount++;
	}
	else if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "ERR"))
	{		
		pSwarm->pData->d_rs.ERRCount++;
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << RESTART, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << RESTART);
	#endif
}

//$SL - Sleep mode
//This command puts the device into a low-power sleep mode.

void ProcessSL(void *pDevice)
{
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_sl.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_sl.OKCount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "WAKE,",sizeof("WAKE")))
	{
		memset(pSwarm->pData->d_sl.WAKE_CAUSE, 0, sizeof(pSwarm->pData->d_sl.WAKE_CAUSE));
		pSwarm->pData->d_sl.WAKECount++;
		strncpy(pSwarm->pData->d_sl.WAKE_CAUSE, (const char*)pSwarm->pNmeaParser->m_Data+sizeof("WAKE"), strlen((const char*)pSwarm->pNmeaParser->m_Data)-sizeof("WAKE"));
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERR,", sizeof("ERR")))
	{		
		memset(pSwarm->pData->d_sl.ERR_CAUSE, 0, sizeof(pSwarm->pData->d_sl.ERR_CAUSE));
		pSwarm->pData->d_sl.ERRCount++;
		strncpy(pSwarm->pData->d_sl.ERR_CAUSE, (const char*)pSwarm->pNmeaParser->m_Data+sizeof("ERR"), strlen((const char*)pSwarm->pNmeaParser->m_Data)-sizeof("ERR"));
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << SLEEP, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << SLEEP);
	#endif
}


//$MM - Messages Received Management
//Manage received messages in the device database. Has ability to return count, mark and delete
//messages. This command is only applicable to Tiles on firmware version 1.0.0+.

void ProcessMM(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_mm.rxCount++;
	
	if (!strcmp((char *)pSwarm->pNmeaParser->m_Data, "OK"))
	{
		pSwarm->pData->d_mm.OKCount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERR,", sizeof("ERR")))
	{				
		memset(pSwarm->pData->d_mm.ERR_CAUSE, 0, sizeof(pSwarm->pData->d_mm.ERR_CAUSE));
		pSwarm->pData->d_mm.ERRCount++;
		strncpy(pSwarm->pData->d_mm.ERR_CAUSE, (const char*)pSwarm->pNmeaParser->m_Data+sizeof("ERR"), strlen((const char*)pSwarm->pNmeaParser->m_Data)-sizeof("ERR"));
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "MARKED,", sizeof("MARKED")))
	{
		pSwarm->pData->d_mm.markedMsgID = atoll((const char*)pSwarm->pNmeaParser->m_Data+sizeof("MARKED"));
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "DELETED,", sizeof("DELETED")))
	{				
		pSwarm->pData->d_mm.deletedMsgID = atoll((const char*)pSwarm->pNmeaParser->m_Data+sizeof("DELETED"));	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be count				
		pSwarm->pData->d_mm.msgCount = atoi((const char*)pSwarm->pNmeaParser->m_Data);
	}
	else 
	{			
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{				
			// appID
			pSwarm->pData->d_mm.appID = atoi((const char*)pField+strlen("AI="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{	
			// data
			strncpy((char*)pSwarm->pData->d_mm.raw_data, (const char*)pField, MAXFIELD);
			HexToASCII((char*)pSwarm->pData->d_mm.msg_data, (const char*)pSwarm->pData->d_mm.raw_data, strlen((const char*)pField));
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// msg_id
			pSwarm->pData->d_mm.msgID = atoll((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{
			// epoch seconds
			pSwarm->pData->d_mm.epoch_sec = atol((const char*)pField);
			EpochToHumanReadable(pSwarm->pData->d_mm.conv_time, sizeof(pSwarm->pData->d_mm.conv_time), pSwarm->pData->d_mm.epoch_sec);
		}	
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << RX_MSGS, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << RX_MSGS);
	#endif
}


//$MT - Messages to Transmit Management
//Manage messages to be transmitted in the device database. Has ability to return count, list, and
//delete unsent messages.

void ProcessMT(void *pDevice)
{
//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_mt.rxCount++;
	
	if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "OK", sizeof("OK")-1))
	{
		pSwarm->pData->d_mt.OKCount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERR", sizeof("ERR")-1))
	{								
		memset(pSwarm->pData->d_mt.ERR_CAUSE, 0, sizeof(pSwarm->pData->d_mt.ERR_CAUSE));
		pSwarm->pData->d_mt.ERRCount++;
		strncpy(pSwarm->pData->d_mt.ERR_CAUSE, (const char*)pSwarm->pNmeaParser->m_Data+sizeof("ERR"), strlen((const char*)pSwarm->pNmeaParser->m_Data)-sizeof("ERR"));
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "DELETED", sizeof("DELETED")-1))
	{				
		pSwarm->pData->d_mt.DELETEDCount++;
		if (pSwarm->pData->d_mt.msgCount > 0)
			pSwarm->pData->d_mt.msgCount--;
	}
	else if (!strchr((const char*)pSwarm->pNmeaParser->m_Data, ','))
	{
		// No commas detected, must be count				
		if (pSwarm->pData->d_mt.req_state == COUNT_REQ)
		{
			pSwarm->pData->d_mt.msgCount = atoi((const char*)pSwarm->pNmeaParser->m_Data);
		}
		else if (pSwarmDevice->pData->d_mt.req_state == DELETE_REQ)
		{
			pSwarm->pData->d_mt.msgCount = 0;
			pSwarm->pData->d_mt.DELETEDCount = atoi((const char*)pSwarm->pNmeaParser->m_Data);
		}
		
		if (pSwarm->pData->d_mt.msgCount > 0)
		{
//			tx_event_flags_set(&LEDEventFlagsGroup, 1 << LED_MSG_PEND, TX_OR);
		}
		else
		{
//			tx_event_flags_set(&LEDEventFlagsGroup, 1 << LED_MSG_PEND_CLR, TX_OR);
		}		
	}
	else 
	{			
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
		{	
			// data			
			memset(pSwarm->pData->d_mt.raw_data, '\0', MAXFIELD);
			memset(pSwarm->pData->d_mt.msg_data, '\0', MAXFIELD);
			strncpy((char*)pSwarm->pData->d_mt.raw_data, (const char*)pField, MAXFIELD);
			HexToASCII((char*)pSwarm->pData->d_mt.msg_data, (const char*)pSwarm->pData->d_mt.raw_data, strlen((const char*)pField));
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{
			// msg_id
			pSwarm->pData->d_mt.msgID = atoll((const char*)pField);
		}		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{
			// epoch seconds
			pSwarm->pData->d_mt.epoch_sec = atol((const char*)pField);
			EpochToHumanReadable(pSwarm->pData->d_mt.conv_time, sizeof(pSwarm->pData->d_mt.conv_time), pSwarm->pData->d_mt.epoch_sec);
		}	
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
//	HAL_GPIO_TogglePin(GPIOG, TICK_TOGGLE_Pin);
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << TX_MSGS, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << TX_MSGS);
	#endif
}

//$RD - Receive Data Message
//This unsolicited message provides an ASCII-encoded hexadecimal string with the user data
//received from the Swarm network. Some fields also include signal quality information for the
//received message. Received data unsolicited messages can be enabled/disabled using the
//$MM command with the message notification option.

void ProcessRD(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_rd.rxCount++;
			
	if (GetField(pSwarm->pNmeaParser->m_Data, pField, 0, MAXFIELD))
	{					
		// appID
		pSwarm->pData->d_rd.appID = atoi((const char*)pField+strlen("AI="));
	}		
	if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
	{
		// RSSI
		pSwarm->pData->d_rd.rssi_sat = atoi((const char*)pField+strlen("RSSI="));
	}		
	if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
	{
		// SNR
		pSwarm->pData->d_rd.snr = atoi((const char*)pField+strlen("SNR="));
	}		
	if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
	{
		// FDEV
		pSwarm->pData->d_rd.fdev = atoi((const char*)pField+strlen("FDEV="));
	}	
	if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
	{
		// data
		strncpy((char*)pSwarm->pData->d_rd.raw_data, (const char*)pField, MAXFIELD);
		HexToASCII((char*)pSwarm->pData->d_rd.msg_data, (const char*)pSwarm->pData->d_rd.raw_data, strlen((const char*)pField));
	}	
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << RX_DATA, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << RX_DATA);
	#endif
}

//$TILE - Tile Status
//These unsolicited status messages indicate the readiness of the Tile for normal operation. This
//includes the conditions at power up, GPS acquisition, and certain error conditions. Tile status
//messages cannot be disabled.

void ProcessSWARM_DEVICE(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_swarm_device.rxCount++;
			
	if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "BOOT,", sizeof("BOOT")))
	{
		pSwarm->pData->d_swarm_device.BOOTCount++;
		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{					
			// BOOT Cause
			strncpy((char*)pSwarm->pData->d_swarm_device.BOOT_CAUSE, (const char*)pField, MAXFIELD);
			
			if (!strncmp((const char*)pField, "POWERON", sizeof("POWERON")) ||
				  !strncmp((const char*)pField, "VERSION", sizeof("VERSION")))
			{
				if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
				{	
					strncpy((char*)pSwarm->pData->d_swarm_device.boot_text,(const char*)pField, MAXFIELD);
				}
			}
		}
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "DATETIME", sizeof("DATETIME")))
	{
		pSwarm->pData->d_swarm_device.DATETIMECount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "POSITION", sizeof("POSITION")))
	{
		pSwarm->pData->d_swarm_device.POSITIONCount++;
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "DEBUG,", sizeof("DEBUG")))
	{
		memset(pSwarm->pData->d_swarm_device.debug_text, 0, sizeof(pSwarm->pData->d_swarm_device.debug_text));
		strncpy((char*)pSwarm->pData->d_swarm_device.debug_text, (const char*)pSwarm->pNmeaParser->m_Data+strlen("DEBUG"), MAXFIELD);
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERROR,", sizeof("ERROR")))
	{		
		memset(pSwarm->pData->d_swarm_device.error_text, 0, sizeof(pSwarm->pData->d_swarm_device.error_text));
		strncpy((char*)pSwarm->pData->d_swarm_device.debug_text, (const char*)pSwarm->pNmeaParser->m_Data+strlen("ERROR"), MAXFIELD);
	}
	
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << SWARM_STATUS, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << SWARM_STATUS);
	#endif
}


//$TD - Transmit data
//This command transmits data to the Swarm network.

void ProcessTD(void *pDevice)
{
	uint8_t pField[MAXFIELD];
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pDevice;
	
	// Get the mutex
	tx_mutex_get(pSwarm->pMutex, TX_WAIT_FOREVER);
	
	pSwarm->pData->d_td.rxCount++;
			
	if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "OK", sizeof("OK")-1))
	{
		pSwarm->pData->d_td.OKCount++;
		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{					
			// MSG ID
			pSwarm->pData->d_td.msgID = atoll((const char*)pField);
//			tx_event_flags_set(&LEDEventFlagsGroup, 1 << LED_MSG_PEND, TX_OR);
		}
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "SENT", sizeof("SENT")-1))
	{
		pSwarm->pData->d_td.SENTCount++;
		
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{					
			// RSSI
			pSwarm->pData->d_td.rssi_sat = atoi((const char*)pField+strlen("RSSI="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
		{					
			// SNR
			pSwarm->pData->d_td.snr = atoi((const char*)pField+strlen("SNR="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 3, MAXFIELD))
		{					
			// FDEV
			pSwarm->pData->d_td.fdev = atoi((const char*)pField+strlen("FDEV="));
		}
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 4, MAXFIELD))
		{					
			uint8_t index = pSwarm->pData->d_meta.logSentIndex++;
			// MSG ID
			pSwarm->pData->d_td.msgID = atoll((const char*)pField);
				pSwarm->pData->d_meta.logSent[index].msgID = pSwarm->pData->d_td.msgID;
			sprintf((char*)pSwarm->pData->d_meta.logSent[index].datetime, "%04d-%02d-%02d %02d:%02d:%02d",
				pSwarm->pData->d_dt.year,pSwarm->pData->d_dt.month, pSwarm->pData->d_dt.day, pSwarm->pData->d_dt.hour, pSwarm->pData->d_dt.minute, pSwarm->pData->d_dt.seconds);
			if (pSwarm->pData->d_meta.logSentIndex > 20)
				pSwarm->pData->d_meta.logSentIndex = 0;
		}		
		// On sent clear the LED
	#ifdef AZURE_RTOS_THREADX
//		tx_event_flags_set(&LEDEventFlagsGroup, 1 << LED_MSG_PEND_CLR, TX_OR);
	#else	
		osThreadFlagsSet(ledTaskHandle, 1 << SWARM_MSG_SENT);
	#endif
	}
	else if (!strncmp((char *)pSwarm->pNmeaParser->m_Data, "ERR,", sizeof("ERR")))
	{
		pSwarm->pData->d_td.ERRCount++;
		if (GetField(pSwarm->pNmeaParser->m_Data, pField, 1, MAXFIELD))
		{					
			strncpy((char*)pSwarm->pData->d_td.ERR_CAUSE, (const char*)pField, MAXFIELD);
			if (!strncmp((const char*)pField, "HOLDTIMEEXPIRED",strlen("HOLDTIMEEXPIRED")))
			{
				if (GetField(pSwarm->pNmeaParser->m_Data, pField, 2, MAXFIELD))
				{
					// MSG ID
					pSwarm->pData->d_td.msgID = atoll((const char*)pField);
				}
			}			
		}
		
	#ifdef AZURE_RTOS_THREADX
	#else	
		osThreadFlagsSet(ledTaskHandle, 1 << SWARM_ERR_COND);
	#endif
	}
		
	// Release the mutex
	tx_mutex_put(pSwarm->pMutex);
	
	#ifdef AZURE_RTOS_THREADX
		tx_event_flags_set(&SwarmDataReadyFlagsGroup, 1 << TX_DATA, TX_OR);
	#else	
		osEventFlagsSet(SwarmDataReadyFlagsHandle, 1 << TX_DATA);
	#endif
}

void GetConfigResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
		snprintf((char*)pResponseBuf, bufLen,
																	
		"Device ID: %s\n" \
		"Device Name: %s\n" \
		"-----------------\n" \
		"Rx Count: %ld\n" \
		"Error Count: %d\n" \
		"\n",
		pSwarmDevice->pData->d_cs.dev_id,
		pSwarmDevice->pData->d_cs.dev_name,
		pSwarmDevice->pData->d_cs.rxCount,
		pSwarmDevice->pData->d_cs.ERRCount
	);
		
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetDateTimeResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{		
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
	if (pSwarmDevice->pData->d_dt.flag == 0)
		pSwarmDevice->pData->d_dt.flag = ' ';
	
	snprintf((char*)pResponseBuf, bufLen, 	
		"Date/Time: %04d-%02d-%02d %02d:%02d:%02d\n" \
		"Flag: %c\n" \
		"-----------------------------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"Error Cause: %s\n" \
		"Rate: %ld\n" \
		"\n", 
		pSwarmDevice->pData->d_dt.year,
		pSwarmDevice->pData->d_dt.month,
		pSwarmDevice->pData->d_dt.day,
		pSwarmDevice->pData->d_dt.hour,
		pSwarmDevice->pData->d_dt.minute,
		pSwarmDevice->pData->d_dt.seconds,
		pSwarmDevice->pData->d_dt.flag,
		pSwarmDevice->pData->d_dt.rxCount,
		pSwarmDevice->pData->d_dt.OKCount,
		pSwarmDevice->pData->d_dt.ERRCount,
		pSwarmDevice->pData->d_dt.ERR_CAUSE,
		pSwarmDevice->pData->d_dt.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetFirmwareVersionResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
		snprintf((char*)pResponseBuf, bufLen, 				
		"Firmware Version: %s\n" \
		"---------------------\n" \
		"Rx Count: %ld\n" \
		"Error Count: %d\n" \
		"\n",																
		pSwarmDevice->pData->d_fv.version_str,
		pSwarmDevice->pData->d_fv.rxCount,
		pSwarmDevice->pData->d_fv.ERRCount
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetGPSJamResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	char spoof_state[30] = {'\0'};
	uint8_t msg;
	
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
	switch (pSwarmDevice->pData->d_gj.spoof_state)
	{
		case SPOOF_UNKNOWN:
			msg = SPOOF_UNKNOWN_MSG;
			break;
		case SPOOF_INDICATED:
			msg = SPOOF_IND_MSG;
			break;
		case SPOOF_NONE:
			msg = SPOOF_NONE_MSG;
			break;
		case SPOOF_MULT:
			msg = SPOOF_MULT_MSG;
			break;
	}
	
	strncpy(spoof_state, (const char *)MESSAGES[msg], strlen((const char *)MESSAGES[msg]));
	
	snprintf((char*)pResponseBuf, bufLen,
		"Jamming Level: %d\n" \
		"Spoof State: %d, %s\n" \
		"---------------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"Rate: %ld\n" \
		"\n", 	
		pSwarmDevice->pData->d_gj.jamming_level,
		pSwarmDevice->pData->d_gj.spoof_state,
		spoof_state,
		pSwarmDevice->pData->d_gj.rxCount,
		pSwarmDevice->pData->d_gj.OKCount,
		pSwarmDevice->pData->d_gj.ERRCount,
		pSwarmDevice->pData->d_gj.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetGeoInfoResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
	snprintf((char*)pResponseBuf, bufLen, 	
		"Latitude: %f\n" \
		"Longitude: %f\n" \
		"Altitude: %f\n" \
		"Course: %f\n" \
		"Speed: %f\n" \
		"--------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"Rate: %ld\n" \
		"\n",																
		pSwarmDevice->pData->d_gn.latitude,
		pSwarmDevice->pData->d_gn.longitude,
		pSwarmDevice->pData->d_gn.altitude,
		pSwarmDevice->pData->d_gn.course,
		pSwarmDevice->pData->d_gn.speed,
		pSwarmDevice->pData->d_gn.rxCount,
		pSwarmDevice->pData->d_gn.OKCount,
		pSwarmDevice->pData->d_gn.ERRCount,
		pSwarmDevice->pData->d_gn.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetGPIOResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	char mode_msg[100] = {'\0'};		
	uint8_t msg;
	
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);

	switch (pSwarmDevice->pData->d_gp.mode)
	{
		case ANALOG:						//0 Analog, pin is internally disconnected and not used (default)
			msg = GPIO_MODE_0_MSG;
			break;
		
		case IN_LO_HI:					//1 Input, low-to-high transition exits sleep mode
			msg = GPIO_MODE_1_MSG;
			break;
		
		case IN_HI_LO:					//2 Input, high-to-low transition exits sleep mode
			msg = GPIO_MODE_2_MSG;
			break;
	
		case OUT_SET_LO:				//3 Output, set low (1)
			msg = GPIO_MODE_3_MSG;
			break;
		
		case OUT_SET_HI:				//4 Output, set high (1)
			msg = GPIO_MODE_4_MSG;
			break;
		
		case OUT_LO_PEND:			//5 Output, low indicates messages pending for client (2)
			msg = GPIO_MODE_5_MSG;
			break;
		
		case OUT_HI_PEND:			//6 Output, high indicates messages pending for client (2)
			msg = GPIO_MODE_6_MSG;
			break;
		
		case OUT_LO_XMIT:			//7 Output, low while transmitting. Otherwise output is high (3)
			msg = GPIO_MODE_7_MSG;
			break;		
		
		case OUT_HI_XMIT:			//8 Output, high while transmitting. Otherwise output is low (3)
			msg = GPIO_MODE_8_MSG;
			break;
		
		case OUT_LO_SLEEP:			//9 Output, low indicates in sleep mode (4). Otherwise output is high
			msg = GPIO_MODE_9_MSG;
			break;
		
		case OUT_HI_SLEEP:			//10 Output, high indicates in sleep mode (4). Otherwise output is low
			msg = GPIO_MODE_10_MSG;
			break;			
	}
	strncpy(mode_msg, (const char *)MESSAGES[msg], strlen((const char *)MESSAGES[msg]));
	
	snprintf((char*)pResponseBuf, bufLen,
		"Mode: %d, %s\n" \
		"------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"\n",										
		pSwarmDevice->pData->d_gp.mode, mode_msg,
		pSwarmDevice->pData->d_gp.rxCount,
		pSwarmDevice->pData->d_gp.OKCount,
		pSwarmDevice->pData->d_gp.ERRCount
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetGPSFixResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{	
	char fix_type[3] = {'\0'};
	char fix_type_msg[MAX_DEBUG_MENU_STRLEN] = {'\0'};
	
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	
	switch (pSwarmDevice->pData->d_gs.fixType)
	{		
		case NF:	//No fix
			strncpy((char*)fix_type, "NF", sizeof("NF"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_NR_MSG], strlen((const char*)MESSAGES[GPSFIX_NR_MSG]));
			break;
		case DR:	//Dead reckoning only solution
			strncpy((char*)fix_type, "DR", sizeof("DR"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_DR_MSG], strlen((const char*)MESSAGES[GPSFIX_DR_MSG]));
			break;
		case G2: //Standalone 2D solution
			strncpy((char*)fix_type, "G2", sizeof("G2"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_G2_MSG], strlen((const char*)MESSAGES[GPSFIX_G2_MSG]));
			break;
		case G3:	//Standalone 3D solution
			strncpy((char*)fix_type, "G3", sizeof("G3"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_G3_MSG], strlen((const char*)MESSAGES[GPSFIX_G3_MSG]));
			break;
		case D2:	//Differential 2D solution
			strncpy((char*)fix_type, "D2", sizeof("D2"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_D2_MSG], strlen((const char*)MESSAGES[GPSFIX_D2_MSG]));
			break;
		case D3:	//Differential 3D solution
			strncpy((char*)fix_type, "D3", sizeof("D3"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_D3_MSG], strlen((const char*)MESSAGES[GPSFIX_D3_MSG]));
			break;
		case RK:	//Combined GNSS + dead reckoning solution
			strncpy((char*)fix_type, "RK", sizeof("RK"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_RK_MSG], strlen((const char*)MESSAGES[GPSFIX_RK_MSG]));
			break;
		case TT:  //Time only solution
			strncpy((char*)fix_type, "TT", sizeof("TT"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_TT_MSG], strlen((const char*)MESSAGES[GPSFIX_TT_MSG]));
			break;
		default:			
			strncpy((char*)fix_type, "??", sizeof("NF"));
			strncpy((char*)fix_type_msg, (const char*)MESSAGES[GPSFIX_UNKNOWN_MSG], strlen((const char*)MESSAGES[GPSFIX_UNKNOWN_MSG]));
			break;
};
	
	snprintf((char*)pResponseBuf, bufLen,
		"HDOP: %d\n" \
		"VDOP: %d\n" \
		"GNSS Satellites: %d\n" \
		"Fix: %s, %s\n" \
		"-------------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"Rate Count: %ld\n" \
		"\n",																
		pSwarmDevice->pData->d_gs.hdop,
		pSwarmDevice->pData->d_gs.vdop,
		pSwarmDevice->pData->d_gs.gnss_sats,
		fix_type, fix_type_msg,
		pSwarmDevice->pData->d_gs.rxCount,
		pSwarmDevice->pData->d_gs.OKCount,
		pSwarmDevice->pData->d_gs.ERRCount,
		pSwarmDevice->pData->d_gs.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetPowerOffResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"\n",
		pSwarmDevice->pData->d_po.rxCount,
		pSwarmDevice->pData->d_po.OKCount,
		pSwarmDevice->pData->d_po.ERRCount
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetPowerStatusResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"Temperature: %3.1f*C, %3.1f*F\n" \
		"-----------------------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"Rate: %ld\n" \
		"\n",	
		pSwarmDevice->pData->d_pw.temp, ConvertCtoF(pSwarmDevice->pData->d_pw.temp),
		pSwarmDevice->pData->d_pw.rxCount,
		pSwarmDevice->pData->d_pw.OKCount,
		pSwarmDevice->pData->d_pw.ERRCount,
		pSwarmDevice->pData->d_pw.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetRestartResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"Error Count: %d\n" \
		"\n",
		pSwarmDevice->pData->d_rs.rxCount,
		pSwarmDevice->pData->d_rs.OKCount,
		pSwarmDevice->pData->d_rs.ERRCount
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetRxDataResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"App ID: %d\n" \
		"RSSI: %d\n" \
		"SNR: %d\n" \
		"FDEV: %d\n" \
		"Raw Data: %s\n" \
		"Decoded: %s\n" \
		"--------------\n" \
		"Rx Count: %ld\n" \
		"\n",
		pSwarmDevice->pData->d_rd.appID,
		pSwarmDevice->pData->d_rd.rssi_sat,
		pSwarmDevice->pData->d_rd.snr,
		pSwarmDevice->pData->d_rd.fdev,
		pSwarmDevice->pData->d_rd.raw_data,
		pSwarmDevice->pData->d_rd.msg_data,
		pSwarmDevice->pData->d_rd.rxCount
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetRxTestResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"RSSI Sat: %d\n" \
		"SNR: %d\n" \
		"FDEV: %d\n" \
		"UTC Time: %s\n" \
		"Sat ID: %s\n" \
		"Background RSSI: %d\n" \
		"-------------------\n" \
		"Rx Count: %ld\n" \
		"Sat Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"ERR Count: %d\n" \
		"Rate: %ld\n" \
		"\n",
		pSwarmDevice->pData->d_rt.rssi_sat,
		pSwarmDevice->pData->d_rt.snr,
		pSwarmDevice->pData->d_rt.fdev,
		pSwarmDevice->pData->d_rt.utc_time,
		pSwarmDevice->pData->d_rt.sat_id,
		pSwarmDevice->pData->d_rt.bg_rssi,
		pSwarmDevice->pData->d_rt.rxCount,
		pSwarmDevice->pData->d_rt.satRxCount,
		pSwarmDevice->pData->d_rt.OKCount,
		pSwarmDevice->pData->d_rt.ERRCount,
		pSwarmDevice->pData->d_rt.Rate
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetStatusResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"Rx Count: %ld\n" \
		"BOOT Count: %d\n" \
		"DATETIME Count: %d\n" \
		"POSITION Count: %d\n" \
		"BOOT Cause: %s\n" \
		"Boot Text: %s\n" \
		"Debug Text: %s\n" \
		"Error Text: %s\n" \
		"\n",
		pSwarmDevice->pData->d_swarm_device.rxCount,
		pSwarmDevice->pData->d_swarm_device.BOOTCount,
		pSwarmDevice->pData->d_swarm_device.DATETIMECount,
		pSwarmDevice->pData->d_swarm_device.POSITIONCount,
		pSwarmDevice->pData->d_swarm_device.BOOT_CAUSE,
		pSwarmDevice->pData->d_swarm_device.boot_text,
		pSwarmDevice->pData->d_swarm_device.debug_text,
		pSwarmDevice->pData->d_swarm_device.error_text
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetSleepResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"WAKE Count: %d\n" \
		"Wake Cause: %s\n" \
		"---------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"ERR Count: %d\n" \
		"Error Cause: %s\n" \
		"\n",
		pSwarmDevice->pData->d_sl.WAKECount,
		pSwarmDevice->pData->d_sl.WAKE_CAUSE,
		pSwarmDevice->pData->d_sl.rxCount,
		pSwarmDevice->pData->d_sl.OKCount,
		pSwarmDevice->pData->d_sl.ERRCount,
		pSwarmDevice->pData->d_sl.ERR_CAUSE
	);
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetTxDataResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);

	snprintf((char*)pResponseBuf, bufLen,
		"MSG ID: %llu\n" \
		"RSSI_SAT: %d\n" \
		"SNR: %d\n" \
		"FDEV: %d\n" \
		"Sent Counter: %d\n" \
		"---------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"ERR Count: %d\n" \
		"Error Cause: %s\n" \
		"\n",
		pSwarmDevice->pData->d_td.msgID,
		pSwarmDevice->pData->d_td.rssi_sat,
		pSwarmDevice->pData->d_td.snr,
		pSwarmDevice->pData->d_td.fdev,
		pSwarmDevice->pData->d_td.SENTCount,
		pSwarmDevice->pData->d_td.rxCount,
		pSwarmDevice->pData->d_td.OKCount,
		pSwarmDevice->pData->d_td.ERRCount,
		pSwarmDevice->pData->d_td.ERR_CAUSE
	);	
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetTxMessagesResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);

	snprintf((char*)pResponseBuf, bufLen,
		"MSG ID: %llu\n" \
		"Epoch Seconds: %ld, %s\n" \
		"Unsent Messages: %d\n" \
		"Raw data: %s\n" \
		"Msg data: %s\n" \
		"DELETED count: %d\n" \
		"---------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"ERR Count: %d\n" \
		"Error Cause: %s\n" \
		"\n",
		pSwarmDevice->pData->d_mt.msgID,
		pSwarmDevice->pData->d_mt.epoch_sec,pSwarmDevice->pData->d_mt.conv_time,
		pSwarmDevice->pData->d_mt.msgCount,
		pSwarmDevice->pData->d_mt.raw_data,
		pSwarmDevice->pData->d_mt.msg_data,
		pSwarmDevice->pData->d_mt.DELETEDCount,
		pSwarmDevice->pData->d_mt.rxCount,
		pSwarmDevice->pData->d_mt.OKCount,
		pSwarmDevice->pData->d_mt.ERRCount,
		pSwarmDevice->pData->d_mt.ERR_CAUSE
	);	
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

void GetRxMessagesResponse(uint8_t *pResponseBuf, uint16_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	snprintf((char*)pResponseBuf, bufLen,
		"Msg count: %d\n" \
		"App ID: %d\n" \
		"Msg ID: %llu\n" \
		"Raw data: %s\n" \
		"Msg data: %s\n" \
		"Epoch Seconds: %ld, %s\n" \
		"DELETED MSG ID: %llu\n" \
		"MARKED MSG ID: %llu\n" \
		"---------------\n" \
		"Rx Count: %ld\n" \
		"OK Count: %d\n" \
		"ERR Count: %d\n" \
		"Error Cause: %s\n" \
		"\n",
		pSwarmDevice->pData->d_mm.msgCount,
		pSwarmDevice->pData->d_mm.appID,
		pSwarmDevice->pData->d_mm.msgID,
		pSwarmDevice->pData->d_mm.raw_data,
		pSwarmDevice->pData->d_mm.msg_data,
		pSwarmDevice->pData->d_mm.epoch_sec, pSwarmDevice->pData->d_mm.conv_time,
		pSwarmDevice->pData->d_mm.deletedMsgID,
		pSwarmDevice->pData->d_mm.markedMsgID,
		pSwarmDevice->pData->d_mm.rxCount,
		pSwarmDevice->pData->d_mm.OKCount,
		pSwarmDevice->pData->d_mm.ERRCount,
		pSwarmDevice->pData->d_mm.ERR_CAUSE
	);	
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}

/**
 *	Package message to transmit
 */

void SWARM_PackageTXMessage(uint8_t *pOut, uint8_t *pIn)
{
	txTemplateFunction(TX_DATA, (char*)pIn, (char*)pOut);
}


/*	JSON Formatted Responses */
/*
 *	Name: GetPositionTsJSON
 *	Returns position information with timestamp data
 */
void SWARM_GetPositionTsJSON(uint8_t *pResponseBuf, uint8_t bufLen)
{
	// Get the mutex
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	if (pResponseBuf && pSwarmDevice->pData)
	{		
		snprintf((char *)pResponseBuf, bufLen,
			"\"{" \
			"\"la\": %f," \
			"\"ln\": %f," \
			"\"al\": %f," \
			"\"dt\": \"%04d-%02d-%02d %02d:%02d:%02d\"," \
			"\"h\": %ld" \
			"}\"",
		pSwarmDevice->pData->d_gn.latitude,
		pSwarmDevice->pData->d_gn.longitude,
		pSwarmDevice->pData->d_gn.altitude,
		pSwarmDevice->pData->d_dt.year,
		pSwarmDevice->pData->d_dt.month,
		pSwarmDevice->pData->d_dt.day,
		pSwarmDevice->pData->d_dt.hour,
		pSwarmDevice->pData->d_dt.minute,
		pSwarmDevice->pData->d_dt.seconds,
		HAL_GetDEVID()
		);
	}
	// Release the mutex
	tx_mutex_put(pSwarmDevice->pMutex);
}


E_SWARM_ERR SWARM_Initialize(S_SWARM_DEVICE *pDevice, TX_MUTEX *pSwarmMutex)
{
	if (!pDevice || !pSwarmMutex)
		return SWARM_NOT_OK;
	
	pSwarmDevice = pDevice;	// Save global pointer
	
	if (pDevice->IsInitialized)	// One-time initialization
		return SWARM_OK;
		
	memset(pDevice, 0, sizeof(S_SWARM_DEVICE));
	pDevice->pMutex = pSwarmMutex;
	pDevice->pNmeaParser = &SWARM_nmeaParser;
	pDevice->pNmeaParser->pCmdTable = swarmNmeaProcCmdTable;
	pDevice->pNmeaParser->m_numTableCmds = NUM_SWARM_NMEA_PROC_CMD;
	pDevice->pNmeaParser->ProcessCommand = SWARM_ProcessCommand;
	pDevice->ParseBuffer = SWARM_ParseBuffer;
	pDevice->Startup = SWARM_Startup;
	pDevice->DebugInterface = SWARM_GetResponse;
	pDevice->GetPositionTsJSON = SWARM_GetPositionTsJSON;
	pDevice->PrepareMessage = SWARM_PackageTXMessage;
	pDevice->pData = &SWARM_data;
	pDevice->IsInitialized = 1;
	pDevice->state = SWARM_IF_STARTUP;
	return SWARM_OK;
}

E_SWARM_STATE SWARM_Startup(uint8_t *pBuff)
{
	static uint8_t seqIdx = 0;
		
	while (seqIdx < NUM_SWARM_DBG_CMDS)
	{
		if (swarm_dbg_cmd_table[seqIdx].mode == STARTUP_MODE)
		{
			txTemplateFunction(seqIdx, "", (char *)pBuff);
			seqIdx++;
			return SWARM_IF_STARTUP;
		}
		seqIdx++;
	}
	return SWARM_IF_READY;
}


uint8_t SWARM_ParseBuffer(uint8_t *pBuff, uint16_t dataLen)
{
	if (!pSwarmDevice)
		return 1;
	
	ParseBuffer(pSwarmDevice->pNmeaParser, pBuff, dataLen);
	return 0;
}

void SWARM_SaveLastCommand(char *lastCmd)
{			
	tx_mutex_get(pSwarmDevice->pMutex, TX_WAIT_FOREVER);
	strcpy((char *)pSwarmDevice->lastCommandRcvd, lastCmd);
	tx_mutex_put(pSwarmDevice->pMutex);
}

uint8_t SWARM_ProcessCommand(void)
{	
	volatile uint32_t len = 0;
	if (!pSwarmDevice)
		return 1;
	
	S_SWARM_DEVICE *pSwarm = (S_SWARM_DEVICE*)pSwarmDevice;
	
	for (uint16_t i=0; i<pSwarm->pNmeaParser->m_numTableCmds; i++)
	{
		len = strlen((const char*)pSwarm->pNmeaParser->m_Command);
		if (!strncmp((char *)pSwarm->pNmeaParser->m_Command,pSwarm->pNmeaParser->pCmdTable[i].cmd,len))
		{
			SWARM_SaveLastCommand((char *)pSwarm->pNmeaParser->m_Command);
			pSwarm->pNmeaParser->pCmdTable[i].fun(pSwarm);
			break;
		}
	}
	return 0;
}



/**********************************************************************/
/*****							Calculate NMEA checksum	for transmit						***/
/**********************************************************************/
uint8_t nmeaChecksum(const char *sz, uint8_t len)
{
	uint8_t i=0;
	uint8_t cs;
	
	if (sz[0] == NMEA_DELIM_CHAR)
		i++;
	
	for (cs=0; (i<len) && sz[i]; i++)
		cs ^= ((uint8_t)sz[i]);
	
	return cs;
}

