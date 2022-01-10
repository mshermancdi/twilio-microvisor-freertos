/**********************************************************************************************


	swarm_tile.h



*************************************************************************************************/

#ifndef __SWARM_TILE_H
#define __SWARM_TILE_H

#include <stdint.h>
#include "debug_utils.h"
#include "nmea.h"
//#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "app_threadx.h"

typedef enum _E_TILE_ERR {
	TILE_OK,
	TILE_NOT_OK
} E_TILE_ERR;
	
typedef enum _E_TILE_STATE {
	TILE_IF_NOT_STARTED,
	TILE_IF_STARTUP,
	TILE_IF_READY
	
} E_TILE_STATE;

enum FLAG {
	INVALID = 'I',
	VALID = 'V'
};

enum SPOOF_STATE {
	SPOOF_UNKNOWN,		//0
	SPOOF_NONE,				//1
	SPOOF_INDICATED,	//2
	SPOOF_MULT				//3
};

enum FIX_TYPE {
	NF,	//No fix
	DR,	//Dead reckoning only solution
	G2, //Standalone 2D solution
	G3,	//Standalone 3D solution
	D2,	//Differential 2D solution
	D3,	//Differential 3D solution
	RK,	//Combined GNSS + dead reckoning solution
	TT  //Time only solution
};


enum TILE_GPIO_MODE {
	ANALOG,						//0 Analog, pin is internally disconnected and not used (default)
	IN_LO_HI,					//1 Input, low-to-high transition exits sleep mode
	IN_HI_LO,					//2 Input, high-to-low transition exits sleep mode
	OUT_SET_LO,				//3 Output, set low (1)
	OUT_SET_HI,				//4 Output, set high (1)
	OUT_LO_PEND,			//5 Output, low indicates messages pending for client (2)
	OUT_HI_PEND,			//6 Output, high indicates messages pending for client (2)
	OUT_LO_XMIT,			//7 Output, low while transmitting. Otherwise output is high (3)
	OUT_HI_XMIT,			//8 Output, high while transmitting. Otherwise output is low (3)
	OUT_LO_SLEEP,			//9 Output, low indicates in sleep mode (4). Otherwise output is high
	OUT_HI_SLEEP			//10 Output, high indicates in sleep mode (4). Otherwise output is low
};

typedef struct _S_TILE_DEVICE_DATA S_TILE_DEVICE_DATA;

typedef struct _S_TILE_DEVICE {	
	TX_MUTEX *pMutex;
	S_TILE_DEVICE_DATA *pData;
	uint8_t				IsInitialized;
	S_NMEAParser 	*pNmeaParser;
	uint8_t (*ParseBuffer)(uint8_t *pBuff, uint16_t dataLen);
	uint8_t (*Startup)(uint8_t *pBuff);
	E_TILE_ERR (*DebugInterface)(uint8_t *pBuff, uint16_t bufLen, ARG_LIST args, char *pTxBuf, void (**cbFunc)(uint8_t *, uint16_t), uint32_t *waitEventFlags, uint32_t *multiplicity);
	E_TILE_STATE state;
	uint8_t lastCommandRcvd[10];
	void (*GetPositionTsJSON)(uint8_t *pBuf, uint8_t len);
	void (*PrepareMessage)(uint8_t *pOut, uint8_t *pIn);
} S_TILE_DEVICE;

typedef enum _E_TILE_CMDS {
	GET_CONFIG,
	DATETIME,
	VERSION,
	GPS_JAM_SPOOF,
	GEO_INFO,
	GPIO_CTL,
	GPS_FIX,
	POWER_OFF,
	POWER_STATUS,
	RX_DATA,
	RESTART,
	RX_TEST,
	TILE_STATUS,
	TX_DATA,
	RX_MSGS,
	TX_MSGS,
	SLEEP,
	TILE_HELP,
	TILE_META,
	
	TILE_NULL_CMD = 255
} E_TILE_CMDS;

typedef enum _E_TILE_MODE {
	NONE_MODE,
	STARTUP_MODE,
	RUNTIME_MODE
} E_TILE_MODE;

typedef enum _E_TILE_LED_MODE {
	TILE_MSG_SENT,
	TILE_ERR_COND,
	TILE_BG_RSSI_GOOD,
	TILE_BG_RSSI_BAD,
} E_TILE_LED_MODE;

E_TILE_ERR TILE_Initialize(S_TILE_DEVICE *pDevice, TX_MUTEX *pMutex);
#endif	/* __SWARM_TILE_H */