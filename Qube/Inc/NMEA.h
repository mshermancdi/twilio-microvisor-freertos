/**********************************************************************************************


	NMEA.h



*************************************************************************************************/

#ifndef __NMEA_H
#define __NMEA_H

#include <stdint.h>

#define NP_MAX_CMD_LEN	6 	// maximum command length 
#define NP_MAX_DATA_LEN	378 // maximum data length
#define MAXFIELD 150
#define NMEA_DELIM_CHAR '$'

typedef enum _E_NP_STATE {
	NP_STATE_SOM = 0,			// Search for start of message
	NP_STATE_CMD,					// Get command
	NP_STATE_DATA,				// Get data
	NP_STATE_CHECKSUM_1,	// Get first checksum character
	NP_STATE_CHECKSUM_2		// Get second checksum character
} E_NP_STATE;

typedef struct _S_NMEA_PROC_CMD_TABLE {
	char cmd[NP_MAX_CMD_LEN];
	void (*fun)(void *pDevice);
} NMEA_PROC_CMD_TABLE;

typedef struct _S_NMEAParser {
	E_NP_STATE 	m_State;										// Current state protocol parser is in
	uint8_t 		m_Checksum;									// Calculated checksum
	uint8_t			m_RxChecksum;								// Received checksum
	uint8_t			m_Data[NP_MAX_DATA_LEN];		// NMEA data
	uint8_t			m_Command[NP_MAX_CMD_LEN];	// NMEA command	
	uint16_t		m_Index;										// Index used for command and data	
	uint16_t		m_numTableCmds;
	NMEA_PROC_CMD_TABLE *pCmdTable;
	uint8_t (*ProcessCommand)(void);
} S_NMEAParser;

uint8_t ParseBuffer(S_NMEAParser *pParser, uint8_t *pBuff, uint16_t dataLen);
void ProcessNMEA(S_NMEAParser *pParser, uint8_t data);
uint8_t GetField(uint8_t* pData, uint8_t* pField, int nFieldNum, int nMaxFieldLen);
#endif /* __NMEA_H */