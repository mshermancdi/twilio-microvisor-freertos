/**
  ******************************************************************************
  * File Name          : NMEA.c
  * Description        : Code for NMEA checksum 
  ******************************************************************************
  */

#include "NMEA.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>


uint8_t ParseBuffer(S_NMEAParser *pParser, uint8_t *pBuff, uint16_t dataLen)
{
	uint8_t retVal = 0;
	
	if (!pParser || !pBuff)
		return 1;

	for (uint16_t i=0; i<dataLen; i++)
	{
		ProcessNMEA(pParser, pBuff[i]);
	}	
	return retVal;
}

void ProcessNMEA(S_NMEAParser *pParser, uint8_t data)
{
	switch(pParser->m_State)
	{
		///////////////////////////////////////////////////////////////////////
		// Search for start of message '$'
		case NP_STATE_SOM:
			if (data == NMEA_DELIM_CHAR)
			{
				pParser->m_Checksum = 0;					// reset checksum
				pParser->m_Index = 0;							// reset index
				pParser->m_State = NP_STATE_CMD;	
			}
			break;
			
		///////////////////////////////////////////////////////////////////////
		// Retrieve command
		case NP_STATE_CMD:
			if (data != ' ' && data != ',' && data != '*')
			{
				pParser->m_Command[pParser->m_Index++] = data;
				pParser->m_Checksum ^= data;
				
				// Check for command overflow
				if (pParser->m_Index > NP_MAX_CMD_LEN)
				{
					pParser->m_State = NP_STATE_SOM;
				}
			}
			else
			{
				pParser->m_Command[pParser->m_Index] = '\0';	// terminate command
				pParser->m_Checksum ^= data;
				pParser->m_Index = 0;
				pParser->m_State = NP_STATE_DATA;		// goto get data state
			}				
			break;
					
		///////////////////////////////////////////////////////////////////////
		// Store data and check for end of sentence or checksum flag
		case NP_STATE_DATA:
			if (data == '*')	// checksum flag?
			{
				pParser->m_Data[pParser->m_Index] = '\0';
				pParser->m_State = NP_STATE_CHECKSUM_1;
			}
			else	// no checksum flag, store data
			{
				// Check for end of sentence with no checksum
				if (data == '\n')
				{
					pParser->m_Data[pParser->m_Index] = '\0';
					pParser->ProcessCommand();
					pParser->m_State = NP_STATE_SOM;
					return;
				}
				
				// Store data and calculate checksum
				pParser->m_Checksum ^= data;
				pParser->m_Data[pParser->m_Index] = data;
				if (++pParser->m_Index >= NP_MAX_DATA_LEN)	// Check for buffer overflow
				{
					pParser->m_State = NP_STATE_SOM;
				}
			}				
			break;
		
		case NP_STATE_CHECKSUM_1:
			if ( (toupper(data) - '0') <= 9)
			{
				pParser->m_RxChecksum = (toupper(data) - '0') << 4;
			}
			else
			{
				pParser->m_RxChecksum = (toupper(data) - 'A' + 10) << 4;
			}
						
			pParser->m_State = NP_STATE_CHECKSUM_2;			
			break;
		
		case NP_STATE_CHECKSUM_2:
			if ( (toupper(data) - '0') <= 9)
			{
				pParser->m_RxChecksum |= (toupper(data) - '0');
			}
			else
			{
				pParser->m_RxChecksum |= (toupper(data) - 'A' + 10);
			}
			
			if (pParser->m_Checksum == pParser->m_RxChecksum)
			{
				pParser->ProcessCommand();
			}
			
			pParser->m_State = NP_STATE_SOM;		
			break;
		
		default:
			break;
	}
}
uint8_t GetField(uint8_t* pData, uint8_t* pField, int nFieldNum, int nMaxFieldLen)
{
	//
	// Validate params
	//
	if(!pData || !pField || nMaxFieldLen <= 0)
	{
		return 0;
	}

	memset(pField, 0, nMaxFieldLen);
	
	//
	// Go to the beginning of the selected field
	//
	int i = 0;
	int nField = 0;
	while(nField != nFieldNum && pData[i])
	{
		if(pData[i] == ',')
		{
			nField++;
		}

		i++;

		if(!pData[i])
		{
			pField[0] = '\0';
			return 0;
		}
	}

	if(pData[i] == ',' || pData[i] == '*')
	{
		pField[0] = '\0';
		return 0;
	}

	//
	// copy field from pData to Field
	//
	int i2 = 0;
	while(pData[i] != ',' && pData[i] != '*' && pData[i])
	{
		pField[i2] = pData[i];
		i2++; i++;

		//
		// check if field is too big to fit on passed parameter. If it is,
		// crop returned field to its max length.
		//
		if(i2 >= nMaxFieldLen)
		{
			i2 = nMaxFieldLen-1;
			break;
		}
	}
	pField[i2] = '\0';

	return 1;
}

// *** PROTOTYPE of ProcessCommand **** 
//uint8_t ProcessCommand(S_NMEAParser *pParser)
//{	
//	for (uint16_t i=0; i<pParser->m_numTableCmds; i++)
//	{
//		if (strncmp((char *)pParser->m_Command,pParser->pCmdTable[i].cmd,strlen((const char*)pParser->m_Command)) == NULL)
//		{
//			pParser->pCmdTable[i].fun(pParser);
//			break;
//		}
//	}
//	return 0;
//}



