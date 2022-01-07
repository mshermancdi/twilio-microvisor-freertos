/*************************************************/
/*		Utilities																	**/
/*************************************************/


#include "debug_utils.h"
#include <stdlib.h>

char GetHexChar(uint8_t val)
{	
	char retVal = '0';
	switch(val)
	{
		case 15:
			retVal = 'F';
			break;
		case 14:
			retVal = 'E';
			break;
		case 13:
			retVal = 'D';
			break;
		case 12:
			retVal = 'C';
			break;
		case 11:
			retVal = 'B';
			break;
		case 10:
			retVal = 'A';
			break;
		default:
			retVal = (char)(val+'0');
			break;
	}		
	return retVal;	
}

void DecToHex(char *out_buff, uint8_t dec)
{
	if (!out_buff)
		return;
	
	// 0 - 15
	if (dec < 16)
	{
		out_buff[0] = '0';
	}
	else
	{
		out_buff[0] = GetHexChar(dec/16);
	}
	
	out_buff[1] = GetHexChar(dec%16);	
	
	return;
}

float ConvertCtoF(float degC)
{
	float result;
	
	result = degC * 1.8f;
	
	result += 32.0f;
	
	return result;
}

void HexToASCII(char *out_buff, const char* in_buff, unsigned int length)
{
	if (out_buff == NULL || in_buff == NULL)
		return;
	
	uint8_t textLen = length/2;				
	char pBuff[3];
	char *pEnd;
	
	for (uint8_t i=0; i<textLen; i++)
	{
		pBuff[0] = in_buff[i*2];
		pBuff[1] = in_buff[i*2+1];
		pBuff[2] = '\0';		

		out_buff[i] = strtol(pBuff, &pEnd, 16);
	}		
}


void EpochToHumanReadable(char *out_buff, uint16_t buff_len, time_t raw)
{
	if (out_buff == NULL)
		return;
	
	struct tm ts;
	
	// Format time	ddd yyyy-mm-dd hh:mm:ss zzz
	ts = *localtime(&raw);
	strftime(out_buff, buff_len, "%a %Y-%m-%d %H:%M:%S %Z", &ts);
	
	return;
}