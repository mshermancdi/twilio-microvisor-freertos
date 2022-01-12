/**********************************************************************************************


	debug_utils.h



*************************************************************************************************/

#ifndef __DEBUG_UTILS_H
#define __DEBUG_UTILS_H

#include <stdint.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#define MAX_ARG_SIZE 20
#define MAX_DBG_ARGS 4
#define MAX_SWARM_MSG_SIZE 192
 
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

char GetHexChar(uint8_t val);
void DecToHex(char *out_buff, uint8_t dec);
void HexToASCII(char *out_buff, const char* in_buff, unsigned int length);
void EpochToHumanReadable(char *out_buff, uint16_t buff_len, time_t raw);
float ConvertCtoF(float degC);
char *GetTimeStamp(void);

typedef struct _ARG_LIST {
	uint8_t arg[MAX_DBG_ARGS][MAX_ARG_SIZE];
	uint8_t count;
} ARG_LIST;


typedef struct _S_SWARM_MSG {
	char data[MAX_SWARM_MSG_SIZE];
	uint8_t idx;
} S_SWARM_MSG;

#ifndef NULL
#define NULL 0
#endif

#endif /* __DEBUG_UTILS_H */
