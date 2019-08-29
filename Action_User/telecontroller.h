#ifndef __TELECONTROLLER_H
#define __TELECONTROLLER_H
#include "stdint.h"

#define MAX_TELECMD_BYTE_LENGTH	3

#define TELENOCMD								0
#define TELENEXT								1
#define TELESTOP								2

#define ANGLESHIFT_NONE					0
#define ANGLESHIFT_POSITIVE			1
#define ANGLESHIFT_NEGIATIVE		2

void TeleCmdBufferInput_BT(uint8_t data);
void TeleCmdPross_BT(char* teleMsg);
void TeleCmdBufferInput_WIFI(uint8_t data);
void TeleCmdPross_WIFI(char* teleMsg);
void AngleShiftLimit(void);
void TeleCmdProssGather(void);
void TeleInit(void);
#endif 
