#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

void GasValveControl(CAN_TypeDef* CANx , uint8_t boardNum , uint8_t valveNum , uint8_t valveState);

#endif
