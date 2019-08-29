#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

//红蓝场开关 PD7
#define COURTID_KEYSWITCH							(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7))
//进入/退出重启开关 PD6
#define RESET_KEYSWITCH								(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6))
//自检开关 PD5
#define SELF_CHECK_KEYSWITCH					(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5))
//复位开关 PD4
#define STEER_RESET_KEYSWITCH		    	(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4))
//调试开关 PD3
#define MANUAL_MODE_KEYSWITCH					(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3))

//手臂气阀开关 PD1
#define VALVE_KEYSWITCH								(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1))

//蜂鸣器 PC3
#define BEEP_ON (GPIO_SetBits(GPIOC,GPIO_Pin_3))
#define BEEP_OFF (GPIO_ResetBits(GPIOC,GPIO_Pin_3))

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void LEDInit(void);
void BeepInit(void);
void PhotoelectricityInit(void);

#endif
