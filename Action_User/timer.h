#ifndef __timer_h
#define __timer_h

#include "stm32f4xx_tim.h"

//宏延时函数
#define SYSCLK 168 //指明MCU工作频率为168MHz
#define ONE_CYCLE_TIME 3        //一次循环所花的周期数
#define TOTAL_CYCLE_TIME 3        //调用、初始化、返回总共所用的周期数
#define delay_us(nus) wait(((nus) * (SYSCLK) - (TOTAL_CYCLE_TIME)) / (ONE_CYCLE_TIME))
#define delay_ms(nms) delay_us((nms)*1000)
#define delay_s(ns) delay_ms((ns)*1000)

//小函道结构体
typedef struct
{
	float LF;
	float RF;
	float LR;
	float RR;
}smallDuckedFan_t;

//精确延时函数调用wait
void wait(uint32_t n);

void TIM_Init(TIM_TypeDef *TIMx, uint16_t arr, uint16_t psr, uint16_t prepri, uint16_t subpri);
void TIM_Delayms(TIM_TypeDef *TIMx, uint32_t DelayMs);
void TIM_Delayus(TIM_TypeDef *TIMx, uint16_t Delayus);
void TIM_Delay100us(TIM_TypeDef *TIMx, uint16_t Delay100us);

void TIM5_Pwm_Init(u32 arr, u32 psc);
void TIM3_Pwm_Init(u32 arr, u32 psc);
void TIM4_Pwm_Init(u32 arr, u32 psc);
void TIM1_Pwm_Init(u32 arr, u32 psc);
void TIM8_Pwm_Init(u32 arr, u32 psc);
void SetCompare(uint8_t Num, float Percent);
void SetSpeed(uint8_t Num, float percent);
float GetSpeed(uint8_t Num);
void IncSpeed(uint8_t Num);
void DecSpeed(uint8_t Num);

void TIM7_Int_Init(u16 arr, u16 psc);

void SetSmallDuckedFanDutyCycle(smallDuckedFan_t dutyCycle);
void SetBigDuckedFanDutyCycle(float dutyCycle);


#endif
