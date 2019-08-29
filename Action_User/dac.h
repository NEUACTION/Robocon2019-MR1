#ifndef __DAC_H
#define __DAC_H

#include "stdint.h"

/**
* @brief  DAC1 init
* @param  PA4
* @author ACTION
*/
void DAC1_Init(void);

/**
* @brief  DAC2 init
* @param  PA5
* @author ACTION
*/
void DAC2_Init(void);

/**
* @brief  DAC voltage set
* @param  channel£ºDAC channel 1||2
* @param  vol£º0-10000 corresponds to0-10V
* @author ACTION
*/
void Dac_Set_Vol(uint8_t channel,uint16_t vol);


#endif
