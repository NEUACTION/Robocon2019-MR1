#include "dac.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_rcc.h"

void DAC1_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//enable GPIOA clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//enable DAC clock
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//analogue input
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//float
	GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIO init
	
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;	//disable trigger TEN1=0
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;//disabled waveform generation
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//shield and amplitude setting
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC2 output buffer off BOFF1=1
	DAC_Init(DAC_Channel_1,&DAC_InitStructure);	 //init DAC channel1
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12 bits align right set DAC value
	
	DAC_Cmd(DAC_Channel_1, ENABLE);  //enable DAC channle2
	
}
void DAC2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//enable GPIOA clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//enable DAC clock
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//analogue input
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//float
	GPIO_Init(GPIOA, &GPIO_InitStructure);//GPIO init
	
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;	//disable trigger TEN1=0
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;//disabled waveform generation
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//shield and amplitude setting
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC2 output buffer off BOFF1=1
	DAC_Init(DAC_Channel_2,&DAC_InitStructure);	 //init DAC channel2
	
	DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12 bits align right set DAC value
	
	DAC_Cmd(DAC_Channel_2, ENABLE);  //enable DAC channle2
	
}


/**
* @brief  DAC voltage set
* @param  channel£ºDAC channel 1||2
* @param  vol£º0-10000 corresponds to0-10V
* @author ACTION
*/
void Dac_Set_Vol(uint8_t channel,uint16_t vol)
{
	double temp=vol;
	temp/=3060;
	temp=temp*4096/3.3;
	if(channel==1)
	{
		DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12 bits align right
	}
	if(channel==2)
	{
		DAC_SetChannel2Data(DAC_Align_12b_R,temp);//12 bits align right
	}
}
