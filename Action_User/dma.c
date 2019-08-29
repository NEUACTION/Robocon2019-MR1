/**
  *******************************************************************************************************
  * @filE	dma.c
  * @author	ACTION_2018
  * @version	V1.0
  * @date	2018/2/15
  * @brief	This file contains
  *
  *******************************************************************************************************
  * @attention
  *
  *
  *******************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "usart.h"
#include "dma.h"

#include "stm32f4xx_usart.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
uint16_t DebugUSARTSendBuffCnt = 0u;
uint8_t DebugUSARTSendBuf[DEBUG_USART_SEND_BUF_CAPACITY] = {0};
uint8_t DebugUSARTDMASendBuf[DEBUG_USART_SEND_BUF_CAPACITY] = {0};

//调试用串口变量定义
static USART_TypeDef* debugUSART = DEBUG_USART;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief 串口DMA使能函数
* @note USARTInit函数类型必须正确！！！使能DMA后要写对应的DMA发送成功中断服务函数
  * @param USARTx：要使能的串口
  * @param *buffAddr：DMA数据缓冲区地址
  * @param *USARTInit：串口使能函数
  * @param baudrate：要配置的波特率
  * @retval None
  */
void USARTDMASendInit(USART_TypeDef* USARTx,uint8_t *buffAddr,void (*USARTInit)(uint32_t baudrate),uint32_t baudrate)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure={0};
	DMA_Stream_TypeDef* DMAy_Streamx;
	uint32_t DMAChannelx;
	uint8_t DMAy_Streamx_IRQn;
	
	DMA_StructInit(&DMA_InitStructure);
	
	switch((uint32_t)USARTx)
	{
		case USART1_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			DMAy_Streamx = DMA2_Stream7;
			DMAChannelx = DMA_Channel_4;
			DMAy_Streamx_IRQn = DMA2_Stream7_IRQn;
		break;
		case USART2_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMAy_Streamx = DMA1_Stream6;
			DMAChannelx = DMA_Channel_4;
			DMAy_Streamx_IRQn = DMA1_Stream6_IRQn;
		break;
		case USART3_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMAy_Streamx = DMA1_Stream3;
			DMAChannelx = DMA_Channel_4;
			DMAy_Streamx_IRQn = DMA1_Stream3_IRQn;
		break;
		case UART4_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMAy_Streamx = DMA1_Stream4;
			DMAChannelx = DMA_Channel_4;
			DMAy_Streamx_IRQn = DMA1_Stream4_IRQn;
		break;
		case UART5_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			DMAy_Streamx = DMA1_Stream7;
			DMAChannelx = DMA_Channel_4;
			DMAy_Streamx_IRQn = DMA1_Stream7_IRQn;
		break;
		case USART6_BASE:
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			DMAy_Streamx = DMA2_Stream6;
			DMAChannelx = DMA_Channel_5;
			DMAy_Streamx_IRQn = DMA2_Stream6_IRQn;
		break;
		default:
			return;	
	}


	USARTInit(baudrate);

	DMA_DeInit(DMAy_Streamx);
	while (DMA_GetCmdStatus(DMAy_Streamx) != DISABLE){}

	DMA_InitStructure.DMA_Channel = DMAChannelx;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(USARTx->DR); // peripheral address, = & USART5->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffAddr;	// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				// data dirction:  memory to peripheral
	DMA_InitStructure.DMA_BufferSize = 0;					//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//8 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMAy_Streamx, &DMA_InitStructure);

	USART_DMACmd(USARTx,USART_DMAReq_Tx,ENABLE);
	DMA_ITConfig(DMAy_Streamx,DMA_IT_TC,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMAy_Streamx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}


void DMA2_Stream7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) != RESET)
	{
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	}
}


/**
  * @brief 使能DMA一次数据传输函数
  * @note debugUSART需要根据实际情况赋值
  * @param USARTx：要使用的串口
  * @param *buffAddr：发送数据缓冲区地址
  * @param *sendBufAddr：DMA发送缓冲区地址
  * @param *buffPointer：数据缓冲区数据个数
  * @retval None
  */
void USARTDMASend(USART_TypeDef* USARTx,uint8_t *buffAddr,uint8_t *sendBufAddr,uint16_t *buffPointer)
{

	DMA_Stream_TypeDef* DMAy_Streamx;
	uint32_t DMA_FLAG_TCIFx;
	uint64_t timeout = 0;
	switch((uint32_t)USARTx)
	{
		case USART1_BASE:
			DMAy_Streamx = DMA2_Stream7;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF7;
		break;
		case USART2_BASE:
			DMAy_Streamx = DMA1_Stream6;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF6;
		break;
		case USART3_BASE:
			DMAy_Streamx = DMA1_Stream3;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF3;
		break;
		case UART4_BASE:
			DMAy_Streamx = DMA1_Stream4;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF4;
		break;
		case UART5_BASE:
			DMAy_Streamx = DMA1_Stream7;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF7;
		break;
		case USART6_BASE:
			DMAy_Streamx = DMA2_Stream6;
			DMA_FLAG_TCIFx = DMA_FLAG_TCIF6;
		break;
		default:
			return;
	}
	while(DMA_GetCurrDataCounter(DMAy_Streamx) != 0u)
	{
		timeout++;
		if(timeout > 0x0fffffff)
		{
			USART_SendData(debugUSART,'D');
			USART_SendData(debugUSART,'M');
			USART_SendData(debugUSART,'A');
			USART_SendData(debugUSART,'C');
			USART_SendData(debugUSART,'O');
			USART_SendData(debugUSART,'U');
			USART_SendData(debugUSART,'N');
			USART_SendData(debugUSART,'T');
			USART_SendData(debugUSART,'E');
			USART_SendData(debugUSART,'R');	
			USART_SendData(debugUSART,'R');
			USART_SendData(debugUSART,'O');
			USART_SendData(debugUSART,'R');
			USART_SendData(debugUSART,'\r');
			USART_SendData(debugUSART,'\n');

		}
	}
	DMA_ClearFlag(DMAy_Streamx,DMA_FLAG_TCIFx);
	DMA_Cmd(DMAy_Streamx, DISABLE);
	memcpy(sendBufAddr,buffAddr, *buffPointer);
	timeout = 0;
	while (DMA_GetCmdStatus(DMAy_Streamx) != DISABLE)
	{
		timeout++;
		if(timeout > 0x0fffffff)
		{
			USART_SendData(debugUSART,'D');
			USART_SendData(debugUSART,'M');
			USART_SendData(debugUSART,'A');
			USART_SendData(debugUSART,'N');
			USART_SendData(debugUSART,'O');
			USART_SendData(debugUSART,'T');
			USART_SendData(debugUSART,'D');
			USART_SendData(debugUSART,'I');
			USART_SendData(debugUSART,'S');
			USART_SendData(debugUSART,'A');	
			USART_SendData(debugUSART,'B');
			USART_SendData(debugUSART,'L');
			USART_SendData(debugUSART,'E');
			USART_SendData(debugUSART,'\r');
			USART_SendData(debugUSART,'\n');

		}	
	}
	DMA_SetCurrDataCounter(DMAy_Streamx, *buffPointer);
	DMA_Cmd(DMAy_Streamx, ENABLE);
	*buffPointer = 0u;
}


/**
  * @brief	向串口DMA数据缓冲区中写数据
  * @note	fix me 并不是环形数组  注意填入数据过多时可能导致Buffer溢出
  * @param	USARTx: 使用的串口
  * @param	data: 需要传输的八位数据
  * @param	*buffAddr: 串口DMA数据缓冲区地址
  * @param	*buffPointer: 缓冲区中的数据数量
  * @param	*sendBufAddr: DMA发送缓冲区地址
  * @param	bufferSize: 串口DMA数据缓冲区的大小
  * @retval	None
  */
void USARTDMASendData(USART_TypeDef* USARTx,uint8_t data,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize)
{
//	uint16_t i = 0u;
	DMA_Stream_TypeDef* DMAy_Streamx;

	switch((uint32_t)USARTx)
	{
		case USART1_BASE:
			DMAy_Streamx = DMA2_Stream7;
		break;
		case USART2_BASE:
			DMAy_Streamx = DMA1_Stream6;
		break;
		case USART3_BASE:
			DMAy_Streamx = DMA1_Stream3;
		break;
		case UART4_BASE:
			DMAy_Streamx = DMA1_Stream4;
		break;
		case UART5_BASE:
			DMAy_Streamx = DMA1_Stream7;
		break;
		case USART6_BASE:
			DMAy_Streamx = DMA2_Stream6;
		break;
		default:
			return;
	}
	if(*buffPointer < bufferSize)
	{
		buffAddr[(*buffPointer)++] = data;
		if(DMA_GetCmdStatus(DMAy_Streamx) == DISABLE && DMA_GetCurrDataCounter(DMAy_Streamx) == 0u)
		{
			if(*buffPointer > 200u)
			{
				 USARTDMASend(USARTx,buffAddr,sendBufAddr,buffPointer);
			}
		}
//		else if(sendBufferCnt > 144u)
//		{
//			while(DMA_GetCmdStatus(DMA1_Stream7) == ENABLE || DMA_GetCurrDataCounter(DMA1_Stream7) != 0u)
//			{
//				i++;
//				if(i == 0)
//					break;
//			}
//			UART5_DMA_Send();
//		}
	}
	else
	{
		USART_SendData(debugUSART,'D');
		USART_SendData(debugUSART,'M');
		USART_SendData(debugUSART,'A');
		USART_SendData(debugUSART,'O');
		USART_SendData(debugUSART,'V');
		USART_SendData(debugUSART,'E');
		USART_SendData(debugUSART,'R');
		USART_SendData(debugUSART,'F');
		USART_SendData(debugUSART,'L');
		USART_SendData(debugUSART,'O');	
		USART_SendData(debugUSART,'W');
		USART_SendData(debugUSART,'\r');
		USART_SendData(debugUSART,'\n');
		*buffPointer = 0;
	}
}
/**
  * @brief	串口DMA格式化输出函数
  * @note	
  * @param	USARTx: 使用的串口
  * @param	*buffAddr: 串口DMA数据缓冲区地址
  * @param	*buffPointer: 缓冲区中的数据数量
  * @param	*sendBufAddr: DMA发送缓冲区地址
  * @param	bufferSize: 串口DMA数据缓冲区的大小
  * @param	*Data：格式化输入字符串
  * @retval	None
  */
void USARTDMAOUT(USART_TypeDef* USARTx,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize,const uint8_t *Data, ...)
{ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data != 0)				                          //判断是否到达字符串结束符
	{
		if(*Data == 0x5c)									  //'\'
		{	
			switch (*++Data)
			{
				case 'r':							          //回车符	   
					USARTDMASendData(USARTx,0x0d,buffAddr,buffPointer ,sendBufAddr,bufferSize);
					Data++;
					break;
				case 'n':	
					//换行符
					USARTDMASendData(USARTx,0x0a,buffAddr,buffPointer ,sendBufAddr,bufferSize);
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}	 
		}
		else if(*Data == '%')									  //
		{
			switch (*++Data)				
			{
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for (; *s; s++) 
				    {
						USARTDMASendData(USARTx,*s,buffAddr,buffPointer ,sendBufAddr,bufferSize);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) 
				    {
						USARTDMASendData(USARTx,*s,buffAddr,buffPointer ,sendBufAddr,bufferSize);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else
		{
			USARTDMASendData(USARTx,*Data++,buffAddr,buffPointer ,sendBufAddr,bufferSize);
		}
	}
	va_end(ap);
}


