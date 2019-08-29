/**
  ******************************************************************************
  * @file    dma.h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017/04/29
  * @brief   This file contains all the functions prototypes for DMA
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx_usart.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define DEBUG_USART USART1

#define DEBUG_USART_SEND_BUF_CAPACITY 400u
	 
extern uint16_t DebugUSARTSendBuffCnt;
extern uint8_t DebugUSARTSendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint8_t DebugUSARTDMASendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
/* Exported functions --------------------------------------------------------*/

/**
  * @brief 串口DMA使能函数
* @note USARTInit函数类型必须正确！！！使能DMA后要写对应的DMA发送成功中断服务函数
  * @param USARTx：要使能的串口
  * @param *buffAddr：DMA数据缓冲区地址
  * @param *USARTInit：串口使能函数
  * @param baudrate：要配置的波特率
  * @retval None
  */
void USARTDMASendInit(USART_TypeDef* USARTx,uint8_t *buffAddr,void (*USARTInit)(uint32_t baudrate),uint32_t baudrate);


/**
  * @brief 使能DMA一次数据传输函数
  * @note debugUSART需要根据实际情况赋值
  * @param USARTx：要使用的串口
  * @param *buffAddr：发送数据缓冲区地址
  * @param *sendBufAddr：DMA发送缓冲区地址
  * @param *buffPointer：数据缓冲区数据个数
  * @retval None
  */
void USARTDMASend(USART_TypeDef* USARTx,uint8_t *buffAddr,uint8_t *sendBufAddr,uint16_t *buffPointer);


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
void USARTDMASendData(USART_TypeDef* USARTx,uint8_t data,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize);
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
void USARTDMAOUT(USART_TypeDef* USARTx,uint8_t *buffAddr,uint16_t *buffPointer ,uint8_t *sendBufAddr,uint16_t bufferSize,const uint8_t *Data, ...);
	 
#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */

