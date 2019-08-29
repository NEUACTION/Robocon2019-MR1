/**
  ******************************************************************************
  * @file    can.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of CAN
  ******************************************************************************
  * @attention
  * 1.the default CAN mode is normal
  * 2.by default, the CAN Rx Interrupt is ON
  * 3.CAN_IT_FMP0, CAN_IT_FMP1 IRQ is auto cleared.
  ******************************************************************************
**/

#include "can.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "String.h"
#include "ucos_ii.h"
#include "cpu.h"
#include "gpio.h"
#include "usart.h"
#include "robot.h"
/**
  * @brief  Initialize the CANx as encoder
  * @param  CANx:  CANx, where x can be 1,2
  * @param  CAN_BaudRate:  CAN BaudRate, the entity is Kb, its value is one of following values:10,20,50,10,125,250,500,1000
  * @param  GPIOx: The specific pins you want to select in group GPIOx.
			ref the datasheet->(Pinouts and pin description) Pin AF to identity GPIOx's x.
  * @param  CAN_RxPin
  * @param  CAN_TxPin
			@note   do not use this function to inialize CAN pin, PI10, PH13
  * @retval None
  * @author Calcus Lee
**/
void CAN_Config(CAN_TypeDef *CANx,
				uint32_t CAN_BaudRate,
				GPIO_TypeDef *GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint8_t CAN_RxSource = 0;
	uint8_t CAN_TxSource = 0;
	uint8_t GPIO_AF_CANx = 0;

	/* CAN GPIOs configuration */

	//确定CAN_RxPin

	switch (CAN_RxPin)
	{
	case GPIO_Pin_0:
	{
		CAN_RxSource = GPIO_PinSource0;
		break;
	}
	case GPIO_Pin_1:
	{
		CAN_RxSource = GPIO_PinSource1;
		break;
	}
	case GPIO_Pin_2:
	{
		CAN_RxSource = GPIO_PinSource2;
		break;
	}
	case GPIO_Pin_3:
	{
		CAN_RxSource = GPIO_PinSource3;
		break;
	}
	case GPIO_Pin_4:
	{
		CAN_RxSource = GPIO_PinSource4;
		break;
	}
	case GPIO_Pin_5:
	{
		CAN_RxSource = GPIO_PinSource5;
		break;
	}
	case GPIO_Pin_6:
	{
		CAN_RxSource = GPIO_PinSource6;
		break;
	}
	case GPIO_Pin_7:
	{
		CAN_RxSource = GPIO_PinSource7;
		break;
	}
	case GPIO_Pin_8:
	{
		CAN_RxSource = GPIO_PinSource8;
		break;
	}
	case GPIO_Pin_9:
	{
		CAN_RxSource = GPIO_PinSource9;
		break;
	}
	case GPIO_Pin_10:
	{
		CAN_RxSource = GPIO_PinSource10;
		break;
	}
	case GPIO_Pin_11:
	{
		CAN_RxSource = GPIO_PinSource11;
		break;
	}
	case GPIO_Pin_12:
	{
		CAN_RxSource = GPIO_PinSource12;
		break;
	}
	case GPIO_Pin_13:
	{
		CAN_RxSource = GPIO_PinSource13;
		break;
	}
	case GPIO_Pin_14:
	{
		CAN_RxSource = GPIO_PinSource14;
		break;
	}
	case GPIO_Pin_15:
	{
		CAN_RxSource = GPIO_PinSource15;
		break;
	}

	default:
		break;
	}

	//确定CAN_TxPin
	switch (CAN_TxPin)
	{
	case GPIO_Pin_0:
	{
		CAN_TxSource = GPIO_PinSource0;
		break;
	}
	case GPIO_Pin_1:
	{
		CAN_TxSource = GPIO_PinSource1;
		break;
	}
	case GPIO_Pin_2:
	{
		CAN_TxSource = GPIO_PinSource2;
		break;
	}
	case GPIO_Pin_3:
	{
		CAN_TxSource = GPIO_PinSource3;
		break;
	}
	case GPIO_Pin_4:
	{
		CAN_TxSource = GPIO_PinSource4;
		break;
	}
	case GPIO_Pin_5:
	{
		CAN_TxSource = GPIO_PinSource5;
		break;
	}
	case GPIO_Pin_6:
	{
		CAN_TxSource = GPIO_PinSource6;
		break;
	}
	case GPIO_Pin_7:
	{
		CAN_TxSource = GPIO_PinSource7;
		break;
	}
	case GPIO_Pin_8:
	{
		CAN_TxSource = GPIO_PinSource8;
		break;
	}
	case GPIO_Pin_9:
	{
		CAN_TxSource = GPIO_PinSource9;
		break;
	}
	case GPIO_Pin_10:
	{
		CAN_TxSource = GPIO_PinSource10;
		break;
	}
	case GPIO_Pin_11:
	{
		CAN_TxSource = GPIO_PinSource11;
		break;
	}
	case GPIO_Pin_12:
	{
		CAN_TxSource = GPIO_PinSource12;
		break;
	}
	case GPIO_Pin_13:
	{
		CAN_TxSource = GPIO_PinSource13;
		break;
	}
	case GPIO_Pin_14:
	{
		CAN_TxSource = GPIO_PinSource14;
		break;
	}
	case GPIO_Pin_15:
	{
		CAN_TxSource = GPIO_PinSource15;
		break;
	}

	default:
		break;
	}
	/* CANx clock source enable */
	switch ((uint32_t)CANx)
	{
	//CANs on APB1
	case CAN1_BASE:
	{
		GPIO_AF_CANx = GPIO_AF_CAN1;
		CAN_FilterInitStructure.CAN_FilterNumber = 0; //Filter 0
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		break;
	}
	case CAN2_BASE:
	{
		GPIO_AF_CANx = GPIO_AF_CAN2;
		CAN_FilterInitStructure.CAN_FilterNumber = 14; //Filter 14
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		break;
	}

	default:
		break;
	}
	/* Enable GPIOx, clock */
	switch ((uint32_t)GPIOx)
	{
	case GPIOA_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;
	}
	case GPIOB_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
	}
	case GPIOC_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
	}
	case GPIOD_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		break;
	}
	case GPIOE_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		break;
	}
	case GPIOF_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		break;
	}
	case GPIOG_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		break;
	}
	case GPIOH_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		break;
	}
	case GPIOI_BASE:
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		break;
	}

	default:
		break;
	}

	/* Connect CAN pins to AF */
	GPIO_PinAFConfig(GPIOx, CAN_RxSource, GPIO_AF_CANx);
	GPIO_PinAFConfig(GPIOx, CAN_TxSource, GPIO_AF_CANx);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = CAN_RxPin | CAN_TxPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	/* CAN register init */
	CAN_DeInit(CANx);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;											//time triggered communication mode
	CAN_InitStructure.CAN_ABOM = ENABLE;											//automatic bus-off management
	CAN_InitStructure.CAN_AWUM = DISABLE;											//automatic wake-up mode
	CAN_InitStructure.CAN_NART = DISABLE;											//non-automatic retransmission mode
	CAN_InitStructure.CAN_RFLM = DISABLE;											//Receive FIFO Locked mode
	CAN_InitStructure.CAN_TXFP = DISABLE;											//transmit FIFO priority
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;									//CAN operating mode
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;										// keep CAN_SJW == 1, never change it
	CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;										//max=16
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;										//max=8
	/* CAN Baudrate =APB1_CLK/((CAN_SJW_tq+CAN_BS1_tq+CAN_BS2_tq)*CAN_Prescaler) */ //?
	switch (CAN_BaudRate)
	{
	case 10:
	{
		CAN_InitStructure.CAN_Prescaler = 200;
		break;
	}
	case 20:
	{
		CAN_InitStructure.CAN_Prescaler = 100;
		break;
	}
	case 50:
	{
		CAN_InitStructure.CAN_Prescaler = 40;
		break;
	}
	case 100:
	{
		CAN_InitStructure.CAN_Prescaler = 20;
		break;
	}
	case 125:
	{
		CAN_InitStructure.CAN_Prescaler = 16;
		break;
	}
	case 250:
	{
		CAN_InitStructure.CAN_Prescaler = 8;
		break;
	}
	case 500:
	{
		CAN_InitStructure.CAN_Prescaler = 4;
		break;
	}
	case 1000:
	{
		CAN_InitStructure.CAN_Prescaler = 2;
		break;
	}

	default:
		break;
	}
	CAN_Init(CANx, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32 Bit
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;				 //32 Bis ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //32 Bit Mask
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //activate Filter
	CAN_FilterInit(&CAN_FilterInitStructure);			   //intialize Filter

	/* Enable FIFO 0 message pending Interrupt */

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief  transmit an array 
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  StdId: the StdId you want to select as StdId.
  * @param  Msg:   a pointer that point the array you want to transmit.
  * @param  len:   the length of the the array that you want to transmit.
  * @retval 1, if transmit successful. 
  * @author Calcus Lee
**/
uint8_t CAN_TxMsg(CAN_TypeDef *CANx,
				  uint32_t StdId,
				  uint8_t *Msg,
				  uint8_t len)
{
	uint8_t mbox;
	uint16_t i = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = StdId;		 // standard identifier=0
	TxMessage.ExtId = StdId;		 // extended identifier=StdId
	TxMessage.IDE = CAN_Id_Standard; // type of identifier for the message is Standard
	TxMessage.RTR = CAN_RTR_Data;	// the type of frame for the message that will be transmitted
	TxMessage.DLC = len;			 // 发送两帧信息
	for (i = 0; i < len; i++)
		TxMessage.Data[i] = Msg[i]; // 第一帧信息

	mbox = CAN_Transmit(CANx, &TxMessage);
	i = 0;
	uint16_t timeOut = 0;
	while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok))
	{
		timeOut++;
		if(timeOut>=60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;
		}
	}
	return 1;
}

/**
  * @brief  Receive an array 
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  StdId: the StdId you want to select as StdId.
  * @param  Msg:   a array you want to transmit.
  * @param  len:   the length of the the array that you want to transmit.
  * @retval 1, if receive successful
  * @author Calcus Lee
**/
uint8_t CAN_RxMsg(CAN_TypeDef *CANx,
				  uint32_t *StdId,
				  uint8_t *buf,
				  uint8_t *len)
{
	uint8_t i = 0;
	CanRxMsg RxMessage;
	if (CAN_MessagePending(CANx, CAN_FIFO0) == 0)
	{
		return 0; //if there is no data, get out of this function
	}
	CAN_Receive(CANx, CAN_FIFO0, &RxMessage); //reveive data
	for (i = 0; i < RxMessage.DLC; i++)
	{
		buf[i] = RxMessage.Data[i];
	}
	*len = RxMessage.DLC;
	*StdId = RxMessage.StdId;

	return 1;
}
/**
  * @brief  利用操作系统互斥型信号量管理CAN发送资源，CAN发送函数
  * @param  CANx:  CANx, where x can be 1,2.
  * @param  TxMessage:   a array you want to transmit.
  * @retval CAN_SEND_OK(whose value is 1), if receive successful
  * @retval CAN_SEND_ERR(which value is -1), if receive unsuccessful
**/
//extern OS_EVENT *CANSendMutex;

//int OSCANSendCmd(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
//{
//	CPU_INT08U os_err;
//	static uint8_t mailBox = INVALID_CANSEND_MAILBOX;
//	//等待互斥型信号量
//	OSMutexPend(CANSendMutex,0,&os_err);
//	//发送CAN消息
//	mailBox = CAN_Transmit(CANx,TxMessage);
//	uint16_t timeout = 0;
//	while(!(CAN_TransmitStatus(CANx,mailBox) == CAN_TxStatus_Ok))
//	{
//		timeout++;
//		if(timeout > 60000)
//		{

//		}
//	}
//	OSMutexPost(CANSendMutex);
//	return CAN_SEND_OK;
//}
