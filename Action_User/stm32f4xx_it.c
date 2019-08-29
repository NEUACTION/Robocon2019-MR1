/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "moveBase.h"
#include "dma.h"
#include "robot.h"
#include "iwdg.h"
#include "steerReset.h"
#include "telecontroller.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

//	uint8_t buffer[8];
	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	uint8_t receiveLength = 0;
	union CanReceive
	{
		uint8_t data8[8];
		int data32[2];
		float dataf[2];
	}canReceiveMsg;
//	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	CAN_RxMsg(CAN1, &StdId, canReceiveMsg.data8, &receiveLength);
	
	canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;

//	if(canReceiveMsg.data32[0]==0x00111111)
//	{
//		
//	}
//	
//	if(canReceiveMsg.data32[0]==0x00111112)
//	{

//	}
	
	switch(canNodeId)
	{
		case  LEFT_FRONT_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
//				gRobot.wheelState.leftFrontAct.direction = WheelAngle2PositionInverseTransform(canReceiveMsg.data32[1]);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.leftFrontAct.vel = Pulse2Vel(canReceiveMsg.data32[1]);
				if(gRobot.wheelState.leftFrontHB<1)
				{
					gRobot.wheelState.leftFrontHB=1;
				}
				gRobot.wheelState.leftFrontHB--;
				gRobot.wheelState.leftFrontHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"1 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.leftFrontHB);
			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.leftFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{					
					gRobot.wheelState.leftFrontHF|=WHEEL_LOST_ENCODER;
				}
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			if(canReceiveMsg.data32[0]==0x00001111)
			{
				gRobot.wheelState.leftFrontOverCycle = canReceiveMsg.data32[1];
			}
			break;
		case  RIGHT_FRONT_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
//				gRobot.wheelState.rightFrontAct.direction = canReceiveMsg.data32[1];
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.rightFrontAct.vel = Pulse2Vel(canReceiveMsg.data32[1]);
				if(gRobot.wheelState.rightFrontHB<1)
				{
					gRobot.wheelState.rightFrontHB=1;
				}
				gRobot.wheelState.rightFrontHB--;
				gRobot.wheelState.rightFrontHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"2 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.rightFrontHB);
			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.rightFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.rightFrontHF|=WHEEL_LOST_ENCODER;
				}
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 2 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case  LEFT_REAR_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
//				gRobot.wheelPosition.wheelPosition4 = canReceiveMsg.data32[1];
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.leftRearAct.vel = Pulse2Vel(canReceiveMsg.data32[1]);
				if(gRobot.wheelState.leftRearHB<1)
				{
					gRobot.wheelState.leftRearHB=1;
				}
				gRobot.wheelState.leftRearHB--;
				gRobot.wheelState.leftRearHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"3 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.leftRearHB);
			}

			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.leftRearHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.leftRearHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 3 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case  RIGHT_REAR_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
//				gRobot.wheelPosition.wheelPosition1 = canReceiveMsg.data32[1];
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.rightRearAct.vel = Pulse2Vel(canReceiveMsg.data32[1]);
				if(gRobot.wheelState.rightRearHB<1)
				{
					gRobot.wheelState.rightRearHB=1;
				}
				gRobot.wheelState.rightRearHB--;
				gRobot.wheelState.rightRearHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"4 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.rightRearHB);
			}
			
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.rightRearHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.rightRearHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			if(canReceiveMsg.data32[0]==0x00001111)
			{
				gRobot.wheelState.rightRearOverCycle = canReceiveMsg.data32[1];
			}
			break;
		case LEFT_FRONT_TURNING_ID:
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.wheelState.leftFrontAct.direction = WheelAngle2PositionInverseTransform(canReceiveMsg.data32[1] , gRobot.wheelState.steerLoopShift.lf);
				if(gRobot.wheelState.leftFrontTurnHB<1)
				{
					gRobot.wheelState.leftFrontTurnHB=1;
				}
				gRobot.wheelState.leftFrontTurnHB--;
				gRobot.wheelState.leftFrontTurnHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"5 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.leftFrontTurnHB);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.turnVel.lf = (float)(((float)canReceiveMsg.data32[1] / COUNTS_PER_ROUND)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
			}
			
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.leftFrontTurnHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.leftFrontTurnHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case RIGHT_FRONT_TURNING_ID:
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.wheelState.rightFrontAct.direction = WheelAngle2PositionInverseTransform(canReceiveMsg.data32[1] , gRobot.wheelState.steerLoopShift.rf);
				if(gRobot.wheelState.rightFrontTurnHB<1)
				{
					gRobot.wheelState.rightFrontTurnHB=1;
				}
				gRobot.wheelState.rightFrontTurnHB--;
				gRobot.wheelState.rightFrontTurnHB_Continious = 0;

//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"6 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.rightFrontTurnHB);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.turnVel.rf = (float)(((float)canReceiveMsg.data32[1] / COUNTS_PER_ROUND)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
			}

			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.rightFrontTurnHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.rightFrontTurnHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case LEFT_REAR_TURNING_ID:
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.wheelState.leftRearAct.direction = WheelAngle2PositionInverseTransform(canReceiveMsg.data32[1] , gRobot.wheelState.steerLoopShift.lr);
				if(gRobot.wheelState.leftRearTurnHB<1)
				{
					gRobot.wheelState.leftRearTurnHB=1;
				}
				gRobot.wheelState.leftRearTurnHB--;
				gRobot.wheelState.leftRearTurnHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"7 %d %d \r\n",(int)(GetTimeCounter()/100),(int)gRobot.wheelState.leftRearTurnHB);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.turnVel.lr = (float)(((float)canReceiveMsg.data32[1] / COUNTS_PER_ROUND)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
			}
			
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.leftRearTurnHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.leftRearTurnHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case RIGHT_REAR_TURNING_ID:
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.wheelState.rightRearAct.direction = WheelAngle2PositionInverseTransform(canReceiveMsg.data32[1] , gRobot.wheelState.steerLoopShift.rr);
				if(gRobot.wheelState.rightRearTurnHB<1)
				{
					gRobot.wheelState.rightRearTurnHB=1;
				}
				gRobot.wheelState.rightRearTurnHB--;
				gRobot.wheelState.rightRearTurnHB_Continious = 0;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"8 %d %d \r\n",(int)GetTimeCounter(),(int)gRobot.wheelState.rightRearTurnHB);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{
				gRobot.wheelState.turnVel.rr = (float)(((float)canReceiveMsg.data32[1] / COUNTS_PER_ROUND)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
			}
			
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
					gRobot.wheelState.rightRearTurnHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
				if(canReceiveMsg.data32[1]==0xcccccccc)
				{
					gRobot.wheelState.rightRearTurnHF|=WHEEL_LOST_ENCODER;
				}
			}
			
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		default:
			break;
	}

	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

//	uint8_t buffer[8];
	uint32_t StdId=0;
	uint8_t canNodeId = 0;
	uint8_t receiveLength = 0;
	union CanReceive
	{
		uint8_t data8[8];
		int data32[2];
		float dataf[2];
	}canReceiveMsg;
//	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	CAN_RxMsg(CAN2, &StdId, canReceiveMsg.data8, &receiveLength);
	
	canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;

	
	switch(canNodeId)
	{
		case  ARM_MOTOR_ID_BLUE:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.robotMotorState.actArmPosBlue = ArmAngle2PositionInverseTransform(canReceiveMsg.data32[1]);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{

			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
//					gRobot.wheelState.leftFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
//					gRobot.wheelState.leftFrontHF|=WHEEL_LOST_ENCODER;
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case  ARM_MOTOR_ID_RED:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.robotMotorState.actArmPosRed = ArmAngle2PositionInverseTransform(canReceiveMsg.data32[1]);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{

			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
//					gRobot.wheelState.leftFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
//					gRobot.wheelState.leftFrontHF|=WHEEL_LOST_ENCODER;
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case  SHOOT_PITCH_LEFT_MOTOR_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.robotMotorState.actLeftPitchPos = ShootAngle2PositionInverseTransform(canReceiveMsg.data32[1]);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{

			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
//					gRobot.wheelState.leftFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
//					gRobot.wheelState.leftFrontHF|=WHEEL_LOST_ENCODER;
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;
		case  SHOOT_PITCH_RIGHT_MOTOR_ID:
			
			//位置
			if(canReceiveMsg.data32[0]==0x00005850)
			{
				gRobot.robotMotorState.actRightPitchPos = ShootAngle2PositionInverseTransform(canReceiveMsg.data32[1]);
			}
		
			//速度
			if(canReceiveMsg.data32[0]==0x00005856)
			{

			}
			if(canReceiveMsg.data32[0]==0x22222222)
			{
				if(canReceiveMsg.data32[1]==0x22222222)
				{
//					gRobot.wheelState.leftFrontHF|=WHEEL_INTO_HF;
				}
			}
			if(canReceiveMsg.data32[0]==0xcccccccc)
			{
//					gRobot.wheelState.leftFrontHF|=WHEEL_LOST_ENCODER;
			}
			if(canReceiveMsg.data32[0]==*(uint32_t *)"STAW")
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",canReceiveMsg.data32[1]);
			}
			break;

		default:
			break;
	}

	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern OS_EVENT *PeriodSem;
extern OS_EVENT *velCtrPeriodSem;
extern OS_EVENT *gunPeriodSem;


void TIM2_IRQHandler(void)
{
#define PERIOD_COUNTER (20)
#define VEL_CTR_PERIOD (10)

	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;
	static uint8_t velCtrPeriod = VEL_CTR_PERIOD;

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{

		//实现10ms 发送1次信号量
		periodCounter--;
		velCtrPeriod--;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			OSSemPost(gunPeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		if(velCtrPeriod==0)
		{
			OSSemPost(velCtrPeriodSem);
			velCtrPeriod = VEL_CTR_PERIOD;
			CountTime();
		}
		IWDG_Feed();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}

/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t count=0;

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		ch = USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		
		if(gRobot.debugInfomation.SDCardLostFlag == 0)
		{
			switch(count)
			{
				case 0:
					if(ch=='A')
						count++;
					else
						count=0;
				break;

				case 1:
					if(ch=='T')
						count++;
					else
						count=0;
				break;

				case 2:
					if(ch=='+')
						count++;
					else
						count=0;
				break;
					
				case 3:
					if(ch=='S')
						count++;
					else
						count=0;
				break;	
					
				case 4:
					if(ch=='D')
						count++;
					else
						count=0;
				break;
					
				case 5:
					if(ch=='\r')
						count++;
					else
						count=0;
				break;

				case 6:
					if(ch=='\n')
					{
						count++;
						gRobot.debugInfomation.SDCardLostFlag = 1;
					}
					else
						count=0;
				break;
					
				default:
					count=0;
				break;		 
			}
		}
	}
	else
	{
		USART_ClearITPendingBit(USART1, USART_IT_PE);
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		USART_ClearITPendingBit(USART1, USART_IT_LBD);
		USART_ClearITPendingBit(USART1, USART_IT_CTS);
		USART_ClearITPendingBit(USART1, USART_IT_ERR);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART1, USART_IT_NE);
		USART_ClearITPendingBit(USART1, USART_IT_FE);
		USART_ReceiveData(USART1);
	}
	OSIntExit();
}

void USART2_IRQHandler(void)
{
	static uint8_t ch;
	
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		
		ch = USART_ReceiveData(USART2);
		
		if(gRobot.gun.mode == GUN_MANUAL_MODE)
		{
			//将收到的数据入队
			GunMsgBufferInput(ch);
		}
		else
		{
			TeleCmdBufferInput_BT(ch);
		}
	}
	else
	{
		USART_ClearITPendingBit(USART2, USART_IT_PE);
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		USART_ClearITPendingBit(USART2, USART_IT_LBD);
		USART_ClearITPendingBit(USART2, USART_IT_CTS);
		USART_ClearITPendingBit(USART2, USART_IT_ERR);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART2, USART_IT_NE);
		USART_ClearITPendingBit(USART2, USART_IT_FE);
		USART_ReceiveData(USART2);
	}
	OSIntExit();
}

//void USART6_IRQHandler(void) //更新频率200Hz
//{
//	static uint8_t ch;
//	static union {
//		uint8_t data[24];
//		float ActVal[6];
//	} posture;
//	static uint8_t count = 0;
//	static uint8_t i = 0;
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//		ch = USART_ReceiveData(USART6);
//		switch (count)
//		{
//		case 0:
//			if (ch == 0x0d)
//				count++;
//			else
//				count = 0;
//			break;

//		case 1:
//			if (ch == 0x0a)
//			{
//				i = 0;
//				count++;
//			}
//			else if (ch == 0x0d)
//				;
//			else
//				count = 0;
//			break;

//		case 2:
//			posture.data[i] = ch;
//			i++;
//			if (i >= 24)
//			{
//				i = 0;
//				count++;
//			}
//			break;

//		case 3:
//			if (ch == 0x0a)
//				count++;
//			else
//				count = 0;
//			break;

//		case 4:
//			if (ch == 0x0d)
//			{

//				posture.ActVal[0] = posture.ActVal[0];
//				posture.ActVal[1] = posture.ActVal[1];
//				posture.ActVal[2] = posture.ActVal[2];
//				posture.ActVal[3] = posture.ActVal[3];
//				posture.ActVal[4] = posture.ActVal[4];
//				posture.ActVal[5] = posture.ActVal[5];
//			}
//			count = 0;
//			break;

//		default:
//			count = 0;
//			break;
//		}
//	}
//	else
//	{
//		USART_ClearITPendingBit(USART6, USART_IT_PE);
//		USART_ClearITPendingBit(USART6, USART_IT_TXE);
//		USART_ClearITPendingBit(USART6, USART_IT_TC);
//		USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
//		USART_ClearITPendingBit(USART6, USART_IT_IDLE);
//		USART_ClearITPendingBit(USART6, USART_IT_LBD);
//		USART_ClearITPendingBit(USART6, USART_IT_CTS);
//		USART_ClearITPendingBit(USART6, USART_IT_ERR);
//		USART_ClearITPendingBit(USART6, USART_IT_ORE_ER);
//		USART_ClearITPendingBit(USART6, USART_IT_NE);
//		USART_ClearITPendingBit(USART6, USART_IT_FE);
//		USART_ReceiveData(USART6);
//	}
//	OSIntExit();
//}

//void USART3_IRQHandler(void)
//{
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	}

//	OSIntExit();
//}

void UART5_IRQHandler(void)
{
	char data = 0;
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		data = USART_ReceiveData(UART5);
		LoopMsgBufferInput(data);
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
	else
	{
		USART_ClearITPendingBit(UART5, USART_IT_PE);
		USART_ClearITPendingBit(UART5, USART_IT_TXE);
		USART_ClearITPendingBit(UART5, USART_IT_TC);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART5, USART_IT_IDLE);
		USART_ClearITPendingBit(UART5, USART_IT_LBD);
		USART_ClearITPendingBit(UART5, USART_IT_CTS);
		USART_ClearITPendingBit(UART5, USART_IT_ERR);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART5, USART_IT_NE);
		USART_ClearITPendingBit(UART5, USART_IT_FE);
		USART_ReceiveData(UART5);
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"NMI Exception! ");
	}
}

void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
  char        hex[] = "0123456789ABCDEF";
  char        *pStr = s;
  for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
  {
    
    /*
    1.*pStr++右结合,并且*索引的是没有++之前的地址
    2.f.移位不会改变指针指向的那个空间的值
    3.对指针指向空间的移位也不会改变指针的指向
    */
    if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
      *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
    *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
  }
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/*屏蔽掉的只是为了返回进硬件中断的错误语句，但是比赛的时候不可能出现错误，只可能有静电等突发因素，所以不用返回了*/
	static uint32_t r_sp ;

	OutputVel2Wheel(0.0f,0.0f,0.0f);

	/*判断发生异常时使用MSP还是PSP*/
	if(__get_PSP()!=0x00) //获取SP的值
		r_sp = __get_PSP(); 
	else
		r_sp = __get_MSP(); 
	
	/*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
	r_sp = r_sp+0x10;
	
	/*串口发数通知*/
	char sPoint[2]={0};
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"%s","0x");
	
	/*获取出现异常时程序的地址*/
	for(int i=3;i>=-28;i--){
		Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"%s",sPoint);
		
		if(i%4==0)
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\n");		
	}
	/*发送回车符*/
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"\r\n");		
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"Hard Fault\r\n");
		
		OutputVel2Wheel(0.0f,0.0f,0.0f);
		
		delay_ms(10);
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Memory Manage Exception! ");
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Bus Fault Exception! ");		
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Usage Fault Exception! ");
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	while(1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"SVCall Exception! ");
	}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	while(1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"Debug Monitor Exception! ");
	}
}
