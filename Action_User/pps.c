/**
  ******************************************************************************
  * @file     
  * @author  lxy and qzj and xfr
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "string.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_it.h"
#include  <ucos_ii.h>
#include "timer.h"
#include "pps.h"
#include "moveBase.h"
#include "motion.h"
#include "math.h"
#include "usart.h"
#include "robot.h"
#include "light.h"
#include "dma.h"
#include "robot.h"

/*告诉定位系统准备开始积分*/
static uint8_t ppsTalkOk = 0;
/*定位系统准备完毕开始发数*/
static uint8_t ppsReady = 0;
/*定义定位系统返回值结构体*/
static Pos_t ppsReturn={0.f};

static uint8_t laserTalkOk = 0;

//extern uint8_t	co3RDFlag, co5THFlag;
uint8_t	send3RDOK = 0, send5THOK = 0;
uint8_t	co3RDOK = 0, co5THOK = 0;

laserValue_t laserValue;

void USART6_IRQHandler(void)
{
		static uint8_t ch;
		static uint8_t count=0;
		static uint8_t i=0;
		OS_CPU_SR  cpu_sr;
		OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
		OSIntNesting++;
		OS_EXIT_CRITICAL();

		static PosSend_t posture={0};
		static LaserSend_t laser={0};
	
		if(USART_GetITStatus(USART6, USART_IT_RXNE)==SET)   
		{
			USART_ClearITPendingBit(USART6,USART_IT_RXNE);
			ch=USART_ReceiveData(USART6);
//			USART_SendData(UART4,ch);
			switch(count)
			{
				case 0:
					if(ch==0x0d||ch=='O')
						count++;
					else
						count=0;
				break;

				case 1:
					if(ch==0x0a)
					{
						i=0;
						count++;
					}
					else if(ch=='K')
					{
						laserTalkOk = 1;
						ppsTalkOk = 1;
						count=0;
					}
					else if(ch=='3')
					{
						send3RDOK = 1;
						count=0;
					}
					else if(ch=='5')
					{
						send5THOK = 1;
						count=0;
					}
					else if(ch=='R')
					{
						co3RDOK = 1;
						count=0;
					}
					else if(ch=='Z')
					{
						co5THOK = 1;
						count=0;
						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"OZGET\r\n");	
					}
				else if(ch==0x0d);
				else
					count=0;
				break;

				case 2:
					if(gRobot.selfCheckFlag==0)
					{
						posture.data[i]=ch;
						i++;
						if(i>=GET_PPS_DATA_NUM)
						{
							i=0;
							count++;
						}
					}
					else if(gRobot.selfCheckFlag==1)
					{
						//自检 接收激光值信息
						laser.data[i]=ch;
						i++;
						if(i>=GET_LASER_DATA_NUM)
						{
							i=0;
							count++;
						}
					}
				break;
					
				case 3:
					if(ch==0x0a)
						count++;
					else
						count=0;
				break;	
					
				case 4:
					if(ch==0x0d)
					{				
						SetLaserOpsReady(1);
						
						if(gRobot.selfCheckFlag==0)
						{
//							/*传入定位系统返回的值*/
//							SetX(posture.value[3]);
//							SetY(posture.value[4]);
//							SetAngle(posture.value[0]);
//							//set速度时要加负号！！！
//							SetSpeedX(-posture.value[1]);
//							SetSpeedY(-posture.value[2]);
//							SetWZ(posture.value[5]);
							//判断坐标是否正确 如果不正确重新预估计算赋值
							CheckPos2(posture);
						}
						else if(gRobot.selfCheckFlag==1)
						{
							SetX(laser.value[0]);
							SetY(laser.value[1]);
							SetAngle(laser.value[2]);
							laserValue.laserOne=laser.value[3];
							laserValue.laserTwo=laser.value[4];
							laserValue.laserThr=laser.value[5];
							laserValue.laserFor=laser.value[6];
							laserValue.laserFiv=laser.value[7];
							laserValue.laserSix=laser.value[8];
							laserValue.laserSev=laser.value[9];
							laserValue.laserEig=laser.value[10];
						}
						
						//计算路程
//						CaculatePath();
						/*定义的全局结构体变量可以在这里赋值*/
						//						=posture.value[0];
						//						=posture.value[1];
						//						=posture.value[2];
						//						=posture.value[3];
						//						=posture.value[4];
						//						=posture.value[5];
					}
					count=0;
				break;

				default:
					count=0;
				break;		 
			}
		}
		else
		{
			USART_ClearITPendingBit(USART6, USART_IT_PE);
			USART_ClearITPendingBit(USART6, USART_IT_TXE);
			USART_ClearITPendingBit(USART6, USART_IT_TC);
			USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
			USART_ClearITPendingBit(USART6, USART_IT_IDLE);
			USART_ClearITPendingBit(USART6, USART_IT_LBD);
			USART_ClearITPendingBit(USART6, USART_IT_CTS);
			USART_ClearITPendingBit(USART6, USART_IT_ERR);
			USART_ClearITPendingBit(USART6, USART_IT_ORE_ER);
			USART_ClearITPendingBit(USART6, USART_IT_NE);
			USART_ClearITPendingBit(USART6, USART_IT_FE);
			USART_ReceiveData(USART6);
		}
		
		OSIntExit();
}

static float continiousAngle = 0.0f;
static int angleLoop = 0;
void SetContiniousAngle(float value)
{
	static float angleRecord = 0.0f;
	
	if(value<-135.0f&&angleRecord>135.0f)
	{
		angleLoop++;
	}
	
	if(value>135.0f&&angleRecord<-135.0f)
	{
		angleLoop--;
	}
	
	angleRecord = value;
	continiousAngle = value + angleLoop*360.0f;
}

int GetAngleLoopNum(void)
{
	return angleLoop;
}

float GetContiniousAngle(void)
{
	return continiousAngle;
}

void SetAngle(float setValue)
{
	SetContiniousAngle(setValue);
	ppsReturn.ppsAngle = setValue;
}

void SetX(float setValue)
{
//	ppsReturn.ppsX = -setValue + DISY_OPS2CENTER * sinf(ANGLE2RAD(ppsReturn.ppsAngle));
	ppsReturn.ppsX = setValue;
}

void SetY(float setValue)
{
//	ppsReturn.ppsY = -setValue - DISY_OPS2CENTER * cosf(ANGLE2RAD(ppsReturn.ppsAngle)) + DISY_OPS2CENTER;
		ppsReturn.ppsY = setValue;
}

void SetSpeedX(float setValue)
{
//	ppsReturn.ppsSpeedX = -setValue;
	ppsReturn.ppsSpeedX = setValue;
}

void SetSpeedY(float setValue)
{
//	ppsReturn.ppsSpeedY = -setValue;
	ppsReturn.ppsSpeedY = setValue;
}

void SetWZ(float setValue)
{
	ppsReturn.ppsWZ = setValue;
}


/*返回定位系统的角度*/
float GetAngle(void)
{
	return ReturnLimitAngle(ppsReturn.ppsAngle + gRobot.manualPos.angleAdjust);
}
/*返回定位系统的X值*/
float GetX(void)
{
	return ppsReturn.ppsX;
}
/*返回定位系统的Y值*/
float GetY(void)
{
	return ppsReturn.ppsY;
}
/*返回定位系统的X轴的速度*/
float GetSpeedX(void)
{
	return ppsReturn.ppsSpeedX;
}
/*返回定位系统的角度*/
float GetSpeedY(void)
{
	return ppsReturn.ppsSpeedY;
}
/*返回定位系统的Z轴角速度值*/
float GetWZ(void)
{
	return ppsReturn.ppsWZ;
}

//返回减去绕机器人中心旋转角速度产生的线速度后的速度
position_t GetSpeedWithoutOmega(void)
{
	position_t vel = {0.0f};
	float rotateVel , rotateVelDirection = 0.0f;
	
	rotateVel = ANGLE2RAD(GetWZ())*sqrtf(DISX_OPS2CENTER*DISX_OPS2CENTER + DISY_OPS2CENTER*DISY_OPS2CENTER);
	
	rotateVelDirection = 90.0f + RAD2ANGLE(atan2f(DISY_OPS2CENTER,DISX_OPS2CENTER)) + GetAngle();
	
	AngleLimit(&rotateVelDirection);
	
	vel.x = GetSpeedX() - rotateVel * cosf(ANGLE2RAD(rotateVelDirection));
	vel.y = GetSpeedY() - rotateVel * sinf(ANGLE2RAD(rotateVelDirection));
	
	return vel;
}


//返回减去绕机器人中心旋转角速度产生的线速度后的速度

/*  ********************************************给定位系统发数矫正，矫正X，Y，angle***************************************************** */

//告诉激光矫正板告诉定位系统X Y ANGLE清零
void TalkLaserOpsToReset(void)
{
	ppsTalkOk=0;
	while(!ppsTalkOk)
	{
		delay_ms(5);
		USART_OUT(USART6,(uint8_t *)"AR\r\n");
	}
}
//一直等待定位系统初始化完成
void WaitLaserOpsReset(void)
{
	  //告诉定位系统准备
	  TalkLaserOpsToReset();
		//等待定位系统准备完成
		SetLaserOpsReady(0);
		while(!GetLaserOpsReady()){};
}


//告诉定位系统开始准备
void TalkLaserOpsToGetReady(void)
{
	laserTalkOk=0;
	while(!laserTalkOk)
	{
		delay_ms(5);
		USART_OUT(USART6,(uint8_t *)"AT+P\r\n");
	}
}

//一直等待定位系统初始化完成
void WaitLaserOpsPrepare(void)
{
	  //告诉定位系统准备
	  TalkLaserOpsToGetReady();
		//等待定位系统准备完成
		SetLaserOpsReady(0);
		while(!GetLaserOpsReady()){};
}

void SetLaserOpsReady(uint8_t flag)
{
	ppsReady = flag;
}
uint8_t GetLaserOpsReady(void)
{
	return ppsReady;
}

//发送获取激光值命令
void SendGetLaserCmd(void)
{
	laserTalkOk=0;
	while(!laserTalkOk)
	{
		delay_ms(1);
		USART_OUT(USART6,(uint8_t *)"AT+8\r\n");
	}
}

//发送红蓝场标志
void SendCourtIDCmd(void)
{
	laserTalkOk=0;
	while(!laserTalkOk)
	{
		delay_ms(1);
		if(gRobot.courtID == BLUE_COURT)
		{
			USART_OUT(USART6,(uint8_t *)"AT+BLUE\r\n");
		}
		else if (gRobot.courtID == RED_COURT)
		{
			USART_OUT(USART6,(uint8_t *)"AT+RED\r\n");
		}
	}
}

//发送靠上3号墙的信息 来矫正取第二个块
void Send3RDWallCorrect(void)
{
	static uint8_t timeOut = 0;
	
	if(!send3RDOK)
	{
		USART_OUT(USART6,(uint8_t *)"AT+C3\r\n");
		timeOut++;
	}
	else 
	{
//		co3RDFlag = 0;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"SEND3RDOK!\r\n");	
	}
	
	if(timeOut > 3)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"3RDTIMEOUT!\r\n");	
		
		timeOut = 0;
//		co3RDFlag = 0;
	}	
}
//发送靠上5号墙的信息 来矫正取第三个块
void Send5THWallCorrect(void)
{
	static uint8_t timeOut = 0;
	
	if(!send5THOK)
	{
		timeOut++;
		USART_OUT(USART6,(uint8_t *)"AT+C5\r\n");
	}
	else 
	{
//		co5THFlag = 0;
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"SEND5THOK!\r\n");	
	}
	if(timeOut > 3)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"5THTIMEOUT!\r\n");	
		
		timeOut = 0;
//		co5THFlag = 0;
	}	
}

//发送开始校正标志
void SendBeginCorrect(void)
{
	laserTalkOk=0;
	while(!laserTalkOk)
	{
		delay_ms(1);

		USART_OUT(USART6,(uint8_t *)"AT+BG\r\n");
	}
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"PPSOKLASEROK!\r\n");
}

//发送停止校正标志
void SendStopCorrect(void)
{
	laserTalkOk=0;
	while(!laserTalkOk)
	{
		delay_ms(1);

		USART_OUT(USART6,(uint8_t *)"AT+ST\r\n");
	}
}

//判断坐标是否正确
void CheckPos1(void)
{
	float tempAngle= 0.0f, tempWZ = 0.0f,tempX = 0.0f, tempY = 0.0f, tempSpeedX = 0.0f, tempSpeedY = 0.0f;
	static float lastAngle= 0.0f, lastWZ= 0.0f, lastX = 0.0f, lastY = 0.0f, lastSpeedX = 0.0f, lastSpeedY = 0.0f;
	float wzFromAngle = 0.0f, accFromSpeed = 0.0f, velFromPos = 0.0f, t = 0.0f;
	static int lastTime;
	uint8_t posXWrong = 0, posYWrong = 0;

	tempAngle = GetAngle();
	tempWZ = GetWZ();
	tempX = GetX();
	tempY = GetY();
	tempSpeedX = GetSpeedX();
	tempSpeedY = GetSpeedY();
	
//	gRobot.debugInfomation.tempX = tempX;
//	gRobot.debugInfomation.tempY = tempY;
//	gRobot.debugInfomation.tempAngle = tempAngle;
//	gRobot.debugInfomation.tempSpeedX = tempSpeedX;
//	gRobot.debugInfomation.tempSpeedY = tempSpeedY;
//	gRobot.debugInfomation.tempWZ = tempWZ;
	
	t = (GetTimeCounter() - lastTime)*0.01f;
	if(t>0.0f)
	{
		//判断角速度是否异常
		if(tempWZ>300.0f)
		{
			tempWZ = lastWZ;
		}
		//判断角度是否异常和突变
		wzFromAngle = ReturnLimitAngle(tempAngle - lastAngle) / t;
		if(fabs(tempAngle) > 200.0f || fabs(wzFromAngle) > 300.0f)
		{
			tempAngle = ReturnLimitAngle(lastAngle + tempWZ * t);
		}
		
		//判断速度是否异常和突变
		//加速度很离散
		accFromSpeed = sqrt(pow(tempSpeedX-lastSpeedX,2) + pow((tempSpeedY-lastSpeedY),2)) / t;
//		if(sqrt(pow(tempSpeedX,2)+pow(tempSpeedY,2)) > (GetVelLimit()+10000) || accFromSpeed > (GetAccLimit() + 20000))
//		{
//			tempSpeedX = lastSpeedX;
//			tempSpeedY = lastSpeedY;
//		}
		//判断做坐标是否异常和突变
		if(gRobot.courtID == BLUE_COURT)
		{
			if(tempX < -11000.0f || tempX > 2000.0f)
			{
				posXWrong = 1;
			}
		}
		else if(gRobot.courtID == RED_COURT)
		{
			if(tempX > 11000.0f || tempX < -2000.0f)
			{
				posXWrong = 1;
			}
		}
		if(tempY > 7000.0f || tempY < -2000.0f)
		{
			posYWrong = 1;
		}
		//坐标不连续导致由坐标得来的有突变，但不能说坐标错误
		velFromPos = sqrt(pow(tempX - lastX,2) + pow(tempY - lastY,2)) / t;
//		if(posXWrong == 1 || posYWrong == 1 || velFromPos > (GetVelLimit()+10000))
//		{
//			tempX = lastX + tempSpeedX*t;
//			tempY = lastY + tempSpeedY*t;
//		}
	}	
	else
	{
		if(tempWZ>300.0f)
		{
			tempWZ = lastWZ;
		}
		if(fabs(tempAngle) > 200.0f)
		{
			tempAngle = lastAngle;
		}
//		if(sqrt(pow(tempSpeedX,2)+pow(tempSpeedY,2)) > (GetVelLimit()+20000))
//		{
//			tempSpeedX = lastSpeedX;
//			tempSpeedY = lastSpeedY;
//		}
	}
	
	lastAngle = tempAngle;
	lastWZ = tempWZ;
	lastX = tempX;
	lastY = tempY;
	lastSpeedX = tempSpeedX;
	lastSpeedY = tempSpeedY;
	lastTime = GetTimeCounter();
	
	gRobot.debugInfomation.tempX = tempX;
	gRobot.debugInfomation.tempY = tempY;
	gRobot.debugInfomation.tempAngle = tempAngle;
	gRobot.debugInfomation.tempSpeedX = tempSpeedX;
	gRobot.debugInfomation.tempSpeedY = tempSpeedY;
	gRobot.debugInfomation.tempWZ = tempWZ;
	
//	SetAngle(tempAngle);
//	SetWZ(tempWZ);
//	SetSpeedX(tempSpeedX);
//	SetSpeedY(tempSpeedY);
//	SetX(tempX);
//	SetY(tempY);
	
	gRobot.debugInfomation.wzFromAngle = wzFromAngle;
	gRobot.debugInfomation.accFromSpeed = accFromSpeed;
	gRobot.debugInfomation.velFromPos = velFromPos;
}

//判断坐标是否正确
void CheckPos2(PosSend_t pos)
{
	float tempAngle= 0.0f, tempWZ = 0.0f,tempX = 0.0f, tempY = 0.0f, tempSpeedX = 0.0f, tempSpeedY = 0.0f;
	static float lastAngle= 0.0f, lastWZ= 0.0f, lastX = 0.0f, lastY = 0.0f, lastSpeedX = 0.0f, lastSpeedY = 0.0f;
	float wzFromAngle = 0.0f, t = 0.0f;
	static int lastTime;
	float vxFromX = 0.0f, vyFromY = 0.0f;
	uint8_t posXWrong = 0, posYWrong = 0;

	tempAngle = pos.value[0];
	tempWZ = pos.value[5];
	tempX = pos.value[3];
	tempY = pos.value[4];
	tempSpeedX = -pos.value[1];
	tempSpeedY = -pos.value[2];
	
	gRobot.debugInfomation.tempX = tempX;
	gRobot.debugInfomation.tempY = tempY;
	gRobot.debugInfomation.tempAngle = tempAngle;
	gRobot.debugInfomation.tempSpeedX = tempSpeedX;
	gRobot.debugInfomation.tempSpeedY = tempSpeedY;
	gRobot.debugInfomation.tempWZ = tempWZ;
	
	if(gRobot.walkStatus != standBy && gRobot.walkStatus != retryState && gRobot.walkStatus != startFromRetry && gRobot.walkStatus != testPara)
	{	
		t = (GetTimeCounter() - lastTime)*0.01f;
		
		//判断角速度是否异常
		if(tempWZ>1000.0f)
		{
			tempWZ = lastWZ;
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"WZ\r\n");
		}
		
		//判断速度是否异常
//		if(sqrt(pow(tempSpeedX,2)+pow(tempSpeedY,2)) > (GetVelLimit()+5000))
//		{
//			tempSpeedX = lastSpeedX;
//			tempSpeedY = lastSpeedY;
//			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"SPEED\r\n");
//		}
		
		//判断做坐标是否异常
		if(gRobot.courtID == BLUE_COURT)
		{
			if(tempX < -11000.0f || tempX > 2000.0f)
			{
				posXWrong = 1;
			}
		}
		else if(gRobot.courtID == RED_COURT)
		{
			if(tempX > 11000.0f || tempX < -2000.0f)
			{
				posXWrong = 1;
			}
		}
		if(tempY > 7000.0f || tempY < -2000.0f)
		{
			posYWrong = 1;
		}
		
		if(t>0.0f)
		{
			//判断角度是否异常		因为是10ms判断一回所以不会出现优弧的情况
			wzFromAngle = ReturnLimitAngle(tempAngle - lastAngle) / t;
			if(fabs(tempAngle) > 200.0f || fabs(wzFromAngle) > 1000.0f)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"tANGLE:%d\r\n",(int)tempAngle);
				tempAngle = ReturnLimitAngle(lastAngle + tempWZ * t);
			}
			
			//判断做坐标是否突变
			vxFromX = (tempX - lastX) / t;
			vyFromY = (tempY - lastY) / t;
			if(fabs(vxFromX) > 10000.0f)
			{
				posXWrong = 1;
			}
			if(fabs(vyFromY) > 10000.0f)
			{
				posYWrong = 1;
			}
			
			if(posXWrong == 1)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"tPOS_X:%d\r\n",(int)tempX);
				tempX = lastX + tempSpeedX*t;
			}
			if(posYWrong == 1)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"tPOS_Y:%d\r\n",(int)tempY);
				tempY = lastY + tempSpeedY*t;
			}
			
			vxFromX = (tempX - lastX)/t;
			vyFromY = (tempY - lastY)/t;
			
			if(fabs(vxFromX) < 100.f && fabs(tempSpeedX) > 2000.f)
			{
	//			tempX = lastX + tempSpeedX*t;
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"tLASERUC_X\r\n");
			}
			if(fabs(vyFromY) < 100.f && fabs(tempSpeedY) > 2000.f)
			{
	//			tempY = lastY + tempSpeedY*t;
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"tLASERUC_Y\r\n");
			}
			
		}	
		else
		{
			if(fabs(tempAngle) > 200.0f)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"ANGLE:%d\r\n",(int)tempAngle);
				tempAngle = lastAngle;
			}
			if(posXWrong == 1)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"POS_X:%d\r\n",(int)tempX);
				tempX = lastX;
			}
			if(posYWrong == 1)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"POS_Y:%d\r\n",(int)tempY);
				tempY = lastY;
			}
		}
	}
	
	lastAngle = tempAngle;
	lastWZ = tempWZ;
	lastX = tempX;
	lastY = tempY;
	lastSpeedX = tempSpeedX;
	lastSpeedY = tempSpeedY;
	lastTime = GetTimeCounter();
	
	SetAngle(tempAngle);
	SetWZ(tempWZ);
	SetSpeedX(tempSpeedX);
	SetSpeedY(tempSpeedY);
	SetX(tempX);
	SetY(tempY);
	
	gRobot.debugInfomation.wzFromAngle = wzFromAngle;
}

/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/
