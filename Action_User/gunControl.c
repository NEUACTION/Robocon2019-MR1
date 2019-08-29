#include "gunControl.h"
#include "stm32f4xx_dac.h"
#include "moveBase.h"
#include "includes.h"
#include <app_cfg.h>
#include "robot.h"
#include "dma.h"
#include "light.h"

uint8_t pushShagaiCount = 0;

gunPos_t gGunPosDataBase[GUN_POS_NUM]= 
{
	//复位姿态 GUN_RESET_POS
	{GUN_CLAW_LOOSE , GUN_BASE_RESET ,SHOOT_GAS_PRESSURE , -21.0f,-21.0f},
	//装弹姿态 GUN_RELOAD_POS
	{GUN_CLAW_LOOSE , GUN_BASE_RESET ,SHOOT_GAS_PRESSURE , -23.9f,-23.9f},
	//装弹完成姿态 GUN_LOADED_POS
	{GUN_CLAW_CLAMP , GUN_BASE_RESET ,SHOOT_GAS_PRESSURE , LEFT_SHOOT_PITCH_ANGLE,RIGHT_SHOOT_PITCH_ANGLE},
	//发射姿态 GUN_SHOOT_POS
	{GUN_CLAW_CLAMP , GUN_BASE_EXTEND ,SHOOT_GAS_PRESSURE , LEFT_SHOOT_PITCH_ANGLE,RIGHT_SHOOT_PITCH_ANGLE}
};

//shootPara[0][]为蓝场参数，shootPara[1][]为红场参数
shootPara_t shootPara[2][MAXPARANUM]=
{
	{
		//蓝场	0712
		//自动投块
		{0.0f, 0.46f, 7.3f, 8.1f},//10号块
		//手动投块
		{30.0f, 0.495f, 8.5f, 9.3f},
		{20.0f, 0.495f, 8.2f, 9.0f},
		{10.0f, 0.48f, 8.1f, 8.9f},
		{0.0f, 0.455f, 7.0f, 7.6f},
		{-4.0f, 0.455f, 7.0f, 7.6f},		
	},
	{
		//红场 全国赛试场后0712
		//自动投块
		{0.0f, 0.455f, 7.6f, 8.4f},
		//手动投块
		{4.0f, 0.455f, 7.2f, 8.0f},
		{0.0f, 0.45f, 7.8f, 8.6f},
		{-10.0f, 0.49f, 8.6f, 9.4f},
		{-20.0f, 0.51f, 8.5f, 9.3f},
		{-30.0f, 0.51f, 8.5f, 9.3f},		
	}		
};

void GunInit(void)
{
	gRobot.gun.targetPos = gGunPosDataBase[GUN_RESET_POS];
	gRobot.gun.mode = GUN_AUTO_MODE;
	
	//检查发射机构到位标志位
	gRobot.gun.ready = GUN_AIMING;
	//手动模式下标志位
	gRobot.gun.aimFlag = GUN_AIM_DONE;
	gRobot.gun.shootFlag = GUN_WAIT_SHOOT_CMD;
	
	if(pushShagaiCount > 0)
	{
		if(gRobot.courtID == BLUE_COURT)
		{
			gRobot.gun.paraNum = 4;
		}
		else if(gRobot.courtID == RED_COURT)
		{
			gRobot.gun.paraNum = 2;
		}
	}
	else if(pushShagaiCount == 0)
	{
		gRobot.gun.paraNum = 0;
	}
	
	SetShootPara();
	
	GunPosReset();
}

void SetShootPara(void)
{
	if(gRobot.courtID == BLUE_COURT)
	{
		gRobot.teleCommand.angleshift = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].posAgl;
		
		gGunPosDataBase[GUN_RESET_POS].gasPressure = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_RELOAD_POS].gasPressure = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_LOADED_POS].gasPressure = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_SHOOT_POS].gasPressure = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].gasPre;
		
		gGunPosDataBase[GUN_LOADED_POS].leftPitchAngle = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].leftAgl;
		gGunPosDataBase[GUN_LOADED_POS].rightPitchAngle = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].rightAgl;
		gGunPosDataBase[GUN_SHOOT_POS].leftPitchAngle = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].leftAgl;
		gGunPosDataBase[GUN_SHOOT_POS].rightPitchAngle = shootPara[BLUE_COURT-1][gRobot.gun.paraNum].rightAgl;
	}
	else if(gRobot.courtID == RED_COURT)
	{
		gRobot.teleCommand.angleshift = shootPara[RED_COURT-1][gRobot.gun.paraNum].posAgl;
		
		gGunPosDataBase[GUN_RESET_POS].gasPressure = shootPara[RED_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_RELOAD_POS].gasPressure = shootPara[RED_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_LOADED_POS].gasPressure = shootPara[RED_COURT-1][gRobot.gun.paraNum].gasPre;
		gGunPosDataBase[GUN_SHOOT_POS].gasPressure = shootPara[RED_COURT-1][gRobot.gun.paraNum].gasPre;
		
		gGunPosDataBase[GUN_LOADED_POS].leftPitchAngle = shootPara[RED_COURT-1][gRobot.gun.paraNum].leftAgl;
		gGunPosDataBase[GUN_LOADED_POS].rightPitchAngle = shootPara[RED_COURT-1][gRobot.gun.paraNum].rightAgl;
		gGunPosDataBase[GUN_SHOOT_POS].leftPitchAngle = shootPara[RED_COURT-1][gRobot.gun.paraNum].leftAgl;
		gGunPosDataBase[GUN_SHOOT_POS].rightPitchAngle = shootPara[RED_COURT-1][gRobot.gun.paraNum].rightAgl;
	}
	gRobot.debugInfomation.posAngleShift = gRobot.teleCommand.angleshift;
	
}

//发射机构发送给主程序的信息
static	communicateMsg_t gunMsgToMain;

//发射机构向主控传递消息序列初始化
void GunMsg2MainInit(void)
{
	for(uint8_t i =0 ; i<10 ; i++)
	{
		gunMsgToMain.msgBuffer[i] = GUN_NO_CMD;
	}
	gunMsgToMain.bufferRear = 0;
	gunMsgToMain.bufferFront = 0;	
}

uint8_t JudgeGunMsg2MainEmpty(void)
{
	if(gunMsgToMain.bufferFront==gunMsgToMain.bufferRear)
	{
		return 1;
	}
	return 0;
}

//发射机构向主程序传递消息
void GunSendMsg2Main(uint8_t msg)
{
	//如果要传送的消息和上一个不同则将新消息放入消息队列
	if(gunMsgToMain.bufferRear>0)
	{
		if(msg!=gunMsgToMain.msgBuffer[gunMsgToMain.bufferRear-1])
		{
			gunMsgToMain.msgBuffer[gunMsgToMain.bufferRear] = msg;
			gunMsgToMain.bufferRear++;
			gunMsgToMain.bufferRear%=10;
		}
	}
	else if(gunMsgToMain.bufferRear==0)
	{
		if(msg!=gunMsgToMain.msgBuffer[9])
		{
			gunMsgToMain.msgBuffer[gunMsgToMain.bufferRear] = msg;
			gunMsgToMain.bufferRear++;
			gunMsgToMain.bufferRear%=10;
		}		
	}	
}

//主程序从消息队列中获取一条来自发射机构的消息
uint8_t GetGunMsg2Main(void)
{
	uint8_t msg = GUN_NO_CMD;
	
	//如果消息队列非空则返回一条有效消息
	if(gunMsgToMain.bufferFront!=gunMsgToMain.bufferRear)
	{
		msg = gunMsgToMain.msgBuffer[gunMsgToMain.bufferFront];
		
		gunMsgToMain.msgBuffer[gunMsgToMain.bufferFront] = GUN_NO_CMD;
		
		gunMsgToMain.bufferFront++;
		
		gunMsgToMain.bufferFront%=10;
	}
	//如果消息队列为空则返回没有消息
	return msg;	
}

static gunCmdBuffer_t gunMsgRecieve = {0};

void GunPosCmdReset(void)
{
	for(uint8_t i = 0;i<10;i++)
	{
		gunMsgRecieve.posCmd.msgBuffer[i] = GUN_NO_CMD;		
	}
	gunMsgRecieve.posCmd.bufferFront = 0;
	gunMsgRecieve.posCmd.bufferRear = 0;
}

void GunShootCmdReset(void)
{
	gunMsgRecieve.shootCmd = GUN_NO_CMD;	
}

void GunCmdReset(void)
{
	GunPosCmdReset();
	
	GunShootCmdReset();
}

gunCmd_t GetGunCmd(void)
{
	gunCmd_t cmd = {GUN_NO_CMD,GUN_NO_CMD};
	
	cmd.shootCmd = gunMsgRecieve.shootCmd;
	
	if(gunMsgRecieve.posCmd.bufferFront!=gunMsgRecieve.posCmd.bufferRear)
	{
		cmd.posCmd = gunMsgRecieve.posCmd.msgBuffer[gunMsgRecieve.posCmd.bufferFront];
		
		gunMsgRecieve.posCmd.bufferFront++;
		
		gunMsgRecieve.posCmd.bufferFront%=10;
	}

	return cmd;
}

//主程序到发射程序消息处理
void Main2GunMsgProcess(void)
{
	uint8_t nn = 0;
	
	while(!JudgeMainMsg2GunEmpty())
	{
		switch(GetMainMsg2Gun())
		{
			case GUN_RELOAD_MSG:
			{
				gunMsgRecieve.posCmd.msgBuffer[gunMsgRecieve.posCmd.bufferRear] = GUN_RELOAD_MSG;
				gunMsgRecieve.posCmd.bufferRear++;
				break;
			}
			case GUN_LOADED_MSG:
			{
				gunMsgRecieve.posCmd.msgBuffer[gunMsgRecieve.posCmd.bufferRear] = GUN_LOADED_MSG;
				gunMsgRecieve.posCmd.bufferRear++;
				break;
			}
			case GUN_PREPARE_SHOOT_MSG:
			{
				gunMsgRecieve.posCmd.msgBuffer[gunMsgRecieve.posCmd.bufferRear] = GUN_PREPARE_SHOOT_MSG;
				gunMsgRecieve.posCmd.bufferRear++;
				break;
			}
			case GUN_SHOOT_CONFIRM_MSG:
			{
				gunMsgRecieve.shootCmd = GUN_SHOOT_CONFIRM_MSG;
				break;
			}
			default:
				break;
		}
		
		gunMsgRecieve.posCmd.bufferRear%=10;
		
		nn++;
		
		if(nn>=10)
		{
			break;
		}
	}
}


//发射机构复位
void GunPosReset(void)
{
	gunPos_t targetPos = gGunPosDataBase[GUN_RESET_POS];

	GunShootPull();	

	GunAim(targetPos);
}

//发射机构瞄准
void GunAim(gunPos_t targetPos)
{
	GunClawControl(targetPos.clawState);
	GunBaseControl(targetPos.baseState);
	GasPressureControl(targetPos.gasPressure);
	ShootPitchControl(targetPos.leftPitchAngle,targetPos.rightPitchAngle);
	
	gRobot.gun.gunPosRecord = targetPos;
};

//检查发射机构参数
void GunCheckAim(void)
{
	//姿态到位检测次数
	#define CHECK_AIM_TIMES (3)
	//检测时间间隔(单位：10ms)
	#define CHECK_AIM_TIME_GAP (2)
	//超时时间(单位：10ms)
	#define CHECK_TIME_OUT (100)
	//超时计数变量
	int timeoutCounter = (CHECK_TIME_OUT/CHECK_AIM_TIME_GAP);
	//发射机构姿态到位次数
	uint8_t gunPosReadyTimes  = 0 ;
	
	//将检测标志复位
	gRobot.gun.ready = GUN_AIMING;
	
	while(timeoutCounter--)
//	while(1)
	{
		//将电机姿态值复位
		gRobot.robotMotorState.actLeftPitchPos  = 0.0f;
		gRobot.robotMotorState.actRightPitchPos = 0.0f;
		
		//读取电机位置
		ReadActualPos(CAN2 , 0);
		
		OSTimeDly(CHECK_AIM_TIME_GAP);
		
		//超时后 条件仍不满足 亮紫灯
		if(timeoutCounter<2)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"SHOOT	NOT	SATISFY!\r\n");
			Purple(LIGHT_BOARD_ID);
		}
		
		//检测气压是否满足标准
		if(fabs(GetGasPressure() - gRobot.gun.targetPos.gasPressure)>=0.002f)
		{
			continue;
		}
		
		//检测左侧俯仰电机是否到位
		if(fabs(-gRobot.robotMotorState.actLeftPitchPos - gRobot.gun.targetPos.leftPitchAngle)>=0.1f)
		{
			continue;
		}

		//检测右侧电机是否到位
		if(fabs(gRobot.robotMotorState.actRightPitchPos - gRobot.gun.targetPos.rightPitchAngle)>=0.1f)
		{
			continue;
		}
		
		gunPosReadyTimes++;
		
		//当姿态到位次数到达标准时跳出循环
		if(gunPosReadyTimes>CHECK_AIM_TIMES)
		{
			break;
		}
	}
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d\r\n",(int)timeoutCounter);
	//将瞄准标志位置位（超时没有到位也会置位）
	gRobot.gun.ready = GUN_AIM_READY;
	
}

//发射机构发射
void GunShoot(void)
{
	if(GunCheckMode()==GUN_AUTO_MODE)
	{
		//如果发射机构到位则发射
		if(gRobot.gun.ready == GUN_AIM_READY)
		{
//			if(pushShagaiCount==0)
//			{
//			OSTimeDly(6000);
//			}
			//打开爪子
			GunClawLoose();
			OSTimeDly(10);
//			OSTimeDly(50);
						
			//发射
			GunShootPush();
			
			pushShagaiCount++;
			
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"SHOOT!\r\n");
						
			OSTimeDly(50);

			GunSendMsg2Main(GUN_SHOOT_DONE_MSG);
			
			GunShootPull();	
			GunShootPull();	
		
			GunBaseIn();
			GunBaseIn();

			OSTimeDly(50);
			
			ShootPitchControl(4.f,4.f);
//			OSTimeDly(165);
		}		
	}
	else if(GunCheckMode()==GUN_MANUAL_MODE)
	{
		//如果发射机构到位则发射
		if(gRobot.gun.ready == GUN_AIM_READY)
		{
			//打开爪子
			GunClawLoose();
			OSTimeDly(10);
			
			gRobot.gun.targetPos.clawState = GUN_CLAW_LOOSE;

			//发射
			GunShootPush();
			OSTimeDly(50);
			
			//机构复位
			GunShootPull();
		}		
	}

}

//检查发射机构模式
uint8_t GunCheckMode(void)
{
	return gRobot.gun.mode;
}

static char gunMsgBuffer[GUN_MSG_BUFFER_SIZE][GUN_MSG_MAX_LENGTH] = {0};

static uint8_t gunMsgBytePtr = 0;

static uint8_t gunMsgBufferRear , gunMsgBufferFront = 0;

//发射机构接收数组初始化
void GunMsgBufferInit(void)
{
	gunMsgBufferFront = 0;
	gunMsgBufferRear = 0;
	gunMsgBytePtr = 0;
}

/**
* @brief  JudgeGunMsgBufferEmpty判断循环队列是否为空
  * @note
* @param  
* @retval 1:队列非空 0：队列为空
  */
uint8_t JudgeGunMsgBufferEmpty(void)
{
	if(gunMsgBufferFront!=gunMsgBufferRear)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

extern float angleNote;
/**
* @brief  GunMsgBufferInput循环队列入队
  * @note
* @param  inputData:要入队的数据
* @retval 
  */
void GunMsgBufferInput(char inputData)
{
	static char lastInputData;
	if(gunMsgBytePtr==0)
	{
		if(inputData=='A')
		{
			//填入数据
			gunMsgBuffer[gunMsgBufferRear][gunMsgBytePtr] = inputData;
			
			//指针自加
			gunMsgBytePtr++;
		}
	}
	else
	{
		//填入数据
		gunMsgBuffer[gunMsgBufferRear][gunMsgBytePtr] = inputData;
		
		//指针自加
		gunMsgBytePtr++;	
	}
	
	
	//收完一条消息后将消息入队
	if(gunMsgBytePtr>=GUN_MSG_MAX_LENGTH||(lastInputData == '\r'&&inputData=='\n'))
	{
		gunMsgBytePtr = 0;
		gunMsgBufferRear++;
		gunMsgBufferRear%=GUN_MSG_BUFFER_SIZE;
	}
	
	lastInputData = inputData;
}

static uint8_t posAngleShiftChangedFlag = 0; 

void GunMsgProcess(void)
{
	union gunMsgRecieve
	{
		uint8_t byteData[4];
		int32_t intData;
		float floatData;
	}gunMsgunion = {0};
		
	//当队列非空时处理消息
	while(JudgeGunMsgBufferEmpty())
	{
		//判断消息头和尾是否正确
		if(strncmp(&gunMsgBuffer[gunMsgBufferFront][0],"AT",2)==0&&\
			strncmp(&gunMsgBuffer[gunMsgBufferFront][5],"\r\n",2)==0)
		{
			//接收到的命令是发射命令
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+1",2)==0)
			{
				if(gunMsgBuffer[gunMsgBufferFront][4]=='1')
				{
					gRobot.gun.shootFlag = GUN_SHOOT_PERMITTED;
				}
				else if(gunMsgBuffer[gunMsgBufferFront][4]=='0')
				{
					gRobot.gun.shootFlag = GUN_WAIT_SHOOT_CMD;
				}
			}

			//接收到的命令是爪子命令
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+2",2)==0)
			{
				if(gunMsgBuffer[gunMsgBufferFront][4]=='1')
				{
					gRobot.gun.targetPos.clawState = GUN_CLAW_CLAMP;
				}
				else if(gunMsgBuffer[gunMsgBufferFront][4]=='0')
				{
					gRobot.gun.targetPos.clawState = GUN_CLAW_LOOSE;
				}
				gRobot.gun.aimFlag = GUN_AIM_CHANGED;
			}			

			//接收到的命令是平台命令
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+3",2)==0)
			{
				if(gunMsgBuffer[gunMsgBufferFront][4]=='1')
				{
					gRobot.gun.targetPos.baseState = GUN_BASE_EXTEND;
				}
				else if(gunMsgBuffer[gunMsgBufferFront][4]=='0')
				{
					gRobot.gun.targetPos.baseState = GUN_BASE_RESET;
				}
				gRobot.gun.aimFlag = GUN_AIM_CHANGED;
			}	
			
			//接收到的命令是轮子使能失能命令
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+4",2)==0)
			{
				if(gunMsgBuffer[gunMsgBufferFront][4]=='1')
				{
					gRobot.wheelState.disenableFlag = 0;
				}
				else if(gunMsgBuffer[gunMsgBufferFront][4]=='0')
				{
					gRobot.wheelState.disenableFlag = 1;
				}
			}
		}
		//判断消息头和尾是否正确
		if(strncmp(&gunMsgBuffer[gunMsgBufferFront][0],"AT",2)==0&&\
			strncmp(&gunMsgBuffer[gunMsgBufferFront][8],"\r\n",2)==0)
		{
			//接收到的命令是气压命令
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+5",2)==0)
			{
				for(int i=0;i<4;i++)
				{
					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+4];
				}	
				gRobot.gun.targetPos.gasPressure = gunMsgunion.floatData;
				gRobot.gun.aimFlag = GUN_AIM_CHANGED;
			}			
			//接收到的命令是轮子方向命令 范围[-90,90]
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+7",2)==0)
			{
				for(int i=0;i<4;i++)
				{
					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+4];
				}	
				
				if(gunMsgunion.floatData < -90.0f)
				{
					gunMsgunion.floatData += 180.f;
				}
				else if(gunMsgunion.floatData > 90.0f)
				{
					gunMsgunion.floatData -= 180.f;
				}
				
				gRobot.wheelState.leftFrontTarget.direction = gunMsgunion.floatData;
				gRobot.wheelState.rightFrontTarget.direction = gunMsgunion.floatData;
				gRobot.wheelState.leftRearTarget.direction = gunMsgunion.floatData;
				gRobot.wheelState.rightRearTarget.direction = gunMsgunion.floatData;
			}
			//接收到的命令是姿态角
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+8",2)==0)
			{
				//姿态角偏移量改变标志位
				posAngleShiftChangedFlag = 1;
				
				for(int i=0;i<4;i++)
				{
					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+4];
				}	
				
				if(fabs(gunMsgunion.floatData)>35.0f)
				{
					gunMsgunion.floatData = gunMsgunion.floatData / fabs(gunMsgunion.floatData) * 35.0f;
					
					USART_OUT(DEBUG_USART,(uint8_t *)"ANGLESHIFT IS MAX\r\n");
				}
				
				gRobot.debugInfomation.posAngleShift = gunMsgunion.floatData;
				
			}	
		}
		
		//判断消息头和尾是否正确
		if(strncmp(&gunMsgBuffer[gunMsgBufferFront][0],"AT",2)==0&&\
			strncmp(&gunMsgBuffer[gunMsgBufferFront][12],"\r\n",2)==0)
		{
			//接收到的命令是俯仰角度命令 fix me 只能接受发射角度
			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+6",2)==0)
			{
				for(int i=0;i<4;i++)
				{
					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+4];
				}	
				gRobot.gun.targetPos.leftPitchAngle = gunMsgunion.floatData;
				for(int i=0;i<4;i++)
				{
					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+8];
				}	
				gRobot.gun.targetPos.rightPitchAngle = gunMsgunion.floatData;
				if(fabs(gRobot.gun.targetPos.leftPitchAngle-gRobot.gun.targetPos.rightPitchAngle) < 1.0f)
				{
					gRobot.gun.aimFlag = GUN_AIM_CHANGED;
				}
				else
				{
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"tabletSendAbnormal!\r\n");
				}
			}
		}
//		//判断消息头和尾是否正确
//		if(strncmp(&gunMsgBuffer[gunMsgBufferFront][0],"AT",2)==0&&\
//			strncmp(&gunMsgBuffer[gunMsgBufferFront][9],"\r\n",2)==0)
//		{
//			//接收到的命令是俯仰角度命令 fix me 只能接受发射角度
//			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+60",3)==0)
//			{
//				for(int i=0;i<4;i++)
//				{
//					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+5];
//				}	
//				gRobot.gun.targetPos.leftPitchAngle = gunMsgunion.floatData;
//				gRobot.gun.targetPos.rightPitchAngle = gunMsgunion.floatData;
//				gRobot.gun.aimFlag = GUN_AIM_CHANGED;
//			}
//			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+61",3)==0)
//			{
//				for(int i=0;i<4;i++)
//				{
//					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+5];
//				}	
//				if(fabs(gunMsgunion.floatData-gRobot.gun.targetPos.rightPitchAngle) < 1.0f)
//				{
//					gRobot.gun.targetPos.leftPitchAngle = gunMsgunion.floatData;
//					gRobot.gun.aimFlag = GUN_AIM_CHANGED;
//				}
//				else
//				{
//					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"tabletSendAbnormal!\r\n");			
//				}
//			}
//			if(strncmp(&gunMsgBuffer[gunMsgBufferFront][2],"+62",3)==0)
//			{
//				for(int i=0;i<4;i++)
//				{
//					gunMsgunion.byteData[i] =  gunMsgBuffer[gunMsgBufferFront][i+5];
//				}	
//				if(fabs(gunMsgunion.floatData-gRobot.gun.targetPos.leftPitchAngle) < 1.0f)
//				{
//					gRobot.gun.targetPos.rightPitchAngle = gunMsgunion.floatData;
//					gRobot.gun.aimFlag = GUN_AIM_CHANGED;
//				}
//				else
//				{
//					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"tabletSendAbnormal!\r\n");
//				}
//			}
//			
//		}
		
		//将处理完的数组清空
		for(uint8_t i = 0 ; i<GUN_MSG_MAX_LENGTH ; i++)
		{
			gunMsgBuffer[gunMsgBufferFront][i] = 0;
		}
		
		gunMsgBufferFront++;
		gunMsgBufferFront%=GUN_MSG_BUFFER_SIZE;
	}
}

void SendGasPressureValue(USART_TypeDef* USARTx , float value)
{
  char s[15]={0};
  int integer=( int )value;
  sprintf( (char*)s, "AT+5%d.%04d\t\r\n", ( int )value, (unsigned int)((fabs(value) - abs(integer))  * 10000));
  USART_OUT(USARTx,(uint8_t*)s);
}

/**
* @brief  GetGasPressure将气压的电压信号转换为气压值
  * @note
* @param  
  * @retval气压值（单位：兆帕）
  */
float GetGasPressure(void)
{
	float gasADCValue = GetGasPressureADCValue();
	
	//fix me hard number
	//jiubilifa
//	return (gasADCValue - GAS_ADC_ZERO)*1.132f/2048.f + 0.003f;
	//xinbilifa
	return (gasADCValue - 338.1667f)*1.16f/2048.f - 0.02f;
}

/**
* @brief  GasPressureControl将气压转换为对应的PWM占空比输出
  * @note
* @param  gasPressure:气压值（单位：兆帕）
  * @retval
  */
void GasPressureControl(float gasPressure)
{
	//对输入气压值进行限幅
	if(gasPressure<=0.0f)
	{
		gasPressure = 0.0f;
	}
	else if(gasPressure>=0.9f)
	{
		gasPressure = 0.9f;
	}
	gRobot.debugInfomation.gasPressure = gasPressure;
	//jiubilifa
//	gasPressure = (10000.f/3060.f*4096/3.3f)*((gasPressure+0.0017f)/0.9f);
	//xinbilifa
	gasPressure = (10000.f/3060.f*4096/3.3f)*((gasPressure+0.0007f)/0.9f);
	//fix me hard number
//	TIM_SetCompare4(TIM4,511-(gasPressure-0.007f)/0.9f*511.f);	
	DAC_SetChannel1Data(DAC_Align_12b_R,gasPressure);//12bit align right
}

/**
* @brief  ShootAngle2PositionTransform将发射机构俯仰角度转化为脉冲
  * @note
* @param  angle:发射机构俯仰角度
* @retval 对应的电机脉冲位置
  */
int ShootAngle2PositionTransform(float angle)
{
	return (int)(angle / 360.0f * M3508_COUNTS_PER_ROUND * SHOOT_PITCH_REDUCTION_RATIO);	
}

/**
* @brief  ShootAngle2PositionInverseTransform将发射机构俯仰脉冲位置转化为角度
  * @note
* @param  position：发射机构俯仰脉冲位置
* @retval 发射机构俯仰角度
  */
float ShootAngle2PositionInverseTransform(int position)
{
	return (float)((float)position / M3508_COUNTS_PER_ROUND / SHOOT_PITCH_REDUCTION_RATIO * 360.0f);
}

/**
* @brief  ShootPitchControl发射机构俯仰角度控制
  * @note
* @param  angle：发射机构俯仰角度
* @retval
  */
void ShootPitchControl(float leftAngle, float rightAngle)
{
	//限制俯仰角
	if(leftAngle<-24.0f){
		leftAngle=-24.0f;
	}
	else if(leftAngle>10.0f){
		leftAngle=10.0f;
	}
	if(rightAngle<-24.0f){
		rightAngle=-24.0f;
	}
	else if(rightAngle>10.0f){
		rightAngle=10.0f;
	}
	//对左右俯仰角限幅 使其差值不超过1度
	if(fabs(leftAngle-rightAngle) >= 1.0f)
	{
		leftAngle = (leftAngle + rightAngle)/2;
		rightAngle = leftAngle;
	}

	gRobot.robotMotorState.expLeftPitchPos=-leftAngle;
	gRobot.robotMotorState.expRightPitchPos=rightAngle;
	

	PosCrl(CAN2, SHOOT_PITCH_LEFT_MOTOR_ID,ABSOLUTE_MODE,-ShootAngle2PositionTransform(leftAngle));
	PosCrl(CAN2, SHOOT_PITCH_RIGHT_MOTOR_ID,ABSOLUTE_MODE,ShootAngle2PositionTransform(rightAngle));
}

//发射机构发射气缸推出
void GunShootPush(void)
{
	GasValveControl(CAN2 , GUN_SHOOT_BOARD_ID , GUN_SHOOT_IO_ID , 1);
}

//发射机构发射气缸收回
void GunShootPull(void)
{
	GasValveControl(CAN2 , GUN_SHOOT_BOARD_ID , GUN_SHOOT_IO_ID , 0);
}

//发射机构平台气缸推出
void GunBaseOut(void)
{
	GasValveControl(CAN2 , GUN_BASE_BOARD_ID , GUN_BASE_IO_ID , 1);
}

//发射机构平台气缸收回7
void GunBaseIn(void)
{
	GasValveControl(CAN2 , GUN_BASE_BOARD_ID , GUN_BASE_IO_ID , 0);
}

//发射机构平台控制 6
void GunBaseControl(uint8_t cmd)
{
	if(cmd==GUN_BASE_EXTEND||cmd==GUN_BASE_RESET)
	{
		GasValveControl(CAN2 , GUN_BASE_BOARD_ID , GUN_BASE_IO_ID , cmd);		
	}
}

//发射机构爪子夹紧
void GunClawClamp(void)
{
	GasValveControl(CAN2 , GUN_CLAW_BOARD_ID , GUN_CLAW_IO_ID , 1);
}
//发射机构爪子复位
void GunClawLoose(void)
{
	GasValveControl(CAN2 , GUN_CLAW_BOARD_ID , GUN_CLAW_IO_ID , 0);
}

//发射机构爪子控制
void GunClawControl(uint8_t cmd)
{
	if(cmd==GUN_CLAW_CLAMP||cmd==GUN_CLAW_LOOSE)
	{
		GasValveControl(CAN2 , GUN_CLAW_BOARD_ID , GUN_CLAW_IO_ID , cmd);		
	}
}

extern float posAngle_EnterManulMode;

//发射机构调试参数动作执行
void ExecuteManualOrder(void)
{
	//轮子
	if(gRobot.wheelState.disenableFlag == 0)
	{
		MotorOn(CAN1,LEFT_FRONT_ID);
		MotorOn(CAN1,LEFT_REAR_ID);
		MotorOn(CAN1,RIGHT_FRONT_ID);
		MotorOn(CAN1,RIGHT_REAR_ID);
	}
	else if(gRobot.wheelState.disenableFlag == 1)
	{
		MotorOff(CAN1,LEFT_FRONT_ID);
		MotorOff(CAN1,LEFT_REAR_ID);
		MotorOff(CAN1,RIGHT_FRONT_ID);
		MotorOff(CAN1,RIGHT_REAR_ID);
	}
	
	//如果姿态角改变 则旋转 结束后5055，3508抱死
	//此时姿态角目标值=进入MANUALMODE时姿态角+姿态角偏移量
	
	if(posAngleShiftChangedFlag == 0)
	{
		gRobot.manualPos.angleAdjust = gRobot.manualPos.angleAdjust + ReturnLimitAngle(gRobot.manualPos.angleNote - GetAngle());
		
		VelCrl(CAN1 , LEFT_FRONT_ID , Vel2Pulse(0.0f));
		VelCrl(CAN1 , RIGHT_FRONT_ID , -Vel2Pulse(0.0f));
		VelCrl(CAN1 , LEFT_REAR_ID , Vel2Pulse(0.0f));
		VelCrl(CAN1 , RIGHT_REAR_ID , -Vel2Pulse(0.0f));

		//轮子角度 复位圈数为0
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(gRobot.wheelState.leftFrontTarget.direction , 0));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(gRobot.wheelState.rightFrontTarget.direction , 0));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(gRobot.wheelState.leftRearTarget.direction , 0));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(gRobot.wheelState.rightRearTarget.direction , 0));
	}
	else if(posAngleShiftChangedFlag == 1)
	{
		static wheelState_t lastWheelState = {0};
//		
//		SetAnglePIDPara(7.f,12.f);
		if(fabs(GetAngle()-(posAngle_EnterManulMode + gRobot.debugInfomation.posAngleShift))>0.1f)
		{
			float angularVel = 0;
		
//			angularVel = AngleControl(GetAngle(),(posAngle_EnterManulMode + gRobot.debugInfomation.posAngleShift));

			OutputVel2Wheel(0.0f,0.0f,angularVel);
			
			lastWheelState = gRobot.wheelState;
		}
		else
		{
			posAngleShiftChangedFlag = 0;
			
			gRobot.manualPos.angleNote = GetAngle();
			
			SendCmd2Driver(0.0f,lastWheelState.leftFrontTarget.direction,0.0f,lastWheelState.rightFrontTarget .direction,\
						0.0f,lastWheelState.leftRearTarget.direction,0.0f,lastWheelState.rightRearTarget.direction);
		}
	}
}

//发射机构手动控制
void ManualControl(void)
{	
	//处理队列中的消息
	GunMsgProcess();
	//向平板发送实时气压
	SendGasPressureValue(USART2,GetGasPressure());
	//执行调试指令
	ExecuteManualOrder();
}

//void ControlPitch()
//{
//	float KpLeft = 0.0f, KpRight = 0.0f;
//	float leftPitchTarget = 0.f, rightPitchTarget = 0.f;
//	float leftPitchAct = 0.f, rightPitchAct = 0.f;
//	float leftPitchOutput = 0.f, rightPitchOutput = 0.f;
//	float leftErr = 0.f, rightErr = 0.f;
//	float pitchDiff = 0.0f;
//	
//	leftPitchTarget = -gRobot.robotMotorState.expLeftPitchPos;
//	rightPitchTarget = gRobot.robotMotorState.expRightPitchPos;
//	
//	leftPitchAct = -gRobot.robotMotorState.actLeftPitchPos;
//	rightPitchAct = gRobot.robotMotorState.actRightPitchPos;
//	
//	leftErr = leftPitchTarget - leftPitchAct;
//	rightErr = rightPitchTarget - rightPitchAct;
//	
//	leftErr*=KpLeft;
//	rightErr*=KpRight;
//	
//	leftPitchOutput = leftPitchTarget + leftErr;
//	rightPitchOutput = rightPitchTarget + rightErr;
//	
//	pitchDiff = leftPitchOutput - rightPitchOutput;
//	
//	if(fabs(pitchDiff) > 1.0f)
//	{
//		leftPitchOutput = leftPitchOutput - pitchDiff/2.0f;
//		rightPitchOutput = rightPitchOutput + pitchDiff/2.0f;
//	}
//	
//}

