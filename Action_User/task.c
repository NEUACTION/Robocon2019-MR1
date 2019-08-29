#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "pps.h"
#include "moveBase.h"
#include "robot.h"
#include "dma.h"
#include "path.h"
#include "adc.h"
#include "dac.h"
#include "iwdg.h"
#include "steerReset.h"
#include "gunControl.h"
#include "telecontroller.h"
#include "light.h"
#include "motion.h"
/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *velCtrPeriodSem;
OS_EVENT *velCtrCmdSem;
OS_EVENT *gunPeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static OS_STK VelCrlTaskStk[VELCRL_TASK_STK_SIZE];
static 	OS_STK  SelfCheckTaskStk[SELFCHECK_TASK_STK_SIZE];
static OS_STK ShootTaskStk[SHOOT_TASK_STK_SIZE];

void VelCrlTask(void);
void SelfCheckTask(void);
void ShootTask(void);

void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);
	
	velCtrPeriodSem =  OSSemCreate(0);

	velCtrCmdSem = OSSemCreate(0);
	
	gunPeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
						  
	os_err = OSTaskCreate((void (*)(void *))VelCrlTask,
						  (void *)0,
						  (OS_STK *)&VelCrlTaskStk[VELCRL_TASK_STK_SIZE - 1],
						  (INT8U)VELCRL_TASK_PRIO);	
							
	os_err = OSTaskCreate(	(void (*)(void *)) SelfCheckTask,
							(void		  * ) 0,
							(OS_STK		* )&SelfCheckTaskStk[SELFCHECK_TASK_STK_SIZE-1],
							(INT8U		   ) SELFCHECK_TASK_PRIO);
							
	os_err = OSTaskCreate((void (*)(void *))ShootTask,
						  (void *)0,
						  (OS_STK *)&ShootTaskStk[SHOOT_TASK_STK_SIZE - 1],
						  (INT8U)SHOOT_TASK_PRIO);	
}

gRobot_t gRobot = {0};
extern laserValue_t laserValue;
uint8_t gunStatus = GUN_RESET_POS;

extern uint8_t shagaiPosionFlag;

void SendDebugInfo(void);
void CheckDriverState(void);
void MotorInit(void);
void MotorTest(void);
void CheckCANSendState(void);
void KeySwitchCheck(void);
void GunModeCheck(void);
void HardwareInit(void);

void WheelDirectionInit(void);
void Valve_KeySwitchCheck(void);
/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void CourtIDInit(void)
{
	static uint8_t timeCounter = 20;
	static uint8_t counter = 0;

	//20次里按下大于等于10次为红场 否则蓝场
	while(timeCounter--)
	{
		delay_ms(5);
		
		if(COURTID_KEYSWITCH)
		{
			counter++;
		}
		else
		{
			counter = 0;
		}
	}
	
	if(counter>=10)
	{
		gRobot.courtID = RED_COURT;
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"CourtID:RED!\r\n");
	}
	else 
	{
		gRobot.courtID = BLUE_COURT;
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"CourtID:BLUE!\r\n");
	}
	
	switch(gRobot.courtID)
	{
		case RED_COURT:
		{
			Red(LIGHT_BOARD_ID);
			break;
		}
		case BLUE_COURT:
		{
			Blue(LIGHT_BOARD_ID);
			break;
		}
		default:
			break;
	}
}
void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
		
	//设置中断向量优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	HardwareInit();
	
	delay_ms(50);
	
	//读按键识别红蓝场 默认为蓝场 按下为红场 红场亮红灯 蓝场亮蓝灯
	CourtIDInit();
	
	//驱动器初始化
	MotorInit();
	
	//红蓝场路径初始化 自检标志位清零 发射机构与主程序间通信初始化 发射机构初始化
	RobotInit();

//	//轨迹规划测试程序
//	delay_ms(5000);
//	BufferZizeInit(200);
//	ClearRingBuffer();
//	InputPoints2RingBuffer(get3rdShagaiPath, GET_3RD_SHAGAI_PATH_NUM);
//  ClearPathLen();
//	while(1)
//	{
//		SendCmd2Driver(1000.0f , 0.0f , 1000.0f , 0.0f,
//						   1000.0f , 0.0f , 1000.0f , 0.0f);
//		delay_ms(10);
//	}
	
	//等待矫正板红蓝场初始化
	SendCourtIDCmd();
	
	//复位 自检 状态判断 初始状态切换 1s
	KeySwitchCheck();

	OSTaskSuspend(OS_PRIO_SELF);
}

void VelCrlTask(void)
{
	CPU_INT08U  os_err;
	float vel = 0.0f;
	float velDireciton = 0.0f;
	position_t actVel = {0.0f};
	os_err = os_err;
	
	OSSemSet(velCtrPeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(velCtrCmdSem,0,&os_err);
		OSSemPend(velCtrPeriodSem, 0, &os_err);
		OSSemSet(velCtrPeriodSem,0,&os_err);
		actVel = GetSpeedWithoutOmega();
		vel = sqrtf(actVel.x * actVel.x + actVel.y * actVel.y);
		
		if(vel>0.01f)
		{
			velDireciton = atan2f(actVel.y,actVel.x)*CHANGE_TO_ANGLE;
		}
		else
		{
			velDireciton = velDireciton;
		}
		
//		VelControl((robotVel_t){vel , velDireciton , GetWZ()});	
	}
}

void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	
	gRobot.walkStatus = waitForStart;
	
	//等待调试开关检查
	OSTimeDly(50);

	if(GunCheckMode()==GUN_AUTO_MODE)
	{
		//轮子方向初始化 BLUE 131 RED 48
		WheelDirectionInit();
	}
	//手臂复位
	ArmRotateReset();
	
	//抓住令牌
	GraspTheGerege();
	
	//定位系统初始化延时18s
	for(int i=0;i<27;i++)
	{
		if(i%2 == 1)
		{
			White(LIGHT_BOARD_ID);
		}
		else
		{
			if(gRobot.courtID == BLUE_COURT)
			{
				Blue(LIGHT_BOARD_ID);
			}
			else if(gRobot.courtID == RED_COURT)
			{
				Red(LIGHT_BOARD_ID);
			}
		}
		
		if(gRobot.debugInfomation.SDCardLostFlag == 1)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"SDCard Lost!\r\n");
		}
		
		OSTimeDly(50);
	}
	White(LIGHT_BOARD_ID);

	//等待定位系统初始化 亮红/蓝灯
	WaitLaserOpsPrepare();
	
//	BufferZizeInit(200);
//	ClearRingBuffer();
//	InputPoints2RingBuffer(getFirstShagaiPath, GET_FIRST_SHAGAI_PATH_NUM);
	
	//初始化NEXT标志位
	gRobot.teleDevice.BT.nextFlag = TELENOCMD;
	gRobot.teleDevice.WIFI.nextFlag = TELENOCMD;
	gRobot.teleDevice.BT.stopFlag = TELENOCMD;
	gRobot.teleDevice.WIFI.stopFlag = TELENOCMD;
	gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NONE;
	gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NONE;				
	gRobot.teleCommand.nextFlag = TELENOCMD;
	gRobot.teleCommand.stopFlag = TELENOCMD;
	gRobot.teleCommand.angleshift = 0.0f;
	
		
	//FIX ME 蜂鸣器响0.5s表示定位系统初始化完成 再按NXT 灯加上后应去掉
	BEEP_ON;
	delay_ms(500);
	BEEP_OFF;
	
	if(gRobot.courtID == BLUE_COURT)
	{
		Blue(LIGHT_BOARD_ID);
	}
	else if(gRobot.courtID == RED_COURT)
	{
		Red(LIGHT_BOARD_ID);
	}
	
	//重启速度衰减系数初始化为1
	gRobot.resetCoeffic = 1.0f;
	
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		OSSemSet(PeriodSem, 0, &os_err);

		Walk();

		//检查各个驱动器状态是否正常
		CheckDriverState();
		
		//检查CAN发送是否正常
		CheckCANSendState();
		
		//发送调试数据
		SendDebugInfo();
	}
}

void ShootTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;

	//初始化为复位状态
	GunPosReset();
	
	OSSemSet(gunPeriodSem, 0, &os_err);	
	while(1)
	{
		OSSemPend(gunPeriodSem, 0, &os_err);
		OSSemSet(gunPeriodSem, 0, &os_err);
		
		Main2GunMsgProcess();
		
		//检查Valve_KeySwitch状态 并切换手臂气缸状态
		Valve_KeySwitchCheck();
		
		//调试模式判断
		GunModeCheck();
		
		if(GunCheckMode()==GUN_AUTO_MODE)
		{
			switch(gunStatus)
			{
				case GUN_RESET_POS:
				{
					//等待装载命令
					if(GetGunCmd().posCmd==GUN_RELOAD_MSG)
					{
						//姿态调整到装载姿态
						gRobot.gun.targetPos = gGunPosDataBase[GUN_RELOAD_POS];
						GunAim(gRobot.gun.targetPos);
						
						gunStatus = GUN_RELOAD_POS;
					}	
					break;
				}
				case GUN_RELOAD_POS:
				{
					//等待装载完成命令
					if(GetGunCmd().posCmd==GUN_LOADED_MSG)
					{
						//姿态调整到装弹完成姿态
						gRobot.gun.targetPos = gGunPosDataBase[GUN_LOADED_POS];
						GunAim(gRobot.gun.targetPos);
						
						gunStatus = GUN_LOADED_POS;
					}
					break;
				}
				case GUN_LOADED_POS:
				{
					//等待准备发射命令
					if(GetGunCmd().posCmd==GUN_PREPARE_SHOOT_MSG)
					{					
						//检查发射机构俯仰角度
						while(-gRobot.robotMotorState.actLeftPitchPos<=0.0f||gRobot.robotMotorState.actRightPitchPos<=0.0f)
						{
							OSTimeDly(2);
						}
						
						//姿态调整到发射姿态
						gRobot.gun.targetPos = gGunPosDataBase[GUN_SHOOT_POS];
						
//						//如果发射姿态角过大则先伸出平台
//						if(gRobot.gun.targetPos.leftPitchAngle>=5.5f||gRobot.gun.targetPos.rightPitchAngle>=5.5f)
//						{
//							GunBaseOut();
//							USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"BaseOut\r\n");
//							OSTimeDly(20);
//						}
						
						GunAim(gRobot.gun.targetPos);
						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"BaseOut\r\n");
						
//						if(shagaiPosionFlag != FIRSTSHAGAIPOSION)
//						{
//							OSTimeDly(5);	
//							GunClawLoose();
//							USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"ClawOpen\r\n");
//								
//							OSTimeDly(15);						
//							GunClawClamp();
//							USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"ClawLamp\r\n");

//							OSTimeDly(10);	
//							GunClawLoose();
//							USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"ClawOpen\r\n");
//								
//							OSTimeDly(10);						
//							GunClawClamp();
//							USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//								(uint8_t *)"ClawLamp\r\n");
//						}
////							
//						OSTimeDly(10);	
//						GunClawLoose();
//						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//							(uint8_t *)"ClawOpen\r\n");
//							
//						OSTimeDly(10);						
//						GunClawClamp();
//						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//							(uint8_t *)"ClawLamp\r\n");
//						if(gRobot.gun.targetPos.leftPitchAngle>=5.5f||gRobot.gun.targetPos.rightPitchAngle>=5.5f)
//						{
//							OSTimeDly(100);
//						}
//						else
//						{
//							OSTimeDly(120);
//						}
						
						if(pushShagaiCount==0)
						{
							GunCheckAim();
						}
						
						gunStatus = GUN_SHOOT_POS;
					}
					break;
				}
				case GUN_SHOOT_POS:
				{
					//等待发射命令
					if(GetGunCmd().shootCmd==GUN_SHOOT_CONFIRM_MSG)
					{
						//检查姿态是否调整到位 1s 20ms一次
						if(pushShagaiCount>0)
						{
							GunCheckAim();
						}
						
						//发射
						GunShoot();
						
						GunShootCmdReset();
						gunStatus = GUN_RESET_POS;
					}
					else if(gRobot.resetFlag == ROBOT_NONE && gRobot.walkStatus != stop)
					{
						//姿态瞄准到最新发射姿态
						gRobot.gun.targetPos = gGunPosDataBase[GUN_SHOOT_POS];
						GunAim(gRobot.gun.targetPos);
					}
					break;
				}
				default:
					break;
			}		
		}
		else if(GunCheckMode()==GUN_MANUAL_MODE)
		{
			
			//如果姿态发生变化则调整姿态
			if(gRobot.gun.aimFlag==GUN_AIM_CHANGED)
			{
				//调整姿态
				GunAim(gRobot.gun.targetPos);
				
				gRobot.gun.aimFlag = GUN_AIM_DONE;
			}
			//如果接收到发射命令则发射
			else if(gRobot.gun.shootFlag==GUN_SHOOT_PERMITTED)
			{
				//发射前将姿态调整到最新姿态
				GunAim(gRobot.gun.targetPos);
				
				//检查是否到达目标姿态
				GunCheckAim();
				
				//发射
				GunShoot();
				
				//发射标志位复位
				gRobot.gun.shootFlag = GUN_WAIT_SHOOT_CMD;
			}
		}
	}
}

void SelfCheckTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;

	static int timeCounter=0;
//	delay_ms(10000);
//	WaitLaserOpsPrepare();
	gRobot.checkStatus=armCheck;
//	gRobot.checkStatus=gpsCheck;
	OSSemSet(PeriodSem, 0, &os_err);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"wheel check!\r\n");
	while(1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		OSSemSet(PeriodSem, 0, &os_err);
		
		CheckDriverState();
		CheckCANSendState();
		switch(gRobot.checkStatus)
		{
			case armCheck:
			{
				delay_ms(1000);

				//手臂旋转
				if(gRobot.courtID==BLUE_COURT)
				{
					ArmPosControl(-90.0f);
				}
				else if(gRobot.courtID==RED_COURT)
				{
					ArmPosControl(0.0f);
				}

				delay_ms(1500);

				//手臂旋转复位
				ArmRotateReset();
				delay_ms(1500);
					
				for(int i=0; i<2; i++)
				{
					//松令牌
					ReleaseTheGerege();
					delay_ms(700);
					//夹令牌
					GraspTheGerege();
					delay_ms(700);
				}
				delay_ms(500);
				gRobot.checkStatus=wheelCheck;
				break;
			}	
			case wheelCheck:
			{
				SendSelfCheckWheelSpeed();
				
				if(WheelMotorCheck2())
				{
					gRobot.checkStatus=cushionCheck;
//					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"arm check!\r\n");	
				}
				
				break;
			}
			case cushionCheck:
			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"cushion check!\r\n");	
				delay_ms(1000);
				
				//缓冲气缸伸出
				CushionCylinderPush();
				delay_ms(1500);
				//缓冲气缸复位
				CushionCylinderRecovery();
				delay_ms(3000);
//				//缓冲气缸泄气
//				CushionCylinderHold();
//				delay_ms(1000);
				
				gRobot.checkStatus=gasSensorCheck;
				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"gas sensor check!\r\n");	
				delay_ms(1000);
				break;
			}
			case gasSensorCheck:
			{
				
				if(timeCounter==0) 
				{
					GasPressureControl(0.3f);
				}
				else if(timeCounter==50)
				{
					GasPressureControl(0.2f);
				}
				else if(timeCounter>=100)
				{
					timeCounter=0;
					gRobot.checkStatus=gunCheck;
//					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"gun check!\r\n");	
				}
				
				timeCounter++;
				break;
			}
			case gunCheck:
			{
				delay_ms(1000);
				
				//发射机构俯仰角上10度下23.5度 2次
				for(int i=0; i<2; i++)
				{
					ShootPitchControl(10.0f,10.0f);
					delay_ms(1500);
					ShootPitchControl(-23.5f,-23.5f);
					delay_ms(1500);
				}
				
				ShootPitchControl(0.0f,0.0f);
				delay_ms(1000);
				
				for(int i=0; i<2; i++)
				{
					//发射机构爪子夹紧
					GunClawClamp();
					delay_ms(700);
					//发射机构爪子复位
					GunClawLoose();
					delay_ms(700);
				}
				
				//发射机构平台发射推出复位 
				GunBaseOut();
				delay_ms(3000);
				GunBaseIn();
				delay_ms(3000);
			
				GunShootPush();
				delay_ms(1500);
				GunShootPull();
				delay_ms(1500);
				
				//等待定位系统初始化
				WaitLaserOpsPrepare();
				
				if(gRobot.courtID == BLUE_COURT)
				{
					Blue(LIGHT_BOARD_ID);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					Red(LIGHT_BOARD_ID);
				}
	
				gRobot.checkStatus=gpsCheck;	
				
				break;
			}
//			case duckedFanCheck:
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"duckedFan check!\r\n");	
//				
//				delay_ms(1000);
//			
//				SetBigDuckedFanDutyCycle(0.5f);
//				delay_ms(2000);
//				SetBigDuckedFanDutyCycle(0.0f);
//				
//				SetSmallDuckedFanDutyCycle((smallDuckedFan_t){0.5f, 0.0f, 0.0f, 0.0f});
//				delay_ms(2000);
//				SetSmallDuckedFanDutyCycle((smallDuckedFan_t){0.0f, 0.5f, 0.0f, 0.0f});
//				delay_ms(2000);
//				SetSmallDuckedFanDutyCycle((smallDuckedFan_t){0.0f, 0.0f, 0.5f, 0.0f});
//				delay_ms(2000);
//				SetSmallDuckedFanDutyCycle((smallDuckedFan_t){0.0f, 0.0f, 0.0f, 0.5f});
//				delay_ms(2000);
//				SetSmallDuckedFanDutyCycle((smallDuckedFan_t){0.0f, 0.0f, 0.0f, 0.0f});
//				
//				WaitLaserOpsPrepare();
//				gRobot.checkStatus=gpsCheck;
//				
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//						(uint8_t *)"\r\n gps check!\r\n");
//				delay_ms(1000);
//				
//				break;
//			}
			case gpsCheck:
			{		
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"P %d %d %d ", (int)GetX(),(int)GetY(),(int)(GetAngle()*100.0f));
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"L %d %d ", (int)laserValue.laserOne,(int)laserValue.laserTwo);
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"%d %d ", (int)laserValue.laserThr,(int)laserValue.laserFor);
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"%d %d ", (int)laserValue.laserFiv,(int)laserValue.laserSix);
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"%d %d ", (int)laserValue.laserSev,(int)laserValue.laserEig);
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\n");	
				
				break;
			}
			default :
				break;
			
		}
	}
}


void HardwareInit(void)
{
	//初始化定时器
	TIM_Init(TIM2, 99, 839, 1, 0);
	
	//蜂鸣器
	GPIO_Init_Pins(GPIOC,GPIO_Pin_3,GPIO_Mode_OUT);
	
	//红蓝场	调试 自检 复位	重启	释放令牌
	GPIO_Init_Pins(GPIOD,GPIO_Pin_1,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOD,GPIO_Pin_3,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOD,GPIO_Pin_4,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOD,GPIO_Pin_5,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOD,GPIO_Pin_6,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOD,GPIO_Pin_7,GPIO_Mode_IN);
	
	GPIO_Init_Pins(GPIOE,GPIO_Pin_3,GPIO_Mode_IN);
	GPIO_Init_Pins(GPIOE,GPIO_Pin_5,GPIO_Mode_IN);
	
	//比例阀dac,adc初始化
	GasPressureADCInit();
	DAC1_Init();
	GasPressureControl(SHOOT_GAS_PRESSURE);
	
	//初始化与激光矫正板串口
	USART6_Init(115200);
	
	//初始化调参数串口 手柄WIFI
	USART2_Init(921600);
	
//	//初始化手柄WIFI串口
//	USART3_Init(921600);
	
	//初始化手柄蓝牙串口
	UART4_Init(921600);
	
	//初始化复位转向串口
	UART5_Init(19200);
	
	//初始化调试蓝牙串口
	USARTDMASendInit(DEBUG_USART,DebugUSARTDMASendBuf,&USART1_Init,921600);
	
	
	//初始化CAN1
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);

	IWDG_Init(1,50); // 11ms-11.2ms	
}

void MotorInit(void)
{
	//驱动器初始化
	ElmoInit(CAN1);
	ElmoInit(CAN2);

	//配置驱动轮电机速度环 acc/dec 100,000,000
	VelLoopCfg(CAN1, LEFT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(CAN1, RIGHT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(CAN1, LEFT_REAR_ID, 100000000, 100000000);
	VelLoopCfg(CAN1, RIGHT_REAR_ID, 100000000, 100000000);
	
	//配置转向电机位置环 acc/dec 1,500,000,000 vel 430*32768 14,000,000
	PosLoopCfg(CAN1, LEFT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(CAN1, RIGHT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(CAN1, LEFT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(CAN1, RIGHT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	
	//配置手臂电机位置环 acc/dec 2,000,000 vel 1,000,000
	PosLoopCfg(CAN2, ARM_MOTOR_ID_BLUE, 2000000, 2000000,1000000);
	PosLoopCfg(CAN2, ARM_MOTOR_ID_RED, 2000000, 2000000,1000000);
	
	//配置发射机构电机位置环 acc/dec 5,000,000 vel 2,000,000
	PosLoopCfg(CAN2, SHOOT_PITCH_LEFT_MOTOR_ID, 5000000, 5000000,2000000);
	PosLoopCfg(CAN2, SHOOT_PITCH_RIGHT_MOTOR_ID, 5000000, 5000000,2000000);
	
	//电机使能
	MotorOn(CAN1,LEFT_FRONT_ID);
	MotorOn(CAN1,RIGHT_FRONT_ID);
	MotorOn(CAN1,LEFT_REAR_ID);
	MotorOn(CAN1,RIGHT_REAR_ID);
	MotorOn(CAN1,LEFT_FRONT_TURNING_ID);
	MotorOn(CAN1,RIGHT_FRONT_TURNING_ID);
	MotorOn(CAN1,LEFT_REAR_TURNING_ID);
	MotorOn(CAN1,RIGHT_REAR_TURNING_ID);

	MotorOn(CAN2,ARM_MOTOR_ID_BLUE);
	MotorOn(CAN2,ARM_MOTOR_ID_RED);
	
	MotorOn(CAN2,SHOOT_PITCH_LEFT_MOTOR_ID);
	MotorOn(CAN2,SHOOT_PITCH_RIGHT_MOTOR_ID);
}

//发送调试数据
void SendDebugInfo(void)
{
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"%d T %d ", (int)gRobot.walkStatus,(int)GetTimeCounter());

	//左前轮给定速度和实际速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"lf %d %d ", (int)gRobot.wheelState.leftFrontTarget.vel , (int)gRobot.wheelState.leftFrontAct.vel);

	//左前轮给定位置和实际位置
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.leftFrontTarget.direction*10.0f) , (int)(gRobot.wheelState.leftFrontAct.direction*10.0f));

	//右前轮给定速度和实际速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)gRobot.wheelState.rightFrontTarget.vel , (int)gRobot.wheelState.rightFrontAct.vel);

	//右前轮给定位置和实际位置
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.rightFrontTarget.direction*10.0f) , (int)(gRobot.wheelState.rightFrontAct.direction*10.0f));

	//左后轮给定速度和实际速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"lr %d %d ", (int)gRobot.wheelState.leftRearTarget.vel , (int)gRobot.wheelState.leftRearAct.vel);

	//左后轮给定位置和实际位置
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.leftRearTarget.direction*10.0f) , (int)(gRobot.wheelState.leftRearAct.direction*10.0f));

	//右后轮给定速度和实际速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)gRobot.wheelState.rightRearTarget.vel , (int)gRobot.wheelState.rightRearAct.vel);

	//右后轮给定位置和实际位置
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.rightRearTarget.direction*10.0f) , (int)(gRobot.wheelState.rightRearAct.direction*10.0f));
	
//	//定位系统坐标和姿态角度
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"op %d %d %d ", (int)(gRobot.debugInfomation.tempX) , (int)(gRobot.debugInfomation.tempY) , (int)(gRobot.debugInfomation.tempAngle*10.0f));
//	//定位系统速度和角速度
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d %d ", (int)(gRobot.debugInfomation.tempSpeedX) , (int)(gRobot.debugInfomation.tempSpeedY) , (int)(gRobot.debugInfomation.tempWZ*10.0f));

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"cp %d %d ", (int)(gRobot.debugInfomation.x) , (int)(gRobot.debugInfomation.y));
				
		//定位系统坐标和姿态角度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"p %d %d %d ", (int)(GetX()) , (int)(GetY()) , (int)(GetAngle()*100.0f));
	//定位系统速度和角速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d %d ", (int)(GetSpeedWithoutOmega().x) , (int)(GetSpeedWithoutOmega().y) , (int)(GetWZ()*100.0f));
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d ", (int)(gRobot.debugInfomation.wzFromAngle*10.0f));
				
	//轨迹跟随虚拟位置点
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"vp %d %d %d ", (int)(gRobot.debugInfomation.virtualPos.point.x) , (int)(gRobot.debugInfomation.virtualPos.point.y) ,\
											(int)(gRobot.debugInfomation.virtualPos.direction));
	
	//轨迹跟随虚拟目标点
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"vt %d %d %d ", (int)(gRobot.debugInfomation.virtualTarget.point.x) , (int)(gRobot.debugInfomation.virtualTarget.point.y) ,\
											(int)(gRobot.debugInfomation.virtualTarget.direction));
//轨迹 disadd r2vp r2vt			
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//	(uint8_t *)"disadd %d r2vp %d r2vt %d %d ", (int)(gRobot.debugInfomation.disAdd) , (int)(gRobot.debugInfomation.disRealPos2VirPos) ,\
//								(int)(gRobot.debugInfomation.disRealPos2VirTarget),(int)gRobot.debugInfomation.VIEW_L);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"2p %d %d %d %d %d ",(int)(gRobot.debugInfomation.disRealPos2VirPos),(int)(gRobot.debugInfomation.VIEW_L),(int)(gRobot.debugInfomation.VIEW_L_K * 100),\
					(int)(gRobot.debugInfomation.scope),(int)(gRobot.debugInfomation.r2vpDetermine * 9.0f));
//					(int)(gRobot.debugInfomation.disRealPos2VirTarget),\(int)(gRobot.debugInfomation.VIEW_L));

	//轨迹跟随原始速度大小、速度调节量大小、和速度大小、姿态角度和角速度大小
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"v %d %d %d %d %d %d %d %d %d ", (int)(gRobot.debugInfomation.originVel) , (int)(gRobot.debugInfomation.originVelDir) ,\
											(int)(gRobot.debugInfomation.adjustVel.vel) , (int)(gRobot.debugInfomation.adjustVel.direction),\
											(int)(gRobot.debugInfomation.sumVel*cos(gRobot.debugInfomation.sumVelDir/180.f*PI)),\
												(int)(gRobot.debugInfomation.sumVel*sin(gRobot.debugInfomation.sumVelDir/180.f*PI)),\
											(int)(gRobot.debugInfomation.posAngle*100),(int)(gRobot.debugInfomation.omega*10),\
												(int)(gRobot.debugInfomation.posAngleVP*100));

	//速度环输出的速度大小和方向
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"ov %d %d ", (int)(gRobot.debugInfomation.outputVel*cos(gRobot.debugInfomation.outputDirection/180.0f*PI)) , \
					(int)(gRobot.debugInfomation.outputVel*sin(gRobot.debugInfomation.outputDirection/180.0f*PI)));
	
//	//分解到每个轮子的速度大小和方向
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.wheelVel.leftFront.vel) , (int)(gRobot.debugInfomation.wheelVel.leftFront.direction));

//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.wheelVel.rightFront.vel) , (int)(gRobot.debugInfomation.wheelVel.rightFront.direction));

//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.wheelVel.leftRear.vel) , (int)(gRobot.debugInfomation.wheelVel.leftRear.direction));

//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.wheelVel.rightRear.vel) , (int)(gRobot.debugInfomation.wheelVel.rightRear.direction));
				
	//轨迹跟随已经走过的路程和总路程
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"l %d %d ", (int)(gRobot.debugInfomation.pathLength) , (int)(gRobot.debugInfomation.totalLength));
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"l %d ", (int)(gRobot.debugInfomation.pathLength));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.robotlenVP) , (int)(gRobot.debugInfomation.robotlenVT));

	//实际手臂角度
	if(gRobot.courtID == BLUE_COURT)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"B %d ", (int)(gRobot.robotMotorState.actArmPosBlue));
	}
	else if (gRobot.courtID == RED_COURT)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"R %d ", (int)(gRobot.robotMotorState.actArmPosRed));
	}
				
	//发射机构的俯仰角、气压值
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"g %d %d %d %d %d %d %d ",(int)gunStatus,(int)(-gRobot.robotMotorState.expLeftPitchPos*100),(int)(-gRobot.robotMotorState.actLeftPitchPos*100),\
					(int)(gRobot.robotMotorState.expRightPitchPos*100),(int)(gRobot.robotMotorState.actRightPitchPos*100),\
						(int)(gRobot.debugInfomation.gasPressure*1000),(int)(GetGasPressure()*1000));
					
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"S %d ", (int)(gRobot.debugInfomation.posAngleShift));
					
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"d %d ", (int)(gRobot.disReduce));
	if(gRobot.walkStatus == standBy)
	{	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"ad %d %d ",(int)(gRobot.manualPos.angleNote*10), (int)(gRobot.manualPos.angleAdjust*1000));	
	}				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"I %d %d %d ",(int)(gRobot.debugInfomation.robotlenIncrement), (int)(gRobot.debugInfomation.robotlenIncrement_last),(int)(gRobot.debugInfomation.robotlenIncrement_or));					
	if(gRobot.walkStatus == omegaTestPara)
	{	
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.debugInfomation.giveOmega),(int)(gRobot.debugInfomation.circleCount));	
	}		
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"f %d %d %d %d ",(int)(gRobot.smallDuckedFan.LF*100),(int)(gRobot.smallDuckedFan.RF*100),\
//					(int)(gRobot.smallDuckedFan.LR*100),(int)(gRobot.smallDuckedFan.RR*100));
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d %d %d %d %d %d ", (int)(gRobot.debugInfomation.vTarget.startPtr),(int)(gRobot.debugInfomation.vTarget.endPtr),(int)(gRobot.debugInfomation.vTarget.u * 100),\
//				(int)(GetRingBufferPointVell(gRobot.debugInfomation.vTarget.startPtr)),(int)(GetRingBufferPointVell(gRobot.debugInfomation.vTarget.endPtr)),\
//				(int)(GetRingBufferPoint(gRobot.debugInfomation.vTarget.startPtr).x),(int)(GetRingBufferPoint(gRobot.debugInfomation.vTarget.startPtr).y));
		
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d ",(int)(GetGasPressureADCValue()));

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d ",(int)(pushShagaiCount));	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d ",(int)(gRobot.debugInfomation.waitStableFlag));
				
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d ",(int)(gRobot.wheelState.leftFrontOverCycle));
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d ",(int)(gRobot.wheelState.rightRearOverCycle));

	//左前轮航向电机速度
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.turnVel.lf) , (int)(gRobot.wheelState.turnVel.rf));
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.turnVel.lr) , (int)(gRobot.wheelState.turnVel.rr));
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"%d", (int)(gRobot.debugInfomation.r2vpDetermine * 20));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\n");	
}

//检查驱动器状态
void CheckDriverState(void)
{
		//判断驱动器通信累计中断次数
		#define WHEEL_LOST_SUM_TIMES (25)
		//判断驱动器通信连续中断次数
		#define WHEEL_LOST_CONTINIOUS_TIMES (8)

	//读取电机位置和速度  驱动5055为0 航向2006为1
		ReadActualPos(CAN1 , 1);
		ReadActualVel(CAN1 , 1);
		ReadActualVel(CAN1 , 0);
		ReadActualPos(CAN2 , 0);
	
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"a %d \r\n",(int)GetTimeCounter());

		//驱动器心跳包自加
		gRobot.wheelState.leftFrontHB++;
		gRobot.wheelState.rightFrontHB++;
		gRobot.wheelState.leftRearHB++;
		gRobot.wheelState.rightRearHB++;
		gRobot.wheelState.leftFrontTurnHB++;
		gRobot.wheelState.rightFrontTurnHB++;
		gRobot.wheelState.leftRearTurnHB++;
		gRobot.wheelState.rightRearTurnHB++;

		gRobot.wheelState.leftFrontHB_Continious++;
		gRobot.wheelState.rightFrontHB_Continious++;
		gRobot.wheelState.leftRearHB_Continious++;
		gRobot.wheelState.rightRearHB_Continious++;
		gRobot.wheelState.leftFrontTurnHB_Continious++;
		gRobot.wheelState.rightFrontTurnHB_Continious++;
		gRobot.wheelState.leftRearTurnHB_Continious++;
		gRobot.wheelState.rightRearTurnHB_Continious++;

		//判断是否有驱动器进入硬件中断
		if(gRobot.wheelState.leftFrontHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftFrontHardFault! ");			
		}
		if(gRobot.wheelState.rightFrontHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightFrontHardFault! ");			
		}
		if(gRobot.wheelState.leftRearHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftRearHardFault! ");			
		}
		if(gRobot.wheelState.rightRearHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightRearHardFault! ");			
		}
		if(gRobot.wheelState.leftFrontTurnHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftFrontTurnHardFault! ");			
		}

		if(gRobot.wheelState.rightFrontTurnHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightFrontTurnHardFault! ");			
		}
		if(gRobot.wheelState.leftRearTurnHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftRearTurnHardFault! ");			
		}
		if(gRobot.wheelState.rightRearTurnHF&WHEEL_INTO_HF)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightRearTurnHardFault! ");			
		}

		//判断驱动器是否能读到编码器
		if(gRobot.wheelState.leftFrontHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftFrontEncoderLost! ");			
		}
		if(gRobot.wheelState.rightFrontHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightFrontEncoderLost! ");			
		}
		if(gRobot.wheelState.leftRearHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftRearEncoderLost! ");			
		}
		if(gRobot.wheelState.rightRearHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightRearEncoderLost! ");			
		}
		if(gRobot.wheelState.leftFrontTurnHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftFrontTurnEncoderLost! ");			
		}
		if(gRobot.wheelState.rightFrontTurnHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightFrontTurnEncoderLost! ");			
		}
		if(gRobot.wheelState.leftRearTurnHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"LeftRearTurnEncoderLost! ");			
		}
		if(gRobot.wheelState.rightRearTurnHF&WHEEL_LOST_ENCODER)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"RightRearTurnEncoderLost! ");			
		}
		
		//检查和驱动器的通信是否正常 连续
		if(gRobot.wheelState.leftFrontHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftFrontSUMLOST! ");
			gRobot.wheelState.leftFrontHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.leftFrontHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.rightFrontHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightFrontSUMLOST! ");
			gRobot.wheelState.rightFrontHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.rightFrontHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.leftRearHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftRearSUMLOST! ");
			gRobot.wheelState.leftRearHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.leftRearHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.rightRearHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightRearSUMLOST! ");
			gRobot.wheelState.rightRearHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.rightRearHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.leftFrontTurnHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftFrontTurnSUMLOST! ");
			gRobot.wheelState.leftFrontTurnHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.leftFrontTurnHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.rightFrontTurnHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightFrontTurnSUMLOST! ");
			gRobot.wheelState.rightFrontTurnHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.rightFrontTurnHB=WHEEL_LOST_SUM_TIMES+1;
		}

		if(gRobot.wheelState.leftRearTurnHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftRearTurnSUMLOST! ");
			gRobot.wheelState.leftRearTurnHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.leftRearTurnHB=WHEEL_LOST_SUM_TIMES+1;
		}
		if(gRobot.wheelState.rightRearTurnHB>WHEEL_LOST_SUM_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightRearTurnSUMLOST! ");
			gRobot.wheelState.rightRearTurnHF|=WHEEL_LOST_SUM_COMMUNICATION;
			gRobot.wheelState.rightRearTurnHB=WHEEL_LOST_SUM_TIMES+1;
		}
		
		//检查和驱动器的通信是否正常 连续
		if(gRobot.wheelState.leftFrontHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftFrontCONTINIOUSLOST! ");
			gRobot.wheelState.leftFrontHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.leftFrontHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.rightFrontHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightFrontCONTINIOUSLOST! ");
			gRobot.wheelState.rightFrontHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.rightFrontHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.leftRearHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftRearCONTINIOUSLOST! ");
			gRobot.wheelState.leftRearHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.leftRearHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.rightRearHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightRearCONTINIOUSLOST! ");
			gRobot.wheelState.rightRearHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.rightRearHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.leftFrontTurnHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftFrontTurnCONTINIOUSLOST! ");
			gRobot.wheelState.leftFrontTurnHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.leftFrontTurnHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.rightFrontTurnHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightFrontTurnCONTINIOUSLOST! ");
			gRobot.wheelState.rightFrontTurnHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.rightFrontTurnHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.leftRearTurnHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"LeftRearTurnCONTINIOUSLOST! ");
			gRobot.wheelState.leftRearTurnHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.leftRearTurnHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}
		if(gRobot.wheelState.rightRearTurnHB_Continious>WHEEL_LOST_CONTINIOUS_TIMES)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"RightRearTurnCONTINIOUSLOST! ");
			gRobot.wheelState.rightRearTurnHF|=WHEEL_LOST_CONTINIOUS_COMMUNICATION;
			gRobot.wheelState.rightRearTurnHB=WHEEL_LOST_CONTINIOUS_COMMUNICATION+1;
		}

		if(gRobot.CANFailFlag)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
								(uint8_t *)"DriverCANBusFail! ");			
		}
		
		if(gRobot.wheelState.leftFrontHF||gRobot.wheelState.leftRearHF||gRobot.wheelState.rightFrontHF||gRobot.wheelState.rightRearHF||\
			gRobot.wheelState.leftFrontTurnHF||gRobot.wheelState.leftRearTurnHF||gRobot.wheelState.rightFrontTurnHF||gRobot.wheelState.rightRearTurnHF)
		{
//			if(gRobot.walkStatus != retryState)
//			{
				gRobot.walkStatus = stop;	
//			}
			
			//驱动器硬件中断 编码器lost CAN通信失败 亮绿灯
			Green(LIGHT_BOARD_ID);
		}
}

//检查CAN发送是否正常
void CheckCANSendState(void)
{
	//检查CAN发送是否成功
	if(gRobot.CANFailFlag&CAN1_FAIL_FLAG)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"CAN1SendFail! ");
	}
	if(gRobot.CANFailFlag&CAN2_FAIL_FLAG)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"CAN2SendFail! ");			
	}
	//fix me此处无论是哪条CAN总线错误都会进入停止状态
	if(gRobot.CANFailFlag)
	{
//		if(gRobot.walkStatus != retryState)
//		{
			gRobot.walkStatus = stop;	
//		}
		
		//发送失败 亮粉灯
		Pink(LIGHT_BOARD_ID);		
	}	
}

void MotorTest(void)
{
	static int timeCounter = 0;
	static float direction = 0.0f;
	
	timeCounter++;
	if(timeCounter<=180)
	{
		direction+=2.0f;
	}
	else if(timeCounter<=360)
	{
		direction-=2.0f;
	}
	timeCounter%=360;
	OutputVel2Wheel(1000.0f ,direction , 0.0f);
}

//轮子初始方向初始化
void WheelDirectionInit(void)
{
	if(gRobot.courtID == BLUE_COURT)
	{
		OutputVel2Wheel(0.0f ,131.4773f , 0.0f);
	}
	else if(gRobot.courtID == RED_COURT)
	{
		OutputVel2Wheel(0.0f ,48.5227f , 0.0f);
	}
//	OutputVel2Wheel(0.0f ,180.f , 0.0f);
}

//自检 转向复位 状态判断 初始状态切换
void KeySwitchCheck(void)
{
	uint8_t i = 100;
	uint8_t steerResetTriggerTimes = 0;
	uint8_t selfCheckTriggerTimes = 0;
	
	uint8_t steerResetFlag = 0;

	while(i--)
	{
		delay_ms(10);
		
		//复位开关
		if(STEER_RESET_KEYSWITCH)
		{
			steerResetTriggerTimes++;
		}
		else
		{
			steerResetTriggerTimes = 0;
		}	
		//自检开关
		if(SELF_CHECK_KEYSWITCH)
		{
			selfCheckTriggerTimes++;
		}
		else
		{
			selfCheckTriggerTimes = 0;
		}
	}
	//复位开关满足条件时 置复位标志
	if(steerResetTriggerTimes >= 80)
	{
		steerResetFlag = 1;
	}
	
	//自检开关满足条件时 置自检标志
	if(selfCheckTriggerTimes >= 80)
	{
		gRobot.selfCheckFlag=ROBOT_SELF_CHECK;
	}
	
	//转向复位
	if(steerResetFlag)
	{
		if(!GetSteerLoopShift())
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"SteerResetTimeOut!\r\n");
			BEEP_ON;
		}
		
		GasPressureControl(0.8f);
		
		while(STEER_RESET_KEYSWITCH)
		{
			static uint8_t timeCounter = 0;
			
			delay_ms(5);
			
			//检查Valve_KeySwitch状态 并切换手臂气缸状态
			Valve_KeySwitchCheck();
			
			GasPressureControl(0.8f);
			//1000轮子往前转
			SendCmd2Driver(1000.0f , 0.0f , 1000.0f , 0.0f,
						   1000.0f , 0.0f , 1000.0f , 0.0f);
			
			if(GetGasPressure()>0.75f)
			{
				timeCounter>100?BEEP_ON:BEEP_OFF;
			}
			
			timeCounter++;
			timeCounter=timeCounter%200;
		}
	}
	
	//自检 挂起走形和速度控制任务 发送获取激光值命令
	if(gRobot.selfCheckFlag == ROBOT_SELF_CHECK)
	{
		SendGetLaserCmd();
		OSTaskSuspend(Walk_TASK_PRIO);
		OSTaskSuspend(VELCRL_TASK_PRIO);
//		OSTaskSuspend(SHOOT_TASK_PRIO);
	}
	else
	{
		OSTaskDel(SELFCHECK_TASK_PRIO);
	}
}

float posAngle_EnterManulMode = 0.0f;
	
//调试发射机构判断 连续触发15个20ms进入调试模式
void GunModeCheck(void)
{
	static uint8_t trigerTimes = 0;

	if(MANUAL_MODE_KEYSWITCH)
	{
		trigerTimes++;
		trigerTimes=trigerTimes>100?100:trigerTimes;
	}
	else
	{
		trigerTimes = 0;
	}

	if(trigerTimes > 15 && gRobot.gun.mode == GUN_AUTO_MODE)
	{
		posAngle_EnterManulMode = GetAngle();
		gRobot.manualPos.angleNote = posAngle_EnterManulMode;
		
		gRobot.gun.mode = GUN_MANUAL_MODE;
		GunSendMsg2Main(GUN_MANUAL_MODE_MSG);
		
		BEEP_ON;
		delay_ms(500);
		BEEP_OFF;
	}
}
//手臂气阀开关检查
void Valve_KeySwitchCheck(void)
{
	//Valve_KEYSWITCH连续为1次数
	static uint8_t pTrigerTimes = 0;
	//Valve_KEYSWITCH连续为0次数
	static uint8_t nTrigerTimes = 0;
	
	static uint8_t valveFlag = 0;

	if(VALVE_KEYSWITCH)
	{
		nTrigerTimes = 0;
		
		pTrigerTimes++;
		pTrigerTimes=pTrigerTimes>100?100:pTrigerTimes;
	}
	else if(!VALVE_KEYSWITCH)
	{
		pTrigerTimes = 0;
		
		nTrigerTimes++;
		nTrigerTimes=nTrigerTimes>100?100:nTrigerTimes;
	}

	if(pTrigerTimes >= 10 && valveFlag == 0)
	{
		valveFlag = 1;
		
		//松开令牌
		ReleaseTheGerege();
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"ValveSwitchOn!\r\n");
	}
		
	if(nTrigerTimes >= 10 && valveFlag == 1)
	{
		valveFlag = 0;
		
		//抓住令牌
		GraspTheGerege();
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"ValveSwitchOff!\r\n");
	}
}
