#include "robot.h"
#include "math.h"
#include "dma.h"
#include "gpio.h"
#include "ucos_ii.h"
#include "telecontroller.h"
#include "light.h"
#include "steerReset.h"
#include "pps.h"

#define VELL_GOTO_GOBI	0.94f//B 0.9f R0.9
#define VELL_GOTO_TZ	0.5f 
#define VELL_GOTO_THROW1STSHAIGAI	0.78f	//B 0.78f	R 0.78f
#define VELL_GOTO_GET2NDSHAIGAI	0.6f	//B 0.6f
#define	VELL_GOTO_THROW2NDSHAIGAI	0.6f	//B 0.6f
#define	VELL_GOTO_GET3RDSHAIGAI	0.55f	//B 0.55f
#define	VELL_GOTO_THROW3RDSHAIGAI	0.6f	//B 0.6f

vector_t finaltVel;
static uint32_t sendMsgFlag = 0;
static uint8_t sendShootCmdFlag = 0;
static mainCmd_t mainCmdRecieve = {0};

//extern uint8_t co3RDFlag,co5THFlag;
extern uint8_t	send3RDOK, send5THOK;
extern uint8_t	co3RDOK, co5THOK;

uint8_t shagaiPosionFlag = 0;

//路径红蓝场初始化 发射机构与主控间通信初始化 发射机构复位
void RobotInit(void)
{
	//路径红蓝场初始化
	PathInit(gRobot.courtID);
	
	gRobot.selfCheckFlag = 0;
	
	//主程序与发射机构间通信初始化
	Gun_Main_CommunicateInit();
	
	//发射机构初始化
	GunInit();
}

//主程序与发射机构间通信初始化
void Gun_Main_CommunicateInit(void)
{
	sendMsgFlag = 0;
	
	//主程序向发射机构传递消息序列初始化
	MainMsg2GunInit();

	//发射机构向主控传递消息序列初始化
	GunMsg2MainInit();
	
	MainCmdReset();
	GunCmdReset();
	
	sendMsgFlag = 0;
	sendShootCmdFlag = 0;
}



void Walk(void)
{
	static int timeCounter = 0;
	static uint8_t testState = 0;
//	static int shootPosAngleLoop = 0;
	static uint8_t resetDoneFlag = 0;
	static uint8_t judgeStopFlag = 0, judgeStopDoneFlag = 0;
	static uint8_t waitStableFlag = 0;
	static wheelState_t lastWheelState = {0};
	
	gRobot.debugInfomation.waitStableFlag = waitStableFlag;
	
	Gun2MainMsgProcess();
	
	//BT WIFI收到的消息处理
	TeleCmdProssGather();
	
	//telestop判断
	if(gRobot.teleCommand.stopFlag == TELESTOP)
	{
		gRobot.walkStatus = stop;
		
		//手柄按下stop 亮青灯
		Blue_green(LIGHT_BOARD_ID);
	}
	
	//调试模式判断
	if(mainCmdRecieve.isManualMode==GUN_MANUAL_MODE_MSG)
	{
		gRobot.walkStatus = standBy;
	}
	
	//重启开关检查
	ResetCheck();
	
	if(gRobot.resetFlag == ROBOT_RETRY)
	{
		gRobot.walkStatus = retryState;
	}
	if(gRobot.resetFlag == ROBOT_RESTART)
	{
		gRobot.walkStatus = startFromRetry;
		gRobot.resetFlag = ROBOT_NONE;
	}
	
	
	switch(gRobot.walkStatus)
	{
		//等待触发0
		case waitForStart:
		{
			if(gRobot.teleCommand.nextFlag == TELENEXT)
			{
				//触发后清楚记录的轨迹长度
//				ClearPathLen();
				
				//开始计时
				SetCountTimeFlag();
				
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				//开始第一段走行
				gRobot.walkStatus = goToInfrontGobi;
//				gRobot.walkStatus = omegaTestPara;
//				gRobot.walkStatus = testPara;
			}
			break;
		}
		//从出发区到交接区1
		case goToInfrontGobi:
		{
			//旋转手臂
			ArmRotate();
			
			//松开令牌 4000松 交接位置4752
			if(GetY()>4050.0f)
			{
				ReleaseTheGerege();
			}	
			
			//铲拐骨
			ShovelShagai(FIRSTSHAGAI);
			
			//缓冲气缸控制
			CushionControl(FIRSTSHAGAI);
			
			float dis2FinalX = GetX() - getFirstShagaiPath[GET_FIRST_SHAGAI_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - getFirstShagaiPath[GET_FIRST_SHAGAI_PATH_NUM -1].point.y;
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<50.0f&&JudgeSpeedLessEqual(1700.0f))\
				||(GetY()>=5100.0f&&JudgeSpeedLessEqual(1700.0f)))
			{
				if(gRobot.courtID == BLUE_COURT)
				{
					OutputVel2Wheel(1000.0f,75.0f,0.0f);
//					OutputVel2Wheel(1000.0f,78.0f,0.0f);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					OutputVel2Wheel(1000.0f,105.0f,0.0f);
//					OutputVel2Wheel(1000.0f,102.0f,0.0f);
				}				
				
				judgeStopFlag = 0;
				
				gRobot.walkStatus = get1stShagai;		
				return;	
			}
			
			//跟随轨迹
//			PathFollowing(VELL_GOTO_GOBI,2);
			break;
		}
		//取第一个兽骨2
		case get1stShagai:
		{
			static uint8_t posAchieve = 0, posTimeOut = 0;
			if(gRobot.courtID == BLUE_COURT)
			{
				OutputVel2Wheel(1000.0f,75.0f,0.0f);
//				OutputVel2Wheel(1000.0f,85.0f,0.0f);
			}
			else if(gRobot.courtID == RED_COURT)
			{
				OutputVel2Wheel(1000.0f,105.0f,0.0f);
//				OutputVel2Wheel(1000.0f,95.0f,0.0f);
			}	
			
			if(JudgeStop2(FIRSTSHAGAI,3.f,5) && judgeStopFlag == 0)
			{
				judgeStopFlag = 1;
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"STOPOK\r\n");
			}
			if(judgeStopFlag)
			{
				if(GetY() > 5382.0f )
				{
					posAchieve	=	1;
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				  (uint8_t *)"POSOK\r\n");
				}
				else
				{
					posTimeOut++;
				}
			}
			if(posAchieve || posTimeOut > 20)
			{
				posAchieve = 0;
				posTimeOut = 0;
				judgeStopFlag = 0;
				
				//旋转手臂复位
				ArmRotateReset();
				
				//夹拐骨
				GunClawClamp();
				
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
//				ClearRingBuffer();
				go2TZPath[0] =  (Pose_t){GetX(),GetY(),GetAngle(),0.0f};
//				InputPoints2RingBuffer(go2TZPath, GO2_TZ_PATH_NUM);		
				
//				ClearPathLen();
				
				//收气缸
				CushionCylinderRecovery();

//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
								
				shagaiPosionFlag = FIRSTSHAGAIPOSION;

				gRobot.walkStatus = goToInfrontTZ;
			}
			break;
		}
		//取完第一个兽骨后到投掷区外等待3
		case goToInfrontTZ:
		{
			//持兽骨
			HoldShagai(FIRSTSHAGAI);
			
			float dis2FinalX = GetX() - go2TZPath[GO2_TZ_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - go2TZPath[GO2_TZ_PATH_NUM -1].point.y;
			Pose_t lastPos;
			
			if(gRobot.courtID == BLUE_COURT)
			{
				lastPos =(Pose_t){-3869.89f, 3049.60f, -73.81f, 0.0f};
			}
			else if(gRobot.courtID == RED_COURT)
			{
				lastPos =(Pose_t){3869.89f, 3049.60f, 73.81f, 0.0f};
			}

			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<50.0f&&JudgeSpeedLessEqual(200.0f))
			{
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
//				ClearRingBuffer();
				
				if(pushShagaiCount==0)
				{
					if(gRobot.courtID == BLUE_COURT)
					{
						PathLineInterpolation(throwFirstShagaiPath,GetPosPresent(),throwFirstShagaiPath[3],\
																	0.33f,0.66f,-77.81f,-77.81f);
					}
					else if(gRobot.courtID == RED_COURT)
					{
						PathLineInterpolation(throwFirstShagaiPath,GetPosPresent(),throwFirstShagaiPath[3],\
																	0.33f,0.66f,77.81f,77.81f);
					}
				}
				else if(pushShagaiCount>0)
				{	
					if(gRobot.courtID == BLUE_COURT)
					{
						PathLineInterpolation(throwFirstShagaiPath,GetPosPresent(),lastPos,\
																	0.33f,0.66f,-73.81f,-73.81f);
					}
					else if(gRobot.courtID == RED_COURT)
					{
						PathLineInterpolation(throwFirstShagaiPath,GetPosPresent(),lastPos,\
																	0.33f,0.66f,73.81f,73.81f);
					}
				}
//				InputPoints2RingBuffer(throwFirstShagaiPath, THROW_FIRST_SHAGAI_PATH_NUM);

//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
				//下一步指令置零
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				gRobot.walkStatus = waitForThrow;
				
				return;
			}
			
//			PathFollowing(VELL_GOTO_TZ,2);

			break;
		}
		//等待投掷第一个兽骨命令4
		case waitForThrow:
		{
//			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			if((sendMsgFlag&0x100)==0)
			{
				MainSendMsg2Gun(GUN_PREPARE_SHOOT_MSG);
				sendMsgFlag|=0x100;
			}
				
			if(gRobot.teleCommand.nextFlag == TELENEXT)
			{
//				if((sendMsgFlag&0x100)==0)
//				{
//					MainSendMsg2Gun(GUN_PREPARE_SHOOT_MSG);
//					sendMsgFlag|=0x100;
//				}

				gRobot.teleCommand.nextFlag = TELENOCMD;
								
//				ClearPathLen();

				gRobot.walkStatus = goToThrow1stShagai;
			}
			break;
		}
		//到投掷第一个兽骨的位置5
		case goToThrow1stShagai:
		{
			float direction = 0.0f, vell = 0.0f;
			Point_t presentPoint;
			Point_t finalPoint;
			
			presentPoint.x = GetX();
			presentPoint.y = GetY();
			finalPoint.x = throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM -1].point.x;
			finalPoint.y = throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM -1].point.y;
			
			float dis2FinalX = GetX() - throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM -1].point.y;
			
			if(fabs(GetX()) < 3955.0f)	
			{	
				if((sendShootCmdFlag&0x01)==0 && pushShagaiCount==0)	
				{
					 if(gRobot.courtID == BLUE_COURT)
					 {
//						 if((GetAngle()+(GetWZ()*0.16f)-GetRingBufferPointPoseAngle(1)) < -4.0f)
//						 {
//							 waitStableFlag = 1;
//						 }
					 }
					 if(gRobot.courtID == RED_COURT)
					 {
//						 if((GetAngle()+(GetWZ()*0.12f)-GetRingBufferPointPoseAngle(1)) > 4.0f)
//						 {
//							 waitStableFlag = 1;
//						 }
					 }
					  
					if(waitStableFlag ==0)
					{
						MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
						MainIsShootDoneReset();
						sendShootCmdFlag|=0x01;	
					}
				}
					
			}
			
//			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<40.0f&&JudgeSpeedLessEqual(200.0f))
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f&&JudgeSpeedLessEqual(300.0f))\
				 || sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f || 
				 (fabs(GetX())<4000.0f	&& fabs(gRobot.debugInfomation.originVel)<100.0f))
			{
				if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 30.0f)
				{	
//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
					//下一步指令置零
					gRobot.teleCommand.nextFlag = TELENOCMD;

					lastWheelState = gRobot.wheelState;
					gRobot.walkStatus = throw1stShagai;
				}	
				else
				{
//					direction = CalculateLineAngle(presentPoint,finalPoint);
					vell = sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) * 3.0f;
					
					OutputVel2Wheel(vell,direction,0.0f);
					
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"OPENLOOPTZ!\r\n");
				}
				
				return;
			}
			
//			PathFollowing(VELL_GOTO_THROW1STSHAIGAI,2);
			break;
		}
		//投掷第一个兽骨6
		case throw1stShagai:
		{
			if(mainCmdRecieve.isShootDone==GUN_SHOOT_DONE_MSG)
			{
				
				//根据情况增删
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
				if(gRobot.courtID == BLUE_COURT)
				{
					gRobot.gun.paraNum = 4;
				}
				else if(gRobot.courtID == RED_COURT)
				{
					gRobot.gun.paraNum = 2;
				}
				SetShootPara();

				MainIsShootDoneReset();
//				ClearRingBuffer();
				get2ndShagaiPath[0] = (Pose_t){GetX(),GetY(),GetAngle(),0.0f};
//				InputPoints2RingBuffer(get2ndShagaiPath, GET_2ND_SHAGAI_PATH_NUM);
				
//				ClearPathLen();				
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				OSTimeDly(10);
					
//				gRobot.teleCommand.nextFlag = TELENOCMD;

				gRobot.walkStatus = goToGet2ndShagai;
			}
			
			if(waitStableFlag && (sendShootCmdFlag&0x01)==0)	
			{
			  if(gRobot.courtID == BLUE_COURT)
				{

//					 	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);	
					 	
					 	OSTimeDly(100);
					
						MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
						MainIsShootDoneReset();
						sendShootCmdFlag|=0x01;				
					
				}
				if(gRobot.courtID == RED_COURT)
				{
					
					
						waitStableFlag = 0;
//				 		OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);	
						
						OSTimeDly(100);
					
						MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
						MainIsShootDoneReset();
						sendShootCmdFlag|=0x01;				
					
				}				
			}				
			if((sendShootCmdFlag&0x01)==0 && pushShagaiCount>0)
			{	
				//fabs(实际角度-(目标角度+偏移角度))>0.1f 输出角速度
				if(fabs(GetAngle()-(gRobot.teleCommand.angleshift))>2.f)
				{
					float angularVel = 0;
				
//					angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_HUGE,D_TINYANGLE_CONTROL_HUGE);

//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
					
					lastWheelState = gRobot.wheelState;
				}
				else if(((fabs(GetAngle()+gRobot.teleCommand.angleshift))>0.1f)&&(fabs(GetAngle()-(gRobot.teleCommand.angleshift))<=2.f))
				{
					float angularVel = 0;
				
//					angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_TINY,D_TINYANGLE_CONTROL_TINY);

//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
					
					lastWheelState = gRobot.wheelState;
				}
				else 
				{
					SendCmd2Driver(0.0f,lastWheelState.leftFrontTarget.direction,0.0f,lastWheelState.rightFrontTarget .direction,\
						0.0f,lastWheelState.leftRearTarget.direction,0.0f,lastWheelState.rightRearTarget.direction);
					
//					shootPosAngleLoop = GetAngleLoopNum();
				}
				
				if(gRobot.teleCommand.nextFlag == TELENEXT)
				{	
					gRobot.teleCommand.nextFlag = TELENOCMD;	
					MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
					MainIsShootDoneReset();
					sendShootCmdFlag|=0x01;
				}
			}
			break;
		}
		//到龙门区取第二个兽骨7
		case goToGet2ndShagai:
		{
			float dis2FinalX = GetX() - get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM -1].point.y;

			//铲兽骨
			ShovelShagai(SECONDSHAGAI);
			
			CushionControl(SECONDSHAGAI);
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<50.0f&&JudgeSpeedLessEqual(700.0f))\
						||(fabs(GetX())>=8700.0f&&JudgeSpeedLessEqual(700.0f)))
			{
				if(gRobot.courtID == BLUE_COURT)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					OutputVel2Wheel(700.0f,0.0f,0.0f);
				}	
				
//				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				gRobot.walkStatus = get2ndShagai;	

				return;
			} 
			
			judgeStopFlag = 0;
			judgeStopDoneFlag = 0;
			
//			if(gRobot.teleCommand.nextFlag == TELENEXT)
//			{	
//				PathFollowing(VELL_GOTO_GET2NDSHAIGAI,2);
//			}
			break;
		}
		
		//取第二个兽骨8
		case get2ndShagai:
		{
			static uint8_t posAchieve = 0, posTimeOut = 0;
			if(judgeStopDoneFlag == 0)
			{
				if(gRobot.courtID == BLUE_COURT)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					OutputVel2Wheel(700.0f,0.0f,0.0f);
				}	
			}
			
			if(JudgeStop2(SECONDSHAGAI,3.f,5) && judgeStopFlag==0)
			{
				judgeStopFlag = 1;
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				  (uint8_t *)"STOPOK\r\n");
			}
			
			if(judgeStopFlag && judgeStopDoneFlag == 0)
			{
				if(fabs(GetX()) > 8980.0f )
				{
					posAchieve	=	1;
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				  (uint8_t *)"POSOK\r\n");
				}
				else
				{
					posTimeOut++;
					posTimeOut = posTimeOut > 100 ? 100 : posTimeOut;
				}
			}
			
			if((posAchieve || posTimeOut > 20) && judgeStopDoneFlag == 0)
			{		
				judgeStopDoneFlag = 1;
				
				//夹拐骨
				GunClawClamp();
				
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);

//				ClearRingBuffer();
				throw2ndShagaiPath[0] = (Pose_t){GetX(),GetY(),GetAngle(),0.0f};
//				InputPoints2RingBuffer(throw2ndShagaiPath, THROW_2ND_SHAGAI_PATH_NUM);
				
				CushionCylinderRecovery();
				
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			}
			
			if(judgeStopDoneFlag == 1 && gRobot.teleCommand.nextFlag == TELENEXT)
			{
				posAchieve = 0;
				posTimeOut = 0;
				judgeStopFlag = 0;
				judgeStopDoneFlag= 0;
				
//				ClearPathLen();
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				shagaiPosionFlag = SECONDSHAGAIPOSION;

				gRobot.walkStatus = goToThrow2ndShagai;
			}
			break;
		}
		//到投掷第二个兽骨的位置9
		case goToThrow2ndShagai:		
		{
			float direction = 0.0f, vell = 0.0f;
			Point_t presentPoint;
			Point_t finalPoint;
			
			presentPoint.x = GetX();
			presentPoint.y = GetY();
			finalPoint.x = throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM -1].point.x;
			finalPoint.y = throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM -1].point.y;
	
			//持拐骨
			HoldShagai(SECONDSHAGAI);
		
			float dis2FinalX = GetX() - throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM -1].point.y;
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f&&JudgeSpeedLessEqual(300.0f))\
				 || sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f || 
					(fabs(GetX())<4000.0f	&& fabs(gRobot.debugInfomation.originVel)<100.0f))
			{
				if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 30.0f)
				{
//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					
					//下一步指令置零
					gRobot.teleCommand.nextFlag = TELENOCMD;

					lastWheelState = gRobot.wheelState;
					gRobot.walkStatus = throw2ndShagai;
				}
				else
				{
//					direction = CalculateLineAngle(presentPoint,finalPoint);
					vell = sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) * 3.0f;
					
					OutputVel2Wheel(vell,direction,0.0f);
					
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"OPENLOOPTZ!\r\n");
				}
				
				return;
			}
			else
			{
//				PathFollowing(VELL_GOTO_THROW2NDSHAIGAI,2);
			}

			break;
		}
		//投掷第二个兽骨10
		case throw2ndShagai:
		{						
			if(mainCmdRecieve.isShootDone==GUN_SHOOT_DONE_MSG)
			{	
				gRobot.teleCommand.nextFlag = TELENOCMD;
					
				if(fabs( ReturnLimitAngle(GetAngle() -  get3rdShagaiPath[0].direction) ) < 7.0f)
				{				
					//根据情况增删
//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
					if(gRobot.courtID == BLUE_COURT)
					{
						gRobot.gun.paraNum = 4;
					}
					else if(gRobot.courtID == RED_COURT)
					{
						gRobot.gun.paraNum = 2;
					}
					SetShootPara();				
					MainIsShootDoneReset();
					
//					ClearRingBuffer();
					get3rdShagaiPath[0] = (Pose_t){GetX(),GetY(),GetAngle(),0.0f};
//					InputPoints2RingBuffer(get3rdShagaiPath,GET_3RD_SHAGAI_PATH_NUM);
//					
//					ClearPathLen();
//					
//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					OSTimeDly(10);
					
					gRobot.walkStatus = goToGet3rdShagai;		
				}
				else 
				{
					if(gRobot.courtID == BLUE_COURT)
					{
//						//get3rdShagaiPath[0].direction正的
//						OutputVel2Wheel(0.0f,0.0f,ContiniuousAngleControl(GetContiniousAngle(), get3rdShagaiPath[0].direction + (shootPosAngleLoop-1)*360.0f));
//						OutputVel2Wheel(0.0f,0.0f,TinyAngleControl(GetAngle(), (get3rdShagaiPath[0].direction + 4.0f),P_TINYANGLE_CONTROL_HUGE,D_TINYANGLE_CONTROL_HUGE));
					}
					else if(gRobot.courtID == RED_COURT)
					{
//						OutputVel2Wheel(0.0f,0.0f,ContiniuousAngleControl(GetContiniousAngle(), get3rdShagaiPath[0].direction + (shootPosAngleLoop+1)*360.0f));
//						OutputVel2Wheel(0.0f,0.0f,TinyAngleControl(GetAngle(), (get3rdShagaiPath[0].direction - 4.0f),P_TINYANGLE_CONTROL_HUGE,D_TINYANGLE_CONTROL_HUGE));
					}
				}	
			}
			else
			{
				//fabs(实际角度-(目标角度+偏移角度))>0.1f 输出角速度
				if(fabs(GetAngle()-(gRobot.teleCommand.angleshift))>2.f)
				{
					float angularVel = 0;
				
//					angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_HUGE,D_TINYANGLE_CONTROL_HUGE);

//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
					
					lastWheelState = gRobot.wheelState;
				}
				else if((fabs(GetAngle()-(gRobot.teleCommand.angleshift))>0.1f)&&(fabs(GetAngle()-(gRobot.teleCommand.angleshift))<=2.f))
				{
					float angularVel = 0;
				
//					angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_TINY,D_TINYANGLE_CONTROL_TINY);

//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
					
					lastWheelState = gRobot.wheelState;
				}
				else 
				{
					SendCmd2Driver(0.0f,lastWheelState.leftFrontTarget.direction,0.0f,lastWheelState.rightFrontTarget .direction,\
						0.0f,lastWheelState.leftRearTarget.direction,0.0f,lastWheelState.rightRearTarget.direction);
					
//					shootPosAngleLoop = GetAngleLoopNum();
				}
				if((sendShootCmdFlag&0x02)==0 && gRobot.teleCommand.nextFlag == TELENEXT)
				{
					gRobot.teleCommand.nextFlag = TELENOCMD;
					
					MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
					MainIsShootDoneReset();
					sendShootCmdFlag|=0x02;
				}
			}
			break;
		}
		//到龙门区取第三个兽骨11
		case goToGet3rdShagai:
		{ 
			CushionControl(THIRDSHAGAI);
			//铲拐骨
			ShovelShagai(THIRDSHAGAI);
			
			float dis2FinalX = GetX() - get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM - 1].point.x;
			float dis2FinalY = GetY() - get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM - 1].point.y;
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 50.0f && JudgeSpeedLessEqual(700.0f))\
					|| (fabs(GetX())>=8700.0f && JudgeSpeedLessEqual(700.0f)))
			{				
				if(gRobot.courtID == BLUE_COURT)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					OutputVel2Wheel(700.0f,0.0f,0.0f);
				}			
				
				judgeStopFlag = 0;
				judgeStopDoneFlag = 0;
				gRobot.walkStatus = get3rdShagai;
				
				return;
			}
			
//			PathFollowing(VELL_GOTO_GET3RDSHAIGAI,2);
			break;
		}
		
		//取第三块兽骨12
		case get3rdShagai:
		{
			static uint8_t posAchieve = 0, posTimeOut = 0;
			if(judgeStopDoneFlag == 0)
			{
				if(gRobot.courtID == BLUE_COURT)
				{
					OutputVel2Wheel(700.0f,180.0f,0.0f);
				}
				else if(gRobot.courtID == RED_COURT)
				{
					OutputVel2Wheel(700.0f,0.0f,0.0f);
				}	
			}
			
			if(JudgeStop2(THIRDSHAGAI,3.f,5) && judgeStopFlag==0)
			{
				judgeStopFlag = 1;
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				  (uint8_t *)"STOPOK\r\n");
			}
			if(judgeStopFlag && judgeStopDoneFlag == 0)
			{
				if(fabs(GetX()) > 8980.0f )
				{
					posAchieve	=	1;
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				  (uint8_t *)"POSOK\r\n");
				}
				else
				{
					posTimeOut++;
					posTimeOut = posTimeOut > 100 ? 100 : posTimeOut;
				}
			}
			if((posAchieve || posTimeOut > 20) && judgeStopDoneFlag == 0)
			{
				judgeStopDoneFlag = 1;
				
				//夹拐骨
				GunClawClamp();
				
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);

//				ClearRingBuffer();
				throw3rdShagaiPath[0] = (Pose_t){GetX(),GetY(),GetAngle(),0.0f};
//				InputPoints2RingBuffer(throw3rdShagaiPath,THROW_3RD_SHAGAI_PATH_NUM);
				
				CushionCylinderRecovery();
								
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			}
			
			if(judgeStopDoneFlag == 1 && gRobot.teleCommand.nextFlag == TELENEXT)
			{
				posAchieve = 0;
				posTimeOut = 0;
				judgeStopFlag = 0;
				judgeStopDoneFlag = 0;
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
//				ClearPathLen();
				
				shagaiPosionFlag = THIRDSHAGAIPOSION;

				gRobot.walkStatus = goToThrow3rdShagai;
			}
			break;
		}
		
		//到投掷第三个兽骨的位置13
		case goToThrow3rdShagai:
		{
			float direction = 0.0f, vell = 0.0f;
			Point_t presentPoint;
			Point_t finalPoint;
			
			presentPoint.x = GetX();
			presentPoint.y = GetY();
			finalPoint.x = throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM -1].point.x;
			finalPoint.y = throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM -1].point.y;
			
			//持拐骨
			HoldShagai(THIRDSHAGAI);			
		
			float dis2FinalX = GetX() - throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM -1].point.y;
			
			if((sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f&&JudgeSpeedLessEqual(300.0f))\
				 || sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY)<30.0f || 
				(fabs(GetX())<4000.0f	&& fabs(gRobot.debugInfomation.originVel)<100.0f))
			{
				if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 30.0f)
				{
//					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					
					//下一步指令置零
					gRobot.teleCommand.nextFlag = TELENOCMD;

					lastWheelState = gRobot.wheelState;
					gRobot.walkStatus = throw3rdShagai;
				}
				else
				{
//					direction = CalculateLineAngle(presentPoint,finalPoint);
					vell = sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) * 3.0f;
					
					OutputVel2Wheel(vell,direction,0.0f);
					
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"OPENLOOPTZ!\r\n");
				}
				
				return;
			}
			else
			{
//				PathFollowing(VELL_GOTO_THROW3RDSHAIGAI,2);
			}
			
			break;
		}
		//投掷第三个兽骨14
		case throw3rdShagai:
		{
			//fabs(实际角度-(目标角度+偏移角度))>0.1f 输出角速度
			if(fabs(GetAngle()-(gRobot.teleCommand.angleshift))>2.f)
			{
				float angularVel = 0;
//			
//				angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_HUGE,D_TINYANGLE_CONTROL_HUGE);

//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
				
				lastWheelState = gRobot.wheelState;
			}
			else if((fabs(GetAngle()-(gRobot.teleCommand.angleshift))>0.1f)&&(fabs(GetAngle()-(gRobot.teleCommand.angleshift))<=2.f))
			{
//				float angularVel = 0;
			
//				angularVel = TinyAngleControl(GetAngle(),(GetRingBufferPointPoseAngle(1)+gRobot.teleCommand.angleshift),P_TINYANGLE_CONTROL_TINY,D_TINYANGLE_CONTROL_TINY);

//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),angularVel);
				
				lastWheelState = gRobot.wheelState;
			}
			else 
			{
				SendCmd2Driver(0.0f,lastWheelState.leftFrontTarget.direction,0.0f,lastWheelState.rightFrontTarget .direction,\
						0.0f,lastWheelState.leftRearTarget.direction,0.0f,lastWheelState.rightRearTarget.direction);
			}
						
			if((sendShootCmdFlag&0x04)==0 && gRobot.teleCommand.nextFlag == TELENEXT)
			{
				gRobot.teleCommand.nextFlag = TELENOCMD;
				
				MainSendMsg2Gun(GUN_SHOOT_CONFIRM_MSG);
				MainIsShootDoneReset();
				sendShootCmdFlag|=0x04;
			}
			
			if(mainCmdRecieve.isShootDone==GUN_SHOOT_DONE_MSG)
			{
//				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				
				MainIsShootDoneReset();
				gRobot.walkStatus = missionAccomplished;			
			}
			break;
		}
		//完成所有流程15
		case missionAccomplished:
		{
//			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			break;
		}
		//重试状态16
		case retryState:
		{
			if(resetDoneFlag == 0)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"RETRY\r\n");
				
				//进入重试 蜂鸣器响				
				BEEP_ON;
				OSTimeDly(50);
				BEEP_OFF;
				
				
				/*发送停止校正标志*/
				SendStopCorrect();
				
				/*手臂复位*/
				ArmRotateReset();
				
				/*抓住令牌*/
				GraspTheGerege();	
				
				/*初始化tele BT WIFI  标志位next angleshift stop置零*/
				TeleInit();			
				gRobot.teleDevice.BT.nextFlag = TELENOCMD;
				gRobot.teleDevice.WIFI.nextFlag = TELENOCMD;
				gRobot.teleDevice.BT.stopFlag = TELENOCMD;
				gRobot.teleDevice.WIFI.stopFlag = TELENOCMD;
				gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NONE;
				gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NONE;				
				gRobot.teleCommand.nextFlag = TELENOCMD;
				gRobot.teleCommand.stopFlag = TELENOCMD;
				gRobot.teleCommand.angleshift = 0.0f;
				
				/*初始化走行所用标志位*/
				judgeStopFlag = 0;
				judgeStopDoneFlag = 0;
				waitStableFlag = 0;
				
				/*初始化靠墙标志位*/
//				co3RDFlag = 0;
//				co5THFlag = 0;
				send3RDOK = 0;
				send5THOK = 0;
				co3RDOK = 0;
				co5THOK = 0;
				
				/*初始化sd卡标志位*/
				gRobot.debugInfomation.SDCardLostFlag = 0;
				
				/*转向归位*/
				//先朝向0°再读取圈数
				SendCmd2Driver(0.0f , 0.0f , 0.0f , 0.0f, 0.0f , 0.0f , 0.0f , 0.0f);
				
				//等待轮子旋转完成 （2s超时）
				for(int i = 0;i<40;i++)
				{
					//检查各个驱动器状态是否正常
					CheckDriverState();
					
					//检查CAN发送是否正常
					CheckCANSendState();
					
					//发送调试数据
					SendDebugInfo();
						
					
					if((fabs(gRobot.wheelState.leftFrontTarget.direction - gRobot.wheelState.leftFrontAct.direction) < 0.1f) &&
							(fabs(gRobot.wheelState.rightFrontTarget.direction - gRobot.wheelState.rightFrontAct.direction) <0.1f) &&
							(fabs(gRobot.wheelState.leftRearTarget.direction - gRobot.wheelState.leftRearAct.direction) < 0.1f) &&
							(fabs(gRobot.wheelState.rightRearTarget.direction - gRobot.wheelState.rightRearAct.direction) < 0.1f))	
					{
						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
							(uint8_t *)"%d\r\n",i);
						OSTimeDly(10);
						break;
					}
								
					OSTimeDly(5);
				}
				
				//更新霍尔圈数				
				if(!GetSteerLoopShift())
				{
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"SteerResetTimeOut!\r\n");
					BEEP_ON;
				}				
				
//				//等待轮子旋转完成 （5s超时）
//				for(int i = 0;i<100;i++)
//				{
//					//检查各个驱动器状态是否正常
//					CheckDriverState();
//					
//					//检查CAN发送是否正常
//					CheckCANSendState();
//					
//					//发送调试数据
//					SendDebugInfo();
//						
//					
//					if((fabs(gRobot.wheelState.leftFrontTarget.direction - gRobot.wheelState.leftFrontAct.direction) < 0.1f) &&
//							(fabs(gRobot.wheelState.rightFrontTarget.direction - gRobot.wheelState.rightFrontAct.direction) <0.1f) &&
//							(fabs(gRobot.wheelState.leftRearTarget.direction - gRobot.wheelState.leftRearAct.direction) < 0.1f) &&
//							(fabs(gRobot.wheelState.rightRearTarget.direction - gRobot.wheelState.rightRearAct.direction) < 0.1f))	
//					{
//						OSTimeDly(10);
//						break;
//					}
//								
//					OSTimeDly(5);
//				}

				SendCmd2Driver(0.0f , 0.0f , 0.0f , 0.0f, 0.0f , 0.0f , 0.0f , 0.0f);
				
				/*CAN发送错误置零 驱动器错误标志位置零 心跳包清零*/
				gRobot.CANFailFlag=0;
			
				gRobot.wheelState.leftFrontHF=0;gRobot.wheelState.leftRearHF=0;gRobot.wheelState.rightFrontHF=0;gRobot.wheelState.rightRearHF=0;
				gRobot.wheelState.leftFrontTurnHF=0;gRobot.wheelState.leftRearTurnHF=0;gRobot.wheelState.rightFrontTurnHF=0;gRobot.wheelState.rightRearTurnHF=0;
				
				gRobot.wheelState.leftFrontHB=0;gRobot.wheelState.rightFrontHB=0;gRobot.wheelState.leftRearHB=0;gRobot.wheelState.rightRearHB=0;
				gRobot.wheelState.leftFrontTurnHB=0;gRobot.wheelState.rightFrontTurnHB=0;gRobot.wheelState.leftRearTurnHB=0;gRobot.wheelState.rightRearTurnHB=0;
				
				gRobot.wheelState.leftFrontHB_Continious=0;gRobot.wheelState.rightFrontHB_Continious=0;gRobot.wheelState.leftRearHB_Continious=0;gRobot.wheelState.rightRearHB_Continious=0;
				gRobot.wheelState.leftFrontTurnHB_Continious=0;gRobot.wheelState.rightFrontTurnHB_Continious=0;gRobot.wheelState.leftRearTurnHB_Continious=0;gRobot.wheelState.rightRearTurnHB_Continious=0;

				
				/*发射机构复位 主程序与发射机构间通信初始化*/
				gunStatus = GUN_RESET_POS;	
				Gun_Main_CommunicateInit();
				GunClawLoose();
				GunBaseIn();	
				GunShootPull();				
				if(gRobot.gun.gunPosRecord.baseState == gGunPosDataBase[GUN_SHOOT_POS].baseState)
				{
					OSTimeDly(150);
				}
				GunShootCmdReset();		
			
				
				resetDoneFlag = 1;
			}
			else
			{
				//可以退出重试 可以擦轮 亮金灯
				Gold(LIGHT_BOARD_ID);
				
				SteerResetCheck();
			}
			
			break;
		}
		//重试后触发状态17
		case startFromRetry:
		{
			//再清除一次标志位 防止移动过程中按其他按键
			/*初始化tele BT WIFI  标志位next angleshift stop置零*/
			TeleInit();			
			gRobot.teleDevice.BT.nextFlag = TELENOCMD;
			gRobot.teleDevice.WIFI.nextFlag = TELENOCMD;
			gRobot.teleDevice.BT.stopFlag = TELENOCMD;
			gRobot.teleDevice.WIFI.stopFlag = TELENOCMD;
			gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NONE;
			gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NONE;				
			gRobot.teleCommand.nextFlag = TELENOCMD;
			gRobot.teleCommand.stopFlag = TELENOCMD;
			gRobot.teleCommand.angleshift = 0.0f;
			
			SendCmd2Driver(0.0f , 0.0f , 0.0f , 0.0f, 0.0f , 0.0f , 0.0f , 0.0f);
			
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"RESTART!\r\n");
			
			//退出重试后亮白灯
			White(LIGHT_BOARD_ID);
			
			resetDoneFlag = 0;
			
			//不等待轮子转完 先清零再转  
			//发送开始矫正标志	并且让laser告诉pps清零
			SendBeginCorrect(); 
			
			//延时 防止期间不进定位系统接收中断
			OSTimeDly(2);
			
			//轮子方向初始化 BLUE 131 RED 48			
			OrientationClear();
			WheelDirectionInit();
			
			//枪复位
			GunInit();
			
			//规划第一段轨迹 清除轨迹长度
//			ClearRingBuffer();
//			InputPoints2RingBuffer(getFirstShagaiPath, GET_FIRST_SHAGAI_PATH_NUM);			
//			ClearPathLen();
			
			BEEP_ON;
			OSTimeDly(50);
			BEEP_OFF;
			
			//红/蓝色
			if(gRobot.courtID == BLUE_COURT)
			{
				Blue(LIGHT_BOARD_ID);
			}
			else if(gRobot.courtID == RED_COURT)
			{
				Red(LIGHT_BOARD_ID);
			}		
			
			//重启速度衰减系数 0.99
			gRobot.resetCoeffic = gRobot.resetCoeffic * 0.99f;
			gRobot.resetCoeffic = gRobot.resetCoeffic > 1.0f ? 1.0f : gRobot.resetCoeffic;
			gRobot.resetCoeffic = gRobot.resetCoeffic < 0.8f ? 0.8f : gRobot.resetCoeffic;
			
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"gRobot.resetCoeffic = %d\r\n",(int)(gRobot.resetCoeffic*100));
			
			gRobot.walkStatus = waitForStart;
			break;
		}
		//从标志线去取第一个兽骨18
		case get1stShagaiFromLine1:
		{
			break;
		}
		//从标志线去取第二个兽骨19
		case get2ndShagaiFromLine1:
		{
			break;
		}
		//从标志线去取第三个兽骨20
		case get3rdShagaiFromLine1:
		{
			break;
		}
		//测试参数用状态21
		case testPara:
		{
			timeCounter++;
			timeCounter%=10000;
			switch(testState)
			{
				case 0:
				{	
//					SetTargetVel(2000.0f,90.0f,0.0f);
					if(fabs(GetX())>2500.0f||fabs(GetY())>2500.0f||timeCounter>=100)
//						||(fabs(GetX())>2000.0f&&fabs(GetSpeedWithoutOmega().x)<300.f)\
//						||(fabs(GetY())>2000.0f&&fabs(GetSpeedWithoutOmega().y)<300.f))
					{
						testState = 1;
						timeCounter = 0;
					}
					break;
				}
				case 1:
				{
//					SetTargetVel(0.0f,90.0f,0.0f);	
					
					if(timeCounter>=25)
					{
						testState = 2;
					}
						
					break;
				}
				case 2:
				{
					OutputVel2Wheel(0.0f,90.0f,0.0f);
					break;
				}
				default:
				{
					OutputVel2Wheel(0.0f,0.0f,0.0f);
					break;
				}
			}
			break;
		}
		//旋转测试22
		case omegaTestPara:
		{
			timeCounter++;
			timeCounter%=10000;
			
			float omega = 0.0f;
			float angle_Temp = GetAngle();
			
			static float angle_Temp_Last = 0.0f;
			static int circleCount = 0;
			
			if(timeCounter < 500 && circleCount < 1 )
			{
				omega=50.0f;
			}
			else
			{
				omega=0.0f;
			}
			
			if((angle_Temp * angle_Temp_Last) < 0.0f && fabs(GetAngle()) > 170.0f)
			{
				circleCount ++;
				circleCount = circleCount%100;
			}
			
			angle_Temp_Last = angle_Temp;
//			
//			SetTargetVel(0.0f ,0.0f , omega);
			
			gRobot.debugInfomation.giveOmega=omega;
			gRobot.debugInfomation.circleCount= circleCount-1;
			
			break;
		}					
		//待命状态23
		case standBy:
		{
			ManualControl();

			break;
		}
		//停止状态24
		case stop:
		{
			//轮子复位 朝向初始位置
			OutputVel2Wheel(0.0f,90.0f,0.0f);
			
			GunBaseIn();
			GunClawLoose();
			GunShootPull();
			
			break;
		}
		default:
			break;
	}
}

static int timeCounter = 0;
static uint8_t countTimeFlag = 0;

int GetTimeCounter(void)
{
	return timeCounter;
}

void CountTime(void)
{
	if(countTimeFlag)
	{
		timeCounter++;
	}
	timeCounter%=500000000;
}

void SetCountTimeFlag(void)
{
	countTimeFlag = 1;
}

//缓冲气缸伸出
void CushionCylinderPush(void)
{
	GasValveControl(CAN2 , CUSHION_PUSH_BOARD_ID , CUSHION_PUSH_IO_ID , 1);
	GasValveControl(CAN2 , CUSHION_RECOVERY_BOARD_ID , CUSHION_RECOVERY_IO_ID , 0);
}

//缓冲气缸收回 5号置1气缸推出 4号置1气缸收回
void CushionCylinderRecovery(void)
{
	GasValveControl(CAN2 , CUSHION_PUSH_BOARD_ID , CUSHION_PUSH_IO_ID , 0);
	GasValveControl(CAN2 , CUSHION_RECOVERY_BOARD_ID , CUSHION_RECOVERY_IO_ID , 1);
}

//缓冲气缸泄气
void CushionCylinderHold(void)
{
	GasValveControl(CAN2 , CUSHION_PUSH_BOARD_ID , CUSHION_PUSH_IO_ID , 0);
	GasValveControl(CAN2 , CUSHION_RECOVERY_BOARD_ID , CUSHION_RECOVERY_IO_ID , 0);
}

/**
* @brief  ArmAngle2PositionTransform将手臂角度转化为脉冲
  * @note
* @param  angle:手臂朝向角度
* @retval 对应的电机脉冲位置
  */
int ArmAngle2PositionTransform(float angle)
{
	return (int)(angle / 360.0f * M3508_COUNTS_PER_ROUND * ARM_REDUCTION_RATIO);	
}

/**
* @brief  ArmAngle2PositionInverseTransform将手臂脉冲位置转化为角度
  * @note
* @param  position：手臂脉冲位置
* @retval 手臂朝向角度
  */
float ArmAngle2PositionInverseTransform(int position)
{
	return (float)((float)position / M3508_COUNTS_PER_ROUND / ARM_REDUCTION_RATIO * 360.0f);
}

/**
* @brief  ArmPosControl手臂位置控制
  * @note
* @param  angle：手臂朝向角度
* @retval 
  */
void ArmPosControl(float angle)
{
	gRobot.robotMotorState.expArmPos = angle;
	if(gRobot.courtID == BLUE_COURT)
	{
		PosCrl(CAN2 , ARM_MOTOR_ID_BLUE , ABSOLUTE_MODE , ArmAngle2PositionTransform(angle));	
	}
	else 	if(gRobot.courtID == RED_COURT)
	{
		PosCrl(CAN2 , ARM_MOTOR_ID_RED , ABSOLUTE_MODE , ArmAngle2PositionTransform(angle));	
	}
}

//手臂旋转
void ArmRotate(void)
{
	if(fabs(GetX())>7000.0f)
	{
		if(gRobot.courtID==BLUE_COURT)
		{
			ArmPosControl(-90.0f);
		}
		else if(gRobot.courtID==RED_COURT)
		{
			ArmPosControl(0.0f);
		}
	}
}
//手臂复位
void ArmRotateReset(void)
{
	if(gRobot.courtID==BLUE_COURT)
	{
		ArmPosControl(0.0f);
	}
	else if(gRobot.courtID==RED_COURT)
	{
		ArmPosControl(-90.0f);
	}
}
//void ArmRotate(uint8_t courtID)
//{
//	static float armAngle = 0.0f;
//	
//	if(fabs(GetX())>7000.0f)
//	{
//		armAngle = RAD2ANGLE(atan2(GetSpeedWithoutOmega().y,GetSpeedWithoutOmega().x)) - 180.0f;
//		
//		AngleLimit(&armAngle);
//		
//		if(fabs(armAngle)>90.0f)
//		{
//		armAngle = armAngle/fabs(armAngle)*90.0f;
//		}
//		if(courtID==BLUE_COURT)
//		{
//			ArmPosControl(armAngle);
//		}
//		else if(courtID==RED_COURT)
//		{
//			ArmPosControl(armAngle);
//		}
//	}
//}

//缓冲气缸控制
void CushionControl(uint8_t shagaiNum)
{
	switch (shagaiNum)
	{
		case FIRSTSHAGAI:
		{
			if(GetY()>=4900.0f&&GetY()<=5050.0f)
			{
				CushionCylinderPush();
			}
			else if(GetY()>5050.0f)
			{
				if(!JudgeSpeedLessEqual(300.0f))
				{
					CushionCylinderHold();
				}
				else
				{
					CushionCylinderRecovery();
				}
			}
			break;
		}
		case SECONDSHAGAI:
		{
			if(fabs(GetX())>8500.0f&&fabs(GetX())<=8650.0f)
			{
				CushionCylinderPush();
			}
			else if(fabs(GetX())>8650.0f)
			{
				if(!JudgeSpeedLessEqual(300.0f))
				{
					CushionCylinderHold();
				}
				else
				{
					CushionCylinderRecovery();
				}
			}
			break;
		}
		case THIRDSHAGAI:
		{
			if(fabs(GetX())>8500.0f&&fabs(GetX())<=8650.0f)
			{
				CushionCylinderPush();
			}
			else if(fabs(GetX())>8650.0f)
			{
				if(!JudgeSpeedLessEqual(300.0f))
				{
					CushionCylinderHold();
				}
				else
				{
					CushionCylinderRecovery();
				}
			}
			break;
		}
		default:
			break;
	}		
}

//爪子气缸控制
void ClawCtr(uint8_t state)
{
	if(state==1||state==0)
	{
		GasValveControl(CAN2 , CLAW_BOARD_ID , CLAW_IO_ID , state);
	}
}

void GraspTheGerege(void)
{
	ClawCtr(0);
}

void ReleaseTheGerege(void)
{
	ClawCtr(1);
}

uint8_t JudgeStop(float disChange,uint8_t countTime)
{
	static uint8_t counter = 0;
	float disX , disY = 0.0f;
	static float posXRecord , posYRecord = 0.0f;
	
	disX = GetX() - posXRecord;
	disY = GetY() - posYRecord;
	
	if(sqrtf(disX * disX + disY * disY)<=disChange)
	{
		counter++;
	}
	else
	{
		counter = 0;
	}
	
	posXRecord = GetX();
	posYRecord = GetY();
	
	if(counter<=countTime)
	{
		return 0;
	}
	else
	{
		counter = 0;
		return 1;
	}
}
uint8_t JudgeStop2(uint8_t shagaiNum,float disChange,uint8_t countTime)
{
	static uint8_t counter = 0;
	float disX , disY = 0.0f;
	static float posXRecord , posYRecord = 0.0f;
	
	disX = GetX() - posXRecord;
	disY = GetY() - posYRecord;
	
	switch(shagaiNum)
	{
		case FIRSTSHAGAI:
		{
			if(fabs(disY) < disChange)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}
			break;
		}
		case SECONDSHAGAI:
		{
			if(fabs(disX) < disChange)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}
			break;
		}
		case THIRDSHAGAI:
		{
			if(fabs(disX) < disChange)
			{
				counter++;
			}
			else
			{
				counter = 0;
			}
			break;
		}
		default:
			break;
	}
	
	posXRecord = GetX();
	posYRecord = GetY();
	
	if(counter<=countTime)
	{
		return 0;
	}
	else
	{
		counter = 0;
		return 1;
	}
}
uint8_t JudgeAgainstWall(void)
{
	static uint8_t counter = 0;
	float  disY = 0.0f, disAngle = 0.0f;
	static float  posYRecord = 0.0f, posAngleRecord = 0.0f;
	
	disY = GetY() - posYRecord;
	disAngle = GetAngle() - posAngleRecord;
	
	if( fabs(disY)<=4.0f && fabs(disAngle)<=0.25f )
	{
		counter++;
	}
	else
	{
		counter = 0;
	}
	
	posYRecord = GetY();
	posAngleRecord = GetAngle();
	
	if(counter<=7)
	{
		return 0;
	}
	else
	{
		counter = 0;
		
		return 1;
	}
}
uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	position_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.x;
	float speedY = actSpeed.y;
	
	float speed = sqrtf(speedX * speedX + speedY * speedY);
	
	if(speed>speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
//主程序发送给发射机构的信息
static	communicateMsg_t mainMsgToGun;

//主程序向发射机构传递消息序列初始化
void MainMsg2GunInit(void)
{
	for(uint8_t i =0 ; i<10 ; i++)
	{
		mainMsgToGun.msgBuffer[i] = GUN_NO_CMD;
	}
	mainMsgToGun.bufferRear = 0;
	mainMsgToGun.bufferFront = 0;
}

uint8_t JudgeMainMsg2GunEmpty(void)
{
	if(mainMsgToGun.bufferFront==mainMsgToGun.bufferRear)
	{
		return 1;
	}
	return 0;
}


//主程序向发射机构传送消息
void MainSendMsg2Gun(uint8_t msg)
{
	//如果要传递的消息和上一个不同则将消息放入队列（避免重复传递消息）
	if(mainMsgToGun.bufferRear>0)
	{
		if(msg!=mainMsgToGun.msgBuffer[mainMsgToGun.bufferRear-1])
		{
			mainMsgToGun.msgBuffer[mainMsgToGun.bufferRear] = msg;
			mainMsgToGun.bufferRear++;
			mainMsgToGun.bufferRear%=10;
		}
	}
	else if(mainMsgToGun.bufferRear==0)
	{
		if(msg!=mainMsgToGun.msgBuffer[9])
		{
			mainMsgToGun.msgBuffer[mainMsgToGun.bufferRear] = msg;
			mainMsgToGun.bufferRear++;
			mainMsgToGun.bufferRear%=10;
		}		
	}
}

//发射机构从消息队列中获取一条来自主程序消息
uint8_t GetMainMsg2Gun(void)
{
	uint8_t msg = GUN_NO_CMD;
	
	//如果消息队列非空则返回一条有效消息
	if(mainMsgToGun.bufferFront!=mainMsgToGun.bufferRear)
	{
		msg = mainMsgToGun.msgBuffer[mainMsgToGun.bufferFront];
		
//		mainMsgToGun.msgBuffer[mainMsgToGun.bufferFront] = GUN_NO_CMD;
		
		mainMsgToGun.bufferFront++;
		
		mainMsgToGun.bufferFront%=10;
	}
	//如果消息队列为空则返回没有消息
	return msg;
}

void MainIsManualCmdReset(void)
{
	mainCmdRecieve.isManualMode = GUN_NO_CMD;
}

void MainIsShootDoneReset(void)
{
	mainCmdRecieve.isShootDone = GUN_NO_CMD;
}

void MainCmdReset(void)
{
	MainIsManualCmdReset();
	
	MainIsShootDoneReset();
}

//发射程序到主程序消息处理
void Gun2MainMsgProcess(void)
{
	uint8_t nn = 0;
	
	while(!JudgeGunMsg2MainEmpty())
	{
		switch(GetGunMsg2Main())
		{
			case GUN_MANUAL_MODE_MSG:
			{
				mainCmdRecieve.isManualMode = GUN_MANUAL_MODE_MSG;				
				break;
			}
			case GUN_SHOOT_DONE_MSG:
			{
				mainCmdRecieve.isShootDone = GUN_SHOOT_DONE_MSG;
				break;				
			}
			default:
				break;
		}
		
		nn++;
		
		if(nn>=10)
		{
			break;
		}
	}
}

//铲拐骨
void ShovelShagai(uint8_t shagaiNum)
{
	switch (shagaiNum)
	{
		case FIRSTSHAGAI:
		{
			if(GetY()>4100.0f)
			{
				if((sendMsgFlag&0x01)==0)
				{
					MainSendMsg2Gun(GUN_RELOAD_MSG);
					sendMsgFlag|=0x01;
				}
			}
			break;
		}		
		case SECONDSHAGAI:
		{
			if(fabs(GetX())>7500.0f)
			{
				if((sendMsgFlag&0x02)==0)
				{
					MainSendMsg2Gun(GUN_RELOAD_MSG);
					sendMsgFlag|=0x02;				
				}
			}
			else if(fabs(GetX())>6200.0f)
			{
				GunPosReset();
			}
			break;
		}		
		case THIRDSHAGAI:
		{
			if(fabs(GetX())>7500.0f)
			{
				if((sendMsgFlag&0x04)==0)
				{
					MainSendMsg2Gun(GUN_RELOAD_MSG);
					sendMsgFlag|=0x04;
				}
			}
			else if(fabs(GetX())>6200.0f)
			{
				GunPosReset();
			}

			break;
		}
		default:
			break;
	}
}
//保持拐骨 三个拐骨动作相同
void HoldShagai(uint8_t shagaiNum)
{
	switch (shagaiNum)
	{
		case FIRSTSHAGAI:
		{
			if(GetY()<5300.0f)
			{
				if((sendMsgFlag&0x08)==0)
				{
					MainSendMsg2Gun(GUN_LOADED_MSG);
					sendMsgFlag|=0x08;
				}
			}
			break;
		}		
		case SECONDSHAGAI:
		{
			if(fabs(GetX())<8900.0f)
			{
				if((sendMsgFlag&0x10)==0)
				{
					MainSendMsg2Gun(GUN_LOADED_MSG);
					sendMsgFlag|=0x10;
				}
			}
			if(fabs(GetX())<8000.0f)
			{
				if((sendMsgFlag&0x20)==0)
				{
					MainSendMsg2Gun(GUN_PREPARE_SHOOT_MSG);
					sendMsgFlag|=0x20;
				}
			}
			break;
		}		
		case THIRDSHAGAI:
		{
			if(fabs(GetX())<8900.0f)
			{
				if((sendMsgFlag&0x40)==0)
				{
					MainSendMsg2Gun(GUN_LOADED_MSG);
					sendMsgFlag|=0x40;
				}
			}
			if(fabs(GetX())<7500.0f)
			{
				if((sendMsgFlag&0x80)==0)
				{
					MainSendMsg2Gun(GUN_PREPARE_SHOOT_MSG);
					sendMsgFlag|=0x80;
				}
			}
			break;
		}
		default:
			break;
	}
}
//重启开关检查
void ResetCheck(void)
{
	//RESET_KEYSWITCH连续为1次数
	static uint8_t pTrigerTimes = 0;
	//RESET_KEYSWITCH连续为0次数
	static uint8_t nTrigerTimes = 0;

	if(RESET_KEYSWITCH)
	{
		nTrigerTimes = 0;
		
		pTrigerTimes++;
		pTrigerTimes=pTrigerTimes>100?100:pTrigerTimes;
	}
	else if(!RESET_KEYSWITCH)
	{
		pTrigerTimes = 0;
		
		nTrigerTimes++;
		nTrigerTimes=nTrigerTimes>100?100:nTrigerTimes;
	}

	if(pTrigerTimes > 10 && gRobot.resetFlag == ROBOT_NONE)
	{
		gRobot.resetFlag = ROBOT_RETRY;
	}
		
	if(nTrigerTimes > 10 && gRobot.resetFlag == ROBOT_RETRY)
	{		
		gRobot.resetFlag = ROBOT_RESTART;
	}
}

//轮子复位开关检查
void SteerResetCheck(void)
{
	//RESET_KEYSWITCH连续为1次数
	static uint8_t pTrigerTimes = 0;
	//RESET_KEYSWITCH连续为0次数
	static uint8_t nTrigerTimes = 0;

	if(STEER_RESET_KEYSWITCH)
	{
		nTrigerTimes = 0;
		
		pTrigerTimes++;
		pTrigerTimes=pTrigerTimes>100?100:pTrigerTimes;
	}
	else if(!STEER_RESET_KEYSWITCH)
	{
		pTrigerTimes = 0;
		
		nTrigerTimes++;
		nTrigerTimes=nTrigerTimes>100?100:nTrigerTimes;
	}

	if(pTrigerTimes > 10)
	{				
		//1000轮子往前转
		SendCmd2Driver(1000.0f , 0.0f , 1000.0f , 0.0f,
						 1000.0f , 0.0f , 1000.0f , 0.0f);
	}
		
	if(nTrigerTimes > 10)
	{		
		SendCmd2Driver(0.0f , 0.0f , 0.0f , 0.0f,
						 0.0f , 0.0f , 0.0f , 0.0f);
	}
}
Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetX();
	pos.point.y = GetY();
	pos.direction   = GetAngle();
	pos.vel = 0.0f;
	return pos;
}
