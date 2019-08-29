/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

  #include "moveBase.h"
  #include "pps.h"
  #include "elmo.h"
  #include "robot.h"
  #include "motion.h"
  #include "dma.h"
  #include "stm32f4xx_gpio.h"

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
* @brief  Vel2Pulse将速度转换为脉冲
  * @note
  * @param  vel:速度（mm/s）
  * @retval 脉冲速度
  */
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*COUNTS_PER_ROUND*WHEEL_REDUCTION_RATIO);
}

/**
* @brief  Pulse2Vel将速度转换为脉冲
  * @note
* @param  pulse:脉冲速度
  * @retval 速度（mm/s）
  */
float Pulse2Vel(int pulse)
{
	return ((float)pulse/COUNTS_PER_ROUND)/WHEEL_REDUCTION_RATIO*PI*WHEEL_DIAMETER;
}

extern vector_t finaltVel;
/**
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
  * @retval 
  */
void OutputVel2Wheel(float vel, float direction, float omega)
{
	finaltVel.module=vel;
	finaltVel.direction=direction;
	
	wheel_t outputVel = {0.0f};
	

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d ", (int)(vel) , (int)(direction) , (int)(omega));
#endif
	//计算每个轮子的和速度大小和方向
	outputVel.leftFront = CalcWheelSpeed(vel , direction , omega , LEFT_FRONT_VERTICAL_ANG , GetAngle());
	
	outputVel.rightFront = CalcWheelSpeed(vel , direction , omega , RIGHT_FRONT_VERTICAL_ANG , GetAngle());
	
	outputVel.leftRear = CalcWheelSpeed(vel , direction , omega , LEFT_REAR_VERTICAL_ANG , GetAngle());
	
	outputVel.rightRear = CalcWheelSpeed(vel , direction , omega , RIGHT_REAR_VERTICAL_ANG , GetAngle());
	
	gRobot.debugInfomation.wheelVel = outputVel;
	
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"WW %d %d %d %d ", (int)(outputVel.leftFront.direction) , (int)(outputVel.rightFront.direction) ,\
				(int)(outputVel.leftRear.direction) , (int)(outputVel.rightRear.direction));
#endif
	
	//将和速度输出
	WheelVelControl(outputVel);
}

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */
//记录各个轮子朝向变量
static float leftFrontAng = 0.0f, rightFrontAng = 0.0f, leftRearAng = 0.0f, rightRearAng = 0.0f;

void WheelVelControl(wheel_t wheelVel)
{
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif
	
	//将定位系统坐标系下角度转换为机器人坐标系下角度 direction-=GetAngle()
	Transform2RobotCoodinate(&wheelVel);

#ifdef SEND_MOVEBASE_DEBUGINFO	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	

	//将机器人坐标系下角度转换为和电机一致 direction = 90.0f - direction
	Transform2WheelCoodinate(&wheelVel);

#ifdef SEND_MOVEBASE_DEBUGINFO	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	
	//判断是否需要将轮速反向
	JudgeVelDirection(&wheelVel.leftFront, leftFrontAng);
	JudgeVelDirection(&wheelVel.rightFront, rightFrontAng);
	JudgeVelDirection(&wheelVel.leftRear, leftRearAng);
	JudgeVelDirection(&wheelVel.rightRear, rightRearAng);

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	
	//保证旋转为劣弧
	leftFrontAng = TurnInferiorArc(wheelVel.leftFront.direction , leftFrontAng);
	rightFrontAng = TurnInferiorArc(wheelVel.rightFront.direction , rightFrontAng);
	leftRearAng = TurnInferiorArc(wheelVel.leftRear.direction , leftRearAng);
	rightRearAng = TurnInferiorArc(wheelVel.rightRear.direction , rightRearAng);

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d \r\n", (int)(leftFrontAng) , (int)(rightFrontAng) ,\
				(int)(leftRearAng) , (int)(rightRearAng));
#endif
	
	//向电机发送速度和位置命令
	SendCmd2Driver(wheelVel.leftFront.vel , leftFrontAng , wheelVel.rightFront.vel , rightFrontAng,
				   wheelVel.leftRear.vel , leftRearAng , wheelVel.rightRear.vel , rightRearAng);
}

void OrientationClear(void)
{
	leftFrontAng = 0.0f;
	rightFrontAng = 0.0f;
	leftRearAng = 0.0f;
	rightRearAng = 0.0f;
}

/**
* @brief  SendCmd2Driver向电机发送速度和位置命令
  * @note
* @param  lfVel:左前轮速度
		  lfDir：左前轮方向
		  rfVel:右前轮速度
		  rfDir：右前轮方向
		  lrVel:左后轮速度
		  lrDir：左后轮方向
		  rrVel:右后轮速度
		  rrDir：右后轮方向

  * @retval 
  */
void SendCmd2Driver(float lfVel , float lfDir , float rfVel , float rfDir,
					float lrVel , float lrDir , float rrVel , float rrDir)
{
	//记录各个轮子实际给出的控制量
	gRobot.wheelState.leftFrontTarget.vel = lfVel;
	gRobot.wheelState.leftFrontTarget.direction = lfDir;
	
	gRobot.wheelState.rightFrontTarget.vel = -rfVel;
	gRobot.wheelState.rightFrontTarget.direction = rfDir;
	
	gRobot.wheelState.leftRearTarget.vel = lrVel;
	gRobot.wheelState.leftRearTarget.direction = lrDir;
	
	gRobot.wheelState.rightRearTarget.vel = -rrVel;
	gRobot.wheelState.rightRearTarget.direction = rrDir;
	
	//将每个轮子实际的速度和方向发送给电机驱动器
	VelCrl(CAN1 , LEFT_FRONT_ID , Vel2Pulse(lfVel));
	PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(lfDir , gRobot.wheelState.steerLoopShift.lf));
	
	VelCrl(CAN1 , RIGHT_FRONT_ID , -Vel2Pulse(rfVel));
	PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(rfDir , gRobot.wheelState.steerLoopShift.rf));
	
	VelCrl(CAN1 , LEFT_REAR_ID , Vel2Pulse(lrVel));
	PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(lrDir , gRobot.wheelState.steerLoopShift.lr));	
	
	VelCrl(CAN1 , RIGHT_REAR_ID , -Vel2Pulse(rrVel));
	PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(rrDir , gRobot.wheelState.steerLoopShift.rr));
	
}
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift)
{
	return (int)(((angle / 360.0f)* WHEEL_TURNING_REDUCTION_RATIO + loopShift)* COUNTS_PER_ROUND );
}
/**
* @brief  WheelAngle2PositionInverseTransform将轮子脉冲位置转化为角度
  * @note
* @param  position:轮子脉冲位置
* @retval 轮子朝向角度
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift)
{
	return (float)(((float)position / COUNTS_PER_ROUND - loopShift)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
}
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
  */
void Transform2RobotCoodinate(wheel_t * wheelVel)
{
	//将定位系统坐标系下角度转换为机器人坐标系下角度
	wheelVel->leftFront.direction-=GetAngle();
	wheelVel->rightFront.direction-=GetAngle();
	wheelVel->leftRear.direction-=GetAngle();
	wheelVel->rightRear.direction-=GetAngle();

	//将角度限制在180度到-180度范围内
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);

}
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
  */
void Transform2WheelCoodinate(wheel_t * wheelVel)
{
	//将机器人坐标系下轮子朝向转换为轮子坐标系下角度
	wheelVel->leftFront.direction = 90.0f - wheelVel->leftFront.direction;
	wheelVel->rightFront.direction = 90.0f - wheelVel->rightFront.direction;
	wheelVel->leftRear.direction = 90.0f - wheelVel->leftRear.direction;
	wheelVel->rightRear.direction = 90.0f - wheelVel->rightRear.direction;
	
	//将角度限制在-180°到180°
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);

}
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(angle>180.0f)
		{
			angle = ReturnLimitAngle(angle - 360.0f);
		}
		else if(angle<-180.0f)
		{
			angle = ReturnLimitAngle(angle + 360.0f);
		}
	}
	
	recursiveTimes--;
	
	return angle;
}


/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子正方向角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle)
{
	int n = 0;
	float angleErr = 0.0f;
	
	//将目标角度和当前实际角度转换到一个360度周期中
	n = (int)(actualAngle/180.0f) - (int)(actualAngle/360.0f);
	
	targetVel->direction = n * 360.0f + targetVel->direction;
	
	//计算目标角度和实际角度的误差
	angleErr = targetVel->direction - actualAngle;
	
	//将误差限制在-180度到180度
	AngleLimit(&angleErr);
	
	//如果角度误差大于90度则将速度反向并将目标角度加180度
	if(fabs(angleErr)>90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;
		
		//保证处理后的目标角度和当前实际角度在一个周期中
		if(targetVel->direction>(n * 360.0f + 180.0f))
		{
			targetVel->direction -=360.0f;
		}
		else if(targetVel->direction<(n * 360.0f - 180.0f))
		{
			targetVel->direction+=360.0f;
		}
	}

}
/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle)
{
	if(targetAngle - actualAngle>180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if(targetAngle - actualAngle<-180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}	
}
/**
* @brief  CalcWheelSpeed计算轮子的和速度大小和方向
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
		  angleN:在机器人坐标系下旋转线速度的正方向(单位：度)
* @retval 
  */
wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN , float postureAngle)
{
	wheelVel_t sumVel = {0.0f};
	float velX , velY = 0.0f;
	float velN ,velNDirection= 0.0f;
	float sumVelX , sumVelY = 0.0f;
	
	//计算平移速度的X，Y分量
	velX = vel*arm_cos_f32(ANGLE2RAD(direction));
	velY = vel*arm_sin_f32(ANGLE2RAD(direction));
	
	//计算旋转的线速度
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	velNDirection = angleN + postureAngle;
	AngleLimit(&velNDirection);
	
	//计算和速度大小和方向
	sumVelX = velX + velN * arm_cos_f32(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * arm_sin_f32(ANGLE2RAD(velNDirection));
	
	arm_sqrt_f32(sumVelX * sumVelX + sumVelY * sumVelY,&sumVel.vel);
	
	//计算合成速度方向时将0向量单独处理
	if(sumVel.vel>0.01f)
	{
		sumVel.direction = RAD2ANGLE(atan2f(sumVelY , sumVelX));
	}
	else
	{
		sumVel.direction = direction;
	}
	
	return sumVel;
}
//每1s给 0 90 180 90 0 -90 -180 0 共8s
//3m/s给2s -3m/s给2s 0m/s给1s
uint8_t WheelMotorCheck2(void)
{
	static int timeCounter = 0;
	static uint8_t turnCnt = 0;
	static float direction = 0.0f;
	float vel = 0.0f;
	uint8_t wheelCheckDoneFlag=0;
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d %d\r\n", (int)direction,(int)timeCounter,(int)turnCnt);
	
	timeCounter++;
	timeCounter%=10000;
	if(timeCounter%50==0)
	{
		turnCnt++;
	}
	turnCnt%=100;
	if(turnCnt<=7)
	{
		if(turnCnt==0)				direction=90.f;
		else if(turnCnt==1)		direction=180.f;
		else if(turnCnt==2)		direction=90.f;
		else if(turnCnt==3)		direction=0.f;
		else if(turnCnt==4)		direction=-90.f;
		else if(turnCnt==5)		direction=-180.f;
		else if(turnCnt==6)		direction=-90.f;
		else if(turnCnt==7)		direction=0.f;
		
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lf));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rf));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lr));	
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rr));
	}
	else
	{
		if(timeCounter>=400&&timeCounter<500)		vel=3000.0f;
		else if(timeCounter<600)		vel=-3000.f;
		else if(timeCounter<650)		vel=0.f;
		else wheelCheckDoneFlag=1;
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(vel));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(vel));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(vel));
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(vel));
	}
	
	return wheelCheckDoneFlag;
}



/**
* @brief  WheelMotorCheck轮子自检函数
* @note 	轮速正转1.8s反转1.8s两次 转向轮正转反转各两圈
* @param  none
* @retval 自检完毕返回1
  */
uint8_t WheelMotorCheck(void)
{
	static int timeCounter = 0;
	static float direction = 0.0f;
	static uint8_t turnCnt=0;
	uint8_t wheelCheckDoneFlag=0;
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d %d\r\n", (int)direction,(int)timeCounter,(int)turnCnt);
	
	
	timeCounter++;
	if(timeCounter<=90)
	{
		direction+=2.0f;
		turnCnt=0;
	}
	else if(timeCounter<=180)
	{
		direction-=2.0f;
		turnCnt=1;
	}
	else if(timeCounter<=270)
	{
		direction-=2.0f;
		turnCnt=2;
	}
	else if(timeCounter<=360)
	{
		direction+=2.0f;
		turnCnt=3;
	}
	else
	{
		turnCnt=4;
	}
	timeCounter%=1000;
	
	if(turnCnt==0)
	{
		//轮速1000 转向正转一周 3.6s
//		OutputVel2Wheel(1000.0f ,direction , 0.0f);
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(1000.f));
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lf));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(1000.f));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rf));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(1000.f));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lr));	
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(1000.f));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rr));
		
	}
	else if(turnCnt==1)
	{
		//轮速-1000 转向反转一周 3.6s
//		OutputVel2Wheel(-1000.0f ,direction , 0.0f);
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(-1000.f));
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lf));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(-1000.f));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rf));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(-1000.f));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lr));	
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(-1000.f));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rr));
	}
	else if(turnCnt==2)
	{
		//轮速正转 转向正转 1.8s
//		OutputVel2Wheel(1000.0f ,direction , 0.0f);
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(1000.f));
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lf));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(1000.f));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rf));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(1000.f));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lr));	
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(1000.f));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rr));
	}
	else if(turnCnt==3)
	{
		//轮速反转 转向反转 1.8s
//		OutputVel2Wheel(-1000.0f ,direction , 0.0f);
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(-1000.f));
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lf));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(-1000.f));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rf));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(-1000.f));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.lr));	
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(-1000.f));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(direction , gRobot.wheelState.steerLoopShift.rr));
	
	}
	else
	{
//		OutputVel2Wheel(0.0f ,0.0f , 0.0f);
		VelCrl(CAN1 , LEFT_FRONT_ID , -Vel2Pulse(0.0f));
		PosCrl(CAN1 , LEFT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(0.0f , gRobot.wheelState.steerLoopShift.lf));
		VelCrl(CAN1 , RIGHT_FRONT_ID , Vel2Pulse(0.0f));
		PosCrl(CAN1 , RIGHT_FRONT_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(0.0f , gRobot.wheelState.steerLoopShift.rf));
		VelCrl(CAN1 , LEFT_REAR_ID , -Vel2Pulse(0.0f));
		PosCrl(CAN1 , LEFT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(0.0f , gRobot.wheelState.steerLoopShift.lr));	
		VelCrl(CAN1 , RIGHT_REAR_ID , Vel2Pulse(0.0f));
		PosCrl(CAN1 , RIGHT_REAR_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(0.0f , gRobot.wheelState.steerLoopShift.rr));
		
		wheelCheckDoneFlag=1;
	}
	return wheelCheckDoneFlag;
}

/**
* @brief  SendSelfCheckWheelSpeed发送轮子自检速度位置信息函数
* @note 	
* @param  none
* @retval 
  */
void SendSelfCheckWheelSpeed(void)
	
{
	//四轮实际速度
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"v %d %d ", (int)gRobot.wheelState.leftFrontAct.vel , (int)gRobot.wheelState.rightFrontAct.vel);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)gRobot.wheelState.leftRearAct.vel , (int)gRobot.wheelState.rightRearAct.vel);
	
	//四轮实际位置
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"agl %d %d ", (int)(gRobot.wheelState.leftFrontAct.direction*10.0f) , (int)(gRobot.wheelState.rightFrontAct.direction*10.0f));
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d %d ", (int)(gRobot.wheelState.leftRearAct.direction*10.0f) , (int)(gRobot.wheelState.rightRearAct.direction*10.0f));
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"\r\n");	
	
}


//用加速度方向计算四个小函道的占空比
void CalcSmallDuckedFanDutyCycle(float vel, float direction)
{
	float velX=0.0f, velY=0.0f;
	static float velXRecord=0.0f, velYRecord=0.0f;
	float accX=0.0f, accY=0.0f;
	float accXOnLFAxis=0.0f,accYOnRFAxis=0.f;
	
	velX=vel*cosf(direction*CHANGE_TO_RADIAN);
	velY=vel*sinf(direction*CHANGE_TO_RADIAN);
	
	accX=velX-velXRecord;
	accY=velY-velYRecord;
	
	accXOnLFAxis=accY*sqrt(2.f)/2.f - accX*sqrt(2.f)/2.f;
	accYOnRFAxis=accY*sqrt(2.f)/2.f + accX*sqrt(2.f)/2.f;
	
	if(fabs(accXOnLFAxis)>fabs(accYOnRFAxis))
	{
		if(accXOnLFAxis>0.f)
		{
			gRobot.smallDuckedFan.LF=0.8f;
			gRobot.smallDuckedFan.RR=0.0f;
		}
		else
		{
			gRobot.smallDuckedFan.LF=0.0f;
			gRobot.smallDuckedFan.RR=0.8f;
		}
		if(accYOnRFAxis>0.f)
		{
			gRobot.smallDuckedFan.RF=0.8f*fabs(accYOnRFAxis)/fabs(accXOnLFAxis);
			gRobot.smallDuckedFan.LR=0.0f;
		}
		else
		{
			gRobot.smallDuckedFan.RF=0.0f;
			gRobot.smallDuckedFan.LR=0.8f*fabs(accYOnRFAxis)/fabs(accXOnLFAxis);
		}
	}
	else if(fabs(accXOnLFAxis)<fabs(accYOnRFAxis))
	{
		if(accXOnLFAxis>0.f)
		{
			gRobot.smallDuckedFan.LF=0.8f*fabs(accXOnLFAxis)/fabs(accYOnRFAxis);
			gRobot.smallDuckedFan.RR=0.0f;
		}
		else
		{
			gRobot.smallDuckedFan.LF=0.0f;
			gRobot.smallDuckedFan.RR=0.8f*fabs(accXOnLFAxis)/fabs(accYOnRFAxis);
		}
		if(accYOnRFAxis>0.f)
		{
			gRobot.smallDuckedFan.RF=0.8f;
			gRobot.smallDuckedFan.LR=0.0f;
		}
		else
		{
			gRobot.smallDuckedFan.RF=0.0f;
			gRobot.smallDuckedFan.LR=0.8f;
		}
	}
	
	velXRecord=velX;
	velYRecord=velY;
}
	
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
