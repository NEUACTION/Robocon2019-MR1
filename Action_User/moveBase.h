/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H



/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/
//机器人速度结构体
typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;

//轮子速度和方向结构体
typedef struct
{
	//轮子速度大小
	float vel;
	//轮子速度方向
	float direction;
}wheelVel_t;

typedef struct
{
	float lf;
	float rf;
	float lr;
	float rr;
}turnVel_t;

//轮子圈数偏移结构体
typedef struct
{
	int32_t lf;
	int32_t rf;
	int32_t lr;
	int32_t rr;
}steerLoopShift_t;

//轮子状态结构体
typedef struct
{
	//左前轮目标速度与方向
	wheelVel_t leftFrontTarget;
	//左前轮实际速度与方向
	wheelVel_t leftFrontAct;
	//右前轮目标速度与方向
	wheelVel_t rightFrontTarget;
	//右前轮实际速度与方向
	wheelVel_t rightFrontAct;
	//左后轮目标速度与方向
	wheelVel_t leftRearTarget;
	//左后轮实际速度与方向
	wheelVel_t leftRearAct;
	//右后轮目标速度与方向
	wheelVel_t rightRearTarget;
	//右后轮实际速度与方向
	wheelVel_t rightRearAct;
	//左前轮心跳包
	uint16_t leftFrontHB;
	//右前轮心跳包
	uint16_t rightFrontHB;
	//左后轮心跳包
	uint16_t leftRearHB;
	//右后轮心跳包
	uint16_t rightRearHB;
	//左前轮转向心跳包
	uint16_t leftFrontTurnHB;
	//右前轮转向心跳包
	uint16_t rightFrontTurnHB;
	//左后轮转向心跳包
	uint16_t leftRearTurnHB;
	//右后轮转向心跳包
	uint16_t rightRearTurnHB;
	
	//左前轮心跳包_连续
	uint16_t leftFrontHB_Continious;
	//右前轮心跳包_连续
	uint16_t rightFrontHB_Continious;
	//左后轮心跳包_连续
	uint16_t leftRearHB_Continious;
	//右后轮心跳包_连续
	uint16_t rightRearHB_Continious;
	//左前轮转向心跳包_连续
	uint16_t leftFrontTurnHB_Continious;
	//右前轮转向心跳包_连续
	uint16_t rightFrontTurnHB_Continious;
	//左后轮转向心跳包_连续
	uint16_t leftRearTurnHB_Continious;
	//右后轮转向心跳包_连续
	uint16_t rightRearTurnHB_Continious;
	
	//左前轮硬件中断标志位
	uint8_t leftFrontHF;
	//右前轮硬件中断标志位
	uint8_t rightFrontHF;
	//左后轮硬件中段标志位
	uint8_t leftRearHF;
	//右后轮硬件中断标志位
	uint8_t rightRearHF;
	//左前轮转向硬件中断标志位
	uint8_t leftFrontTurnHF;
	//右前轮转向硬件中断标志位
	uint8_t rightFrontTurnHF;
	//左后轮转向硬件中断标志位
	uint8_t leftRearTurnHF;
	//右后轮硬件中断标志位
	uint8_t rightRearTurnHF;
	
	//左前轮超周期标志位
	int leftFrontOverCycle;
	//右后轮超周期标志位
	int rightRearOverCycle;
	
	//轮子圈数偏移
	steerLoopShift_t steerLoopShift;
	//轮子失能标志位
	uint8_t disenableFlag; 
	
	turnVel_t turnVel;
	
}wheelState_t;

//轮子结构体
typedef struct
{
	//左前轮
	wheelVel_t leftFront;
	//右前轮
	wheelVel_t rightFront;
	//左后轮
	wheelVel_t leftRear;
	//右后轮
	wheelVel_t rightRear;
}wheel_t;
/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

#define PULSE_2_VEL(pulse) (((float)pulse/COUNTS_PER_ROUND)/WHEEL_REDUCTION_RATIO*PI*WHEEL_DIAMETER)

//#define SEND_MOVEBASE_DEBUGINFO


//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (32768)
//电机最大转速
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*100)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (70.0f)
//定位系统X轴方向到中心距离
#define DISX_OPS2CENTER (0.0f)
//定位系统Y轴方向到中心距离
#define DISY_OPS2CENTER (-307.0f)
//驱动轮减速比
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//M2006减速比
#define M2006_REDUCTION_RATIO (36.0f/1.0f)
//转向齿轮减速比
#define TURNING_REDUCTION_RATIO (4.0f/1.0f)
//轮子转向减速比
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO*TURNING_REDUCTION_RATIO)
//底盘旋转半径
#define MOVEBASE_RADIUS (362.039f)
//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)

//左前轮ID号
#define LEFT_FRONT_ID (1)
//右前轮ID号
#define RIGHT_FRONT_ID (2)
//左后轮ID号
#define LEFT_REAR_ID (3)
//右后轮ID号
#define RIGHT_REAR_ID (4)
//左前轮转向ID号
#define LEFT_FRONT_TURNING_ID (5)
//右前轮转向ID号
#define RIGHT_FRONT_TURNING_ID (6)
//左后轮转向ID号
#define LEFT_REAR_TURNING_ID (7)
//右后轮转向ID号
#define RIGHT_REAR_TURNING_ID (8)

//左前轮与中心连线切线方向
#define LEFT_FRONT_VERTICAL_ANG (-135.0f)
//右前轮与中心连线切线方向
#define RIGHT_FRONT_VERTICAL_ANG (135.0f)
//左后轮与中心连线切线方向
#define LEFT_REAR_VERTICAL_ANG (-45.0f)
//右后轮与中心连线切线方向
#define RIGHT_REAR_VERTICAL_ANG (45.0f)

//
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
* @brief  Vel2Pulse将速度转换为脉冲
  * @note
  * @param  vel:速度（mm/s）
  * @retval 脉冲速度
  */
int Vel2Pulse(float vel);
/**
* @brief  Pulse2Vel将速度转换为脉冲
  * @note
* @param  pulse:脉冲速度
  * @retval 速度（mm/s）
  */
float Pulse2Vel(int pulse);
/**
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
  * @retval 
  */
void OutputVel2Wheel(float vel, float direction, float omega);

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */
void WheelVelControl(wheel_t wheelVel);

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
					float lrVel , float lrDir , float rrVel , float rrDir);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift);
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
  */
void Transform2RobotCoodinate(wheel_t * wheelVel);
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
  */
void Transform2WheelCoodinate(wheel_t * wheelVel);
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle);
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle);
/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle);
/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle);
/**
* @brief  CalcWheelSpeed计算轮子的和速度大小和方向
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
		  angleN:在机器人坐标系下旋转线速度的正方向(单位：度)
		  postureAngle：机器人姿态角
* @retval 
  */
wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN, float postureAngle);
#endif /* ___H */

//轮子自检函数
uint8_t WheelMotorCheck(void);
uint8_t WheelMotorCheck2(void);
//发送轮子自检速度位置信息函数
void SendSelfCheckWheelSpeed(void);
//用加速度方向计算四个小函道的占空比
void CalcSmallDuckedFanDutyCycle(float vel, float direction);

void OrientationClear(void);

/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/
