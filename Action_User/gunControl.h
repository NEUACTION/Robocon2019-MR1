#ifndef __GUNCONTROL_H
#define __GUNCONTROL_H

#include "adc.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "timer.h"
#include "usart.h"
#include "robot.h"

#define MAXPARANUM (6)

////蓝场俯仰角
//#ifndef SHOOT_PITCH_ANGLE
//#define LEFT_SHOOT_PITCH_ANGLE (7.1f)
//#define RIGHT_SHOOT_PITCH_ANGLE (7.0f)
//#endif
extern float posAngle_EnterManulMode;
extern uint8_t pushShagaiCount;

//红场俯仰角
#ifndef SHOOT_PITCH_ANGLE
#define LEFT_SHOOT_PITCH_ANGLE (7.5f)
#define RIGHT_SHOOT_PITCH_ANGLE (7.6f)
#endif

#ifndef SHOOT_GAS_PRESSURE
#define SHOOT_GAS_PRESSURE (0.423f)
//#define SHOOT_GAS_PRESSURE (0.2f)
#endif

//发射机构姿态数目
#define GUN_POS_NUM (4)

//发射机构复位姿态
#define GUN_RESET_POS (0)
//发射机构装弹姿态
#define GUN_RELOAD_POS (1)
//发射机构装弹完成姿态
#define GUN_LOADED_POS (2)
//发射机构发射姿态
#define GUN_SHOOT_POS (3)

//发射机构自动模式
#define GUN_AUTO_MODE (0)

//发射机构手动模式
#define GUN_MANUAL_MODE (1)

//发射机构等待发射命令
#define GUN_WAIT_SHOOT_CMD (0)

//发射机构可以发射
#define GUN_SHOOT_PERMITTED (1)

//发射机构爪子关闭
#define GUN_CLAW_CLAMP (1)

//发射机构爪子打开
#define GUN_CLAW_LOOSE (0)

//发射机构平台复位
#define GUN_BASE_RESET (0)

//发射机构平台伸出
#define GUN_BASE_EXTEND (1)

//发射机构正在瞄准
#define GUN_AIMING (0)

//发射机构瞄准完毕
#define GUN_AIM_READY (1)

//发射机构目标姿态改变
#define GUN_AIM_CHANGED (1)

//发射机构姿态到达给定姿态
#define GUN_AIM_DONE (0)



//接收发射信息队列长度
#define GUN_MSG_BUFFER_SIZE (50)

//接收消息长度
#define GUN_MSG_MAX_LENGTH (20)

//ADC转气压零点
//#define GAS_ADC_ZERO (372.5f)
#define GAS_ADC_ZERO (366.9f)

//发射俯仰机构减速比
#define SHOOT_PITCH_MECHENIC_REDUCTION_RATIO (10.0f/1.0f)
//发射机构俯仰减速比
#define SHOOT_PITCH_REDUCTION_RATIO (M3508_REDUCTION_RATIO * SHOOT_PITCH_MECHENIC_REDUCTION_RATIO)

//发射机构爪子机构减速比
#define GUN_CLAW_MECHENIC_REDUCTION_RATIO (14.0f/1.0f)
//发射机构爪子减速比
#define GUN_CLAW_REDUCTION_RATIO (M3508_REDUCTION_RATIO * GUN_CLAW_MECHENIC_REDUCTION_RATIO)

//发射机构俯仰角电机驱动器ID号
#define SHOOT_PITCH_LEFT_MOTOR_ID (2)
#define SHOOT_PITCH_RIGHT_MOTOR_ID (3)



//发射机构发射气缸气阀板号
#define GUN_SHOOT_BOARD_ID (0)

//发射机构平台气缸气阀板号
#define GUN_BASE_BOARD_ID (0)

//发射机构爪子气缸气阀板号
#define GUN_CLAW_BOARD_ID (0)

//发射机构发射气缸气阀口号  
#define GUN_SHOOT_IO_ID (3)

//发射机构平台气缸气阀口号
#define GUN_BASE_IO_ID (6)

//发射机构爪子气缸气阀口号
#define GUN_CLAW_IO_ID (8)


/**
* @brief  GetGasPressure将气压的电压信号转换为气压值
  * @note
* @param  
  * @retval气压值（单位：兆帕）
  */
float GetGasPressure(void);

/**
* @brief  GasPressureControl将气压转换为对应的PWM占空比输出
  * @note
* @param  gasPressure:气压值（单位：兆帕）
  * @retval
  */
void GasPressureControl(float gasPressure);

/**
* @brief  ShootAngle2PositionTransform将发射机构俯仰角度转化为脉冲
  * @note
* @param  angle:发射机构俯仰角度
* @retval 对应的电机脉冲位置
  */
int ShootAngle2PositionTransform(float angle);

/**
* @brief  ShootAngle2PositionInverseTransform将发射机构俯仰脉冲位置转化为角度
  * @note
* @param  position：发射机构俯仰脉冲位置
* @retval 发射机构俯仰角度
  */
float ShootAngle2PositionInverseTransform(int position);

/**
* @brief  ShootPitchControl发射机构俯仰角度控制
  * @note
* @param  angle：发射机构俯仰角度
* @retval
  */
void ShootPitchControl(float leftAngle, float rightAngle);

//发射机构发射气缸推出
void GunShootPush(void);

//发射机构发射气缸收回
void GunShootPull(void);

//发射机构平台气缸推出
void GunBaseOut(void);

//发射机构平台气缸收回
void GunBaseIn(void);

//发射机构爪子夹紧
void GunClawClamp(void);

//发射机构爪子复位
void GunClawLoose(void);

//发射机构调试参数动作执行
void ExecuteManualOrder(void);

//发射机构手动控制
void ManualControl(void);

/**
* @brief  GunClawAngle2PositionTransform将发射机构爪子角度转化为脉冲
  * @note
* @param  angle:发射机构爪子角度
* @retval 对应的电机脉冲位置
  */
int GunClawAngle2PositionTransform(float angle);

/**
* @brief  GunClawAngle2PositionInverseTransform将发射机构爪子脉冲位置转化为角度
  * @note
* @param  position：发射机构爪子脉冲位置
* @retval 发射机构爪子角度
  */
float GunClawAngle2PositionInverseTransform(int position);

/**
* @brief  GunClawPosControl发射机构爪子角度控制
  * @note
* @param  angle：发射机构爪子角度
* @retval
  */
void GunClawPosControl(float angle);

void GunMsgBufferInit(void);

/**
* @brief  JudgeGunMsgBufferEmpty判断循环队列是否为空
  * @note
* @param  
* @retval 1:队列非空 0：队列为空
  */
uint8_t JudgeGunMsgBufferEmpty(void);

/**
* @brief  GunMsgBufferInput循环队列入队
  * @note
* @param  inputData:要入队的数据
* @retval 
  */
void GunMsgBufferInput(char inputData);

void GunMsgProcess(void);

void SendGasPressureValue(USART_TypeDef* USARTx , float value);

//发射机构平台控制
void GunBaseControl(uint8_t cmd);

//发射机构爪子控制
void GunClawControl(uint8_t cmd);

//发射机构瞄准
void GunAim(gunPos_t targetPos);

//发射机构复位
void GunPosReset(void);

//检查发射机构模式
uint8_t GunCheckMode(void);

extern gunPos_t gGunPosDataBase[GUN_POS_NUM];

void GunInit(void);

//检查发射机构参数
void GunCheckAim(void);

//发射机构发射
void GunShoot(void);

//发射机构向主控传递消息序列初始化
void GunMsg2MainInit(void);

uint8_t JudgeGunMsg2MainEmpty(void);

//发射机构向主程序传递消息
void GunSendMsg2Main(uint8_t msg);

//主程序从消息队列中获取一条来自发射机构的消息
uint8_t GetGunMsg2Main(void);

void GunPosCmdReset(void);

void GunShootCmdReset(void);

void GunCmdReset(void);

gunCmd_t GetGunCmd(void);

//主程序到发射程序消息处理
void Main2GunMsgProcess(void);

//设置投射参数
void SetShootPara(void);

#endif
