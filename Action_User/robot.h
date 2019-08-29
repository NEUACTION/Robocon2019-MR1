#ifndef __ROBOT_H
#define __ROBOT_H
#include "motion.h"
#include "moveBase.h"
#include "adc.h"
#include "timer.h"
#include "pps.h"
#include "path.h"
#include "gasvalveControl.h"
#include "elmo.h"

#define	ROBOT_NONE	(0)
#define ROBOT_RETRY	(1)
#define	ROBOT_RESTART	(2)

#define FIRSTSHAGAIPOSION (1)
#define SECONDSHAGAIPOSION (2)
#define THIRDSHAGAIPOSION (3)

#define GUN_NO_CMD (255)

//发射机构发送给主程序信息
#define GUN_MANUAL_MODE_MSG (1)
#define GUN_SHOOT_DONE_MSG (2)

//主程序发送给发射机构信息
#define GUN_RELOAD_MSG (0)
#define GUN_LOADED_MSG (1)
#define GUN_PREPARE_SHOOT_MSG (2)
#define GUN_SHOOT_CONFIRM_MSG (3)

//是否进入自检
#define ROBOT_SELF_CHECK (1)

#define BLUE_COURT (0x01)
#define RED_COURT (0x02)

#define FIRSTSHAGAI 0u
#define SECONDSHAGAI 1u
#define THIRDSHAGAI 2u

//驱动轮硬件中断标志位
#define WHEEL_INTO_HF (0x01)
//驱动轮编码器错误标志位
#define WHEEL_LOST_ENCODER (0x02)
//驱动轮累计不回复标志位
#define WHEEL_LOST_SUM_COMMUNICATION (0x04)
//驱动轮连续不回复标志位
#define WHEEL_LOST_CONTINIOUS_COMMUNICATION (0x08)

//CAN1发送失败标志位
#define CAN1_FAIL_FLAG (0x01)
//CAN2发送失败标志位
#define CAN2_FAIL_FLAG (0x02)

//手臂电机驱动ID号 蓝场5 红场4
#define ARM_MOTOR_ID_BLUE (5)
#define ARM_MOTOR_ID_RED (4)

//M3508一周的脉冲数
#define M3508_COUNTS_PER_ROUND (8192)
//M3508电机减速比
#define M3508_REDUCTION_RATIO (3591.f/187.0f)
//手臂传动减速比
#define ARM_MECHENIC_REDUCTION_RATIO (1.0f/1.0f)
//手臂减速比
#define ARM_REDUCTION_RATIO (M3508_REDUCTION_RATIO*ARM_MECHENIC_REDUCTION_RATIO)


//缓冲气缸推出气阀板号
#define CUSHION_PUSH_BOARD_ID (0)
//缓冲气缸复位气阀板号
#define CUSHION_RECOVERY_BOARD_ID (0)
//爪子气缸气阀板号
#define CLAW_BOARD_ID (0)

//缓冲气缸推出气阀口号
#define CUSHION_PUSH_IO_ID (5)
//缓冲气缸收回气阀口号
#define CUSHION_RECOVERY_IO_ID (4)
//爪子气缸气阀口号 蓝方3 
#define CLAW_IO_ID (9)

//调试数据结构体
typedef struct
{
	//虚拟位置点坐标
	PointU_t virtualPos;
	//虚拟目标点坐标
	PointU_t virtualTarget;
	//分解到各个轮子的速度大小和方向
	wheel_t wheelVel;
	
	float robotlenVP;
	float robotlenVT;
	
	//已经走过的路程
	float pathLength;
	//轨迹的总路程
	float totalLength;
	//轨迹跟随速度调节量
	robotVel_t adjustVel;
	//原始速度大小
	float originVel;
	//原始速度方向
	float originVelDir;
	//和速度大小
	float sumVel;
	//和速度方向
	float sumVelDir;
	//角速度大小（度/秒）
	float omega;
	//速度环输出的速度大小
	float outputVel;
	//速度环输出的速度方向
	float outputDirection;
	
	float velXErr;
	float velYErr;
	float disAdd;
	float disRealPos2VirPos;
	float disRealPos2VirTarget;
	float VIEW_L;
	float VIEW_L_K;	
	float posAngle;
	float giveOmega;
	uint8_t circleCount;
	
	float tempX;
	float tempY;
	float tempAngle;
	float tempSpeedX;
	float tempSpeedY;
	float tempWZ;
	
	//投块时姿态角偏移量
	float posAngleShift;
	
	PointU_t vTarget;	
	float posAngleVP;
	
	float gasPressure;
	
	float wzFromAngle;
	float accFromSpeed;
	float velFromPos;
	uint8_t SDCardLostFlag;
	uint8_t waitStableFlag;
	
	float x;
	float y;
	
	float robotlenIncrement;
	float robotlenIncrement_last;
	float robotlenIncrement_or;
	
	uint8_t r2vpDetermine;
	uint8_t scope;
}debugInfo_t;

//自检状态变量枚举类型变量
typedef enum
{
	armCheck,
	wheelCheck,
	cushionCheck,
	gasSensorCheck,
	gunCheck,
	duckedFanCheck,
	gpsCheck
}checkStatus_t;

//走行状态变量枚举类型变量
typedef enum
{
	waitForStart,
	
	goToInfrontGobi,
	get1stShagai,
	goToInfrontTZ,
	waitForThrow,
	goToThrow1stShagai,
	throw1stShagai,
	
	goToGet2ndShagai,
	get2ndShagai,
	goToThrow2ndShagai,
	throw2ndShagai,
	
	goToGet3rdShagai,
	get3rdShagai,
	goToThrow3rdShagai,
	throw3rdShagai,
	
	missionAccomplished,
	
	retryState,
	startFromRetry,
	get1stShagaiFromLine1,
	get2ndShagaiFromLine1,
	get3rdShagaiFromLine1,
	testPara,
	omegaTestPara,
	standBy,
	stop
}walkStatus_t;

//机器人电机位置结构体
typedef struct
{
	//期望手臂角度
	float expArmPos;
	//实际手臂角度
	float actArmPosBlue;
	float actArmPosRed;
	//期望发射机构左侧电机俯仰角度
	float expLeftPitchPos;
	//实际发射机构左侧电机俯仰角度
	float actLeftPitchPos;	
	//期望发射机构右侧电机俯仰角度
	float expRightPitchPos;
	//实际发射机构右侧电机俯仰角度
	float actRightPitchPos;
}robotMotor_t;

//发射机构状态结构体
typedef struct
{
	//爪子状态
	uint8_t clawState;
	//平台状态
	uint8_t baseState;
	//气压值
	float gasPressure;
	//俯仰角度
	float leftPitchAngle;
	float rightPitchAngle;
}gunPos_t;

//发射参数结构体
typedef struct
{
	float posAgl;
	float gasPre;
	float leftAgl;
	float rightAgl;
}shootPara_t;
//枪结构体
typedef struct
{
	//参数序号
	uint8_t paraNum;
//	//发射参数结构体
//	para_t para;
	//瞄准标志位
	uint8_t aimFlag;
	//发射机构模式
	uint8_t mode;
	//发射标志位
	uint8_t shootFlag;
	//发射准备标志
	uint8_t ready;
	//发射机构目标姿态
	gunPos_t targetPos;
	//发射机构状态记录
	gunPos_t gunPosRecord;
}gun_t;

typedef struct
{
	uint8_t msgBuffer[10];
	uint8_t bufferRear;
	uint8_t bufferFront;
}communicateMsg_t;

typedef struct
{
	communicateMsg_t posCmd;
	uint8_t shootCmd;
}gunCmdBuffer_t;

typedef struct
{
	uint8_t posCmd;
	uint8_t shootCmd;
}gunCmd_t;

typedef struct
{
	uint8_t isManualMode;
	uint8_t isShootDone;
}mainCmd_t;	

//手柄接受指令
typedef struct
{
	float angleshift;
	uint8_t nextFlag;
	uint8_t stopFlag;
}teleCmd_t;
//手柄接受指令
typedef struct
{
	uint8_t angleShiftSign;
	uint8_t nextFlag;
	uint8_t stopFlag;
}teleDeviceCmd_t;

//手柄接受指令
typedef struct
{
	teleDeviceCmd_t BT;
	teleDeviceCmd_t WIFI;
}teleDevice_t;

typedef struct
{
	float angleAdjust;
	float angleNote;
}manualPos_t;

//全局变量结构体
typedef struct{
	wheelState_t wheelState;
	debugInfo_t debugInfomation;
	walkStatus_t walkStatus;
	//发射机构电机，摆臂电机状态
	robotMotor_t robotMotorState;
	uint8_t CANFailFlag;
	uint8_t courtID;
	//发射机构目标，实际状态
	gun_t gun;
	//小函道结构体
	smallDuckedFan_t smallDuckedFan;
	//自检标志位
	uint8_t selfCheckFlag;
	//自检状态枚举变量
	checkStatus_t checkStatus;
	//手柄命令
	teleCmd_t teleCommand;
	//手柄命令接受媒介
	teleDevice_t teleDevice;
	//重启标志位
	uint8_t resetFlag;
	//减小的距离
	float disReduce;
	
	manualPos_t manualPos;
	
	//重启计数/速度衰减系数
	float resetCoeffic;
	
}gRobot_t;

extern gRobot_t gRobot;

void Walk(void);

int GetTimeCounter(void);

void CountTime(void);

void SetCountTimeFlag(void);

//缓冲气缸伸出
void CushionCylinderPush(void);

//缓冲气缸收回
void CushionCylinderRecovery(void);

//缓冲气缸泄气
void CushionCylinderHold(void);

/**
* @brief  ArmAngle2PositionTransform将手臂角度转化为脉冲
  * @note
* @param  angle:手臂朝向角度
* @retval 对应的电机脉冲位置
  */
int ArmAngle2PositionTransform(float angle);

/**
* @brief  ArmAngle2PositionInverseTransform将手臂脉冲位置转化为角度
  * @note
* @param  position：手臂脉冲位置
* @retval 手臂朝向角度
  */
float ArmAngle2PositionInverseTransform(int position);

void ArmPosControl(float angle);


//手臂旋转
void ArmRotate(void);
//手臂旋转复位
void ArmRotateReset(void);
//缓冲气缸控制
void CushionControl(uint8_t shagaiNum);
//爪子气缸控制
void ClawCtr(uint8_t state);

void GraspTheGerege(void);

void ReleaseTheGerege(void);

uint8_t JudgeStop(float disChange,uint8_t countTime);
uint8_t JudgeStop2(uint8_t shagaiNum,float disChange,uint8_t countTime);

uint8_t JudgeAgainstWall(void);
	
uint8_t JudgeSpeedLessEqual(float speedCompared);

//铲拐骨
void ShovelShagai(uint8_t shagaiNum);

//保持拐骨
void HoldShagai(uint8_t shagaiNum);

void RobotInit(void);

//主控与发射机构间通信初始化
void Gun_Main_CommunicateInit(void);

//主程序向发射机构传递消息序列初始化
void MainMsg2GunInit(void);

//主程序向发射机构传送消息
void MainSendMsg2Gun(uint8_t msg);

//发射机构从消息队列中获取一条来自主程序消息
uint8_t GetMainMsg2Gun(void);

void MainIsManualCmdReset(void);

void MainIsShootDoneReset(void);

void MainCmdReset(void);

//发射程序到主程序消息处理
void Gun2MainMsgProcess(void);

uint8_t JudgeMainMsg2GunEmpty(void);

void ResetCheck(void);
//轮子复位开关检查
void SteerResetCheck(void);

extern void WheelDirectionInit(void);
extern uint8_t gunStatus;

//检查各个驱动器状态是否正常
extern void	CheckDriverState(void);
				
//检查CAN发送是否正常
extern void CheckCANSendState(void);
				
//发送调试数据
extern void SendDebugInfo(void);

Pose_t GetPosPresent(void);

#include "gunControl.h"

#endif
