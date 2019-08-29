#ifndef __PPS_H
#define __PPS_H
#include "stdint.h"


/*接受几个来自定位系统的float数据*/
#define GET_PPS_VALUE_NUM      6
/*接受几个来自定位系统的uint8_t数据*/ /* 6 * 4byte = 24*/
#define GET_PPS_DATA_NUM       24

//接收激光值数据
#define GET_LASER_VALUE_NUM      11
#define GET_LASER_DATA_NUM       44

typedef union{
	uint8_t data[GET_PPS_DATA_NUM];
	float  value[GET_PPS_VALUE_NUM];
}PosSend_t;

typedef union{
	uint8_t data[GET_LASER_DATA_NUM];
	float  value[GET_LASER_VALUE_NUM];
}LaserSend_t;

/*定位系统返回的值*/
typedef struct{
		/*定位系统返回的角度*/
		float ppsAngle ;
		/*定位系统返回的X值*/
		float ppsX ;
		/*定位系统返回的Y值*/
		float ppsY ;
		/*定位系统返回的X轴速度*/
		float ppsSpeedX;
		/*定位系统返回的Y轴速度*/
		float ppsSpeedY;
		/*定位系统的z轴角速度*/
		float ppsWZ ;
	
}Pos_t;

typedef struct
{
	float x;
	float y;
}position_t;

//八个激光值信息
typedef struct
{
	float laserOne;
	float laserTwo;
	float laserThr;
	float laserFor;
	float laserFiv;
	float laserSix;
	float laserSev;
	float laserEig;
	
}laserValue_t;


//void TalkOpsToGetReady(void);
//void TalkOpsToReset(void);
///*初始化并且让程序等待定位系统发数*/
//void WaitOpsPrepare(void);

void SetOpsReady(uint8_t flag);
void SetAngle(float setValue);
void SetX(float setValue);
void SetY(float setValue);
void SetSpeedX(float setValue);
void SetSpeedY(float setValue);
void SetWZ(float setValue);

/*定位系统准备完毕*/
uint8_t GetOpsReady(void);
/*返回定位系统的角度*/
float GetAngle(void);
/*返回定位系统的X值*/
float GetX(void);
/*返回定位系统的Y值*/
float GetY(void);
/*返回定位系统的X轴的速度*/
float GetSpeedX(void);
/*返回定位系统的角度*/
float GetSpeedY(void);
/*返回定位系统的Z轴角速度值*/
float GetWZ(void);

//返回减去绕机器人中心旋转角速度产生的线速度后的速度
position_t GetSpeedWithoutOmega(void);
position_t GetSpeedWithoutOmega_c(void);

//发送发送获取激光值命令
void SendGetLaserCmd(void);

void SendCourtIDCmd(void);

//发送开始校正标志
void SendBeginCorrect(void);

//发送停止校正标志
void SendStopCorrect(void);

void Send3RDWallCorrect(void);

void Send5THWallCorrect(void);

float GetContiniousAngle(void);

int GetAngleLoopNum(void);

void TalkLaserOpsToReset(void);

void TalkLaserOpsToGetReady(void);

void SetLaserOpsReady(uint8_t flag);

uint8_t GetLaserOpsReady(void);

void WaitLaserOpsPrepare(void);

void WaitLaserOpsReset(void);

//判断坐标是否正确
void CheckPos1(void);

void CheckPos2(PosSend_t pos);

#endif 

