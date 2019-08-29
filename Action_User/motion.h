#ifndef __MOTION_H
#define __MOTION_H
//点的结构体 单位mm
typedef struct
{
	float x;
	float y;
}Point_t;
//点斜式结构体 ，斜率用角度制的角度代替
typedef struct
{
	Point_t point;
	//角度制
	float   direction;
	//速度
	float vel;
}Pose_t;
typedef struct
{
	Point_t point;
	float u;
	float direction;
	unsigned short startPtr;
	unsigned short endPtr;
}PointU_t;
typedef struct
{
	float module;
	float direction;
}vector_t;
//角度制转换为弧度制系数
#define CHANGE_TO_RADIAN    0.01745329251994f   
//弧度制转换为角度制系数
#define CHANGE_TO_ANGLE     57.29577951308232f				
//圆周率
#ifndef PI
#define PI                  3.1415926f
#endif

#endif 
