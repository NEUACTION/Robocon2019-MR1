#ifndef __PATH_H
#define __PATH_H

#include "motion.h"
#include "moveBase.h"

extern Pose_t testPath[6];

//从出发区到取第一个兽骨轨迹长度
#define GET_FIRST_SHAGAI_PATH_NUM (78)
//#define GET_FIRST_SHAGAI_PATH_NUM (107)
//#define GET_FIRST_SHAGAI_PATH_NUM (116)

extern Pose_t getFirstShagaiPath[GET_FIRST_SHAGAI_PATH_NUM];

//取完第一个兽骨到投掷区前等待轨迹长度
#define GO2_TZ_PATH_NUM (22)

extern Pose_t go2TZPath[GO2_TZ_PATH_NUM];

//从投掷区前到投掷第一个兽骨位置轨迹长度
#define THROW_FIRST_SHAGAI_PATH_NUM (4)

extern Pose_t throwFirstShagaiPath[THROW_FIRST_SHAGAI_PATH_NUM];

//取第二个兽骨轨迹长度
#define GET_2ND_SHAGAI_PATH_NUM (29)

extern Pose_t get2ndShagaiPath[GET_2ND_SHAGAI_PATH_NUM];

//投掷第二个兽骨轨迹长度
#define THROW_2ND_SHAGAI_PATH_NUM (25)

extern Pose_t throw2ndShagaiPath[THROW_2ND_SHAGAI_PATH_NUM];

//取第三个兽骨轨迹长度
#define GET_3RD_SHAGAI_PATH_NUM (33)

extern Pose_t get3rdShagaiPath[GET_3RD_SHAGAI_PATH_NUM];

//投掷第三个兽骨轨迹长度
#define THROW_3RD_SHAGAI_PATH_NUM (27)

extern Pose_t throw3rdShagaiPath[THROW_3RD_SHAGAI_PATH_NUM];


void PathLineInterpolation(Pose_t *path , Pose_t startPos , Pose_t endPos ,\
						   float percent1 , float percent2 , float posture1 , float posture2);

void PathInit(uint8_t courtId);
#endif
