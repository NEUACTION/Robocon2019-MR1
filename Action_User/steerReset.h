#ifndef __STEER_RESET_H
#define __STEER_RESET_H

#include "stdint.h"

//循环队列长度
#define LOOP_MSG_BUFFER_SIZE (20)

//每一条消息长度
#define LOOP_MSG_MAX_LENGTH (9)

//圈数接收标志位
#define LF_RECIEVE_FLAG (0x01)
#define RF_RECIEVE_FLAG (0x02)
#define LR_RECIEVE_FLAG (0x04)
#define RR_RECIEVE_FLAG (0x08)

void SendReadLoopCmd(void);

void LoopMsgBufferInit(void);

uint8_t JudgeLoopMsgBufferEmpty(void);

void LoopMsgBufferInput(char inputData);

int f2int(float floatData);

void LoopMsgProcess(void);

uint8_t GetSteerLoopShift(void);

#endif
