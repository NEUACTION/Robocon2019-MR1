#include "steerReset.h"
#include "usart.h"
#include "string.h"
#include "robot.h"
#include "timer.h"
#include "dma.h"

//转向电机圈数偏移接收循环队列
static char loopMsgBuffer[LOOP_MSG_BUFFER_SIZE][LOOP_MSG_MAX_LENGTH] = {0};

//每条消息内字节指针
static uint8_t loopMsgBytePtr = 0;

//循环队列头尾
static uint8_t loopMsgBufferRear , loopMsgBufferFront = 0;

//接收标志位
static uint8_t loopMsgRecieveFlag = 0;

//发送读取圈数命令
void SendReadLoopCmd(void)
{
	USART_OUT(UART5,(uint8_t *)"AT");
}

//循环数组初始化
void LoopMsgBufferInit(void)
{
	loopMsgBufferFront = 0;
	loopMsgBufferRear = 0;
	loopMsgBytePtr = 0;
}

/**
* @brief  JudgeLoopMsgBufferEmpty判断循环队列是否为空
  * @note
* @param  
* @retval 1:队列非空 0：队列为空
  */
uint8_t JudgeLoopMsgBufferEmpty(void)
{
	if(loopMsgBufferFront!=loopMsgBufferRear)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
* @brief  LoopMsgBufferInput循环队列入队
  * @note
* @param  inputData:要入队的数据
* @retval 
  */
void LoopMsgBufferInput(char inputData)
{
	//填入数据
	loopMsgBuffer[loopMsgBufferRear][loopMsgBytePtr] = inputData;
	
	//指针自加
	loopMsgBytePtr++;
	
	//收完一条消息后将消息入队
	if(loopMsgBytePtr>=LOOP_MSG_MAX_LENGTH)
	{
		loopMsgBytePtr = 0;
		loopMsgBufferRear++;
		loopMsgBufferRear%=LOOP_MSG_BUFFER_SIZE;
	}
}

/**
* @brief  f2int浮点数四舍五入转换为整数
  * @note
* @param  floatData:浮点数
* @retval 转换后的整数
  */
int f2int(float floatData)
{
	floatData+=floatData>0?0.5:-0.5;
	
	return (int)floatData;
}

//处理队列中的消息
void LoopMsgProcess(void)
{
	union loopMsgRecieve
	{
		uint8_t byteData[4];
		float floatData;
	}loopMsgunion;
	
	//当队列非空时处理消息
	while(JudgeLoopMsgBufferEmpty())
	{
		//判断消息头和尾是否正确
		if(strncmp(&loopMsgBuffer[loopMsgBufferFront][0],"AT",2)==0&&\
			strncmp(&loopMsgBuffer[loopMsgBufferFront][7],"\r\n",2)==0)
		{
			//将圈数信息填入联合体
			loopMsgunion.byteData[0] = loopMsgBuffer[loopMsgBufferFront][3];
			loopMsgunion.byteData[1] = loopMsgBuffer[loopMsgBufferFront][4];
			loopMsgunion.byteData[2] = loopMsgBuffer[loopMsgBufferFront][5];
			loopMsgunion.byteData[3] = loopMsgBuffer[loopMsgBufferFront][6];
			
			//判断圈数是否正常
			if(fabs(loopMsgunion.floatData)<=(3.0f*144.0f))
			{
				//根据收到的编号接收圈数并置位标志位
				switch(loopMsgBuffer[loopMsgBufferFront][2])
				{
					case '5':
						loopMsgRecieveFlag|=LF_RECIEVE_FLAG;
						gRobot.wheelState.steerLoopShift.lf = f2int(loopMsgunion.floatData);
						break;
					case '6':
						loopMsgRecieveFlag|=RF_RECIEVE_FLAG;
						gRobot.wheelState.steerLoopShift.rf = f2int(loopMsgunion.floatData);
						break;
					case '7':
						loopMsgRecieveFlag|=LR_RECIEVE_FLAG;
						gRobot.wheelState.steerLoopShift.lr = f2int(loopMsgunion.floatData);
						break;
					case '8':
						loopMsgRecieveFlag|=RR_RECIEVE_FLAG;
						gRobot.wheelState.steerLoopShift.rr = f2int(loopMsgunion.floatData);
						break;
					default:
						break;
				}
			}
		}
		
		for(uint8_t i = 0 ; i<LOOP_MSG_MAX_LENGTH ; i++)
		{
			loopMsgBuffer[loopMsgBufferFront][i] = 0;
		}
		
		loopMsgBufferFront++;
		loopMsgBufferFront%=LOOP_MSG_BUFFER_SIZE;
	}
}

//读取转向电机偏移圈数
uint8_t GetSteerLoopShift(void)
{
	uint8_t timeOut = 0;
	
	//初始化接收队列
	LoopMsgBufferInit();
	
	while(1)
	{
		//发送读圈数命令
		SendReadLoopCmd();
		
		delay_ms(20);
		
		//超时计数，如果超时则将圈数偏移置位0并结束等待
		timeOut++;
		if(timeOut>=100)
		{
			gRobot.wheelState.steerLoopShift = (steerLoopShift_t){0,0,0,0};
			return 0;
		}
		
		//处理队列中的消息
		LoopMsgProcess();
		
		//如果收完四个电机的圈数则结束循环跳出
		if(loopMsgRecieveFlag==0x0f)
		{
//			int a=0;
//			a=1;
			return 1;
		}
	}
}
