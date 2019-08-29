#include "elmo.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "robot.h"

/*******************************控制驱动器命令************************************/
/**
* @brief  Elmo驱动器初始化
* @param  CANx：所使用的CAN通道编号
* @author ACTION
*/
void ElmoInit(CAN_TypeDef* CANx)
{
	uint32_t data[1][2]={0x00000001,00000000};
	CAN_TxMsg(CANx,ELMO_BROADCAST_ID,(uint8_t*)&data[0],8);
}

/**
* @brief  电机使能（通电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note ELMO驱动器默认初始状态为电机失能，使用电机时需要对其进行使能
*       部分驱动器参数需要在电机失能状态下才可以配置
*/
void MotorOn(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//第一个数发送MO命令，第二个数发送1给电机使能（通电）
	uint32_t data[1][2]={
							0x00004F4D,0x00000001,
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	//为发送结构体赋值 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted

	TxMessage.DLC=8;

 	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	//发送数据
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;
		}
	}
}

/**
* @brief  电机失能（断电）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
*/
void MotorOff(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//第一个数据发送MO命令，第二个数据发送0给电机失能（断电）
	uint32_t data[1][2]={
						0x00004F4D,0x00000000,      //MO  0
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//在这里应加入异常处理
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;
		}
	}

}

/**
* @brief  驱动器速度环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION
* @note 在速度环初始化后才可以使能电机！！
*/
void VelLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
	SetUnitMode(CANx, ElmoNum, SPEED_CONTROL_MODE);
	
	SetSmoothFactor(CANx, ElmoNum, 0);
	
	SetAccAndDec(CANx, ElmoNum, acc, dec);
}

/**
* @brief  驱动器位置环初始化
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note 在位置环初始化后才可以使能电机！！
*/
void PosLoopCfg(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec,uint32_t vel)
{
	SetUnitMode(CANx, ElmoNum, SINGLE_POSITION_MODE);
	
	SetSmoothFactor(CANx, ElmoNum, 0);

	SetAccAndDec(CANx, ElmoNum, acc, dec);

	SetPosLoopVel(CANx, ElmoNum, vel);	
}

/**
* @brief  电机速度控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
*/
void VelCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	SetJoggingVel(CANx, ElmoNum, vel);
	
//	BeginMotion(CANx, ElmoNum);
}

/**
* @brief  电机位置控制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION
*/
void PosCrl(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
{
	SendPosCmd(CANx, ElmoNum, posMode, pos);
	
//	BeginMotion(CANx, ElmoNum);	
}

/**
* @brief  配置驱动器工作模式
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  unitMode：驱动器工作模式，范围：
			TORQUE_CONTROL_MODE：力矩控制模式，在该模式下可以执行TC电流命令
			SPEED_CONTROL_MODE：速度控制模式，在该模式下通过设置JV值控制速度
			MICRO_STEPPER_MODE：直流电机不能使用该模式
			DUAL_POSITION_MODE：双位置闭环模式
			SINGLE_POSITION_MODE：单位置闭环模式，在该模式下可以配置PA、PR、JV、PT或PVT运动
* @author ACTION
* @note 只有在电机失能时可以配置该参数
*/
void SetUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t unitMode)
{

	//第一个数据发送UM命令，第二个数据发送模式
	uint32_t data[1][2]={
						0x00004D55,0x00000000,      //UM
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = unitMode;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;	
		}
	}
}

/**
* @brief  配置驱动器是否对输出进行平滑
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  smoothFactor：是否平滑，范围：0~100（单位：毫秒）
* @author ACTION
* @note ：只有在电机失能时可以配置
*/
void SetSmoothFactor(CAN_TypeDef* CANx, uint8_t ElmoNum, uint8_t smoothFactor)
{
	//第一个数据发送SF命令，第二个数据发送SF命令值
	uint32_t data[1][2]={
						0x00004653,0x00000000,      //SF
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = smoothFactor;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  配置加速度与减速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  acc：加速度，单位：脉冲每二次方秒
* @param  dec：减速度，单位：脉冲每二次方秒
* @author ACTION
* @note
*/
void SetAccAndDec(CAN_TypeDef* CANx, uint8_t ElmoNum, uint32_t acc, uint32_t dec)
{
	//第一个数据发送AC\DC命令，第二个数据发送命令值
	uint32_t data[2][2]={
							0x00004341,0x00000000,		//AC
							0x00004344,0x00000000		//DC
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = acc;
	data[1][1] = dec;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//等待发送成功
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//在这里应加入异常处理
				if(CANx==CAN1)
				{
					gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
				}
				else if(CANx==CAN2)
				{
					gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
				}
				break;
			}
		}
	}
}

/**
* @brief  配置电机速度限制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向速度限制，单位：脉冲每秒，范围：0~80,000,000
* @param  lowerLimit：反方向速度限制，单位：脉冲每秒，范围：-80,000,000~0
* @author ACTION
* @note：只有在电机失能时可以配置该参数
*/
void SetVelLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//第一个数据发送VH[2]\VL[2]命令，第二个数据发送命令值
	uint32_t data[2][2]={
							0x00024856,0x00000000,		//VH[2]
							0x00024C56,0x00000000		//VL[2]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//等待发送成功
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//在这里应加入异常处理
				if(CANx==CAN1)
				{
					gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
				}
				else if(CANx==CAN2)
				{
					gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
				}
				break;	
			}
		}
	}
}

/**
* @brief  配置电机位置限制
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向位置限制，单位：脉冲，范围：-2^31~2^31-1
* @param  lowerLimit：反方向位置限制，单位：脉冲，范围：-2^31~upperLimit
* @author ACTION
* @note：只有在电机失能时可以配置该参数,upperLimit必须大于lowerLimit
*/
void SetPosLimit(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//第一个数据发送VH[3]\VL[3]命令，第二个数据发送命令值
	uint32_t data[2][2]={
							0x00034856,0x00000000,		//VH[3]
							0x00034C56,0x00000000		//VL[3]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//等待发送成功
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//在这里应加入异常处理
				if(CANx==CAN1)
				{
					gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
				}
				else if(CANx==CAN2)
				{
					gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
				}
				break;
			}
		}
	}
}

/**
* @brief  配置位置记录范围
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  upperLimit：正方向位置限制，单位：脉冲，范围：-10^9~10^9
* @param  lowerLimit：反方向位置限制，单位：脉冲，范围：-10^9~10^9
* @author ACTION
* @note：只有在电机失能时可以配置该参数,upperLimit必须大于lowerLimit且upperLimit-lowerLimit结果必须为偶数
*/
void SetPosCountingRange(CAN_TypeDef* CANx, uint8_t ElmoNum, int32_t upperLimit, int32_t lowerLimit)
{
	//第一个数据发送VH[3]\VL[3]命令，第二个数据发送命令值
	uint32_t data[2][2]={
							0x00024D58,0x00000000,    //XM[2]
							0x00014D58,0x00000000    //XM[1]
		};
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = upperLimit;
	data[1][1] = lowerLimit;

	for(uint8_t i =  0;i < 2;i++)
	{
		TxMessage.Data[0] = *(unsigned long*)&data[i][0]&0xff;
		TxMessage.Data[1] = (*(unsigned long*)&data[i][0]>>8)&0xff;
		TxMessage.Data[2] = (*(unsigned long*)&data[i][0]>>16)&0xff;
		TxMessage.Data[3] = (*(unsigned long*)&data[i][0]>>24)&0xff;
		TxMessage.Data[4] = *(unsigned long*)&data[i][1]&0xff;
		TxMessage.Data[5] = (*(unsigned long*)&data[i][1]>>8)&0xff;
		TxMessage.Data[6] = (*(unsigned long*)&data[i][1]>>16)&0xff;
		TxMessage.Data[7] = (*(unsigned long*)&data[i][1]>>24)&0xff;
		mbox= CAN_Transmit(CANx, &TxMessage);
		
		//等待发送成功
		uint16_t timeout = 0;
		while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
		{
			timeout++;
			if(timeout > 60000)
			{
				//在这里应加入异常处理
				if(CANx==CAN1)
				{
					gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
				}
				else if(CANx==CAN2)
				{
					gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
				}
				break;		
			}
		}
	}
}

/**
* @brief  配置位置环运行最大速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SetPosLoopVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	//第一个数据发送SP命令，第二个数据发送命令值
	uint32_t data[1][2]={
							0x00005053,0x00000000,		//SP
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = vel;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//在这里应加入异常处理
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  配置运行速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  vel: 速度，单位：脉冲每秒，范围：最小速度限制到最大速度限制
* @author ACTION
* @note：速度正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SetJoggingVel(CAN_TypeDef* CANx, uint8_t ElmoNum,int32_t vel)
{
	//第一个数据发送JV命令，第二个数据发送命令值
	uint32_t data[1][2]={
							0x0000564A,0x00000000,		//JV
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
	data[0][1] = vel;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//在这里应加入异常处理
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  配置位置环命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @param  posMode: 位置环运行模式，范围：
				ABSOLUTE_MODE: 绝对位置模式
				RELATIVE_MODE: 相对位置模式
* @param  pos:位置命令，单位：脉冲，范围：最大位置限制到最小位置限制
* @author ACTION
* @note：位置正负号代表旋转的方向，大于零为正方向，小于零为负方向
*/
void SendPosCmd(CAN_TypeDef* CANx, uint8_t ElmoNum,uint8_t posMode,int32_t pos)
{

	uint32_t data[1][2]={
							0x00000000,0x00000000,      //PA
						 };

	uint8_t mbox;
	CanTxMsg TxMessage;

	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					 // standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					 // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	if(posMode==ABSOLUTE_MODE)
	{
		data[0][0]= 0x00004150;  //绝对
	}
	else if(posMode==RELATIVE_MODE)
	{
		data[0][0]= 0x00005250;   //相对
	}

	data[0][1]= pos;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;			
		}
	}

}

/**
* @brief  开始新一次运动命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note：
*/
void BeginMotion(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	//第一个数据发送BG命令
	uint32_t data[1][2]={
							0x40004742,0x00000000,    //BG
					 };
	uint8_t mbox;
	CanTxMsg TxMessage;
					 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;		// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;	  	// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;			 			// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;			 				// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] = *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;
	mbox= CAN_Transmit(CANx, &TxMessage);
	
	//等待发送成功
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			//在这里应加入异常处理
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;	
		}
	}
}

/**********************************读取驱动器数据命令*************************************/

/**
* @brief  读取电机电压
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
* @note：
*/
void ReadActualVoltage(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005155,0x00000000,      //UQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;			
		}
	}
 }

/**
* @brief  读取电机电流
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x80005149
*/
void ReadActualCurrent(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005149,0x00000000,      //IQ
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	} 
 }

/**
* @brief  读取电机位置
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00005850
*/
void ReadActualPos(CAN_TypeDef* CANx, uint8_t ElmoNum)
 {
	 uint32_t data[1][2]={
							0x40005850,0x00000000,      //PX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	} 
 }

/**
* @brief  读取电机速度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00005856
*/
void ReadActualVel(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40005856,0x00000000,      //VX
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  读取驱动器温度
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00014954
*/
void ReadActualTemperature(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40014954,0x00000000,      //TI[1]
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;			
		}
	}
}

/**
* @brief  读取驱动器电流限制标志位
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000434C
*/
void ReadCurrentLimitFlag(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000434C,0x00000000,      //LC
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;
		}
	}
}

/**
* @brief  读取驱动器速度误差（驱动器输出速度与实际速度的误差）
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadVelocityError(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004556,0x00000000,      //VE
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
	
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;			
		}
	}
}

/**
* @brief  读取驱动器输出的速度命令值
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x00025644
*/
void ReadCommandVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40025644,0x00000000,      //DV[2]
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
		
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  读取电机接收到的速度命令
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000564A
*/
void ReadJoggingVelocity(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000564A,0x00000000,      //JV
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);        
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  读取电机模式
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadUnitMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D55,0x00000000,      //UM
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}

/**
* @brief  读取驱动器RM状态
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：
*/
void ReadReferenceMode(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x40004D52,0x00000000,      //RM
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;


	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage);         
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;			
		}
	}
}

/**
* @brief  读取驱动器错误代码
* @param  CANx：所使用的CAN通道编号
* @param  ElmoNum：驱动器ID号，范围：0~128，0为广播用ID号
* @author ACTION
 * @note：接收标识符为：0x0000464D
*/
void ReadMotorFailure(CAN_TypeDef* CANx, uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x4000464D,0x00000000,      //MF
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID + ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID + ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CANx, &TxMessage); 
						 
	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CANx, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 60000)
		{
			if(CANx==CAN1)
			{
				gRobot.CANFailFlag|=CAN1_FAIL_FLAG;
			}
			else if(CANx==CAN2)
			{
				gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			}
			break;		
		}
	}
}


