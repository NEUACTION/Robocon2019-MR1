#include "light.h"
#include "can.h"
#include "moveBase.h"
#include "elmo.h"
#include "usart.h"
#include "robot.h"

/*红场亮红灯，蓝场亮蓝灯，定位系统初始化/清零后亮白灯，驱动器HF亮绿灯，
	发射过程中气压或俯仰超时亮紫灯，CAN发送失败亮粉灯，手柄按下STOP亮青灯，进入重启后亮金灯*/

/** 
  * @brief  亮白灯
  * @note
  * @param  None
  * @retval None
  */
  
void White(uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x5247424C,0xFFFFFF00,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  亮红灯
  * @note
  * @param  None
  * @retval None
  */

void Red(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF000000,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}
	
/** 
  * @brief  亮绿灯
  * @note
  * @param  None
  * @retval None
  */

void Green(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00FF0000,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  亮蓝灯
  * @note
  * @param  None
  * @retval None
  */
void Blue(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x0000FF00,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  亮金灯
  * @note
  * @param  None
  * @retval None
  */
void Gold(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFFFF0000,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  亮紫灯
  * @note
  * @param  None
  * @retval None
  */
void Purple(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF00FF00,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}


/** 
  * @brief  亮青灯
  * @note
  * @param  None
  * @retval None
  */
void Blue_green(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00FFFF00,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  亮粉灯
  * @note
  * @param  None
  * @retval None
  */
void Pink(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF003200,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
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

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}

/** 
  * @brief  灭灯
  * @note
  * @param  None
  * @retval None
  */
void LightsOff(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00000000,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;		 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;		 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 6000)
		{
			gRobot.CANFailFlag|=CAN2_FAIL_FLAG;
			break;
		}
	} 
}














