#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include "telecontroller.h"
#include "usart.h"
#include "string.h"
#include "robot.h"
#include "dma.h"
#include "light.h"

static char teleMsg_BT[MAX_TELECMD_BYTE_LENGTH] = {0};
static uint8_t teleCntCnt_BT = 0;
static uint8_t teleCntFlag_BT = 0;
static char teleMsg_WIFI[MAX_TELECMD_BYTE_LENGTH] = {0};
static uint8_t teleCntCnt_WIFI = 0;
static uint8_t teleCntFlag_WIFI = 0;

//WIFI
void USART3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	uint8_t data = 0;

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		data = USART_ReceiveData(USART3);
		
		TeleCmdBufferInput_WIFI(data);
	}
	else
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_LBD);
		USART_ClearITPendingBit(USART3, USART_IT_CTS);
		USART_ClearITPendingBit(USART3, USART_IT_ERR);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART3, USART_IT_NE);
		USART_ClearITPendingBit(USART3, USART_IT_FE);
		USART_ReceiveData(USART3);
	}
	OSIntExit();
}
//BlueTooth
void UART4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	uint8_t data = 0;
		
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		data = USART_ReceiveData(UART4);
		
		TeleCmdBufferInput_WIFI(data);
		
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
	else
	{
		USART_ClearITPendingBit(UART4, USART_IT_PE);
		USART_ClearITPendingBit(UART4, USART_IT_TXE);
		USART_ClearITPendingBit(UART4, USART_IT_TC);
		USART_ClearITPendingBit(UART4, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART4, USART_IT_IDLE);
		USART_ClearITPendingBit(UART4, USART_IT_LBD);
		USART_ClearITPendingBit(UART4, USART_IT_CTS);
		USART_ClearITPendingBit(UART4, USART_IT_ERR);
		USART_ClearITPendingBit(UART4, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART4, USART_IT_NE);
		USART_ClearITPendingBit(UART4, USART_IT_FE);
		USART_ReceiveData(UART4);
	}
	OSIntExit();
}

//手柄字符存入
//先判断指令格式正确再判断指令内容正确 节省中断内运行时间
//BT
void TeleCmdBufferInput_BT(uint8_t ch)
{	
	static uint8_t count = 0;
	static uint8_t i=0;
	
	switch(count)
	{
		case 0:
		{
			if(ch=='A')
				count++;
			else
				count=0;
			break;
		}
		case 1:
		{
			if(ch=='T')
			{
				count++;
			}
			else
				count=0;
			break;
		}
		case 2:
		{
			if(ch=='+')
			{
				i=0;
				count++;
			}
			else
				count=0;
			break;
		}
		case 3:
		{
			teleMsg_BT[i]=ch;
			
			i++;
			
			if(i>=MAX_TELECMD_BYTE_LENGTH)
			{
				i=0;
				count++;
			}
			break;
		}
		case 4:
		{
			if(ch=='\r')
				count++;
			else
				count=0;
			break;
		}
		case 5:
		{
			if(ch=='\n')
			{
				//手柄消息处理BT
				TeleCmdPross_BT(teleMsg_BT);
			}
			count=0;
			break;
		}
		default:
			count=0;
		break;		 
	}
}

//WIFI
void TeleCmdBufferInput_WIFI(uint8_t ch)
{	
	static uint8_t count = 0;
	static uint8_t i=0;

	switch(count)
	{
		case 0:
		{
			if(ch=='A')
				count++;
			else
				count=0;
			break;
		}
		case 1:
		{
			if(ch=='T')
			{
				count++;
			}
			else
				count=0;
			break;
		}
		case 2:
		{
			if(ch=='+')
			{
				i=0;
				count++;
			}
			else
				count=0;
			break;
		}
		case 3:
		{
			teleMsg_WIFI[i]=ch;
			
			i++;
			
			if(i>=MAX_TELECMD_BYTE_LENGTH)
			{
				i=0;
				count++;
			}
			break;
		}
		case 4:
		{
			if(ch=='\r')
				count++;
			else
				count=0;
			break;
		}
		case 5:
		{
			if(ch=='\n')
			{
				//手柄消息处理WIFI
				TeleCmdPross_WIFI(teleMsg_WIFI);
			}
			count=0;
			break;
		}
		default:
			count=0;
		break;		 
	}
}

//手柄消息处理
//stop 直接停止 left/right 累计(限幅15.0f) next 置下一步标志位 
//接收到“AT+CNT\r\n”后 回复“AT+WF\r\n” "AT+BT\r\n"
void TeleCmdPross_BT(char* teleMsg)
{
	if(strncmp(&teleMsg[0],"NXT",3)==0 || strncmp(&teleMsg[0],"STP",3)==0 || strncmp(&teleMsg[0],"LFT",3)==0 || strncmp(&teleMsg[0],"RGT",3)==0 || strncmp(&teleMsg[0],"CNT",3)==0)
	{	
		if(strncmp(&teleMsg[0],"NXT",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1)
			{
				gRobot.teleDevice.BT.nextFlag = TELENEXT;
				
				teleCntFlag_BT = 0;
							
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTNEXT!!\r\n");
				
				USART_OUT(USART2,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINN\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"STP",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1)
			{
				gRobot.teleDevice.BT.stopFlag = TELESTOP;
				
				teleCntFlag_BT = 0;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTSTOP!!\r\n");
				
				USART_OUT(USART2,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINS\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"LFT",3)==0)
		{
			teleCntCnt_BT = 0;
		
			if(teleCntFlag_BT == 1 && (gRobot.walkStatus == goToThrow1stShagai || gRobot.walkStatus == throw1stShagai || gRobot.walkStatus ==goToThrow2ndShagai ||\
					gRobot.walkStatus == throw2ndShagai || gRobot.walkStatus ==goToThrow3rdShagai || gRobot.walkStatus == throw3rdShagai))
			{

				gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NEGIATIVE;
				
				teleCntFlag_BT = 0;
				
	//			AngleShiftLimit();
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTLEFT!!\r\n");
				
				USART_OUT(USART2,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINL\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"RGT",3)==0)
		{
			teleCntCnt_BT = 0;
			
			if(teleCntFlag_BT == 1 && (gRobot.walkStatus == goToThrow1stShagai || gRobot.walkStatus == throw1stShagai || gRobot.walkStatus ==goToThrow2ndShagai ||\
					gRobot.walkStatus == throw2ndShagai || gRobot.walkStatus ==goToThrow3rdShagai || gRobot.walkStatus == throw3rdShagai))
			{
				gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_POSITIVE;
				
				teleCntFlag_BT = 0;
				
	//			AngleShiftLimit();
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"BTRIGHT!!\r\n");
				
				USART_OUT(USART2,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"BTINR\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"CNT",3)==0)
		{
			//连续2次收到CNT 开始下一次读取指令 即相邻两次按键时间间隔必须大于150*2ms！！！
			teleCntCnt_BT++;
			
			if(teleCntCnt_BT >= 2)
			{		
				teleCntFlag_BT = 1;
			}
					
	//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	//				(uint8_t *)"BTCNTCMD!!\r\n");
			
			USART_OUT(USART2,(uint8_t *)"AT+BT\r\n");
		}
	}
	else
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"BT%s ERROR TELECMD\r\n",teleMsg);
	}
}

void TeleCmdPross_WIFI(char* teleMsg)
{
	if(strncmp(&teleMsg[0],"NXT",3)==0 || strncmp(&teleMsg[0],"STP",3)==0 || strncmp(&teleMsg[0],"LFT",3)==0 || strncmp(&teleMsg[0],"RGT",3)==0 || strncmp(&teleMsg[0],"CNT",3)==0)
	{
		if(strncmp(&teleMsg[0],"NXT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				gRobot.teleDevice.WIFI.nextFlag = TELENEXT;
				
				teleCntFlag_WIFI = 0;
							
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFNEXT!!\r\n");
				
				USART_OUT(UART4,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINN\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"STP",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1)
			{
				gRobot.teleDevice.WIFI.stopFlag = TELESTOP;
				
				teleCntFlag_WIFI = 0;
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFSTOP!!\r\n");
				
				USART_OUT(UART4,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINS\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"LFT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1 && (gRobot.walkStatus == goToThrow1stShagai || gRobot.walkStatus == throw1stShagai || gRobot.walkStatus ==goToThrow2ndShagai ||\
					gRobot.walkStatus == throw2ndShagai || gRobot.walkStatus ==goToThrow3rdShagai || gRobot.walkStatus == throw3rdShagai))
			{
				gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NEGIATIVE;
				
				teleCntFlag_WIFI = 0;
				
	//			AngleShiftLimit();
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFLEFT!!\r\n");
				
				USART_OUT(UART4,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINL!\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"RGT",3)==0)
		{
			teleCntCnt_WIFI = 0;
			
			if(teleCntFlag_WIFI == 1 && (gRobot.walkStatus == goToThrow1stShagai || gRobot.walkStatus == throw1stShagai || gRobot.walkStatus ==goToThrow2ndShagai ||\
					gRobot.walkStatus == throw2ndShagai || gRobot.walkStatus ==goToThrow3rdShagai || gRobot.walkStatus == throw3rdShagai))
			{
				gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_POSITIVE;
				
				teleCntFlag_WIFI = 0;
				
	//			AngleShiftLimit();
				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"WFRIGHT!!\r\n");
				
				USART_OUT(UART4,(uint8_t *)"AT+FH\r\n");
			}
//			else
//			{
//				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//					(uint8_t *)"WFINR\r\n");
//			}
		}
		if(strncmp(&teleMsg[0],"CNT",3)==0)
		{
			//连续2次收到CNT 开始下一次读取指令 即相邻两次按键时间间隔必须大于150*2ms！！！
			teleCntCnt_WIFI++;
			
			if(teleCntCnt_WIFI >= 2)
			{		
				teleCntFlag_WIFI = 1;
			}
			
	//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	//				(uint8_t *)"WIFICNTCMD!!\r\n");

			USART_OUT(UART4,(uint8_t *)"AT+WF\r\n");
		}
	}	
	else
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"WIFI%s ERROR TELECMD\r\n",teleMsg);
	}
}
//void AngleShiftLimit(void)
//{
//	if(fabs(gRobot.teleCommand.angleshift)>35.0f)
//	{
//		gRobot.teleCommand.angleshift = gRobot.teleCommand.angleshift / fabs(gRobot.teleCommand.angleshift) * 35.0f;
//		
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//				(uint8_t *)"ANGLESHIFT IS MAX\r\n");
//	}
//}

void TeleCmdProssGather(void)
{
	if(gRobot.teleDevice.BT.nextFlag == TELENEXT || gRobot.teleDevice.WIFI.nextFlag == TELENEXT)
	{
		gRobot.teleCommand.nextFlag = TELENEXT;
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"TELENEXT!!\r\n");
		
		gRobot.teleDevice.BT.nextFlag = TELENOCMD;
		gRobot.teleDevice.WIFI.nextFlag = TELENOCMD;
		
		TeleInit(); 
	}
	if(gRobot.teleDevice.BT.stopFlag == TELESTOP || gRobot.teleDevice.WIFI.stopFlag == TELESTOP)
	{
		gRobot.teleCommand.stopFlag = TELESTOP;
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"TELESTOP!!\r\n");
		
		gRobot.teleDevice.BT.stopFlag = TELENOCMD;
		gRobot.teleDevice.WIFI.stopFlag = TELENOCMD;
		
		TeleInit();
	}
	if(gRobot.teleDevice.BT.angleShiftSign == ANGLESHIFT_POSITIVE || gRobot.teleDevice.WIFI.angleShiftSign == ANGLESHIFT_POSITIVE)
	{
		if(gRobot.gun.paraNum < (MAXPARANUM-1))
		{
			gRobot.gun.paraNum++;
		}
		else
		{
			gRobot.gun.paraNum = (MAXPARANUM-1);
		}
		
		SetShootPara();
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"TELERIGHT!!\r\n");
		
		gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NONE;
		gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NONE;
		
		TeleInit();
	}
	if(gRobot.teleDevice.BT.angleShiftSign == ANGLESHIFT_NEGIATIVE || gRobot.teleDevice.WIFI.angleShiftSign == ANGLESHIFT_NEGIATIVE)
	{
		if(gRobot.gun.paraNum > 1 && gRobot.gun.paraNum <= (MAXPARANUM-1))
		{
			gRobot.gun.paraNum--;
		}
		else
		{
			gRobot.gun.paraNum = 1;
		}
		
		SetShootPara();
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"TELELEFT!!\r\n");
		
		gRobot.teleDevice.BT.angleShiftSign = ANGLESHIFT_NONE;
		gRobot.teleDevice.WIFI.angleShiftSign = ANGLESHIFT_NONE;
		
		TeleInit();
	} 
}
void TeleInit(void)
{
	for(uint8_t i = 0;i < MAX_TELECMD_BYTE_LENGTH;i++)
	{
		teleMsg_BT[i] = 0;
		teleMsg_WIFI[i] = 0;
	}
	
	teleCntCnt_BT = 0;
	teleCntFlag_BT = 0;
	
	teleCntCnt_WIFI = 0;
	teleCntFlag_WIFI = 0;
}


