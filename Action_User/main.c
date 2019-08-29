#include  <includes.h>
#include <app_cfg.h>

static  OS_STK  App_TaskStartStk[APP_TASK_START_STK_SIZE];


INT32S main (void)
{
	CPU_INT08U  os_err;
	os_err = os_err; /* prevent warning... */

	/* Note:  由于使用UCOS, 在OS运行之前运行,注意别使能任何中断. */
	CPU_IntDis();                     /* Disable all ints until we are ready to accept them.  */

	OSInit();                        /* Initialize "uC/OS-II, The Real-Time Kernel".         */
	
	os_err = OSTaskCreateExt((void (*)(void *)) App_TaskStart,  /* Create the start task.                               */
						 (void          * ) 0,
						 (OS_STK        * )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
						 (INT8U           ) APP_TASK_START_PRIO,
						 (INT16U          ) APP_TASK_START_PRIO,
						 (OS_STK        * )&App_TaskStartStk[0],
						 (INT32U          ) APP_TASK_START_STK_SIZE,
						 (void          * )0,
						 (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

	OSStart();                        /* Start multitasking (i.e. give control to uC/OS-II).  */
	return (0);
}

static  void  App_TaskStart ()
{   
	  OS_CPU_SysTickInit(SystemCoreClock/OS_TICKS_PER_SEC);                 /* 10ms，已仿真测试     */
	
#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                                            /* Determine CPU capacity.                              */
#endif
     
     App_Task();                                        /* Create application tasks.                            */

	for(;;)
   	{
      	OSTimeDlyHMSM(0, 1, 0, 0);							 /* Delay One minute */
    }	
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
