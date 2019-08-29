/*
*********************************************************************************************************
*                                          uC/OS-II APP HOOKS

*
*                               (c) Copyright 2005-2009, Micrium, Weston, FL
*                                          All Rights Reserved
*
*				 
* File    : OS_AppHooks.c
* By      : 
* Version : V2.91
*
* LICENSING TERMS:
* ---------------
*   uC/OS-II is provided in source form for FREE evaluation, for educational use or for peaceful research.
* If you plan on using  uC/OS-II  in a commercial product you need to contact Micriµm to properly license
* its use in your product. We provide ALL the source code for your convenience and to help you experience
* uC/OS-II.   The fact that the  source is provided does  NOT  mean that you can use it without  paying a
* licensing fee.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                          
*********************************************************************************************************
*********************************************************************************************************
*/	   


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <includes.h>

#define  DEF_DISABLED                    0
#define  DEF_ENABLED                     1

#define  OS_VIEW_MODULE                  DEF_DISABLED	     	/* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */

/*
*********************************************************************************************************
*                                      TASK RETURN HOOK (APPLICATION)
*
* Description : 
*
* Argument : ptcb   is a pointer to the task control block of the task being created.
*
* Note     : 
*********************************************************************************************************
*/
void          OSTaskReturnHook(OS_TCB          *ptcb)
{

}



#if OS_APP_HOOKS_EN > 0u
/*
*********************************************************************************************************
*                                      TASK CREATION HOOK (APPLICATION)
*
* Description : This function is called when a task is created.
*
* Argument : ptcb   is a pointer to the task control block of the task being created.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void          App_TaskCreateHook      (OS_TCB          *ptcb)
{
#if (OS_VIEW_MODULE == DEF_ENABLED)
    OSView_TaskCreateHook(ptcb);
#endif
}



/*
*********************************************************************************************************
*                                    TASK DELETION HOOK (APPLICATION)
*
* Description : This function is called when a task is deleted.
*
* Argument : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note     : (1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
void          App_TaskDelHook         (OS_TCB          *ptcb)
{ 
	(void)ptcb; 
}

/*
*********************************************************************************************************
*                                      IDLE TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskIdleHook(), which is called by the idle task.  This hook
*               has been added to allow you to do such things as STOP the CPU to conserve power.
*
* Argument : none.
*
* Note     : (1) Interrupts are enabled during this call.
*********************************************************************************************************
*/
#if OS_VERSION >= 251
void          App_TaskIdleHook        (void)
{		
}
#endif

/*
*********************************************************************************************************
*                                      TASK RETURN HOOK (APPLICATION)
*
* Description :
*
* Argument : none.
*
* Note     : 
*********************************************************************************************************
*/
void          App_TaskReturnHook      (OS_TCB          *ptcb)
{
}

/*
*********************************************************************************************************
*                                        STATISTIC TASK HOOK (APPLICATION)
*
* Description : This function is called by OSTaskStatHook(), which is called every second by uC/OS-II's
*               statistics task.  This allows your application to add functionality to the statistics task.
*
* Argument : none.
*********************************************************************************************************
*/
void          App_TaskStatHook        (void)
{
}


/*
*********************************************************************************************************
*                                        TASK SWITCH HOOK (APPLICATION)
*
* Description : This function is called when a task switch is performed.  This allows you to perform other
*               operations during a context switch.
*
* Argument : none.
*
* Note     : 1 Interrupts are disabled during this call.
*
*            2  It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                   will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the
*                  task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
#if OS_TASK_SW_HOOK_EN > 0u

void          App_TaskSwHook          (void)
{ 
#if (OS_VIEW_MODULE == DEF_ENABLED)
    OSView_TaskSwHook();
#endif 
}

#endif


/*
*********************************************************************************************************
*                                     OS_TCBInit() HOOK (APPLICATION)
*
* Description : This function is called by OSTCBInitHook(), which is called by OS_TCBInit() after setting
*               up most of the TCB.
*
* Argument : ptcb    is a pointer to the TCB of the task being created.
*
* Note     : (1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if OS_VERSION >= 204
void          App_TCBInitHook         (OS_TCB          *ptcb)
{
	(void)ptcb;	
}
#endif


/*
*********************************************************************************************************
*                                     App_TimeTickHook() HOOK (APPLICATION)
*
* Description : 
*
* Argument : 
*
* Note     : 
*********************************************************************************************************
*/
#if OS_TIME_TICK_HOOK_EN > 0u
void          App_TimeTickHook        (void)
{ 
#if (OS_VIEW_MODULE == DEF_ENABLED)
    OSView_TickHook();
#endif
}

#endif

#endif	




 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
