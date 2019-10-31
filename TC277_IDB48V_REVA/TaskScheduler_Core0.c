// ------------------------------------- DISCLAIMER -----------------------------------------//
// THE INFORMATION GIVEN IN THE DOCUMENTS (APPLICATION NOTE, SOFTWARE PROGRAM ETC.)
// IS GIVEN AS A HINT FOR THE IMPLEMENTATION OF THE INFINEON TECHNOLOGIES COMPONENT ONLY
// AND SHALL NOT BE REGARDED AS ANY DESCRIPTION OR WARRANTY OF A CERTAIN FUNCTIONALITY,
// CONDITION OR QUALITY OF THE INFINEON TECHNOLOGIES COMPONENT.
// YOU MUST VERIFY ANY FUNCTION DESCRIBED IN THE DOCUMENTS IN THE REAL APPLICATION.
// INFINEON TECHNOLOGIES AG HEREBY DISCLAIMS ANY AND ALL WARRANTIES AND LIABILITIES OF ANY KIND
// (INCLUDING WITHOUT LIMITATION WARRANTIES OF NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS
// OF ANY THIRD PARTY) WITH RESPECT TO ANY AND ALL INFORMATION GIVEN IN THE DOCUMENTS.
// ------------------------------------------------ -----------------------------------------//
/*******************************************************************************
**                                                                            **
** Copyright (C) Infineon Technologies (2010)                                 **
**                                                                            **
** All rights reserved.                                                       **
**                                                                            **
** This document contains proprietary information belonging to Infineon       **
** Technologies. Passing on and copying of this document, and communication   **
** of its contents is not permitted without prior written authorization.      **
**                                                                            **
********************************************************************************
**                                                                            **
**  FILENAME  : TaskScheduler.c                                              **
**                                                                            **
**  VERSION   : 0.0.1                                                         **
**                                                                            **
**  DATE      : 2013-05-26                                                    **
**                                                                            **
**  VARIANT   : VariantPB                                                     **
**                                                                            **
**  PLATFORM  : AURIX                                                         **
**                                                                            **
**  COMPILER  : Tasking                                                       **
**                                                                            **
**  AUTHOR    : DL-AUTOSAR-Engineering                                        **
**                                                                            **
**  VENDOR    : Infineon Technologies                                         **
**                                                                            **
**  DESCRIPTION  : This file contains                                         **
**                 - sample Demo Test for all the  modules                    **
**                                                                            **
**  SPECIFICATION(S) :                                                        **
**                                                                            **
**  MAY BE CHANGED BY USER [Yes/No]: Yes                                      **
*******************************************************************************/
/*******************************************************************************
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** K,S,H,W      KIM SUNG HOON , WAYNE                                         **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
********************************************************************************

*******************************************************************************/

//****************************************************************************
// @Project            Demo
// @Filename          TaskScheduler.c
// @Author            IFKOR SMD ATV SAE / KIM SUNG HOON , WAYNE
// @Controller        Infineon TC275T-B
// @Compiler          TASKING VX-toolset for Tricore v4.2r2
//****************************************************************************
/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/

//****************************************************************************
// @Project Includes
//****************************************************************************


#include "TaskScheduler.h"
#include "Gpt.h"
#include "Dio.h"
#include "Dio_Ver.h"
#include "main.h"
//#if ASIC_SPI_H_
#include "Test_SPI.h"
//#endif
#if MOT_SPI_H_
#include	"UsrMTR.h"
#include "Pwm_17_Gtm.h"
#include "Adc.h"
#include "Test_ADC.h"
#include "Test_CAN.h"

#endif
#if EPB_SPI_H_
#include "IfxScu_reg.h"
#include "Pwm_17_Gtm.h"
//#include "UsrCan.h"

#endif

#include "IfxPort_reg.h"
#include "6Step_Lib.h"


#if(SYSTEM_TICK==10 || SYSTEM_TICK==100 || SYSTEM_TICK==200 || SYSTEM_TICK==1000)
#endif

//****************************************************************************
// @Defines
//****************************************************************************

//****************************************************************************
// @Prototypes Of Local Functions
//****************************************************************************

#if(SYSTEM_TICK==10 || SYSTEM_TICK==100)
static    void TaskScheduler_100us(void);
#endif
#if(SYSTEM_TICK==10 || SYSTEM_TICK==100)
static    void TaskScheduler_200us(void);
#endif
static    void TaskScheduler_1ms(void);
static    void TaskScheduler_5ms(void);
static    void TaskScheduler_10ms(void);
static    void TaskScheduler_20ms(void);
static    void TaskScheduler_50ms(void);
static    void TaskScheduler_100ms(void);
static    void TaskScheduler_1s(void);
static    void TaskScheduler_5s(void);
static    void TaskScheduler_TaskCalculation(void);
#if EPB_SPI_H_
#endif




//****************************************************************************
// @Global Variables
//****************************************************************************
TASK_CONTROL    TaskControl_Core0;
uint32		time_counter_10ms=0;
uint32		time_counter_1ms =0;
uint32		time_counter_10us=0;
uint32		time_counter_1s  =0;
uint16 		uTesflagPWM = 0;
uint8 		uTesflagMotor=0;

//****************************************************************************
// @ Imported Global Variables
//****************************************************************************
extern CanMsg_t 	DiagInfo;
extern uint32		Init_func_MTR;
extern uint8 		Init_Count;
//****************************************************************************
// @ Imported Global Functions
//****************************************************************************
extern void	ASIC_test(uint32 time_counter);
extern void Adc_Initialization(void);
extern void ADC_Conversion(void);
extern void UsrCan_Task_10ms (void);
#if EPB_SPI_H_
extern	void 	EPB_Int(void);
extern	void	EPB_WD(void);
extern	void	YG_Init(void);
#endif

//*********************************************************************************************
// @Function         void Gpt_Notification_SystemTick_Core0(void)
// @Description       Gpt notification for system tick generation
// @Returnvalue        None
// @Parameters        None
//*********************************************************************************************
void Gpt_Notification_SystemTick_Core0(void)
{

    TaskControl_Core0.B.TickCount++;
    TaskControl_Core0.B.Enable = 1;

} /* Gpt_Notification_SystemTick_Core0() */

void TaskScheduler_Initialization_Core0(unsigned int maxTask)
{
    TaskControl_Core0.U = 0;

    TaskControl_Core0.B.Tick        = SYSTEM_TICK;
    TaskControl_Core0.B.TaskMax     = maxTask;

    //Dio_WriteChannel(DioConf_DioChannel_DioChannel_22_0_MOT_DRV_EN, OFF);
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_22_0_MOT_DRV_EN, ON); // MOT_DRV_EN

    // Enable notification for system tick generation
    Gpt_EnableNotification(GptConf_GptChannel_SystemTick_Core0);

    // Start timer for system tick generation
    Gpt_StartTimer(GptConf_GptChannel_SystemTick_Core0, SYSTEM_TICK_TIMER_PERIOD);

    Can_Initialization();

    //init_func();
    /*Dio_WriteChannel(DIO_CHANNEL_21_4,ON);
    Dio_WriteChannel(DIO_CHANNEL_33_10,OFF);
    Dio_WriteChannel(DIO_CHANNEL_2_4,OFF);
    Dio_WriteChannel(DIO_CHANNEL_33_9, OFF);
    Dio_WriteChannel(DIO_CHANNEL_21_5, OFF);// MTR_IC_EN
    Dio_WriteChannel(DIO_CHANNEL_22_0, OFF);// MOT_DRV_EN*/


    Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, ON);//MTR_IC_EN
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, ON);//MTR_IC_EN
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_23_2_GD_RESETN, ON);//P22_OUT.B.P7 = ON;

    /*Dio_WriteChannel(DIO_CHANNEL_2_3, ON);
    Dio_WriteChannel(DIO_CHANNEL_2_4, ON);
    Dio_WriteChannel(DIO_CHANNEL_21_5, ON);// MTR_IC_EN*/

    Pwm_17_Gtm_SetPeriodAndDuty(Pwm_17_GtmConf_PwmChannel_PwmChannel_MOC_SYNC1, 391, 195);
    Adc_Initialization();


#if ASIC_SPI_H_
    init_func();
#endif


#if EPB_SPI_H_
	EPB_Int();
	init_YG();
#endif

#if MOT_SPI_H_
	UsrMTR_Init();
#endif
} // End of TaskScheduler_Initialization()


//*********************************************************************************************
// @Function         static    void TaskScheduler_TaskCalculation(void)
// @Description       Calculate task job to be executed.
// @Returnvalue        None
// @Parameters        None
//*********************************************************************************************
static void TaskScheduler_TaskCalculation(void)
{
    if((TaskControl_Core0.B.TickCount % TASK_100us) == 0) TaskControl_Core0.B.TaskRun = TASK_100us;
    if((TaskControl_Core0.B.TickCount % TASK_200us) == 0) TaskControl_Core0.B.TaskRun = TASK_200us;
    if((TaskControl_Core0.B.TickCount % TASK_1ms)   == 0) TaskControl_Core0.B.TaskRun = TASK_1ms;
    if((TaskControl_Core0.B.TickCount % TASK_5ms)   == 0) TaskControl_Core0.B.TaskRun = TASK_5ms;
    if((TaskControl_Core0.B.TickCount % TASK_10ms)  == 0) TaskControl_Core0.B.TaskRun = TASK_10ms;
    if((TaskControl_Core0.B.TickCount % TASK_20ms)  == 0) TaskControl_Core0.B.TaskRun = TASK_20ms;
    if((TaskControl_Core0.B.TickCount % TASK_50ms)  == 0) TaskControl_Core0.B.TaskRun = TASK_50ms;
    if((TaskControl_Core0.B.TickCount % TASK_100ms) == 0) TaskControl_Core0.B.TaskRun = TASK_100ms;
    if((TaskControl_Core0.B.TickCount % TASK_1s)    == 0) TaskControl_Core0.B.TaskRun = TASK_1s;
    if((TaskControl_Core0.B.TickCount % TASK_5s)    == 0) TaskControl_Core0.B.TaskRun = TASK_5s;

}  // End of TaskScheduler_TaskCalculation()

//*********************************************************************************************
// @Function         void TaskScheduler_ActivateTask(void)
// @Description       Execute task job.
// @Returnvalue        None
// @Parameters        None
//*********************************************************************************************
void TaskScheduler_ActivateTask_Core0(void)
{
    if(TaskControl_Core0.B.TickCount > TaskControl_Core0.B.TaskMax)        TaskControl_Core0.B.TickCount = 1;

    if(TaskControl_Core0.B.Enable == 1)
    {
        TaskControl_Core0.B.Enable = 0;           // Stop task scheduler

        TaskScheduler_TaskCalculation();

        switch(TaskControl_Core0.B.TaskRun)
        {
                case TASK_100us:
                                TaskScheduler_100us();
                                break;
                case TASK_200us:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                break;
                case TASK_1ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                break;
                case TASK_5ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                break;
                case TASK_10ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                break;
                case TASK_20ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                TaskScheduler_20ms();
                                break;
                case TASK_50ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                TaskScheduler_20ms();
                                TaskScheduler_50ms();
                                break;
                case TASK_100ms:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                TaskScheduler_20ms();
                                TaskScheduler_50ms();
                                TaskScheduler_100ms();
                                break;
                case TASK_1s:

                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                TaskScheduler_20ms();
                                TaskScheduler_50ms();
                                TaskScheduler_100ms();
                                TaskScheduler_1s();
                                break;
                case TASK_5s:
                                TaskScheduler_100us();
                                TaskScheduler_200us();
                                TaskScheduler_1ms();
                                TaskScheduler_5ms();
                                TaskScheduler_10ms();
                                TaskScheduler_20ms();
                                TaskScheduler_50ms();
                                TaskScheduler_100ms();
                                TaskScheduler_1s();
                                TaskScheduler_5s();
                                break;
            default:        break;
        }
    }
}//end of TaskScheduler_ActivateTask()

///////////////   100us Task Job    //////////////////
static    void TaskScheduler_100us(void)
{
    __nop();
    //    P13_OUT.B.P6 = !P13_OUT.B.P6;
    //    P13_OUT.B.P7 = !P13_OUT.B.P7;
        ADC_Conversion();
        switch(DiagInfo.data[0])
        {
        case	0:
        	Dio_WriteChannel(DioConf_DioChannel_DioChannel_23_2_GD_RESETN, OFF);//P22_OUT.B.P7 = OFF;
        	uTesflagPWM = 0;
        	uTesflagMotor = 0;

        	break;
        case	1:
        	uTesflagPWM = 0;
        	uTesflagMotor = 2;
        	break;
    	case	2:
        	uTesflagPWM = 0;
        	uTesflagMotor = 1;
    		break;
    	case	3:
    		Dio_WriteChannel(DioConf_DioChannel_DioChannel_23_2_GD_RESETN, ON);//P22_OUT.B.P7 = ON;
        	uTesflagPWM = 1;
        	uTesflagMotor=0;
        	Dead_Time=DiagInfo.data[1];
        	U_HI= (DiagInfo.data[2]<<8)|(DiagInfo.data[3]);
        	V_HI =(DiagInfo.data[4]<<8)|(DiagInfo.data[5]);
        	W_HI =(DiagInfo.data[6]<<8)|(DiagInfo.data[7]);
            UsrMTR_func();
    		break;
    	case	4:
        	uTesflagPWM = 0;
        	uTesflagMotor=0;
    		//EPB_test();
    		//EPB_current();
    		break;
    	default:
        	uTesflagPWM = 0;
        	uTesflagMotor =0;
    		break;

        }
#if 1//MOT_SPI_H_
//    if(uTesflagPWM == 1) 	{	UsrMTR_func();}
    if(uTesflagMotor ==1 )
    {
		Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, ON);//MTR_IC_EN
    	MC_StartMotor();
    }
    else
    {
    	if(uTesflagMotor == 2)
    	{
    		MC_StopMotor();
    		Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, OFF);//MTR_IC_EN
    	}
    }
#endif
    __nop();
} // End of TaskScheduler_100us();

///////////////   200us Task Job    //////////////////
static    void TaskScheduler_200us(void)
{
    __nop();
    __nop();

} // End of TaskScheduler_200us();

///////////////   1ms Task Job    //////////////////
static    void TaskScheduler_1ms(void)
{

    __nop();
    time_counter_1ms++;
    Dio_FlipChannel(DioConf_DioChannel_DioChannel_11_3_PRN);
#if EPB_SPI_H_
    EPB_current();
#endif
    __nop();

} // End of TaskScheduler_1ms();


///////////////   5ms Task Job    //////////////////
static    void TaskScheduler_5ms(void)
{
    __nop();
     __nop();

} // End of TaskScheduler_5ms();


///////////////   10ms Task Job    /////////////////
static    void TaskScheduler_10ms(void)
{
    __nop();
    time_counter_10ms++;
    ASIC_test(time_counter_10ms);

#if 0//MOT_SPI_H_
//    if(uTesflagPWM == 1) 	{	UsrMTR_func();}
    if(uTesflagMotor ==1 )
    {
    	MC_StartMotor();
    }
    else
    {
    	if(uTesflagMotor == 2)
    	{
    		MC_StopMotor();
    	}
    }
#endif

    ADC_Conversion();
    //ASIC_test(time_counter_10ms);
	#if	TEST_SPI_H
    switch(time_counter_10ms % 5)
    {
    case	0:
    	EPB_WD();
    	break;
    case	1:
    	EPB_WD();
    	break;
	case	2:
		EPB_WD();
		break;
	case	3:
		EPB_WD();
		break;
	case	4:
		EPB_WD();
		//EPB_test();
		//EPB_current();
		break;
	default:
		break;
    }
	#endif
    __nop();

} // End of TaskScheduler_10ms();

///////////////   20ms Task Job    /////////////////
static    void TaskScheduler_20ms(void)
{
    __nop();
    __nop();

} // End of TaskScheduler_20ms();

///////////////   50ms Task Job    /////////////////
static    void TaskScheduler_50ms(void)
{
    __nop();
    __nop();

} // End of TaskScheduler_50ms();


///////////////   100ms Task Job    /////////////////
static    void TaskScheduler_100ms(void)
{
    __nop();
    UsrCan_Task_10ms();
//    Can_Test();
    __nop();
} // End of TaskScheduler_100ms();


static    void TaskScheduler_1s(void)
{
    __nop();
    //   Dio_FlipChannel(DioConf_DioChannel_MOC_RESET);
    time_counter_1s++;
    //EPB_test();
    //EPB_Actuator_operation();
    __nop();

} // End of TaskScheduler_1s();

///////////////   5s Task Job    /////////////////
static    void TaskScheduler_5s(void)
{
    __nop();
    __nop();

} // End of TaskScheduler_5s();
