/*
 * main.c
 *
 *  Created on: 2016. 8. 26.
 *      Author: mando
 */


#include "Std_Types.h"
#include "TaskScheduler.h"
#include "Dio.h"

//#include "Mcal_Compiler.h"


void core0_main(void)
{
	TaskScheduler_Initialization_Core0(TASK_1s);

	while(1)
	{
		__nop();
		TaskScheduler_ActivateTask_Core0();
		//Dio_FlipChannel(DioConf_DioChannel_VDD_DRV);
		//Dio_FlipChannel(DioConf_DioChannel_EN_DR);
		__nop();
	}

}

