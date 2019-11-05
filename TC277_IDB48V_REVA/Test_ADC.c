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
// @Project			Demo
// @Filename      	TaskScheduler.c
// @Author			IFKOR SMD ATV SAE / KIM SUNG HOON , WAYNE
// @Controller    	Infineon TC275T-B
// @Compiler      	TASKING VX-toolset for Tricore v4.1r1
//****************************************************************************

#include "Adc.h"
#include "Dio.h"
#include "Test_ADC.h"
#include "IfxVadc_reg.h"
#include "main.h"
#include "Test_ADC_KMW.h"

/*******************************************************************************
**            			Private Macro Definitions                             **
*******************************************************************************/

/*******************************************************************************
**                      Private Type Definitions                              **
*******************************************************************************/

typedef struct
{
	uint16	ADC0_SCAN;
	uint16	ADC1_SCAN;
	uint16	ADC2_SCAN;
	uint16	ADC3_SCAN;
	uint16	ADC4_SCAN;
	uint16	ADC5_SCAN;
	uint16	ADC6_SCAN;
	uint16	ADC7_SCAN;
	uint16	ADC8_SCAN;

} ADC_GROUP_STATUS;

/*******************************************************************************
**                      Private Function Declarations                         **
*******************************************************************************/

/*******************************************************************************
**                      Global Constant Definitions                           **
*******************************************************************************/
#define ADC_CHANNEL_NUMBER	(12)
/*******************************************************************************
**                      Global Variable Definitions                           **
*******************************************************************************/

// Result buffer declaration
Adc_ValueGroupType 	Adc0_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc1_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc2_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc3_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc4_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc5_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc6_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc7_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc4_Result_Scan_HW[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc3_Result_Scan_HW[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};
Adc_ValueGroupType 	Adc8_Result_Scan[ADC_CHANNEL_NUMBER]	= {0,0,0,0,0,0,0,0,0,0,0,0};

// Notification counter
ADC_GROUP_STATUS	Adc_NotificationCount={0};

// Result buffer status
ADC_GROUP_STATUS	Adc_ResultBufferStatus={0};


/*******************************************************************************
**                      Private  Constant Definitions                         **
*******************************************************************************/

/*******************************************************************************
**                     Private  Variable Definitions                          **
*******************************************************************************/
Adc_StreamNumSampleType AdcSWGroupNoSamples = 8;

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/
Adc_ValueGroupType  testdata[24][10];
uint32 limit1 =0;
uint32 limit2 =0;

ADC_value_t ADC_value;


//extern void Adc0_Initialization(void);

void Adc_Initialization(void);
void ADC_Conversion(void) ;


//******************************************************************************
// @Function	 	void Adc0_Initialization(void)
// @Description   	Initialize ADC0 module
// @Returnvalue		None
// @Parameters    	None
//******************************************************************************
void Adc_Initialization(void)
{

#if 0
	Adc_ResultBufferStatus.ADC0_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_4, &Adc0_Result_Scan[0]);
	Adc_ResultBufferStatus.ADC1_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_4_HW_Trig, &Adc1_Result_Scan[0]);
#endif
#if 1

#endif

#if 1
	if(Adc_ResultBufferStatus.ADC0_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC0_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_4, &Adc4_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_4);

	}
#endif

	if(Adc_ResultBufferStatus.ADC1_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC1_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_4_HW_Trig, &Adc4_Result_Scan_HW[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_4_HW_Trig);
		Adc_EnableHardwareTrigger(AdcConf_AdcGroup_AdcGroup_4_HW_Trig);

	}
	if(Adc_ResultBufferStatus.ADC2_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC2_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_3_HW_Trig, &Adc3_Result_Scan_HW[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_3_HW_Trig);
		Adc_EnableHardwareTrigger(AdcConf_AdcGroup_AdcGroup_3_HW_Trig);


	}
	if(Adc_ResultBufferStatus.ADC3_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC3_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_0, &Adc0_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_0);

	}
	if(Adc_ResultBufferStatus.ADC4_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC4_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_1, &Adc1_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_1);

	}
	if(Adc_ResultBufferStatus.ADC5_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC5_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_2, &Adc2_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_2);

	}
	if(Adc_ResultBufferStatus.ADC6_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC6_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_3, &Adc3_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_3);

	}
	if(Adc_ResultBufferStatus.ADC7_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC7_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_5, &Adc5_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_5);

	}
	if(Adc_ResultBufferStatus.ADC8_SCAN == E_OK)
	{
		Adc_ResultBufferStatus.ADC8_SCAN = Adc_SetupResultBuffer(AdcConf_AdcGroup_AdcGroup_6, &Adc8_Result_Scan[0]);
		Adc_EnableGroupNotification(AdcConf_AdcGroup_AdcGroup_6);

	}
/*
AdcConf_AdcGroup_AdcGroup_4
AdcConf_AdcGroup_AdcGroup_4_HW_Trig
AdcConf_AdcGroup_AdcGroup_7_HW_Trig
AdcConf_AdcGroup_AdcGroup_0
AdcConf_AdcGroup_AdcGroup_1
AdcConf_AdcGroup_AdcGroup_2
AdcConf_AdcGroup_AdcGroup_3
AdcConf_AdcGroup_AdcGroup_5
*/

}



void ADC_Conversion(void)
{
	 while( (Adc_17_GetStartupCalStatus()) != E_OK);

	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_4);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_0);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_1);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_2);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_3);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_5);
	Adc_StartGroupConversion(AdcConf_AdcGroup_AdcGroup_6);



}



/* ADC Voltage conversion (ex : VBATT01)
 * VBATT01_MON : declared by 8bit, So, max value = 255
 * 5V : 255 = xV : VBATT01_MON
 * VBATT01_MON = 255*x / 5
 * x = VBATT * 5.1/35.1 = VBATT * 510/3510
 * VBATT01_MON = 255/5 * VBATT * 510 / 3510
 * VBATT = VBATT01_MON *5 *3510 / (255*510)
 * VBATT = VBATT01_MON *500 *3510 / (255*510) --> to express floating --> so , result = ?? /100 --> ex: 331 = 3.31V
 *
 */


void ADC_notification_0(void)
{


	__nop();
	//	ADC_value.TEMP_MON		= Adc4_Result_Scan[0];
		ADC_value.MPS_5V_MON	= Adc4_Result_Scan[0];
		ADC_value.SIMV_DRN_MON	= Adc4_Result_Scan[1];
		ADC_value.PDF_5V_MON	= Adc4_Result_Scan[2];
		ADC_value.RLNO_MON		= Adc4_Result_Scan[4];
		ADC_value.RRNO_MON		= Adc4_Result_Scan[5];
		ADC_value.PDT_SIG_MON	= Adc4_Result_Scan[6];

	Adc_NotificationCount.ADC0_SCAN++;
	if(Adc_NotificationCount.ADC0_SCAN >100)
	{
		Adc_NotificationCount.ADC0_SCAN = 0;
	}

	__nop();

}


void ADC_notification_1(void)
{
	__nop();
	ADC_value.W_PHASE_MON = Adc4_Result_Scan_HW[0];
	Adc_NotificationCount.ADC1_SCAN++;
	if(Adc_NotificationCount.ADC1_SCAN >16000)
	{
		Adc_NotificationCount.ADC1_SCAN = 0;
	}
	__nop();

}
void ADC_notification_2(void)
{
	__nop();
	//	ADC_value.W_PHASE_MON = Adc3_Result_Scan_HW[0];
	ADC_value.U_PHASE_MON = Adc3_Result_Scan_HW[0];
	Adc_NotificationCount.ADC2_SCAN++;
	if(Adc_NotificationCount.ADC2_SCAN >16000)
	{
		Adc_NotificationCount.ADC2_SCAN = 0;
	}

	__nop();
}

void ADC_notification_3(void)
{
	__nop();
	ADC_value.WDV_FR_MON = 		Adc0_Result_Scan[0];
	ADC_value.CUTV_P_MON = 		Adc0_Result_Scan[1];
	ADC_value.WDV_RL_MON = 		Adc0_Result_Scan[2];
	ADC_value.CE_MON	 = 		Adc0_Result_Scan[3];
	ADC_value.FSP_ABS_MON = 	Adc0_Result_Scan[4];
	ADC_value.IF_G1_AD_MON = 	Adc0_Result_Scan[5];
	ADC_value.PDF_SIG_MON = 	Adc0_Result_Scan[6];
	ADC_value.SW4LINE_F_MON = 	Adc0_Result_Scan[7];
	Adc_NotificationCount.ADC3_SCAN++;
	if(Adc_NotificationCount.ADC3_SCAN >100)
	{
		Adc_NotificationCount.ADC3_SCAN = 0;
	}
	__nop();
}

void ADC_notification_4(void)
{
	__nop();
	ADC_value.SW3LINE_E_MON	= Adc1_Result_Scan[0];
	ADC_value.VDD5_MON		= Adc1_Result_Scan[1];
	ADC_value.SW1LINE_C_MON	= Adc1_Result_Scan[2];
	ADC_value.VBATT02_MON	= Adc1_Result_Scan[3];
	ADC_value.FSP_CBS_MON	= Adc1_Result_Scan[4];
	ADC_value.PDT_5V_MON	= Adc1_Result_Scan[5];
	ADC_value.VDD3_MON		= Adc1_Result_Scan[6];
	ADC_value.V5_INT_MON	= Adc1_Result_Scan[7];
	Adc_NotificationCount.ADC4_SCAN++;
	if(Adc_NotificationCount.ADC4_SCAN >100)
	{
		Adc_NotificationCount.ADC4_SCAN = 0;
	}
	__nop();
}

void ADC_notification_5(void)
{
	__nop();
	ADC_value.SIMV_MON			= Adc2_Result_Scan[0];
	ADC_value.RELV1_MON			= Adc2_Result_Scan[1];
	ADC_value.RELV2_MON		= Adc2_Result_Scan[2];
	ADC_value.V5_EXT_MON			= Adc2_Result_Scan[3];
	ADC_value.FLNO_MON			= Adc2_Result_Scan[4];
	ADC_value.FRNO_MON			= Adc2_Result_Scan[5];
	ADC_value.MOT_POW_MON		= Adc2_Result_Scan[6];
	ADC_value.WDV_FR_DRN_MON	= Adc2_Result_Scan[7];
	Adc_NotificationCount.ADC5_SCAN++;
	if(Adc_NotificationCount.ADC5_SCAN >100)
	{
		Adc_NotificationCount.ADC5_SCAN = 0;
	}
	__nop();
}

void ADC_notification_6(void)
{
	__nop();
	ADC_value.VBATT01_MON			= Adc3_Result_Scan[0];
	ADC_value.PD1_MON		= Adc3_Result_Scan[1];
	ADC_value.MOC_POW_MON			= Adc3_Result_Scan[2];
	ADC_value.SW2LINE_B_MON		= Adc3_Result_Scan[3];
	ADC_value.IF_A3_MON		= Adc3_Result_Scan[4];
	ADC_value.RELV1_DRN_MON			= Adc3_Result_Scan[5];
	ADC_value.CUTV_P_DRN_MON		= Adc3_Result_Scan[6];

	Adc_NotificationCount.ADC6_SCAN++;
	if(Adc_NotificationCount.ADC6_SCAN >100)
	{
		Adc_NotificationCount.ADC6_SCAN = 0;
	}
	__nop();
}

void ADC_notification_7(void)
{
	__nop();
	ADC_value.VDD_MON		= Adc5_Result_Scan[0];
	ADC_value.PD2_MON	= Adc5_Result_Scan[1];
	ADC_value.WDV_RL_DRN_MON= Adc5_Result_Scan[2];
	ADC_value.TEMP_MON2		= Adc5_Result_Scan[3];
	ADC_value.BAT2_CURR_MON		= Adc5_Result_Scan[4];
	ADC_value.U_OUT_MON		= Adc5_Result_Scan[5];
	ADC_value.W_OUT_MON		= Adc5_Result_Scan[6];
	ADC_value.V_OUT_MON		= Adc5_Result_Scan[7];
	Adc_NotificationCount.ADC7_SCAN++;
	if(Adc_NotificationCount.ADC7_SCAN >100)
	{
		Adc_NotificationCount.ADC7_SCAN = 0;
	}
	__nop();
}
void ADC_notification_8(void)
{
	__nop();
	ADC_value.TEMP_MON		= Adc5_Result_Scan[0];
	ADC_value.GD_VDD5V_MON	= Adc5_Result_Scan[1];
	if(Adc_NotificationCount.ADC8_SCAN >100)
	{
		Adc_NotificationCount.ADC8_SCAN = 0;
	}
	__nop();
}





