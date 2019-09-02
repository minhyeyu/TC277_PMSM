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
**  FILENAME  : Test_SPI.c 		                                              **
**                                                                            **
**  VERSION   : 1.0.0                                                         **
**                                                                            **
**  DATE      : 2015-02-14                                                    **
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
** H2S     		HAN SUNG SU , STEVE                                           **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
********************************************************************************

*******************************************************************************/

//****************************************************************************
// @Project			SPI
// @Filename      	Test_SPI.c
// @Author			IFKOR SMD ATV SAE / HAN SUNG SU , STEVE
// @Controller    	Infineon TC277
// @Compiler      	TASKING VX-toolset for Tricore v4.2r1
//****************************************************************************
/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include "main.h"
#include "UsrMTR.h"
#include "Dio.h"
#include "Std_Types.h"
#include "Mcal.h"
#include "Spi.h"
#include "Pwm_17_Gtm.h"

#if MTR_PROCESSING_MEAS_EXEC_TIME == STD_ON
#include "IfxStm_reg.h"
#endif
#include "IfxGtm_regdef.h"
#include "Gtm.h"
#define MTR_MSB_THIRD_MASK		(0x0000FF00)
#define MTR_LSB_MASK			(0x000000FF)
#define	MTR_DUTY_TEST			(1000U)
#define	MTR_PROCESSING_MEAS_EXEC_TIME			(STD_OFF)

uint32 MC_frequency;

/*******************************************************************************
**                      Private Variable Declarations                         **
*******************************************************************************/



//SPI_MOT_Tx SPI_Tx_MSG;
SPI_MOT_DRV3256_Tx	SPI_Tx_MSG;
SPI_MOT_Rx SPI_Rx_MSG;
uint16 Rx_temp;
#pragma align 2
Spi_DataType MTR_TX_Command_8[2];
#pragma align restore
#pragma align 2
Spi_DataType MTR_RX_Command_8[2];
#pragma align restore
uint8 count_spi_MTR;

Rx_Msg0x00_t	RxMsg0x00;
Rx_Msg0x01_t 	RxMsg0x01;
Rx_Msg0x02_t	RxMsg0x02;
Rx_Msg0x03_t	RxMsg0x03;
Rx_Msg0x04_t	RxMsg0x04;
Rx_Msg0x05_t	RxMsg0x05;
Rx_Msg0x06_t	RxMsg0x06;
Tx_Msg0x02_t	TxMsg0x02;
Tx_Msg0x03_t	TxMsg0x03;
Tx_Msg0x04_t	TxMsg0x04;
Tx_Msg0x05_t	TxMsg0x05;
Tx_Msg0x06_t	TxMsg0x06;

//SPI_MOT_Tx_t MTR_Tx_SPI;
//MTR_Tx_SPI_t	MTR_Tx_SPI;
MTR_Rx_SPI_t	MTR_Rx_SPI;
SPI_MOT_Tx_t MOT_SPI_TX;
uint32		Init_func_MTR=0;
uint16	U_HI = 0;
uint16 U_LO = 5000;
uint16 W_HI = 0;
uint16 W_LO = 5000;
uint16 V_HI = 0;
uint16 V_LO = 5000;
uint16 value_write, value_read;
uint8 Dead_Time=100;

static uint16 Mtr_Ctrl_Timeout = 300u;

/*==============================================================================
 *                  GLOBAL VARIABLE DEFINITIONS
 =============================================================================*/

/* Internal Bus */
//Mtr_Processing_Diag_HdrBusType Mtr_Processing_DiagBus;

/* Input Data Element */

/* Output Data Element */


/*******************************************************************************
**                      Private Function Declarations                           **
*******************************************************************************/
void EnableInput_CH1_E_CH2_E_CH3_D(void);
void EnableInput_CH1_E_CH2_D_CH3_E(void);
void EnableInput_CH1_D_CH2_E_CH3_E(void);
void DisableInput_CH1_D_CH2_D_CH3_D(void);
static uint8 Mtr_Process_MotorCtrState(const Mtr_Processing_Internal_t *pMtr_Internal);




/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/

void Send_MTR_Spi(uint16 TX_Command_16, uint16 RX_Command_16);
uint8 parity_check(uint16 input_data);
void spi_amt49100(uint8 Address_5B, uint8 write_1B, uint16 data_9B);


void MTR_Center_Frequency(void)
{
	__nop();
	MC_frequency++;
	__nop();
}


void UsrMTR_Init(void)
{
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, ON);
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_21_5_MTR_IC_EN, ON);
    //Dio_WriteChannel(DioConf_DioChannel_DioChannel_23_6_MTR_RVP_OFF, ON);
    Dio_WriteChannel(DioConf_DioChannel_DioChannel_23_6_MTR_RVP_OFF, OFF);

    Pwm_17_Gtm_SetPeriodAndDuty(Pwm_17_GtmConf_PwmChannel_PwmChannel_MTR_Center_REF, 5000, 2500);
	spi_amt49100(0x12, 1, 0);
	spi_amt49100(0x1C, 0, 0);//11100000_00000000
	spi_amt49100(0x1D, 0, 0);//11101000_00000001
	spi_amt49100(0x1E, 0, 0);//11110000_00000001
	spi_amt49100(0x13, 1, 0);
	spi_amt49100(0x1F, 0, 0);
	spi_amt49100(0x07, 0, 0);
	spi_amt49100(0x07, 1, 0xE0);
	spi_amt49100(0x07, 0, 0);
	spi_amt49100(0x1C, 0, 0);//11100000_00000000
	spi_amt49100(0x1C, 0, 0);//11100000_00000000
	spi_amt49100(0x1F, 1, 0x000);
	spi_amt49100(0x04, 0, 0);
	spi_amt49100(0x04, 1, 0x03F);
	spi_amt49100(0x04, 0, 0);
	spi_amt49100(0x02, 0, 0);
	spi_amt49100(0x02, 1, 0x03F);
	spi_amt49100(0x02, 0, 0);
	spi_amt49100(0x03, 0, 0);
	spi_amt49100(0x03, 1, 0x11F);
	spi_amt49100(0x03, 0, 0);
	spi_amt49100(0x05, 0, 0);


/*
/*
    Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI
	Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO
	Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI
	Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO
	Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI
	Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO
	*/
#if 0 // all high
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,0x8000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,0);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,0x8000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,0);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,0x8000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,0);
#endif
#if 0 //HI LOW, LO HIGH
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,0x4000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,0x4000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,0x4000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,0x4000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,0x4000);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,0x4000);
#endif

#if 0 // all 50% duty
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,2500);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,2500);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,2500);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,2500);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,2500);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,2500);
#endif
	U_HI=0;
	V_HI=0;
	W_HI=0;
    U_LO= ((U_HI)-Dead_Time);
    V_LO= ((V_HI)-Dead_Time);
    W_LO= ((W_HI)-Dead_Time);
    if(U_LO<0) {U_LO=0;}
    else {if(U_LO>5000){U_LO=5000;}}
    if(V_LO<0) {V_LO=0;}
    else {if(V_LO>5000){V_LO=5000;}}
    if(W_LO<0) {W_LO=0;}
    else {if(W_LO>5000){W_LO=5000;}}
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,U_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,U_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,W_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,W_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,V_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,V_LO);
    //Ifx_GTM_TOM_CH_CM0_Bits.CM0;

}

void UsrMTR_func(void)
{
	  	spi_amt49100(0x1F, 0, 0);
		spi_amt49100(0x1C, 0, 0);//11100000_00000000
		spi_amt49100(0x1D, 0, 0);//11101000_00000001
		spi_amt49100(0x1E, 0, 0);//11110000_00000001
		if(U_HI>5000) {U_HI=5000;}
		if(V_HI>5000) {V_HI=5000;}
		if(W_HI>5000) {W_HI=5000;}
	    U_LO= ((U_HI)-Dead_Time);
	    V_LO= ((V_HI)-Dead_Time);
	    W_LO= ((W_HI)-Dead_Time);
	    if(U_LO<0) {U_LO=0;}
	    else {if(U_LO>5000){U_LO=5000;}}
	    if(V_LO<0) {V_LO=0;}
	    else {if(V_LO>5000){V_LO=5000;}}
	    if(W_LO<0) {W_LO=0;}
	    else {if(W_LO>5000){W_LO=5000;}}
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,U_HI);
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,U_LO);
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,W_HI);
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,W_LO);
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,V_HI);
	    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,V_LO);
	    Gtm_SetTomCompareValCm1((uint8) 0,(uint8) 8,(uint16) 1000);

	    //Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,(U_LO));


	Init_func_MTR++;

}





void Send_MTR_Spi(uint16 TX_Command_16, uint16 RX_Command_16)
{
	RX_Command_16 = 0;

	MTR_TX_Command_8[0] = (Spi_DataType)((TX_Command_16&MTR_LSB_MASK)>>0);
	MTR_TX_Command_8[1] = (Spi_DataType)((TX_Command_16&MTR_MSB_THIRD_MASK)>>8);

	Spi_SetupEB(SpiConf_SpiChannel_SpiChannel_1_DRV8353RS,MTR_TX_Command_8,MTR_RX_Command_8,1);

	Spi_SyncTransmit(SpiConf_SpiSequence_SpiSequence_3);

	while(Spi_GetSequenceResult(SpiConf_SpiSequence_SpiSequence_3) != SPI_SEQ_OK)
	{
		if(Spi_GetSequenceResult(SpiConf_SpiSequence_SpiSequence_3) == SPI_SEQ_FAILED)
		{
		  return;
		}
	}
	count_spi_MTR++;

	RX_Command_16 = RX_Command_16|(((uint16)MTR_RX_Command_8[0]<<0) & MTR_LSB_MASK);
	RX_Command_16 = RX_Command_16|(((uint16)MTR_RX_Command_8[1]<<8) & MTR_MSB_THIRD_MASK);

	value_read = RX_Command_16;


}


void spi_amt49100(uint8 Address_5B, uint8 write_1B, uint16 data_9B)
{
	//https://www.eevblog.com/forum/microcontrollers/reading-a-16-bit-word-via-spi-(stm32)/
	 MOT_SPI_TX.All = 0;
	 MOT_SPI_TX.B.MTRAddr_5b = Address_5B;
	 MOT_SPI_TX.B.MTRwrite_1b = write_1B;
	 MOT_SPI_TX.B.MTRdata_9b = data_9B;
	 MOT_SPI_TX.B.MTRparity_1b = parity_check( MOT_SPI_TX.All);
	 value_write = MOT_SPI_TX.All;
	 Send_MTR_Spi(value_write,value_read);
}


uint8 parity_check(uint16 input_data)
{
	uint16 cal_data = input_data;
	sint8 i = 0;
	uint8 parity = 0;

	while(i<15)
	{
		 if(cal_data & 0x8000){
			 parity++;
			 cal_data = (cal_data <<1);
		 }
		 else{
			 cal_data = (cal_data <<1);
		 }
		 i++;
	 }
	if(parity%2){
		return 0;
	}
	else{
		return 1;
	}
}
void EnableInput_CH1_E_CH2_E_CH3_D()
{
	//spi_amt49100(0x1F, 0, 0);
	//spi_amt49100(0x1C, 0, 0);//11100000_00000000
	//spi_amt49100(0x1D, 0, 0);//11101000_00000001
	//spi_amt49100(0x1E, 0, 0);//11110000_00000001
	U_HI=5000;
	V_HI=0;
	W_HI=0;
	U_LO= ((U_HI)-Dead_Time);
	V_LO= ((V_HI)-Dead_Time);
	W_LO= 0;//((W_HI)-Dead_Time);
	if(V_LO<0) {V_LO=0;}
	  else {if(V_LO>5000){V_LO=5000;}}
	if(W_LO<0) {W_LO=0;}
	  else {if(W_LO>5000){W_LO=5000;}}
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,U_HI);
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,U_LO);
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,W_HI);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI);
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,W_LO);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,V_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,V_LO);

}
void EnableInput_CH1_E_CH2_D_CH3_E()
{
	//spi_amt49100(0x1F, 0, 0);
	//spi_amt49100(0x1C, 0, 0);//11100000_00000000
	//spi_amt49100(0x1D, 0, 0);//11101000_00000001
	//spi_amt49100(0x1E, 0, 0);//11110000_00000001
	U_HI=0;
	V_HI-0;
	W_HI=5000;
	U_LO= ((U_HI)-Dead_Time);
	V_LO= ((V_HI)-Dead_Time);
	W_LO= 0;//((W_HI)-Dead_Time);
    if(U_LO<0) {U_LO=0;}
	  else {if(U_LO>5000){U_LO=5000;}}
	if(V_LO<0) {V_LO=0;}
	  else {if(V_LO>5000){V_LO=5000;}}

	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,U_HI);
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,U_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,W_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,W_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,V_HI);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,V_LO);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO);
}
void EnableInput_CH1_D_CH2_E_CH3_E()
{
	//spi_amt49100(0x1F, 0, 0);
	//spi_amt49100(0x1C, 0, 0);//11100000_00000000
	//spi_amt49100(0x1D, 0, 0);//11101000_00000001
	//spi_amt49100(0x1E, 0, 0);//11110000_00000001
	U_HI=0;
	V_HI=5000;
	W_HI=0;
	U_LO= 0;//((U_HI)-Dead_Time);
    V_LO= ((V_HI)-Dead_Time);
	W_LO= ((W_HI)-Dead_Time);
	if(U_LO<0) {U_LO=0;}
	  else {if(U_LO>5000){U_LO=5000;}}
	if(W_LO<0) {W_LO=0;}
	  else {if(W_LO>5000){W_LO=5000;}}
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI,U_HI);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_HI);
	Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO,U_LO);//Pwm_17_Gtm_SetOutputToIdle(Pwm_17_GtmConf_PwmChannel_PwmChannel_U_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_HI,W_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_W_LO,W_LO);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_HI,V_HI);
    Pwm_17_Gtm_SetDutyCycle(Pwm_17_GtmConf_PwmChannel_PwmChannel_V_LO,V_LO);

}

void DisableInput_CH1_D_CH2_D_CH3_D()
{

	UsrMTR_Init();
}
static uint8 Mtr_Process_MotorCtrState(const Mtr_Processing_Internal_t *pMtr_Internal)
{
    uint8 ret = DEF_MTR_REQUEST_NONE;

    switch(pMtr_Internal->Mtr_Status)
    {
        case DEF_MTR_ERROR:
        case DEF_MTR_INIT_ERROR:
        case DEF_MTR_STATE_HOLD:
            ret = DEF_MTR_REQUEST_ERR;
            break;

        case DEF_MTR_INIT:
            ret = DEF_MTR_REQUEST_PWM_INIT;
            break;

        case DEF_MTR_MPS_CALIBRATION:
        case DEF_MTR_CURRENT_OFFSET_CALIBRATION:
        case DEF_MTR_CURRENT_GAINCALIBRATION:
        case DEF_MTR_INVERTER_TEST:

		#if((M_IDB_BSW_TEST == ENABLE) && ((M_IDB_BSW_ENDURANCE_AUTO_PATTERN_TEST == ENABLE) || (M_IDB_BSW_PATTERN_TEST == ENABLE)))
        case DEF_MTR_HW_TEST:
        #endif

        case DEF_MTR_DIAG_MODE:
            ret = DEF_MTR_REQUEST_DIAG_PWM;
            break;

        case DEF_MTR_TARGET_CTR:
        case DEF_MTR_REPEAT_CTR:
            ret = DEF_MTR_REQUEST_DIAG_CTR;
            break;

        case DEF_MTR_NORMAL:
            ret = DEF_MTR_REQUEST_CONTROL;
            break;

        default:
            /* Intentionally Empty */
            break;

    }

    return ret;
}




//https://github.com/realsosy/AurixTutorial/blob/master/docs/SynchronizedPwm.md
//IfxGtm_Pwm_updateDuty

//void Gtm_SetTomCompareValCm1((uint8) 0,(uint8) 8,(uint16) 1000);
