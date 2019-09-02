/******************************************************************************
**                                                                           **
** Copyright (C) Infineon Technologies (2013)                                **
**                                                                           **
** All rights reserved.                                                      **
**                                                                           **
** This document contains proprietary information belonging to Infineon      **
** Technologies. Passing on and copying of this document, and communication  **
** of its contents is not permitted without prior written authorization.     **
**                                                                           **
*******************************************************************************
**                                                                           **
**  $FILENAME   : Test_Print.h $                                             **
**                                                                           **
**  $CC VERSION : \main\4 $                                                  **
**                                                                           **
**  $DATE       : 2013-11-07 $                                               **
**                                                                           **
**  AUTHOR      : DL-AUTOSAR-Engineering                                     **
**                                                                           **
**  VENDOR      : Infineon Technologies                                      **
**                                                                           **
**  DESCRIPTION :  This file exports the functionality of Test_Print.c       **
**                                                                           **
**  MAY BE CHANGED BY USER [yes/no]: Yes                                     **
**                                                                           **
******************************************************************************/


/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/

#include "Std_Types.h"


typedef unsigned char       uBitType;

/*******************************************************************************
**                      Global Symbols                                        **
*******************************************************************************/


/*******************************************************************************
**                      Global Type Definitions                               **
*******************************************************************************/
//#define MCH_MAX_BUFFER_SIZE     100u//60u

//#define MCH_SPI_DMA_ENABLE

/* SPI ADDRESS */
/* TI DRV8353RS SPI MAP */
#define DRV8353RS_ADDR_FAULT_STATUS_1              	(0x01U)
#define DRV8353RS_ADDR_VGS_STATUS_2              	(0x01U)
#define DRV8353RS_ADDR_DRIVER_CONTROL              	(0x02U)
#define DRV8353RS_ADDR_GATE_DRIVER_HS             	(0x03U)
#define DRV8353RS_ADDR_GATE_DRIVER_LS            	(0x04U)
#define DRV8353RS_ADDR_OCP_CONTROL             		(0x05U)
#define DRV8353RS_ADDR_CSA_CONTROL             		(0x06U)
#define DRV8353RS_ADDR_RESERVED             		(0x07U)


//*******************************************************************************
//**                      Global Function Declarations                          **
//*******************************************************************************

 //*==============================================================================
 //*                  GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 //=============================================================================


// Rx
typedef union
{
    uint16  ALL;
    struct
    {
        uint16 MTRdata      :8;
        uint16 MTRAddr     	:7;
        uint16 MTRRW    	:1;

    }d;
}SPI_MOT_DRV3256_Tx;


typedef union
{
    uint16  ALL;
    struct
    {
        uint16 MTRdata      :11;
        uint16 MTRAddr     	:4;
        uint16 MTRRW    	:1;

    }d;
}SPI_MOT_Tx;


typedef union
{
    uint16  ALL;
    struct
    {
        uint16 MTRdata1      :11;
        uint16 Reserved      :5;

    }d;
}SPI_MOT_Rx;


//Tx

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_VDS_LC		:1;
        uBitType 		Rx_VDS_HC		:1;
        uBitType 		Rx_VDS_LB		:1;
        uBitType 		Rx_VDS_HB		:1;
        uBitType 		Rx_VDS_LA		:1;
        uBitType 		Rx_VDS_HA		:1;
        uBitType 		Rx_OTSD		:1;
        uBitType 		Rx_UVLO		:1;
        uBitType 		Rx_GDF		:1;
        uBitType 		Rx_VDS_OCP		:1;
        uBitType 		Rx_FAULT		:1;

	}B;
}Rx_Msg0x00_t;


typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_VGS_LC		:1;
        uBitType 		Rx_VGS_HC		:1;
        uBitType 		Rx_VGS_LB		:1;
        uBitType 		Rx_VGS_HB		:1;
        uBitType 		Rx_VGS_LA		:1;
        uBitType 		Rx_VGS_HA		:1;
        uBitType 		Rx_GDUV		:1;
        uBitType 		Rx_OTW		:1;
        uBitType 		Rx_SC_OC		:1;
        uBitType 		Rx_SB_OC		:1;
        uBitType 		Rx_SA_OC		:1;
	}B;
}Rx_Msg0x01_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_CLR_FLT		:1;
        uBitType 		Rx_BRAKE		:1;
        uBitType 		Rx_COAST		:1;
        uBitType 		Rx_1PWM_DIR		:1;
        uBitType 		Rx_1PWM_COM		:1;
        uBitType 		Rx_PWM_MODE		:2;
        uBitType 		Rx_OTW_REP		:1;
        uBitType 		Rx_DIS_GDF		:1;
        uBitType 		Rx_DIS_GDUV		:1;
        uBitType 		Rx_OCP_ACT		:1;
	}B;
}Rx_Msg0x02_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_IDRVEN_HS		:4;
        uBitType 		Rx_IDRIVEP_HS		:4;
        uBitType 		Rx_LOCK		:3;
	}B;
}Rx_Msg0x03_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_IDRIVEN_LS		:4;
        uBitType 		Rx_IDRVEP_LS		:4;
        uBitType 		Rx_TDRIVE		:2;
        uBitType 		Rx_CBC		:1;
	}B;
}Rx_Msg0x04_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Rx_VDS_LVL		:4;
        uBitType 		Rx_OCP_DEG		:2;
        uBitType 		Rx_OCP_MODE		:2;
        uBitType 		Rx_DEAD_TIME		:2;
        uBitType 		Rx_TRETRY		:1;
	}B;
}Rx_Msg0x05_t;

typedef union
{
	uint16 R;
	struct
	{
		uBitType 		Rx_SEN_LVL		:2;
		uBitType 		Rx_CSA_CAL_C		:1;
		uBitType 		Rx_CSA_CAL_B		:1;
		uBitType 		Rx_CSA_CAL_A		:1;
		uBitType 		Rx_DIS_SEN		:1;
		uBitType 		Rx_CSA_GAIN		:2;
		uBitType 		Rx_LS_REF		:1;
		uBitType 		Rx_VREF_DIV		:1;
		uBitType 		Rx_CSA_FET		:1;

	}B;
}Rx_Msg0x06_t;

typedef union
{
    uint32  R;
    struct
    {
        uint32 DATA      :11;
        uint32 ADD_FBK   :5;

    }B;
}MTR_Rx_SPI_t;		//RX_data_t;

typedef union
{
    uint32  R;
    struct
    {
        uint32 DATA      :11;
        uint32 ADD_FBK   :4;
        uBitType RW		 :1;

    }B;
}MTR_Tx_SPI_t;		//TX_data_t;

typedef union
{
    uint16  All;
    struct
    {
    	uint8 MTRparity_1b		:1;
        uint16 MTRdata_9b     	:9;
        uint8 MTRwrite_1b   	:1;
        uint8 MTRAddr_5b     	:5;

    }B;
}SPI_MOT_Tx_t;
//****************************************** Tx ***************************************************************
typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Tx_CLR_FLT		:1;
        uBitType 		Tx_BRAKE		:1;
        uBitType 		Tx_COAST		:1;
        uBitType 		Tx_1PWM_DIR		:1;
        uBitType 		Tx_1PWM_COM		:1;
        uBitType 		Tx_PWM_MODE		:2;
        uBitType 		Tx_OTW_REP		:1;
        uBitType 		Tx_DIS_GDF		:1;
        uBitType 		Tx_DIS_GDUV		:1;
        uBitType 		Tx_OCP_ACT		:1;

	}B;
}Tx_Msg0x02_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Tx_IDRVEN_HS		:4;
        uBitType 		Tx_IDRIVEP_HS		:4;
        uBitType 		Tx_LOCK				:3;

	}B;
}Tx_Msg0x03_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Tx_IDRIVEN_LS		:4;
        uBitType 		Tx_IDRVEP_LS		:4;
        uBitType 		Tx_TDRIVE			:2;
        uBitType 		Tx_CBC				:1;

	}B;
}Tx_Msg0x04_t;

typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Tx_VDS_LVL			:4;
        uBitType 		Tx_OCP_DEG			:2;
        uBitType 		Tx_OCP_MODE			:2;
        uBitType 		Tx_DEAD_TIME		:2;
        uBitType 		Tx_TRETRY			:1;

	}B;
}Tx_Msg0x05_t;
typedef union
{
	uint16 R;
	struct
	{
        uBitType 		Tx_SEN_LVL			:2;
        uBitType 		Tx_CSA_CAL_C		:1;
        uBitType 		Tx_CSA_CAL_B		:1;
        uBitType 		Tx_CSA_CAL_A		:1;
        uBitType 		Tx_DIS_SEN			:1;
        uBitType 		Tx_CSA_GAIN			:2;
        uBitType 		Tx_LS_REF			:1;
        uBitType 		Tx_VREF_DIV			:1;
        uBitType 		Tx_CSA_FET			:1;


	}B;
}Tx_Msg0x06_t;
extern void UsrMTR_Init(void);
extern void UsrMTR_func(void);



extern uint32 MC_frequency;
extern uint16	U_HI;
extern uint16   W_HI;
extern uint16   V_HI;

extern uint8 Dead_Time;


typedef struct
{
    uint8 Mtr_Init_Mtr_State;
    uint8 Mtr_Init_Vbat1_Check;
    uint8 Mtr_Init_Vbat2_Check;
    uint8 Mtr_Init_VDC_Check;
    uint8 Mtr_Valve_Worikg_Count;
    uint8 Mtr_Init_Mtr_Chk_Fault_Info;
    uint16 Mtr_Init_Mtr_State_Cnt; /* MAX value change 255ms -> 1st : 300ms, 2and3 : 450ms*/
    uint32 CalcedPosition;
    uint32 CalcedRePosition;
    uint32 Mtr_Init_U_Position_D1;
    uint32 Mtr_Init_V_Position_D1;
    uint32 Mtr_Init_W_Position_D1;
    uint32 Mtr_Init_U_Position_D2;
    uint32 Mtr_Init_V_Position_D2;
    uint32 Mtr_Init_W_Position_D2;
}Mtr_Init_Chk_Variables_t;


typedef struct
{
    uint8   Mtr_CWPosiOffsetFlg;
    uint8   Mtr_CCWPosiOffsetFlg;
    uint8	Mtr_Current_CompleteFlg;
    uint8	Mtr_CWPositionAVRCnt;		/* CW Offset Save Count*/
    uint8	Mtr_CCWPositionAVRCnt;		/* CCW Offset Save Count*/
	sint16 	Mtr_CWPosition_D1_1stVal;
	sint16 	Mtr_CWPosition_D2_1stVal;
	sint16 	Mtr_CCWPosition_D1_1stVal;
	sint16 	Mtr_CCWPosition_D2_1stVal;
    uint16  Mtr_CWPhaseOffset_D1;		/* CW Offset Save result  */
    uint16  Mtr_CWPhaseOffset_D2;
    sint16  Mtr_CWPhaseOffset_D1_temp;
    sint16  Mtr_CWPhaseOffset_D2_temp;
    sint16  Mtr_CWPhaseOffset_D1_1stVal;
    sint16  Mtr_CWPhaseOffset_D2_1stVal;
    sint32  Mtr_CWPositionAddValue_D1;	/* CW Offset whole value */
    sint32  Mtr_CWPositionAddValue_D2;
    uint16  Mtr_CCWPhaseOffset_D1; 		/* CCW Offset save result */
    uint16  Mtr_CCWPhaseOffset_D2;
    sint16  Mtr_CCWPhaseOffset_D1_temp; 		/* CCW Offset save result */
    sint16  Mtr_CCWPhaseOffset_D2_temp;
    sint16  Mtr_CCWPhaseOffset_D1_1stVal;
    sint16  Mtr_CCWPhaseOffset_D2_1stVal;
    sint32  Mtr_CCWPositionAddValue_D1;	/* CCW Offset whole value */
    sint32  Mtr_CCWPositionAddValue_D2;	/* CCW Offset whole value */
    sint16  Mtr_WholeCWOffsetPosition_D1;	/* CW Position value for 4time's Motor work */
    sint16  Mtr_WholeCCWOffsetPosition_D1;	/* CCW Position value for 4time's Motor work */
    sint16  Mtr_WholeCWOffsetPosition_D2;	/* CW Position value for 4time's Motor work */
    sint16  Mtr_WholeCCWOffsetPosition_D2;	/* CCW Position value for 4time's Motor work */
}Mtr_Processing_DaigPhase_Type_t;


typedef struct
{
    uint8 Mtr_Calibration_Init;
    uint8 Mtr_MPS_HOLD;
    uint8 Mtr_MPS_Direction_State;
    uint8 Mtr_CW_PositionState;
	uint8 Mtr_CW_Compare_Delay_cnt;
    uint8 Mtr_CCW_PositionState;
	uint8 Mtr_CCW_Compare_Delay_cnt;
    uint8 Mtr_MPS_CAL_Phase_State;
    uint16 Mtr_MPS_CAL_Wait_Count;
    uint16 Mtr_MPS_CAL_Motion_Count;
	uint8  Mtr_MPS_CAL_Direc_ChangeFlg;
    uint32 Mtr_MPS_Offset_U_D1;
    uint32 Mtr_MPS_Offset_V_D1;
    uint32 Mtr_MPS_Offset_W_D1;
    uint32 Mtr_MPS_Offset_U_D2;
    uint32 Mtr_MPS_Offset_V_D2;
    uint32 Mtr_MPS_Offset_W_D2;

    Mtr_Processing_DaigPhase_Type_t Mtr_Inter_U_Phase_Info;
    Mtr_Processing_DaigPhase_Type_t Mtr_Inter_V_Phase_Info;
    Mtr_Processing_DaigPhase_Type_t Mtr_Inter_W_Phase_Info;
}Mtr_MPS_Calibration_Variables_t;
typedef struct
{
    uint8 Mtr_Current_Waiting_Cnt;
    uint8 Mtr_Current_Getting_Cnt_1;
    uint8 Mtr_Current_Getting_Cnt_2;
    uint8 Mtr_Current_Offset_Cnt_1;
    uint8 Mtr_Current_Offset_Cnt_2;
    sint32 Mtr_Current_Phase_Pre_1;
    sint32 Mtr_Current_Phase_Pre_2;
    sint32 Mtr_Current_Phase_1;
    sint32 Mtr_Current_Phase_2;
    sint32 Mtr_Current_Phase_1_add;
    sint32 Mtr_Current_Phase_2_add;
    sint32 Mtr_Current_Phase_1_Offset[5];
    sint32 Mtr_Current_Phase_2_Offset[5];
}Mtr_Current_Calibration_Variables_t;

typedef struct
{
    uint16 Mtr_Current_Saturation_Cnt;
    uint16 Mtr_Current_woriking_Cnt;
    uint16 Mtr_Current_Div_Cnt;

}Mtr_Inverter_Test_Variables_t;

typedef struct
{
    uint8    MotorRptState;
    uint8    MotorMonData;
    uint8    MotorMonDirection;
    uint8    u8TestReady;
    sint8    s8TestActive ;
    sint8    s8TestMode;
    sint16  s16DeltaStrk;
    sint16  s16TestState;
    sint16  s16WrpmRef;
    sint16  s16Iq_Request;
    sint16  s16Id_Request;
    sint32  s32TgtPosi;
	uint8   MtrRptMonCheckFlg;
	uint8   Mtr_Sys_Fail;
}Mtr_ProcessingDiag_Ctr_Internal_t;


typedef struct
{
    uint8 Mtr_Status;
    uint8 Mtr_MotorCtrMode;
    uint8 Mtr_Init_State;
    uint8 Mtr_MPS_Calibration_State;
    uint8 Mtr_Current_Calibration_State;
    uint8 Mtr_Current_GainCalibration_State;

    uint8 Mtr_Inverter_Test_State;
    uint8 Mtr_DeCal_State;
    uint8 Mtr_NO_Working_Cnt;
    uint16 Mtr_PWM_U_Phase;
    uint16 Mtr_PWM_V_Phase;
    uint16 Mtr_PWM_W_Phase;

	uint8 Mtr_InverterTest_PhPWMRes;
	uint8 Mtr_Ctr_Process_RepeatRes;
	uint8 Mtr_Ctr_Process_0PRes;
	uint8 Mtr_Ctr_Process_StrokeRes;

    Mtr_Init_Chk_Variables_t Mtr_Init_Chk_Variables;
    Mtr_MPS_Calibration_Variables_t Mtr_MPS_Calibration_Variables;
    Mtr_Current_Calibration_Variables_t Mtr_Current_Calibration_Variables;
    Mtr_Inverter_Test_Variables_t Mtr_Inverter_Test_Varialbes;
    Mtr_ProcessingDiag_Ctr_Internal_t Mtr_ProcessingDiag_Ctr_Internal_Variables;
}Mtr_Processing_Internal_t;

typedef struct
{
    uint8 Mtr_MotorFail;
    uint8 Mtr_SysFail;
    uint8 Mtr_Power_Fail;
    uint8 Mtr_CurrentCalibrationFail;
}Mtr_Processing_Input_t;


#define DEF_MTR_INIT							0x00u
#define DEF_MTR_NORMAL							0x01u
#define DEF_MTR_INIT_ERROR  					0x02u
#define DEF_MTR_ERROR							0x03u
#define DEF_MTR_MPS_CALIBRATION  				0x04u
#define DEF_MTR_CURRENT_OFFSET_CALIBRATION    	0x05u
#define DEF_MTR_INVERTER_TEST    				0x06u
#define DEF_MTR_DIAG_MODE  						0x07u
#define DEF_MTR_DECAL_MODE 						0x08u
#define DEF_MTR_CURRENT_GAINCALIBRATION 		0x09u
#define DEF_MTR_STATE_HOLD 						0x0Au
#define DEF_MTR_CLEAR 							0x0Bu
#define DEF_MTR_REPEAT_CTR 						0x0Cu
#define DEF_MTR_TARGET_CTR 						0x0Du
#define DEF_MTR_HW_TEST 						0x0Eu

#define DEF_MTR_REQUEST_NONE		0x00u
#define DEF_MTR_REQUEST_ERR     	0x01u
#define DEF_MTR_REQUEST_PWM_INIT 	0x02u
#define DEF_MTR_REQUEST_DIAG_PWM    0x03u
#define DEF_MTR_REQUEST_DIAG_CTR    0x04u
#define DEF_MTR_REQUEST_CONTROL    	0x05u
#define DEF_MTR_REQUEST_FAILSAFE   	0x06u

/* Motor Init Current Saturation Status */
#define DEF_CURRENT_NOT_SATURATED 	0x00u
#define DEF_CURRENT_SATURATED 		0x01u
/*==============================================================================
 *                  GLOBAL MACROS AND DEFINES
 =============================================================================*/
#define MTR_PHASE_CURR_BAND_1 	((uint32)470) /* 47.0 % */
#define MTR_PHASE_CURR_BAND_2 	((uint32)530) /* 53.0 % */
#define MTR_PHASE_CURR_BAND     500U /* 50.0 % */
#define MTR_PHASE_CURR_GAIN		1000

#define MTR_PHASE_10A   100
#define MTR_PHASE_20A   200
#define MTR_PHASE_30A   300
#define MTR_PHASE_40A   400
#define MTR_PHASE_50A   500

#define MTR_INIT    1U
#define MTR_INVERTER_TEST   2U
/**************** 3 phase Motor Macro ****************/
/* Motor Status in Mtr_Processing; Mtr_Processing_DiagMotState */
#define DEF_MTR_INIT		0x00u
#define DEF_MTR_NORMAL		0x01u
#define DEF_MTR_INIT_ERROR  0x02u
#define DEF_MTR_ERROR		0x03u
#define DEF_MTR_MPS_CALIBRATION  0x04u
#define DEF_MTR_CURRENT_OFFSET_CALIBRATION    0x05u
#define DEF_MTR_INVERTER_TEST    0x06u
#define DEF_MTR_DIAG_MODE  	0x07u
#define DEF_MTR_DECAL_MODE 	0x08u
#define DEF_MTR_CURRENT_GAINCALIBRATION 0x09u
#define DEF_MTR_STATE_HOLD 	0x0Au
#define DEF_MTR_CLEAR 		0x0Bu
#define DEF_MTR_REPEAT_CTR 	0x0Cu
#define DEF_MTR_TARGET_CTR 	0x0Du
#define DEF_MTR_HW_TEST 	0x0Eu

/* Voltage */
#define    U16_CALCVOLT_0V1             (100)
#define    U16_CALCVOLT_0V12            (120)
#define    U16_CALCVOLT_0V187           (187)
#define    U16_CALCVOLT_0V2             (200)
#define    U16_CALCVOLT_0V25            (250)
#define    U16_CALCVOLT_0V3             (300)
#define    U16_CALCVOLT_0V35            (350)
#define    U16_CALCVOLT_0V375           (375)
#define    U16_CALCVOLT_0V4             (400)
#define    U16_CALCVOLT_0V5             (500)
#define    U16_CALCVOLT_0V6             (600)
#define    U16_CALCVOLT_0V7             (700)
#define    U16_CALCVOLT_0V8             (800)
#define    U16_CALCVOLT_0V9             (900)
#define    U16_CALCVOLT_1V0             (1000)
#define    U16_CALCVOLT_1V2             (1200)
#define    U16_CALCVOLT_1V25            (1250)
#define    U16_CALCVOLT_1V4             (1400)
#define    U16_CALCVOLT_1V5             (1500)
#define    U16_CALCVOLT_1V75            (1750)
#define    U16_CALCVOLT_1V9             (1900)
#define    U16_CALCVOLT_2V0             (2000)
#define    U16_CALCVOLT_2V1             (2100)
#define    U16_CALCVOLT_2V25            (2250)
#define    U16_CALCVOLT_2V36            (2360)
#define    U16_CALCVOLT_2V5             (2500)
#define    U16_CALCVOLT_2V64            (2640)
#define    U16_CALCVOLT_2V75            (2750)
#define    U16_CALCVOLT_2V9             (2900)
#define    U16_CALCVOLT_2V97            (2970)
#define    U16_CALCVOLT_3V0             (3000)
#define    U16_CALCVOLT_3V1             (3100)
#define    U16_CALCVOLT_3V25            (3250)
#define    U16_CALCVOLT_3V5             (3500)
#define    U16_CALCVOLT_3V63            (3630)
#define    U16_CALCVOLT_3V75            (3750)
#define	   U16_CALCVOLT_3V9 			(3900)
#define    U16_CALCVOLT_4V0             (4000)
#define    U16_CALCVOLT_4V2             (4200)
#define    U16_CALCVOLT_4V25            (4250)
#define    U16_CALCVOLT_4V5             (4500)
#define    U16_CALCVOLT_4V7             (4700)
#define    U16_CALCVOLT_4V75            (4750)
#define    U16_CALCVOLT_4V8             (4800)
#define    U16_CALCVOLT_4V85            (4850)
#define    U16_CALCVOLT_5V0             (5000)
#define    U16_CALCVOLT_5V25            (5250)
#define    U16_CALCVOLT_5V5             (5500)
#define    U16_CALCVOLT_5V75            (5750)
#define    U16_CALCVOLT_5V9             (5900)
#define    U16_CALCVOLT_6V0             (6000)
#define    U16_CALCVOLT_6V25            (6250)
#define    U16_CALCVOLT_6V4             (6400)
#define    U16_CALCVOLT_6V5             (6500)
#define    U16_CALCVOLT_6V75            (6750)
#define    U16_CALCVOLT_7V0             (7000)
#define    U16_CALCVOLT_7V2             (7200)
#define    U16_CALCVOLT_7V25            (7250)
#define    U16_CALCVOLT_7V5             (7500)
#define    U16_CALCVOLT_7V75            (7750)
#define    U16_CALCVOLT_7V8             (7800)
#define    U16_CALCVOLT_8V0             (8000)
#define    U16_CALCVOLT_8V2             (8200)
#define    U16_CALCVOLT_8V4             (8400)
#define    U16_CALCVOLT_8V25            (8250)
#define    U16_CALCVOLT_8V5             (8500)
#define    U16_CALCVOLT_8V6             (8600)
#define    U16_CALCVOLT_8V75            (8750)
#define    U16_CALCVOLT_8V8             (8800)
#define    U16_CALCVOLT_9V0             (9000)
#define    U16_CALCVOLT_9V2             (9200)
#define    U16_CALCVOLT_9V25            (9250)
#define    U16_CALCVOLT_9V4             (9400)
#define    U16_CALCVOLT_9V5             (9500)
#define    U16_CALCVOLT_9V6             (9600)
#define    U16_CALCVOLT_9V75            (9750)
#define    U16_CALCVOLT_10V0            (10000)
#define    U16_CALCVOLT_10V25           (10250)
#define    U16_CALCVOLT_10V5            (10500)
#define    U16_CALCVOLT_10V75           (10750)
#define    U16_CALCVOLT_11V0            (11000)
#define    U16_CALCVOLT_11V25           (11250)
#define    U16_CALCVOLT_11V5            (11500)
#define    U16_CALCVOLT_11V75           (11750)
#define    U16_CALCVOLT_12V0            (12000)
#define    U16_CALCVOLT_12V25           (12250)
#define    U16_CALCVOLT_12V5            (12500)
#define    U16_CALCVOLT_12V75           (12750)
#define    U16_CALCVOLT_13V0            (13000)
#define    U16_CALCVOLT_13V25           (13250)
#define    U16_CALCVOLT_13V5            (13500)
#define    U16_CALCVOLT_13V75           (13750)
#define    U16_CALCVOLT_14V0            (14000)
#define    U16_CALCVOLT_14V25           (14250)
#define    U16_CALCVOLT_14V5            (14500)
#define    U16_CALCVOLT_14V75           (14750)
#define    U16_CALCVOLT_15V0            (15000)
#define    U16_CALCVOLT_15V25           (15250)
#define    U16_CALCVOLT_15V5            (15500)
#define    U16_CALCVOLT_15V75           (15750)
#define    U16_CALCVOLT_16V0            (16000)
#define    U16_CALCVOLT_16V25           (16250)
#define    U16_CALCVOLT_16V5            (16500)
#define    U16_CALCVOLT_16V7            (16700)
#define    U16_CALCVOLT_16V75           (16750)
#define    U16_CALCVOLT_16V8            (16800)
#define    U16_CALCVOLT_17V0            (17000)
#define    U16_CALCVOLT_17V25           (17250)
#define    U16_CALCVOLT_17V5            (17500)
#define    U16_CALCVOLT_17V75           (17750)
#define    U16_CALCVOLT_18V0            (18000)
#define    U16_CALCVOLT_18V25           (18250)
#define    U16_CALCVOLT_18V5            (18500)
#define    U16_CALCVOLT_18V75           (18750)
#define    U16_CALCVOLT_19V0            (19000)
#define    U16_CALCVOLT_19V25           (19250)
#define    U16_CALCVOLT_19V5            (19500)
#define    U16_CALCVOLT_19V75           (19750)
#define    U16_CALCVOLT_20V0            (20000)
#define    U16_CALCVOLT_50V0            (50000)

#define VOLTAGE_NORMAL 		0x01u
#define VOLTAGE_ABNORMAL 	0x02u

#define MTR_PROC_IN_V_BATT1		Mtr_Processing_DiagBus.Mtr_Processing_DiagVBatt1Mon
#define MTR_PROC_IN_V_BATT2		Mtr_Processing_DiagBus.Mtr_Processing_DiagVBatt2Mon
#define MTR_PROC_IN_V_MTRPWR	Mtr_Processing_DiagBus.Mtr_Processing_DiagVdcLink

#define MTR_PROC_IN_U_CURRENT	Mtr_Processing_DiagBus.Mtr_Processing_DiagAcmioSenInfo.MotCurrPhUMeasd
#define MTR_PROC_IN_V_CURRENT	Mtr_Processing_DiagBus.Mtr_Processing_DiagAcmioSenInfo.MotCurrPhVMeasd
#define MTR_PROC_IN_W_CURRENT	Mtr_Processing_DiagBus.Mtr_Processing_DiagAcmioSenInfo.MotCurrPhWMeasd

#define DEF_NON_PHASE  		0x00u
#define DEF_U_PHASE     	0x01u
#define DEF_V_PHASE     	0x02u
#define DEF_W_PHASE    		0x03u


typedef uint32 TimerVal;        /**< \brief Used in timer values */

/** Tom channels */
typedef enum
{
    IfxGtmTom_Ch_0  = 0,
    IfxGtmTom_Ch_1  = 1,
    IfxGtmTom_Ch_2  = 2,
    IfxGtmTom_Ch_3  = 3,
    IfxGtmTom_Ch_4  = 4,
    IfxGtmTom_Ch_5  = 5,
    IfxGtmTom_Ch_6  = 6,
    IfxGtmTom_Ch_7  = 7,
    IfxGtmTom_Ch_8  = 8,
    IfxGtmTom_Ch_9  = 9,
    IfxGtmTom_Ch_10 = 10,
    IfxGtmTom_Ch_11 = 11,
    IfxGtmTom_Ch_12 = 12,
    IfxGtmTom_Ch_13 = 13,
    IfxGtmTom_Ch_14 = 14,
    IfxGtmTom_Ch_15 = 15
} IfxGtmTom_Ch;



typedef struct IfxGtmTom_PwmHl_s IfxGtmTom_PwmHl;

/** PWM configuration
 *
 */
typedef struct
{
    boolean inverted;                                               /**< \brief Inverted configuration for the selected mode */
    void (*update)(IfxGtmTom_PwmHl *driver, TimerVal *tOn);         /**< \brief update call back function for the selected mode */
} IfxGtmTom_PwmHl_Mode;
;



