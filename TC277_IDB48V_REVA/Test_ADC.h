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

#ifndef _TEST_ADC_H_
#define _TEST_ADC_H_
void ADC_Conversion(void);
//****************************************************************************
// @Macros
//****************************************************************************

#define ADC0_GetResultData(RegNum)  		(ADC0_GetResultData_RESULT_REG_##RegNum)
#define ADC1_GetResultData(RegNum)  		(ADC1_GetResultData_RESULT_REG_##RegNum)
#define ADC2_GetResultData(RegNum)  		(ADC2_GetResultData_RESULT_REG_##RegNum)
#define ADC3_GetResultData(RegNum)  		(ADC3_GetResultData_RESULT_REG_##RegNum)
#define ADC4_GetResultData(RegNum)  		(ADC4_GetResultData_RESULT_REG_##RegNum)
#define ADC5_GetResultData(RegNum)  		(ADC5_GetResultData_RESULT_REG_##RegNum)
#define ADC6_GetResultData(RegNum)  		(ADC6_GetResultData_RESULT_REG_##RegNum)
#define ADC7_GetResultData(RegNum)  		(ADC7_GetResultData_RESULT_REG_##RegNum)


#define ADC0_GetResultData_RESULT_REG_0 	((VADC_G0_RESD0.U & 0x80000000) ? (VADC_G0_RES0.U) : 0)
#define ADC0_GetResultData_RESULT_REG_1 	((VADC_G0_RESD1.U & 0x80000000) ? (VADC_G0_RES1.U) : 0)
#define ADC0_GetResultData_RESULT_REG_2 	((VADC_G0_RESD2.U & 0x80000000) ? (VADC_G0_RES2.U) : 0)
#define ADC0_GetResultData_RESULT_REG_3 	((VADC_G0_RESD3.U & 0x80000000) ? (VADC_G0_RES3.U) : 0)
#define ADC0_GetResultData_RESULT_REG_4 	((VADC_G0_RESD4.U & 0x80000000) ? (VADC_G0_RES4.U) : 0)
#define ADC0_GetResultData_RESULT_REG_5 	((VADC_G0_RESD5.U & 0x80000000) ? (VADC_G0_RES5.U) : 0)
#define ADC0_GetResultData_RESULT_REG_6 	((VADC_G0_RESD6.U & 0x80000000) ? (VADC_G0_RES6.U) : 0)
#define ADC0_GetResultData_RESULT_REG_7 	((VADC_G0_RESD7.U & 0x80000000) ? (VADC_G0_RES7.U) : 0)

#define ADC1_GetResultData_RESULT_REG_0 	((VADC_G1_RESD0.U & 0x80000000) ? (VADC_G1_RES0.U) : 0)
#define ADC1_GetResultData_RESULT_REG_1 	((VADC_G1_RESD1.U & 0x80000000) ? (VADC_G1_RES1.U) : 0)
#define ADC1_GetResultData_RESULT_REG_2 	((VADC_G1_RESD2.U & 0x80000000) ? (VADC_G1_RES2.U) : 0)
#define ADC1_GetResultData_RESULT_REG_3 	((VADC_G1_RESD3.U & 0x80000000) ? (VADC_G1_RES3.U) : 0)
#define ADC1_GetResultData_RESULT_REG_4 	((VADC_G1_RESD4.U & 0x80000000) ? (VADC_G1_RES4.U) : 0)
#define ADC1_GetResultData_RESULT_REG_5 	((VADC_G1_RESD5.U & 0x80000000) ? (VADC_G1_RES5.U) : 0)
#define ADC1_GetResultData_RESULT_REG_6 	((VADC_G1_RESD6.U & 0x80000000) ? (VADC_G1_RES6.U) : 0)
#define ADC1_GetResultData_RESULT_REG_7 	((VADC_G1_RESD7.U & 0x80000000) ? (VADC_G1_RES7.U) : 0)

#define ADC2_GetResultData_RESULT_REG_0 	((VADC_G2_RESD0.U & 0x80000000) ? (VADC_G2_RES0.U) : 0)
#define ADC2_GetResultData_RESULT_REG_1 	((VADC_G2_RESD1.U & 0x80000000) ? (VADC_G2_RES1.U) : 0)
#define ADC2_GetResultData_RESULT_REG_2 	((VADC_G2_RESD2.U & 0x80000000) ? (VADC_G2_RES2.U) : 0)
#define ADC2_GetResultData_RESULT_REG_3 	((VADC_G2_RESD3.U & 0x80000000) ? (VADC_G2_RES3.U) : 0)
#define ADC2_GetResultData_RESULT_REG_4 	((VADC_G2_RESD4.U & 0x80000000) ? (VADC_G2_RES4.U) : 0)
#define ADC2_GetResultData_RESULT_REG_5 	((VADC_G2_RESD5.U & 0x80000000) ? (VADC_G2_RES5.U) : 0)
#define ADC2_GetResultData_RESULT_REG_6 	((VADC_G2_RESD6.U & 0x80000000) ? (VADC_G2_RES6.U) : 0)
#define ADC2_GetResultData_RESULT_REG_7 	((VADC_G2_RESD7.U & 0x80000000) ? (VADC_G2_RES7.U) : 0)

#define ADC3_GetResultData_RESULT_REG_0 	((VADC_G3_RESD0.U & 0x80000000) ? (VADC_G3_RES0.U) : 0)
#define ADC3_GetResultData_RESULT_REG_1 	((VADC_G3_RESD1.U & 0x80000000) ? (VADC_G3_RES1.U) : 0)
#define ADC3_GetResultData_RESULT_REG_2 	((VADC_G3_RESD2.U & 0x80000000) ? (VADC_G3_RES2.U) : 0)
#define ADC3_GetResultData_RESULT_REG_3 	((VADC_G3_RESD3.U & 0x80000000) ? (VADC_G3_RES3.U) : 0)
#define ADC3_GetResultData_RESULT_REG_4 	((VADC_G3_RESD4.U & 0x80000000) ? (VADC_G3_RES4.U) : 0)
#define ADC3_GetResultData_RESULT_REG_5 	((VADC_G3_RESD5.U & 0x80000000) ? (VADC_G3_RES5.U) : 0)
#define ADC3_GetResultData_RESULT_REG_6 	((VADC_G3_RESD6.U & 0x80000000) ? (VADC_G3_RES6.U) : 0)
#define ADC3_GetResultData_RESULT_REG_7 	((VADC_G3_RESD7.U & 0x80000000) ? (VADC_G3_RES7.U) : 0)

#define ADC4_GetResultData_RESULT_REG_0 	((VADC_G4_RESD0.U & 0x80000000) ? (VADC_G4_RES0.U) : 0)
#define ADC4_GetResultData_RESULT_REG_1 	((VADC_G4_RESD1.U & 0x80000000) ? (VADC_G4_RES1.U) : 0)
#define ADC4_GetResultData_RESULT_REG_2 	((VADC_G4_RESD2.U & 0x80000000) ? (VADC_G4_RES2.U) : 0)
#define ADC4_GetResultData_RESULT_REG_3 	((VADC_G4_RESD3.U & 0x80000000) ? (VADC_G4_RES3.U) : 0)
#define ADC4_GetResultData_RESULT_REG_4 	((VADC_G4_RESD4.U & 0x80000000) ? (VADC_G4_RES4.U) : 0)
#define ADC4_GetResultData_RESULT_REG_5 	((VADC_G4_RESD5.U & 0x80000000) ? (VADC_G4_RES5.U) : 0)
#define ADC4_GetResultData_RESULT_REG_6 	((VADC_G4_RESD6.U & 0x80000000) ? (VADC_G4_RES6.U) : 0)
#define ADC4_GetResultData_RESULT_REG_7 	((VADC_G4_RESD7.U & 0x80000000) ? (VADC_G4_RES7.U) : 0)

#define ADC5_GetResultData_RESULT_REG_0 	((VADC_G5_RESD0.U & 0x80000000) ? (VADC_G5_RES0.U) : 0)
#define ADC5_GetResultData_RESULT_REG_1 	((VADC_G5_RESD1.U & 0x80000000) ? (VADC_G5_RES1.U) : 0)
#define ADC5_GetResultData_RESULT_REG_2 	((VADC_G5_RESD2.U & 0x80000000) ? (VADC_G5_RES2.U) : 0)
#define ADC5_GetResultData_RESULT_REG_3 	((VADC_G5_RESD3.U & 0x80000000) ? (VADC_G5_RES3.U) : 0)
#define ADC5_GetResultData_RESULT_REG_4 	((VADC_G5_RESD4.U & 0x80000000) ? (VADC_G5_RES4.U) : 0)
#define ADC5_GetResultData_RESULT_REG_5 	((VADC_G5_RESD5.U & 0x80000000) ? (VADC_G5_RES5.U) : 0)
#define ADC5_GetResultData_RESULT_REG_6 	((VADC_G5_RESD6.U & 0x80000000) ? (VADC_G5_RES6.U) : 0)
#define ADC5_GetResultData_RESULT_REG_7 	((VADC_G5_RESD7.U & 0x80000000) ? (VADC_G5_RES7.U) : 0)

#define ADC6_GetResultData_RESULT_REG_0 	((VADC_G6_RESD0.U & 0x80000000) ? (VADC_G6_RES0.U) : 0)
#define ADC6_GetResultData_RESULT_REG_1 	((VADC_G6_RESD1.U & 0x80000000) ? (VADC_G6_RES1.U) : 0)
#define ADC6_GetResultData_RESULT_REG_2 	((VADC_G6_RESD2.U & 0x80000000) ? (VADC_G6_RES2.U) : 0)
#define ADC6_GetResultData_RESULT_REG_3 	((VADC_G6_RESD3.U & 0x80000000) ? (VADC_G6_RES3.U) : 0)
#define ADC6_GetResultData_RESULT_REG_4 	((VADC_G6_RESD4.U & 0x80000000) ? (VADC_G6_RES4.U) : 0)
#define ADC6_GetResultData_RESULT_REG_5 	((VADC_G6_RESD5.U & 0x80000000) ? (VADC_G6_RES5.U) : 0)
#define ADC6_GetResultData_RESULT_REG_6 	((VADC_G6_RESD6.U & 0x80000000) ? (VADC_G6_RES6.U) : 0)
#define ADC6_GetResultData_RESULT_REG_7 	((VADC_G6_RESD7.U & 0x80000000) ? (VADC_G6_RES7.U) : 0)

#define ADC7_GetResultData_RESULT_REG_0 	((VADC_G7_RESD0.U & 0x80000000) ? (VADC_G7_RES0.U) : 0)
#define ADC7_GetResultData_RESULT_REG_1 	((VADC_G7_RESD1.U & 0x80000000) ? (VADC_G7_RES1.U) : 0)
#define ADC7_GetResultData_RESULT_REG_2 	((VADC_G7_RESD2.U & 0x80000000) ? (VADC_G7_RES2.U) : 0)
#define ADC7_GetResultData_RESULT_REG_3 	((VADC_G7_RESD3.U & 0x80000000) ? (VADC_G7_RES3.U) : 0)
#define ADC7_GetResultData_RESULT_REG_4 	((VADC_G7_RESD4.U & 0x80000000) ? (VADC_G7_RES4.U) : 0)
#define ADC7_GetResultData_RESULT_REG_5 	((VADC_G7_RESD5.U & 0x80000000) ? (VADC_G7_RES5.U) : 0)
#define ADC7_GetResultData_RESULT_REG_6 	((VADC_G7_RESD6.U & 0x80000000) ? (VADC_G7_RES6.U) : 0)
#define ADC7_GetResultData_RESULT_REG_7 	((VADC_G7_RESD7.U & 0x80000000) ? (VADC_G7_RES7.U) : 0)

//****************************************************************************
// @Typedefs
//****************************************************************************

//****************************************************************************
// @Prototypes Of Global Functions
//****************************************************************************

#endif  // ifndef _TEST_ADC_H_


