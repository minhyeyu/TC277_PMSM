/**
 ******************************************************************************
 * @file    6Step_Lib.h
 * @author  System lab
 * @version V1.0.0
 * @date    06-July-2015
 * @brief   This header file provides the set of functions for Motor Control 
            library 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __6STEP_LIB_H
#define __6STEP_LIB_H

//#include "stm32_nucleo_ihm07m1.h"

#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "Platform_Types.h"

  /* @brief  Six Step parameters
  */
typedef enum 
{
    IDLE,                               /* 0 */
    STARTUP,                            /* 1 */
    VALIDATION,                         /* 2 */
    STOP,                               /* 3 */
    START,                              /* 4 */
    RUN,                                /* 5 */
    ALIGNMENT,                          /* 6 */
    SPEEDFBKERROR,                      /* 7 */
    OVERCURRENT,                        /* 8 */
    STARTUP_FAILURE,                    /* 9 */  
    STARTUP_BEMF_FAILURE                /* 10 */        
} SIXSTEP_Base_SystStatus_t;


/** @defgroup Exported_types  Exported_types
* @{
*/
/** 
  * @brief  Six Step parameters
  */


typedef struct
{
  uint32 LF_TIMx_PSC;                  /*!< Prescaler variable for low frequency timer*/
  uint32 LF_TIMx_ARR;                  /*!< ARR variable for low frequency timer*/
  uint32 HF_TIMx_PSC;                  /*!< Prescaler variable for high frequency timer*/
  uint32 HF_TIMx_ARR;                  /*!< ARR variable for high frequency timer*/
  uint32 HF_TIMx_CCR;                  /*!< CCR variable for high frequency timer*/
  uint8 step_position;                 /*!< Step number variable for SixStep algorithm*/
  SIXSTEP_Base_SystStatus_t STATUS;      /*!< Status variable for SixStep algorithm*/
  uint8  status_prev;                  /*!< Previous status variable for SixStep algorithm*/
  uint16 pulse_value;                  /*!< CCR value for SixStep algorithm*/
  uint16 ARR_value;                    /*!< ARR vector for Accell compute*/
  uint32 Regular_channel[4];           /*!< Buffer for ADC regular channel */
  uint32 CurrentRegular_BEMF_ch;       /*!< ADC regular channel to select */
  uint32 prescaler_value;              /*!< Prescaler value for low freq timer*/
  uint16 numberofitemArr;              /*!< Number of elements */
  uint32 ADC_BUFFER[4];                /*!< Buffer for ADC regular channel */
  uint32 ADC_SEQ_CHANNEL[4];           /*!< Buffer for ADC regular channel */
  uint32 ADC_Regular_Buffer[5];        /*!< Buffer for ADC regular channel */
  uint16 ADC_BEMF_threshold_UP;        /*!< Voltage threshold for BEMF detection in up direction*/
  uint16 ADC_BEMF_threshold_DOWN;      /*!< Voltage threshold for BEMF detection in down direction*/
  uint16 demagn_counter;               /*!< Demagnetization counter*/
  uint16 demagn_value;                 /*!< Demagnetization value*/
  sint16 speed_fdbk;                    /*!< Motor speed variable*/
  sint16 speed_fdbk_filtered;           /*!< Filtered Motor speed variable*/
  sint16 filter_depth;                  /*!< Filter depth for speed measuring*/
  uint16 Current_Reference;            /*!< Currrent reference for SixStep algorithm*/
  uint16 Ireference;                   /*!< Currrent reference for SixStep algorithm*/
  sint32 Integral_Term_sum;             /*!< Global Integral part for PI*/
  uint8 CMD;                           /*!< Flag control for Motor Start/Stop*/
  uint8 ALIGN_OK;                      /*!< Flag control for Motor Alignment*/
  uint8 ALIGNMENT;                     /*!< Flag control for Motor Alignment ongoing*/
  uint8 bemf_state_1;                  /*!< Bemf variable */
  uint8 bemf_state_2;                  /*!< Bemf variable */
  uint8 bemf_state_3;                  /*!< Bemf variable */
  uint8 bemf_state_4;                  /*!< Bemf variable */
  uint8 bemf_state_5;                  /*!< Bemf variable */
  uint8 bemf_state_6;                  /*!< Bemf variable */
  uint16 Speed_Loop_Time;              /*!< Speed loop variable for timing */
  uint16 Speed_Ref_filtered;           /*!< Filtered Reference Motor Speed variable */
  uint16 RUN_Motor;                    /*!< Flag for Motor status */
  uint8 ARR_OK;                        /*!< ARR flag control for Accell status */
  uint8 VALIDATION_OK;                 /*!< Validation flag for Closed loop control begin */
  uint8 SPEED_VALIDATED;               /*!< Validation flag for Speed before closed loop control */
  uint16 Speed_target_ramp;            /*!< Target Motor Speed */
  uint16 Speed_target_time;            /*!< Target Motor Ramp time */
  uint16 Ramp_Start;                   /*!< Ramp time start*/
  uint16 Bemf_delay_start;             /*!< Bemf variable */
  uint16 MediumFrequencyTask_flag;     /*!< Flag for Medium Task Frequency */
  uint32 SYSCLK_frequency;             /*!< System clock main frequency */
  uint32 Uart_cmd_to_set;              /*!<  */
  uint32 Uart_value_to_set;            /*!<  */
  uint8 Button_ready;                  /*!<  */
  uint8 BEMF_OK;                       /*!<  */
  uint8 CL_READY;                      /*!<  */
  uint8 BEMF_Tdown_count;              /*!< BEMF Consecutive Threshold Falling Crossings Counter */
  uint16 IREFERENCE;                   /*!< Currrent reference*/
  uint16 NUMPOLESPAIRS;                /*!< Number of motor pole pairs  */
  uint32 ACCEL;                        /*!< Acceleration start-up parameter*/
  uint16 KP;                           /*!< KP parameter for PI regulator */
  uint16 KI;                           /*!< KI parameter for PI regulator */
  uint8 CW_CCW;                        /*!< Set the motor direction */
  uint8 Potentiometer;                 /*!< Enable/Disable potentiometer for speed control */
}  SIXSTEP_Base_InitTypeDef;             /*!< Six Step Data Structure */

typedef struct
{
  sint16 Reference;                    /*!< Refence value for PI regulator */
  sint16 Kp_Gain;                      /*!< Kp value for PI regulator */
  sint16 Ki_Gain;                      /*!< Ki value for PI regulator */
  sint16 Lower_Limit_Output;           /*!< Min output value for PI regulator */
  sint16 Upper_Limit_Output;           /*!< Max output value for PI regulator */
  sint8 Max_PID_Output;                /*!< Max Saturation indicator flag */
  sint8 Min_PID_Output;                /*!< Min Saturation indicator flag */
} SIXSTEP_PI_PARAM_InitTypeDef_t, *SIXSTEP_pi_PARAM_InitTypeDef_t;  /*!< PI Data Structure */



/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01
} HAL_LockTypeDef;
/** @defgroup Exported_function_6StepLib  Exported_function_6StepLib
* @{
*/

void MC_SixStep_INIT(void);
void MC_SixStep_RESET(void);
void MC_StartMotor(void);
void MC_StopMotor(void);
void MC_Set_Speed(uint16);
void MC_EXT_button_SixStep(void);

  #define HF_TIMx               htim1
  #define LF_TIMx               htim6
  #define HALL_ENCODER_TIMx     htim2
  #define ADCx                  hadc1
  #define REFx                  htim16
  #define UART                  huart2

  #define GPIO_PORT_1           GPIOC
  #define GPIO_CH1              GPIO_PIN_10
  #define GPIO_PORT_2           GPIOC
  #define GPIO_CH2              GPIO_PIN_11
  #define GPIO_PORT_3           GPIOC
  #define GPIO_CH3              GPIO_PIN_12
  #define GPIO_SET              GPIO_PIN_SET
  #define GPIO_RESET            GPIO_PIN_RESET

  #define ADC_CH_1              ADC_CHANNEL_7    /*CURRENT*/
  #define ADC_CH_2              ADC_CHANNEL_12   /*SPEED*/
  #define ADC_CH_3              ADC_CHANNEL_2    /*VBUS*/
  #define ADC_CH_4              ADC_CHANNEL_8    /*TEMP*/
  #define ADC_Bemf_CH1          ADC_CHANNEL_9    /*BEMF1*/
  #define ADC_Bemf_CH2          ADC_CHANNEL_11   /*BEMF2*/
  #define ADC_Bemf_CH3          ADC_CHANNEL_15   /*BEMF3*/

  #define ADC_CH_1_ST           ADC_SAMPLETIME_1CYCLE_5    /*CURRENT sampling time */
  #define ADC_CH_2_ST           ADC_SAMPLETIME_181CYCLES_5 /*SPEED sampling time*/
  #define ADC_CH_3_ST           ADC_SAMPLETIME_181CYCLES_5 /*VBUS sampling time*/
  #define ADC_CH_4_ST           ADC_SAMPLETIME_181CYCLES_5 /*TEMP sampling time*/
  #define ADC_Bemf_CH1_ST       ADC_SAMPLETIME_61CYCLES_5  /*BEMF1 sampling time*/
  #define ADC_Bemf_CH2_ST       ADC_SAMPLETIME_61CYCLES_5  /*BEMF2 sampling time*/
  #define ADC_Bemf_CH3_ST       ADC_SAMPLETIME_61CYCLES_5  /*BEMF3 sampling time*/

#define DAC_ENABLE 1

/* Exported types ------------------------------------------------------------*/




 
/**  MIDDLEWARES
  * @} 
  */

#endif
