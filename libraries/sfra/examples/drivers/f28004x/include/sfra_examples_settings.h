//###########################################################################
//
// FILE:   sfra_examples_settings.h
//
// AUTHOR: Manish Bhardwaj (C2000 Systems Solutions, Houston , TX)
//
//#############################################################################
// $TI Release: C2000 Software Frequency Response Analyzer Library v1.40.00.00 $
// $Release Date: Tue Sep 21 16:33:07 CDT 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef _SFRA_EXAMPLES_SETTINGS_H
#define _SFRA_EXAMPLES_SETTINGS_H

#ifdef __cplusplus

extern "C" {
#endif

//
//defines
//

#define CPU_SYS_CLOCK      (100*1000000)
#define PWMSYSCLOCK_FREQ   (100*1000000)
#define ECAPSYSCLOCK_FREQ   (100*1000000)

//
// Project Options
//

//
// 1 means control runs on C28x , otherwise it will run on CLA
//
#define C28x_CORE 1
#define CLA_CORE 2
#define CONTROL_RUNNING_ON 1

//
// Power Stage Related Values
//
#define PFC_PWM_SWITCHING_FREQUENCY ((float)100*1000)
#define PFC_PWM_PERIOD ((PWMSYSCLOCK_FREQ) / (PFC_PWM_SWITCHING_FREQUENCY))

//
// Control Loop Design
//
#define CONTROL_ISR_FREQUENCY ((PFC_PWM_SWITCHING_FREQUENCY) / (CNTRL_ISR_FREQ_RATIO))
#define CNTRL_ISR_FREQ_RATIO    1
#define VOLTAGE_LOOP_RUN_RATIO  1

//
//SFRA Options
//
#define SFRA_ISR_FREQ       CONTROL_ISR_FREQUENCY
#define SFRA_FREQ_START 2
//
//SFRA step Multiply = 10^(1/No of steps per decade(40))
//
#define SFRA_FREQ_STEP_MULTIPLY (float)1.105
#define SFRA_AMPLITUDE (float)0.005
#define SFRA_FREQ_LENGTH 100
#define SCI_VBUS_CLK 50000000
#define SFRA_GUI_SCI_BAUDRATE 57600


#define PI_VALUE 3.141592653589

#define GI_PI_KP    (float) 0.3496503407
#define GI_PI_KI    (float) 0.0020000510

#define C28x_CONTROLISR_INTERRUPT_PIE_GROUP_NO INTERRUPT_ACK_GROUP3

#define C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE EPWM1_BASE
#define C28x_CONTROLISR_INTERRUPT INT_EPWM1

#define GI_PI_MAX   1.0
#define GI_PI_MIN   -1.0

//
// PWM pin, ADC, SDFM, Relay Selection related variables
//

#define PWM_BASE                   EPWM1_BASE

#define PWM_H_GPIO                 0
#define PWM_H_GPIO_PIN_CONFIG      GPIO_0_EPWM1A

#define PWM_L_GPIO                 1
#define PWM_L_GPIO_PIN_CONFIG      GPIO_1_EPWM1B

#define GPIO_PROFILING1 16
#define GPIO_PROFILING1_SET GPIO_GPASET_GPIO16
#define GPIO_PROFILING1_CLEAR GPIO_GPACLEAR_GPIO16
#define GPIO_PROFILING1_PIN_CONFIG GPIO_16_GPIO16

#define GPIO_PROFILING2 17
#define GPIO_PROFILING2_SET GPIO_GPASET_GPIO17
#define GPIO_PROFILING2_CLEAR GPIO_GPACLEAR_GPIO17
#define GPIO_PROFILING2_PIN_CONFIG GPIO_17_GPIO17

#define SFRA_GUI_SCI_BASE SCIA_BASE
#define SFRA_GUI_VBUS_CLK 50000000
#define SFRA_GUI_SCI_BAUDRATE 57600

#define SFRA_GUI_SCIRX_GPIO 28
#define SFRA_GUI_SCITX_GPIO 29

#define SFRA_GUI_SCIRX_GPIO_PIN_CONFIG GPIO_28_SCIRXDA
#define SFRA_GUI_SCITX_GPIO_PIN_CONFIG GPIO_29_SCITXDA

//
// if the following #define is set to 1 SFRA GUI indicates status on an LED
// otherwise LED code is ignored
//
#define SFRA_GUI_LED_INDICATOR 1
#define SFRA_GUI_LED_GPIO 31
#define SFRA_GUI_LED_GPIO_PIN_CONFIG GPIO_31_GPIO31

/* USER CODE END (section: User_Section) */


#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif
