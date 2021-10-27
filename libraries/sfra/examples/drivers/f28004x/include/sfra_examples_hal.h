//###########################################################################
//
// FILE:   sfra_examples_hal.h
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

#ifndef SFRA_EXAMPLES_HAL_H
#define SFRA_EXAMPLES_HAL_H

#ifdef __cplusplus

extern "C" {
#endif


//
// the includes
//
#ifndef __TMS320C28XX_CLA__
#include "inc/hw_types.h"
#include "driverlib.h"
#include "device.h"
#include "sfra_examples_settings.h"
#endif
//
//defines
//

#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define PWM_TRIP_STATUS EPWM_getTripZoneFlagStatus

#define ADC_PU_SCALE_FACTOR  (float)(0.000244140625)
#define ADC_PU_PPB_SCALE_FACTOR 0.000488281250 //1/2^11

#define GET_TASK_A_TIMER_OVERFLOW_STATUS CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE)
#define CLEAR_TASK_A_TIMER_OVERFLOW_FLAG CPUTimer_clearOverflowFlag(CPUTIMER0_BASE)

#define GET_TASK_B_TIMER_OVERFLOW_STATUS CPUTimer_getTimerOverflowStatus(CPUTIMER1_BASE)
#define CLEAR_TASK_B_TIMER_OVERFLOW_FLAG CPUTimer_clearOverflowFlag(CPUTIMER1_BASE)

//
// globals
//

//
// the function prototypes
//

void setupDevice(void);

void setupADC(void);

void setupUpDwnCountPWM(uint32_t base1, uint16_t pwm_period_ticks);

void disablePWMCLKCounting(void);
void enablePWMCLKCounting(void);
void setPinsAsPWM();

void setupProfilingGPIO();

//
// ISR related
//
#if CONTROL_RUNNING_ON == C28x_CORE

#ifndef __TMS320C28XX_CLA__
    #pragma INTERRUPT (controlISR, HPI)
    #pragma CODE_SECTION(controlISR,"isrcodefuncs");
    interrupt void controlISR(void);
#endif

#endif


//
// Inline functions
//

//
// setProfilingGPIO
//
static inline void setProfilingGPIO(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_PROFILING1_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// resetProfilingGPIO
//
static inline void resetProfilingGPIO(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_PROFILING1_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// setProfilingGPIO
//
static inline void setProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  + GPIO_O_GPASET ) = GPIO_PROFILING2_SET;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// resetProfilingGPIO
//
static inline void resetProfilingGPIO2(void)
{
    #pragma diag_suppress = 770
    #pragma diag_suppress = 173
    HWREG(GPIODATA_BASE  + GPIO_O_GPACLEAR ) = GPIO_PROFILING2_CLEAR;
    #pragma diag_warning = 770
    #pragma diag_warning = 173
}

//
// clearPWM Interrupt Flag
//
static inline void clearPWMInterruptFlag(uint32_t base)
{
    EPWM_clearEventTriggerInterruptFlag(base);
}

//
//enable PWM Interrupt generation
//
static inline void enablePWMInterruptGeneration(uint32_t base)
{
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(base, CNTRL_ISR_FREQ_RATIO);
    EPWM_enableInterrupt(base);
    EPWM_clearEventTriggerInterruptFlag(base);
}


#ifndef __TMS320C28XX_CLA__
//
// clearInterrupt
//
static inline void clearInterrupt(uint16_t pie_group_no)
{
    Interrupt_clearACKGroup(pie_group_no);
}


static inline void setupInterrupt(void)
{

#if CONTROL_RUNNING_ON == C28x_CORE
    Interrupt_register(C28x_CONTROLISR_INTERRUPT, &controlISR);
    enablePWMInterruptGeneration(C28x_CONTROLISR_INTERRUPT_TRIG_PWM_BASE);
    clearInterrupt(C28x_CONTROLISR_INTERRUPT_PIE_GROUP_NO);
    Interrupt_enable(C28x_CONTROLISR_INTERRUPT);
#endif

    EALLOW;
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    EDIS;
}

#endif

#ifdef __cplusplus
}
#endif                                  /* extern "C" */


#endif
