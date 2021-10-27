//############################################################################
//
// FILE:   sfra_examples_hal.c
//
// AUTHOR: Manish Bhardwaj (C2000 Systems Solutions, Houston , TX)
//
//############################################################################
// $TI Release: C2000 Software Frequency Response Analyzer Library v1.40.00.00 $
// $Release Date: Tue Sep 21 16:33:07 CDT 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//############################################################################

#include "sfra_examples_hal.h"

//
// Global variables used that are applicable to this board
//

//
//  This routine sets up the basic device configuration such as initializing PLL
//  copying code from FLASH to RAM
//
void setupDevice(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // initialize CPU timers
    //

    //
    // Initialize timer period to 100Hz
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ / 100 );
    //
    // Initialize timer period to 10Hz
    //
    CPUTimer_setPeriod(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ / 10 );
    //
    // Initialize timer period to 10KHz
    //
    CPUTimer_setPeriod(CPUTIMER2_BASE, DEVICE_SYSCLK_FREQ / 10000 );
    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);
    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setEmulationMode(CPUTIMER2_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    CPUTimer_resumeTimer(CPUTIMER0_BASE);
    CPUTimer_resumeTimer(CPUTIMER1_BASE);
    CPUTimer_resumeTimer(CPUTIMER2_BASE);

}

//
// setupADC()
//
void setupADC(void)
{

    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);

    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);
    ADC_enableConverter(ADCC_BASE);

    DEVICE_DELAY_US(1000);

    //
    // setup ADC conversions for current and voltage signals
    //

}

//
// configurePWM1chUpCnt()
//
void configurePWM1chUpCnt(uint32_t base1, uint16_t period)
{

    //
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(base1, period - 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // configure PWM 1 as master and Phase 2 as slaves and
    // let it pass the sync in pulse from PWM1
    //
    EPWM_disablePhaseShiftLoad(base1);
    EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    //
    // Counter Compare Submodule Registers
    // set duty 0% initially
    //
    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A, 0);
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Action Qualifier SubModule Registers
    //
    HWREGH(base1 + EPWM_O_AQCTLA) = 0 ;
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A ,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

}

//
// disablePWMCLKCounting
//
void disablePWMCLKCounting(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//
// enablePWMCLKCounting
//
void enablePWMCLKCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//
// setupProfilingGPIO()
//
void setupProfilingGPIO(void)
{
    GPIO_setDirectionMode(GPIO_PROFILING1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(GPIO_PROFILING2, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(GPIO_PROFILING1, GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(GPIO_PROFILING2, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_PROFILING1_PIN_CONFIG);
    GPIO_setPinConfig(GPIO_PROFILING2_PIN_CONFIG);

#if CONTROL_RUNNING_ON == CLA_CORE
    GPIO_setMasterCore(GPIO_PROFILING1, GPIO_CORE_CPU1_CLA1);
    GPIO_setMasterCore(GPIO_PROFILING2, GPIO_CORE_CPU1_CLA1);
#endif
}

void setupUpDwnCountPWM(uint32_t base1, uint16_t pwm_period_ticks)
{

    //
    // PWM clock on F2837x is divided by 2
    // ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1
    // Deadband needs to be 0.5us => 10ns*50=500ns
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBasePeriod(base1, pwm_period_ticks >> 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

}

void setPinsAsPWM()
{
    GPIO_writePin(PWM_H_GPIO, 0);
    GPIO_writePin(PWM_L_GPIO, 0);

    GPIO_setDirectionMode(PWM_H_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(PWM_H_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(PWM_H_GPIO_PIN_CONFIG );

    GPIO_setDirectionMode(PWM_L_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(PWM_L_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(PWM_L_GPIO_PIN_CONFIG );
}

