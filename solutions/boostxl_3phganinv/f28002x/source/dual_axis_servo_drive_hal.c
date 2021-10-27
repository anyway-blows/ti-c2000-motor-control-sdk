//#############################################################################
//
// FILE:    dual_axis_servo_drive_hal.c
//
// TITLE:   define initialize the handle functions of device
//
// Group:   C2000
//
// Target Family: F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// includes
//
#include "dual_axis_servo_drive_settings.h"
#include "dual_axis_servo_drive_hal.h"

#include "dual_axis_servo_drive_cpu.h"

#include "stdbool.h"
#include "stdint.h"

//
// SD Trip Level - scope for additional work
//
uint16_t hlt = 0x7FFF;
uint16_t llt = 0x0;

//
// These are defined by the linker file
//
extern uint32_t Cla1funcsLoadStart;
extern uint32_t Cla1funcsLoadSize;
extern uint32_t Cla1funcsRunStart;

extern uint32_t Cla1ConstRunStart;
extern uint32_t Cla1ConstLoadStart;
extern uint32_t Cla1ConstLoadSize;

//
// interrupt routines for CPU
//
extern __interrupt void motor1ControlISR(void);
extern __interrupt void motor2ControlISR(void);

//
// tasks 1-4 are owned by the FCL for motor 1
//
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);
extern __interrupt void Cla1Task3(void);
extern __interrupt void Cla1Task4(void);

//
// tasks 5-8 are owned by the FCL for motor 2
//
extern __interrupt void Cla1Task5(void);
extern __interrupt void Cla1Task6(void);
extern __interrupt void Cla1Task7(void);
extern __interrupt void Cla1Task8(void);

//
// Enables interrupts
//
void HAL_enableInterrupts(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // clear pending INT event
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    if(handle == &halMtr[MTR_1])
    {
        // Enable PWM1INT in PIE group 3
        Interrupt_enable(M1_INT_PWM);
    }
    else if(handle == &halMtr[MTR_2])
    {
        Interrupt_enable(M2_INT_PWM);        // Enable PWM1INT in PIE group 3
    }

    // Enable group 3 interrupts - EPWM1 is here
    Interrupt_enableInCPU(INTERRUPT_CPU_INT3);

    return;
}

//
// initialize the handles
//
HAL_Handle HAL_init(void *pMemory, const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    //
    // assign the handle
    //
    handle = (HAL_Handle)pMemory;

    //
    // assign the object
    //
    obj = (HAL_Obj *)handle;

    //
    // disable watchdog
    //
    SysCtl_disableWatchdog();

    //
    // initialize the DAC handles
    // No DAC on F28002x

    //
    // initialize CLA handle
    // No CLA on F28002x

    //
    // initialize SCI handle
    // only one SCI on F28002x
    obj->sciHandle[0] = SCIA_BASE;

    //
    // initialize timer handles
    //
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    //
    // initialize ADC handles
    // Only two ADC on F28002x
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCC_BASE;

    //
    // initialize the ADC results
    // Only two ADC on F28002x
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCCRESULT_BASE;

    return(handle);
} // end of HAL_init() function

//
// Initializes the hardware abstraction layer (HAL) object for motors
//
HAL_MTR_Handle HAL_MTR_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    //
    // assign the handle
    //
    handle = (HAL_MTR_Handle)pMemory;

    //
    // assign the object
    //
    obj = (HAL_MTR_Obj *)handle;

    if(handle == &halMtr[MTR_1])
    {
        //
        // initialize SPI handle
        //
        obj->spiHandle = M1_SPI_BASE;

        //
        // initialize PWM handles for motor_1
        //
        obj->pwmHandle[0] = M1_U_PWM_BASE;
        obj->pwmHandle[1] = M1_V_PWM_BASE;
        obj->pwmHandle[2] = M1_W_PWM_BASE;

        //
        // initialize CMPSS handle
        //
        obj->cmpssHandle[0] = M1_U_CMPSS_BASE;
        obj->cmpssHandle[1] = M1_V_CMPSS_BASE;
        obj->cmpssHandle[2] = M1_W_CMPSS_BASE;

        // initialize QEP driver
        obj->qepHandle = M1_QEP_BASE;
    }
    else if(handle == &halMtr[MTR_2])
    {
        //
        // initialize SPI handle
        //
        obj->spiHandle = M2_SPI_BASE;

        //
        // initialize PWM handles for motor_2
        //
        obj->pwmHandle[0] = M2_U_PWM_BASE;
        obj->pwmHandle[1] = M2_V_PWM_BASE;
        obj->pwmHandle[2] = M2_W_PWM_BASE;

        //
        // initialize CMPSS handle for motor_2
        //
        obj->cmpssHandle[0] = M2_U_CMPSS_BASE;
        obj->cmpssHandle[1] = M2_V_CMPSS_BASE;
        obj->cmpssHandle[2] = M2_W_CMPSS_BASE;

        //
        // initialize QEP driver
        //
        obj->qepHandle = M2_QEP_BASE;
    }

     return(handle);
} // end of HAL_MTR_init() function

//
// sets the HAL parameters for motor
//
void HAL_setMotorParams(HAL_MTR_Handle handle)
{
    //
    // setup the PWMs
    //
    HAL_setupMotorPWMs(handle);

    //
    // setup the CMPSS
    //
    HAL_setupCMPSS(handle);

    //
    // setup the eqep
    //
    HAL_setupQEP(handle);

    return;
}


//
// sets the HAL parameters
//
void HAL_setParams(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // Make sure the LSPCLK divider is set to divide by 2
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_2); // 50MHz for SFRA

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    Interrupt_initModule();


    //
    // Clear all interrupts and initialize PIE vector table:
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    //
    Interrupt_initVectorTable();

    //
    // Timing sync for background loops
    //
    HAL_setupCpuTimer(obj->timerHandle[0], MICROSEC_50);    // A tasks
    HAL_setupCpuTimer(obj->timerHandle[1], MICROSEC_100);   // B tasks
    HAL_setupCpuTimer(obj->timerHandle[2], MICROSEC_150);   // C tasks

    //
    // Sets up the GPIO (General Purpose I/O) pins
    //
    HAL_setupGPIOs(handle);

    //
    // Sets up the ADC
    //
    HAL_setupADCs(handle);



    return;
}


//
// Configure ADC
//
void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;

    // Enable internal voltage reference
    SysCtl_delay(100U);
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    SysCtl_delay(100U);

    // setup ADC modules A, B, C
    for(cnt = 0; cnt < 2; cnt++)
    {
        // Set main clock scaling factor (50MHz max clock for the ADC module)
        ADC_setPrescaler(obj->adcHandle[cnt], ADC_CLK_DIV_2_0);

        // set the ADC interrupt pulse generation to end of conversion
        ADC_setInterruptPulseMode(obj->adcHandle[cnt], ADC_PULSE_END_OF_CONV);

        // enable the ADC
        ADC_enableConverter(obj->adcHandle[cnt]);

        // set priority of SOCs
        ADC_setSOCPriority(obj->adcHandle[cnt], ADC_PRI_ALL_HIPRI);
    }

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);


    //-------------------------------------------------------------------------
    // For motor 1
    //-------------------------------------------------------------------------
    // Shunt Motor Currents (M1-Iu) @ B2->SOCB0
    // SOC0 will convert pin B2, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M1_IU_ADC_BASE, M1_IU_ADC_SOC_NUM,
                 M1_ADC_TRIGGER_SOC, M1_IU_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M1_IU_ADC_BASE, M1_IU_ADC_PPB_NUM, M1_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M1_IU_ADC_BASE, M1_IU_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iv) @ C0->SOCC0
    // SOC0 will convert pin C0, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M1_IV_ADC_BASE, M1_IV_ADC_SOC_NUM,
                 M1_ADC_TRIGGER_SOC, M1_IV_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M1_IV_ADC_BASE, M1_IV_ADC_PPB_NUM, M1_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M1_IV_ADC_BASE, M1_IV_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iw) @ A9->SOCA0
    // SOC0 will convert pin A9, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M1_IW_ADC_BASE, M1_IW_ADC_SOC_NUM,
                 M1_ADC_TRIGGER_SOC, M1_IW_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M1_IW_ADC_BASE, M1_IW_ADC_PPB_NUM, M1_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M1_IW_ADC_BASE, M1_IW_ADC_PPB_NUM, 0);

    // Phase Voltage (M1-Vfb-dc) @ A5->SOCA1
    // SOC1 will convert pin A5, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M1_VDC_ADC_BASE, M1_VDC_ADC_SOC_NUM,
                 M1_ADC_TRIGGER_SOC, M1_VDC_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC1
    ADC_setupPPB(M1_VDC_ADC_BASE, M1_VDC_ADC_PPB_NUM, M1_VDC_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M1_VDC_ADC_BASE, M1_VDC_ADC_PPB_NUM, 0);

    //-------------------------------------------------------------------------
    // For motor 2
    //-------------------------------------------------------------------------
    // Shunt Motor Currents (M2-Iu) @ C3->SOCC2
    // SOC1 will convert pin C3, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M2_IU_ADC_BASE, M2_IU_ADC_SOC_NUM,
                 M2_ADC_TRIGGER_SOC, M2_IU_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC2
    ADC_setupPPB(M2_IU_ADC_BASE, M2_IU_ADC_PPB_NUM, M2_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M2_IU_ADC_BASE, M2_IU_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M2-Iv) @ C5->SOCC3
    // SOC2 will convert pin C5, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M2_IV_ADC_BASE, M2_IV_ADC_SOC_NUM,
                 M2_ADC_TRIGGER_SOC, M2_IV_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC3
    ADC_setupPPB(M2_IV_ADC_BASE, M2_IV_ADC_PPB_NUM, M2_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M2_IV_ADC_BASE, M2_IV_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M2-Iw) @ A3->SOCA2
    // SOC2 will convert pin A3, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M2_IW_ADC_BASE, M2_IW_ADC_SOC_NUM,
                 M2_ADC_TRIGGER_SOC, M2_IW_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC2
    ADC_setupPPB(M2_IW_ADC_BASE, M2_IW_ADC_PPB_NUM, M2_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M2_IW_ADC_BASE, M2_IW_ADC_PPB_NUM, 0);

    // Phase Voltage (M2-Vfb-dc) @ A6->SOCA3
    // SOC3 will convert pin A6, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M2_VDC_ADC_BASE, M2_VDC_ADC_SOC_NUM,
                 M2_ADC_TRIGGER_SOC, M2_VDC_ADC_CH_NUM, 12);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC3
    ADC_setupPPB(M2_VDC_ADC_BASE, M2_VDC_ADC_PPB_NUM, M2_VDC_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M2_VDC_ADC_BASE, M2_VDC_ADC_PPB_NUM, 0);

    return;
}

//
// setup CMPSS
//
void HAL_setupCMPSS(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    //
    // Refer to the Figure 12-4 and Table 12-1 in Chapter 12 of TMS320F28002x
    // Technical Reference Manual (SPRUI33B)
    //
    if(handle == &halMtr[MTR_1])
    {
        ASysCtl_selectCMPHPMux(M1_IU_CMPHP_SEL, M1_IU_CMPHP_MUX);

        ASysCtl_selectCMPHPMux(M1_IV_CMPHP_SEL, M1_IV_CMPHP_MUX);
        ASysCtl_selectCMPLPMux(M1_IV_CMPLP_SEL, M1_IV_CMPLP_MUX);

        ASysCtl_selectCMPLPMux(M1_IW_CMPLP_SEL, M1_IW_CMPLP_MUX);
    }
    else if(handle == &halMtr[MTR_2])
    {
        ASysCtl_selectCMPHPMux(M2_IU_CMPHP_SEL, M2_IU_CMPHP_MUX);

        ASysCtl_selectCMPHPMux(M2_IV_CMPHP_SEL, M2_IV_CMPHP_MUX);
        ASysCtl_selectCMPLPMux(M2_IV_CMPLP_SEL, M2_IV_CMPLP_MUX);

        ASysCtl_selectCMPLPMux(M2_IW_CMPLP_SEL, M2_IW_CMPLP_MUX);
    }

    DEVICE_DELAY_US(500);

    //** U-Phase Current Protection using CMPSS
    // Set up COMPCTL register
    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);


    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                            (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 2);

    // set up COMPDACCTL register
    // VDDA is REF for CMPSS DACs, DAC updated on sysclock, Ramp bypassed
    CMPSS_configDAC(obj->cmpssHandle[0],
            (CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW));

    // Load DACs - High and Low
    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], 1024);

    // digital filter settings - HIGH side
    // set time between samples, max : 1023, # of samples in window,
    // max : 31, recommended : thresh > sampWin/2
    // Init samples to filter input value
    CMPSS_configFilterHigh(obj->cmpssHandle[0], 20, 30, 18);
    CMPSS_initFilterHigh(obj->cmpssHandle[0]);

    // Clear the status register for latched comparator events
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    // Enable CMPSS
    CMPSS_enableModule(obj->cmpssHandle[0]);

    //** V-Phase Current Protection using CMPSS
    // Set up COMPCTL register
    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

    // NEG signal from DAC for COMP-L, COMP-L output is inverted
    CMPSS_configLowComparator(obj->cmpssHandle[0],
                              (CMPSS_INSRC_DAC | CMPSS_INV_INVERTED)) ;

    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                            (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[0],
                           (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 2);

    // set up COMPDACCTL register
    // VDDA is REF for CMPSS DACs, DAC updated on sysclock, Ramp bypassed
    CMPSS_configDAC(obj->cmpssHandle[0],
            (CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW));

    // Load DACs - High and Low
    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], 1024);

    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[0], 1024);

    // digital filter settings - HIGH side
    // set time between samples, max : 1023, # of samples in window,
    // max : 31, recommended : thresh > sampWin/2
    // Init samples to filter input value
    CMPSS_configFilterHigh(obj->cmpssHandle[0], 20, 30, 18);
    CMPSS_initFilterHigh(obj->cmpssHandle[0]);

    // digital filter settings - LOW side
    // set time between samples, max : 1023, # of samples in window,
    // max : 31, recommended : thresh > sampWin/2
    // Init samples to filter input value
    CMPSS_configFilterLow(obj->cmpssHandle[0], 20, 30, 18);
    CMPSS_initFilterLow(obj->cmpssHandle[0]);

    // Clear the status register for latched comparator events
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    // Enable CMPSS
    CMPSS_enableModule(obj->cmpssHandle[0]);

    //** W-Phase Current Protection using CMPSS
    // Set up COMPCTL register
    // NEG signal from DAC for COMP-L, COMP-L output is inverted
    CMPSS_configLowComparator(obj->cmpssHandle[2],
                              (CMPSS_INSRC_DAC | CMPSS_INV_INVERTED)) ;

    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[2],
                           (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[2], 2);

    // set up COMPDACCTL register
    // VDDA is REF for CMPSS DACs, DAC updated on sysclock, Ramp bypassed
    CMPSS_configDAC(obj->cmpssHandle[2],
            (CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW));

    // Load DACs - High and Low
    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[2], 1024);

    // digital filter settings - LOW side
    // set time between samples, max : 1023, # of samples in window,
    // max : 31, recommended : thresh > sampWin/2
    // Init samples to filter input value
    CMPSS_configFilterLow(obj->cmpssHandle[2], 20, 30, 18);
    CMPSS_initFilterLow(obj->cmpssHandle[2]);

    // Clear the status register for latched comparator events
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Enable CMPSS
    CMPSS_enableModule(obj->cmpssHandle[2]);

    return;
}

//
// Setup OCP limits and digital filter parameters of CMPSS
//
void HAL_setupCMPSS_DACValue(HAL_MTR_Handle handle, uint16_t curHi, uint16_t curLo)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // comparator references
    // Set DAC-H to allowed MAX +ve current
    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], curHi);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], curHi);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], curLo);

    CMPSS_setDACValueLow(obj->cmpssHandle[2], curLo);

    return;
}

//
// Setup interrupts
//
void HAL_setupInterrupts(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;


    // Enable EPWM1 INT to generate MotorControlISR
    #if(SAMPLING_METHOD == SINGLE_SAMPLING)
    // Select INT @ ctr = 0
    EPWM_setInterruptSource(obj->pwmHandle[0],
                            EPWM_INT_TBCTR_ZERO);
    #elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
    // Select INT @ ctr = 0 or ctr = prd
    EPWM_setInterruptSource(obj->pwmHandle[0],
                            EPWM_INT_TBCTR_ZERO_OR_PERIOD);
    #endif

    // Enable Interrupt Generation from the PWM module
    EPWM_enableInterrupt(obj->pwmHandle[0]);

    // This needs to be 1 for the INTFRC to work
    EPWM_setInterruptEventCount(obj->pwmHandle[0], 1);

    // Clear ePWM Interrupt flag
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    if(handle == &halMtr[MTR_1])
    {
        Interrupt_register(M1_INT_PWM, &motor1ControlISR);

        // Enable AdcA-ADCINT1- to help verify EoC before result data read
        ADC_setInterruptSource(M1_IW_ADC_BASE, M1_INT_ADC_NUM,
                               M1_IW_ADC_SOC_NUM);
        ADC_enableContinuousMode(M1_IW_ADC_BASE, M1_INT_ADC_NUM);
        ADC_enableInterrupt(M1_IW_ADC_BASE, M1_INT_ADC_NUM);
    }
    else if(handle == &halMtr[MTR_2])
    {
        Interrupt_register(M2_INT_PWM, &motor2ControlISR);

        // Enable AdcA-ADCINT1- to help verify EoC before result data read
        ADC_setInterruptSource(M2_IW_ADC_BASE, M2_INT_ADC_NUM,
                               M2_IW_ADC_SOC_NUM);
        ADC_enableContinuousMode(M2_IW_ADC_BASE, M2_INT_ADC_NUM);
        ADC_enableInterrupt(M2_IW_ADC_BASE, M2_INT_ADC_NUM);

    }

    return;
}

//
// setup CPU Timer
//
void HAL_setupCpuTimer(uint32_t base, uint32_t periodCount)
{
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);  // divide by 1 (SYSCLKOUT)
    CPUTimer_setPeriod(base, periodCount);
    CPUTimer_stopTimer(base);               // Stop timer / reload / restart
    CPUTimer_setEmulationMode(base,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_reloadTimerCounter(base);      // Reload counter with period value
    CPUTimer_resumeTimer(base);

    return;
}


//
// Sets up the GPIO (General Purpose I/O) pins
//
void HAL_setupGPIOs(HAL_Handle handle)
{
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO5->SPIA_STE
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO5->
    GPIO_setPinConfig(GPIO_5_GPIO5);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->Reserve
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO09->SPI_SCLK*
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO9->Reserve
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO10->SPIA_SOMI->M1_DRV_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO10->Reserve
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO11->SPIA_SIMO->M1_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO11->Reserve
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->EQEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 4);

    // GPIO15->EPWM3B->M1_WL*
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO22->SPIB_CLK->M2_DRV_SCLK
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO23-> En_GATE(site 1)
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);

    // GPIO24->Reserve
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->EQEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 4);

    // GPIO26->EQEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);

    // GPIO27->Reserve
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIA_RX->XDS110
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->SCIA_TX->XDS110
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO30->SPIB_SIMO->M2_DRV_SDI*
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO31->SPIB_SOMI->M2_DRV_SDO*
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO32->Reserve*
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->SPIB_STE
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO34->LPD_LED5
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 1);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->Reserve
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->SPIB_STE
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 4);

    // GPIO39->EN_GATE (site 2)
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 0);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);

    // GPIO40->Reserve
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->reserve
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->OT (Site 1)
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_PULLUP);

    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 4);

    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 4);

    // GPIO45->OT (site 2)
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_PULLUP);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL3PHGAN_REVA

    return;
}


// Sets up the PWMs (Pulse Width Modulators) for motor
void HAL_setupMotorPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    uint16_t  cnt;
    uint16_t  halfPeriod = 0;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // *****************************************
    // Inverter PWM configuration - PWM 1, 2, 3
    // *****************************************
    for(cnt = 0; cnt < 3; cnt++)
    {
        // Time Base SubModule Registers
        // set Immediate load
        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);
        EPWM_setTimeBasePeriod(obj->pwmHandle[cnt], 0);
        EPWM_setPhaseShift(obj->pwmHandle[cnt], 0);
        EPWM_setTimeBaseCounter(obj->pwmHandle[cnt], 0);
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);

        EPWM_setClockPrescaler(obj->pwmHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);

        // Counter Compare Submodule Registers
        // set duty 0% initially
        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_A, 0);
        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        // Action Qualifier SubModule Registers
        EPWM_setActionQualifierActionComplete(obj->pwmHandle[cnt],
                                              EPWM_AQ_OUTPUT_A,
                (EPWM_ActionQualifierEventAction)(EPWM_AQ_OUTPUT_LOW_UP_CMPA |
                                               EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA));

        // Active high complementary PWMs - Set up the deadband
        EPWM_setRisingEdgeDeadBandDelayInput(obj->pwmHandle[cnt],
                                             EPWM_DB_INPUT_EPWMA);
        EPWM_setFallingEdgeDeadBandDelayInput(obj->pwmHandle[cnt],
                                              EPWM_DB_INPUT_EPWMA);

        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt], EPWM_DB_RED,
                                      EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt], EPWM_DB_DELAY_RISE);
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt], EPWM_DB_DELAY_FALL);

        EPWM_enablePhaseShiftLoad(obj->pwmHandle[cnt]);

        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);
        // configure sync
        EPWM_enableSyncOutPulseSource(obj->pwmHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
    }

    SysCtl_setSyncOutputConfig(SYSCTL_SYNC_OUT_SRC_EPWM1SYNCOUT);


    if(handle == &halMtr[MTR_1])
    {
        halfPeriod = M1_INV_PWM_TICKS / 2;     // 100MHz EPWMCLK

        EPWM_disablePhaseShiftLoad(obj->pwmHandle[0]);  //EPWM1

        EPWM_enableSyncOutPulseSource(obj->pwmHandle[0],
                                      EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

        EPWM_setPhaseShift(obj->pwmHandle[0], 0);    //EPWM1
        EPWM_setPhaseShift(obj->pwmHandle[1], 0);    //EPWM2
        EPWM_setPhaseShift(obj->pwmHandle[2], 0);    //EPWM3
    }
    else if(handle == &halMtr[MTR_2])
    {
        halfPeriod = M2_INV_PWM_TICKS / 2;     // 100MHz EPWMCLK

        // sync "down-stream"
        EPWM_setPhaseShift(obj->pwmHandle[0], (halfPeriod>>1));         //EPWM7
        EPWM_setPhaseShift(obj->pwmHandle[1], (halfPeriod>>1));         //EPWM4
        EPWM_setPhaseShift(obj->pwmHandle[2], (halfPeriod>>1));         //EPWM5
    }

    EPWM_setTimeBasePeriod(obj->pwmHandle[0], halfPeriod);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], halfPeriod);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], halfPeriod);

    // Setting up link from EPWM to ADC
    // EPWM1/EPWM4 - Inverter currents at sampling frequency
    //               (@ PRD or @ (PRD&ZRO) )
#if(SAMPLING_METHOD == SINGLE_SAMPLING)
    // Select SOC from counter at ctr = prd
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
    // Select SOC from counter at ctr = 0 or ctr = prd
    EPWM_setADCTriggerSource(obj->pwmHandle[0], EPWM_SOC_A,
                             EPWM_SOC_TBCTR_ZERO_OR_PERIOD);
#endif

    // Generate pulse on 1st event
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);

    // Enable SOC on A group
    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    return;
}

//
// Configure Motor Fault Protection Against Over Current
//
void HAL_setupMotorFaultProtection(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    uint16_t  cnt;

    uint16_t dcTripSet, tzOSTSet;

    if(handle == &halMtr[MTR_1])
    {
        // Configure TRIP 4 to OR the High and Low trips from both
        // comparator 1 & 3, clear everything first
        EALLOW;
        HWREG(XBAR_EPWM_CFG_REG_BASE + M1_XBAR_TRIP_ADDRL) = 0;
        HWREG(XBAR_EPWM_CFG_REG_BASE + M1_XBAR_TRIP_ADDRH) = 0;
        EDIS;

        //Select GPIOn as INPUTXBARn
        XBAR_setInputPin(INPUTXBAR_BASE, M1_XBAR_INPUT_NUM, M1_XBAR_INPUT_GPIO);
        XBAR_lockInput(INPUTXBAR_BASE, M2_XBAR_INPUT_NUM);

        // Enable Muxes for ored input of CMPSS1H and 1L, mux for Mux0x
        //cmpss3 - tripH
        XBAR_setEPWMMuxConfig(M1_XBAR_TRIP_NUM, M1_IU_XBAR_EPWM_MUX);

        //cmpss1 - tripH or tripL
        XBAR_setEPWMMuxConfig(M1_XBAR_TRIP_NUM, M1_IV_XBAR_EPWM_MUX);

        //cmpss4 - tripL
        XBAR_setEPWMMuxConfig(M1_XBAR_TRIP_NUM, M1_IW_XBAR_EPWM_MUX);

        // Disable all the muxes first
        XBAR_disableEPWMMux(M1_XBAR_TRIP_NUM, 0xFFFF);

        // Enable Mux 0  OR Mux 4 to generate TRIP4
        XBAR_enableEPWMMux(M1_XBAR_TRIP_NUM,
                           M1_IU_XBAR_MUX | M1_IV_XBAR_MUX | M1_IW_XBAR_MUX);

        dcTripSet = M1_DC_TRIP_NUM;
        tzOSTSet = M1_TZ_OSHT_NUM;
    }
    else if(handle == &halMtr[MTR_2])
    {
        // Configure TRIP 4 to OR the High and Low trips from both
        // comparator 1 & 3, clear everything first
        EALLOW;
        HWREG(XBAR_EPWM_CFG_REG_BASE + M2_XBAR_TRIP_ADDRL) = 0;
        HWREG(XBAR_EPWM_CFG_REG_BASE + M2_XBAR_TRIP_ADDRH) = 0;
        EDIS;

        //Select GPIOn as INPUTXBARn
        XBAR_setInputPin(INPUTXBAR_BASE, M2_XBAR_INPUT_NUM, M2_XBAR_INPUT_GPIO);
        XBAR_lockInput(INPUTXBAR_BASE, M2_XBAR_INPUT_NUM);

        // Enable Muxes for ored input of CMPSS1H and 1L, mux for Mux0x
        //cmpss3 - tripH
        XBAR_setEPWMMuxConfig(M2_XBAR_TRIP_NUM, M2_IU_XBAR_EPWM_MUX);

        //cmpss1 - tripH or tripL
        XBAR_setEPWMMuxConfig(M2_XBAR_TRIP_NUM, M2_IV_XBAR_EPWM_MUX);

        //cmpss4 - tripL
        XBAR_setEPWMMuxConfig(M2_XBAR_TRIP_NUM, M2_IW_XBAR_EPWM_MUX);

        // Disable all the muxes first
        XBAR_disableEPWMMux(M2_XBAR_TRIP_NUM, 0xFFFF);

        // Enable Mux 0  OR Mux 4 to generate TRIP4
        XBAR_enableEPWMMux(M2_XBAR_TRIP_NUM,
                           M2_IU_XBAR_MUX | M2_IV_XBAR_MUX | M2_IW_XBAR_MUX);

        dcTripSet = M2_DC_TRIP_NUM;
        tzOSTSet = M2_TZ_OSHT_NUM;
    }


    //
    // Configure TRIP for motor inverter phases
    //
    for(cnt = 0; cnt < 3; cnt++)
    {
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], tzOSTSet);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                                      dcTripSet, EPWM_DC_TYPE_DCAH);

        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                                      dcTripSet, EPWM_DC_TYPE_DCBH);

        // Trigger event when DCAH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        // Trigger event when DCBH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_B1,
                                                     EPWM_TZ_EVENT_DCXL_HIGH);

        // Configure the DCA path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_A,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        // Configure the DCB path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_B,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_A,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_B,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        // Enable DCA as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Enable DCB as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCBEVT1);

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB

        // EPWMxA will go low
        // EPWMxB will go low
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZA,
                               EPWM_TZ_ACTION_LOW);

        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZB,
                               EPWM_TZ_ACTION_LOW);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZFLAG_ALL);


    }

    return;
}

void HAL_setupQEP(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Configure the decoder for quadrature count mode, counting both
    // rising and falling edges (that is, 2x resolution)
    EQEP_setDecoderConfig(obj->qepHandle, (EQEP_CONFIG_2X_RESOLUTION |
                                           EQEP_CONFIG_QUADRATURE |
                                           EQEP_CONFIG_NO_SWAP) );

    EQEP_setEmulationMode(obj->qepHandle, EQEP_EMULATIONMODE_RUNFREE);

    // Configure the position counter to be latched on a unit time out
    // and latch on rising edge of index pulse
    EQEP_setLatchMode(obj->qepHandle, (EQEP_LATCH_RISING_INDEX |
                                       EQEP_LATCH_UNIT_TIME_OUT) );

    // Configure the position counter to reset on a maximum position
    if(handle == &halMtr[MTR_1])
    {
        EQEP_setPositionCounterConfig(obj->qepHandle,
                                      EQEP_POSITION_RESET_MAX_POS,
                                      ((4 * M1_ENCODER_LINES) - 1) );

        // Enable the unit timer, setting the frequency to 10KHz
        EQEP_enableUnitTimer(obj->qepHandle, M1_QEP_UNIT_TIMER_TICKS - 1);
    }
    else if(handle == &halMtr[MTR_2])
    {
        EQEP_setPositionCounterConfig(obj->qepHandle,
                                      EQEP_POSITION_RESET_MAX_POS,
                                      ((4 * M2_ENCODER_LINES) - 1) );

        // Enable the unit timer, setting the frequency to 10KHz
        EQEP_enableUnitTimer(obj->qepHandle, M2_QEP_UNIT_TIMER_TICKS - 1);
    }

    // Disables the eQEP module position-compare unit
    EQEP_disableCompare(obj->qepHandle);

    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/128. The unit-position event divider is QCLK/32.
    EQEP_setCaptureConfig(obj->qepHandle, EQEP_CAPTURE_CLK_DIV_128,
                                          EQEP_UNIT_POS_EVNT_DIV_32);

    // Enable QEP edge-capture unit
    EQEP_enableCapture(obj->qepHandle);

    // Enable UTO on QEP
    EQEP_enableInterrupt(obj->qepHandle, EQEP_INT_UNIT_TIME_OUT);

    // Enable the eQEP module
    EQEP_enableModule(obj->qepHandle);

    return;
}


void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount=0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }

    return;
}   //end of HAL_clearDataRAM() function

// end of the file
