//#############################################################################
//
// FILE:    multi_axis_slave_hal.c
//
// TITLE:   define initialize the handle functions of device
//
// Group:   C2000
//

// Target Family: F28002x
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
#include "sensored_foc_hal.h"
#include "sensored_foc_main.h"
//
// interrupt routines for CPU, motor control
//
extern __interrupt void motorControlISR(void);

//
// Enables interrupts
//
void HAL_enableInterrupts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // clear pending INT event
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    // Enable PWMxINT in PIE group 3
    Interrupt_enable(M_INT_PWM);

    // Enable group 3 interrupts - EPWMx is here
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
    // initialize SCI handle
    //
    obj->sciHandle = SCIA_BASE;

    //
    // initialize timer handles
    //
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    //
    // initialize ADC handles
    //
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCC_BASE;

    //
    // initialize the ADC results
    //
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCCRESULT_BASE;

    //
    // initialize SPI handle
    //
    obj->spiHandle = M_SPI_BASE;

    //
    // initialize PWM handles for motor_1
    //
    obj->pwmHandle[0] = M_U_PWM_BASE;
    obj->pwmHandle[1] = M_V_PWM_BASE;
    obj->pwmHandle[2] = M_W_PWM_BASE;

    //
    // initialize CMPSS handle
    //
    obj->cmpssHandle[0] = M_U_CMPSS_BASE;
    obj->cmpssHandle[1] = M_V_CMPSS_BASE;
    obj->cmpssHandle[2] = M_W_CMPSS_BASE;
    obj->cmpssHandle[3] = M_VDC_CMPSS_BASE;

    // initialize QEP driver
    obj->qepHandle = M_QEP_BASE;

    // initialize FSI handle
    obj->fsiTxHandle = FSITXA_BASE;         //!< the FSITX handle
    obj->fsiRxHandle = FSIRXA_BASE;         //!< the FSIRX handle

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;              //!< the DMA handle

    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_CH1_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_CH2_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_CH3_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_CH4_BASE;     //!< the DMA Channel handle

    return(handle);
} // end of HAL_init() function

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

    // Enable internal voltage reference
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    // setup ADC modules A, B
    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);

    // set priority of SOCs
    ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);

    // enable the ADC
    ADC_enableConverter(obj->adcHandle[0]);
    ADC_enableConverter(obj->adcHandle[1]);

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);

    //
    // Shunt Motor Currents (M1-Iu) @ B2->SOCB0
    // SOC0 will convert pin B2, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M_IU_ADC_BASE, M_IU_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IU_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IU_ADC_BASE, M_IU_ADC_PPB_NUM, M_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IU_ADC_BASE, M_IU_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iv) @ C0->SOCC0
    // SOC0 will convert pin C0, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M_IV_ADC_BASE, M_IV_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IV_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IV_ADC_BASE, M_IV_ADC_PPB_NUM, M_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IV_ADC_BASE, M_IV_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iw) @ A9->SOCA0
    // SOC0 will convert pin A9, sample window in SYSCLK cycles
    // trigger on ePWM6 SOCA/C
    ADC_setupSOC(M_IW_ADC_BASE, M_IW_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IW_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IW_ADC_BASE, M_IW_ADC_PPB_NUM, M_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IW_ADC_BASE, M_IW_ADC_PPB_NUM, 0);

    // Phase Voltage (M1-Vfb-dc) @ A5->SOCA1
    // SOC1 will convert pin A5, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M_VDC_ADC_BASE, M_VDC_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_VDC_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC1
    ADC_setupPPB(M_VDC_ADC_BASE, M_VDC_ADC_PPB_NUM, M_VDC_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_VDC_ADC_BASE, M_VDC_ADC_PPB_NUM, 0);

    return;
}

//
// setup CMPSS
//
void HAL_setupCMPSS(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Set up COMPCTL register
        // NEG signal from DAC for COMP-H
        CMPSS_configHighComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);

        // NEG signal from DAC for COMP-L, COMP-L output is inverted
        CMPSS_configLowComparator(obj->cmpssHandle[cnt],
                                  (CMPSS_INSRC_DAC | CMPSS_INV_INVERTED)) ;

        // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
        CMPSS_configOutputsHigh(obj->cmpssHandle[cnt],
                                (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

        // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
        CMPSS_configOutputsLow(obj->cmpssHandle[cnt],
                               (CMPSS_TRIP_FILTER | CMPSS_TRIPOUT_FILTER));

        // Set up COMPHYSCTL register
        // COMP hysteresis set to 2x typical value
        CMPSS_setHysteresis(obj->cmpssHandle[cnt], 2);

        // set up COMPDACCTL register
        // VDDA is REF for CMPSS DACs, DAC updated on sysclock, Ramp bypassed
        CMPSS_configDAC(obj->cmpssHandle[cnt],
                (CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW));

        // Load DACs - High and Low
        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], 1024);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], 1024);

        // digital filter settings - HIGH side
        // set time between samples, max : 1023, # of samples in window,
        // max : 31, recommended : thresh > sampWin/2
        // Init samples to filter input value
        CMPSS_configFilterHigh(obj->cmpssHandle[cnt], 20, 30, 18);
        CMPSS_initFilterHigh(obj->cmpssHandle[cnt]);

        // digital filter settings - LOW side
        // set time between samples, max : 1023, # of samples in window,
        // max : 31, recommended : thresh > sampWin/2
        // Init samples to filter input value
        CMPSS_configFilterLow(obj->cmpssHandle[cnt], 20, 30, 18);
        CMPSS_initFilterLow(obj->cmpssHandle[cnt]);

        // Clear the status register for latched comparator events
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Enable CMPSS
        CMPSS_enableModule(obj->cmpssHandle[cnt]);
    }

    //
    // Refer to the Figure 14-1 and Table 14-1 in Chapter 14 of TMS320F28002x
    // Technical Reference Manual (SPRUIN7)
    //
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 0);   //CMP1P_H
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 0);   //CMP1P_L

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 0);   //CMP1P_H
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 0);   //CMP1P_L

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3, 4);   //CMP3P_H
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3, 4);   //CMP3P_L

    DEVICE_DELAY_US(500);

    return;
}

//
// Setup OCP limits and digital filter parameters of CMPSS
//
void HAL_setupCMPSS_DACValue(HAL_Handle handle,
                             const uint16_t curHi, const uint16_t curLo)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // comparator references
        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], curHi);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], curLo);
    }

    return;
}

//
// Setup interrupts
//
void HAL_setupInterrupts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

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

    Interrupt_register(M_INT_PWM, &motorControlISR);

    // Enable AdcA-ADCINT1- to help verify EoC before result data read
    ADC_setInterruptSource(M_IW_ADC_BASE, ADC_INT_NUMBER1,
                           M_IW_ADC_SOC_NUM);
    ADC_enableContinuousMode(M_IW_ADC_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(M_IW_ADC_BASE, ADC_INT_NUMBER1);

    return;
}

//
// setup CPU Timer
//
void HAL_setupCpuTimer(uint32_t base, uint32_t periodCount)
{
    CPUTimer_setPreScaler(base, 0);         // divide by 1 (SYSCLKOUT)
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
    // EPWM1A->UH
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // EPWM1B->UL
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // EPWM2A->VH
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // EPWM2B->VL
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->WH
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // EPWM3B->WL
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO06
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO07
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

#ifdef _FLASH
    // GPIO08
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO09
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // GPIO10
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // GPIO11
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // GPIO12
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
#endif

    // GPIO14->CLR_Fault -- Connect to H13-14
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_writePin(14, 0);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // GPIO15->TRIP_CC1
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);

#ifdef _FLASH
    // GPIO16->GPIO16
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->X2 (N/A for GPIO)
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->X1 (N/A for GPIO)
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
#endif

    // GPIO23->EQEP1I
    GPIO_setPinConfig(GPIO_23_EQEP1_INDEX);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->SPIB_SDI
    GPIO_setPinConfig(GPIO_24_SPIB_SIMO);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);

    // GPIO25->SPIB_SDO
    GPIO_setPinConfig(GPIO_25_SPIB_SOMI);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);

    // GPIO26->SPIB_CLK
    GPIO_setPinConfig(GPIO_26_SPIB_CLK);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);

    // GPIO27->SPIB_CS
    GPIO_setPinConfig(GPIO_27_SPIB_STE);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIRXDA
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->SCITXDA
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

    // GPIO30->
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_writePin(30, 0);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // GPIO31->LED1
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_writePin(31, 0);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

#ifdef _FLASH
    // GPIO32
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

    // GPIO33
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif

    // GPIO34->GPIO34->LED2
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 0);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->TDI
    GPIO_setPinConfig(GPIO_35_TDI);

    // GPIO37->TDO
    GPIO_setPinConfig(GPIO_37_TDO);

    // GPIO39->nFault(TZn)
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_INVERT);

    // GPIO40->EQEP1A
    GPIO_setPinConfig(GPIO_40_EQEP1_A);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->EQEP1B
    GPIO_setPinConfig(GPIO_41_EQEP1_B);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->ENC_PWREN
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_writePin(42, 0);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

#ifdef _FLASH
    // GPIO43->
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
#endif

    return;
}

// Sets up the PWMs (Pulse Width Modulators) for motor
void HAL_setupMotorPWMs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;
    uint16_t  halfPeriod = 0;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // *****************************************
    // Inverter PWM configuration - PWM 1, 2, 3
    // *****************************************
    for(cnt = 0; cnt < 3; cnt++)
    {
        // Time Base SubModule Registers
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);

        EPWM_setClockPrescaler(obj->pwmHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                               EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);


        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);

        EPWM_setEmulationMode(obj->pwmHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmHandle[cnt], 0);

        // Counter Compare Submodule Registers
        // set duty 0% initially
        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_A, 0);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C);

        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D);

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
    }

    halfPeriod = M_INV_PWM_TICKS / 2;     // 100MHz EPWMCLK

    EPWM_disablePhaseShiftLoad(obj->pwmHandle[0]);
    EPWM_enablePhaseShiftLoad(obj->pwmHandle[1]);
    EPWM_enablePhaseShiftLoad(obj->pwmHandle[2]);

    EPWM_enableSyncOutPulseSource(obj->pwmHandle[0],
                                  EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_enableSyncOutPulseSource(obj->pwmHandle[1],
                                  EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_disableSyncOutPulseSource(obj->pwmHandle[2],
                                  EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

    EPWM_setPhaseShift(obj->pwmHandle[0], 0);     //EPWM1
    EPWM_setPhaseShift(obj->pwmHandle[1], 2);     //EPWM2
    EPWM_setPhaseShift(obj->pwmHandle[2], 4);     //EPWM3

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

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}

//
// Configure Motor Fault Protection Against Over Current
//
void HAL_setupMotorFaultProtection(HAL_Handle handle,
                                   const float32_t currentLimit)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    EPWM_DigitalCompareTripInput tripInSet = EPWM_DC_TRIP_TRIPIN4;

    // High and Low Compare event trips
    uint16_t curHi = 0;
    uint16_t curLo = 0;

    tripInSet = EPWM_DC_TRIP_TRIPIN4;

    curHi = 2048 + M_CURRENT_SCALE(currentLimit);
    curLo = 2048 - M_CURRENT_SCALE(currentLimit);

    //Select GPIO58 as INPUTXBAR1
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT1, M_XBAR_INPUT_GPIO);

    // Configure TRIP 4 to OR the High and Low trips from both
    // comparator 1 & 3, clear everything first
    EALLOW;
    HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP4MUX0TO15CFG) = 0;
    HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP4MUX16TO31CFG) = 0;
    EDIS;

    // Enable Muxes for ored input of CMPSS1H and 1L, mux for Mux0x
    //cmpss1 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);

    //cmpss1 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);

    //cmpss3 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L);

    //inputxbar1 trip
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX01_INPUTXBAR1);

    // Disable all the muxes first
    XBAR_disableEPWMMux(XBAR_TRIP4, 0xFFFF);

    // Enable Mux 0, Mux 1 or Mux 4 to generate TRIP4
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX00 | XBAR_MUX00 | XBAR_MUX04 |
                                   XBAR_MUX01);

    //
    // Configure TRIP for motor inverter phases
    //
    for(cnt = 0; cnt < 3; cnt++)
    {
        // comparator references
        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], curHi);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], curLo);

        //Trip 4 is the input to the DCAHCOMPSEL
        EPWM_selectDigitalCompareTripInput(obj->pwmHandle[cnt],
                                           tripInSet,
                                           EPWM_DC_TYPE_DCAH);

        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_A,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_A,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Emulator Stop
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_CBC6);

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
    }

    // clear EPWM trip flags
    DEVICE_DELAY_US(1L);

    for(cnt = 0; cnt < 3; cnt++)
    {
        // clear any spurious  OST & DCAEVT1 flags
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], (EPWM_TZ_FLAG_OST |
                                                     EPWM_TZ_FLAG_DCAEVT1 |
                                                     EPWM_TZ_FLAG_CBC ));

        // clear any spurious  HLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // clear any spurious  LLATCH - (not in TRIP gen path)
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);
    }

    DEVICE_DELAY_US(1L);

    return;
}

void HAL_setupQEP(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

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
    EQEP_setPositionCounterConfig(obj->qepHandle,
                                  EQEP_POSITION_RESET_MAX_POS,
                                  ((4 * M_ENC_SLOTS) - 1) );

    // Enable the unit timer, setting the frequency to 10KHz
    EQEP_enableUnitTimer(obj->qepHandle, M_QEP_UNIT_TIMER_TICKS - 1);

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
