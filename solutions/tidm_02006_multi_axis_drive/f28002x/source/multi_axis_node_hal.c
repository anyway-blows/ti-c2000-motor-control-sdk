//#############################################################################
//
// FILE:    multi_axis_node_hal.c
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
#include "multi_axis_node_main.h"

#ifdef CLB_PWM_SYNC
#include "clb_config.h"
#endif // CLB_PWM_SYNC

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
// tasks 1-4 are owned by the FCL for motor
//
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);
extern __interrupt void Cla1Task3(void);
extern __interrupt void Cla1Task4(void);

//
// tasks 5-8 are reserved
//
extern __interrupt void Cla1Task5(void);
extern __interrupt void Cla1Task6(void);
extern __interrupt void Cla1Task7(void);
extern __interrupt void Cla1Task8(void);

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
    // initialize CLB handle
    //
    obj->clbHandle = CLB1_BASE;

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

    obj->pwmHandle[3] = M_PWM_SYNC_BASE;
    //
    // initialize CMPSS handle
    //
    obj->cmpssHandle[0] = M_U_CMPSS_BASE;
    obj->cmpssHandle[1] = M_V_CMPSS_BASE;
    obj->cmpssHandle[2] = M_W_CMPSS_BASE;

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

    // Timing sync for background loops
    HAL_setupCpuTimer(obj->timerHandle[0], MICROSEC_50);    // A tasks
    HAL_setupCpuTimer(obj->timerHandle[1], MICROSEC_100);   // B tasks
    HAL_setupCpuTimer(obj->timerHandle[2], MICROSEC_150);   // C tasks

    // Sets up the GPIO (General Purpose I/O) pins
    HAL_setupGPIOs(handle);

    // Sets up the ADC
    HAL_setupADCs(handle);

    #ifdef CLB_PWM_SYNC
    // Sets up the CLB
    HAL_setupCLB(handle);
    #endif  // CLB_PWM_SYNC

    // setup the CMPSS
    HAL_setupCMPSS(handle);

    // setup the PWMs
    HAL_setupMotorPWMs(handle);

    // setup the eqep
    HAL_setupQEP(handle);

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
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    SysCtl_delay(100U);

    // setup ADC modules A, B
    for(cnt = 0; cnt < 2; cnt++)
    {
        // Set main clock scaling factor (50MHz max clock for the ADC module)
        ADC_setPrescaler(obj->adcHandle[cnt], ADC_CLK_DIV_2_0);

        // set the ADC interrupt pulse generation to end of conversion
        ADC_setInterruptPulseMode(obj->adcHandle[cnt], ADC_PULSE_END_OF_CONV);

        // set priority of SOCs
        ADC_setSOCPriority(obj->adcHandle[cnt], ADC_PRI_ALL_HIPRI);

        // enable the ADC
        ADC_enableConverter(obj->adcHandle[cnt]);
    }

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

#ifdef CLB_PWM_SYNC
//
// Configure CLB
//
void HAL_setupCLB(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // Configure CLB
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);

    CLB_enableCLB(obj->clbHandle);

    initTILE1(obj->clbHandle);

    // Select Global input instead of local input for all CLB IN
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(obj->clbHandle, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

    // Select FSI_PING_PKT_RCVD for CLB1, IN0
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN0, CLB_GLOBAL_IN_MUX_FSIRXA_DATA_PACKET_RCVD);

    // Unused Inputs below
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN1, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM4A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM4A);

    // Select External for CLB1, IN0
    CLB_configGPInputMux(obj->clbHandle, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);

    // Unused inputs to GP register
    CLB_configGPInputMux(obj->clbHandle, CLB_IN1, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN2, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN3, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN4, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN5, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN6, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(obj->clbHandle, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

    // CLB output 31 is a copy of CLB output 7
    // This output is used to trigger the FSI Tx module
    // CLB output 05 will have delayed EPWMSYNCIN signal
//    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_07 | CLB_OUTPUT_05, true);
    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_05, true);

    // GPIO for monitoring SYNCH
//    XBAR_setOutputMuxConfig(CLBOUTPUTXBAR_BASE, XBAR_OUTPUT7, XBAR_OUT_MUX07_CLB1_OUT7);
//    XBAR_enableOutputMux(CLBOUTPUTXBAR_BASE, XBAR_OUTPUT7, XBAR_MUX07);
//    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
//    GPIO_setPinConfig(GPIO_14_CLB_OUTPUTXBAR7);

    // GPIO for monitoring SYNCH with delay
    XBAR_setOutputMuxConfig(CLBOUTPUTXBAR_BASE, XBAR_OUTPUT5, XBAR_OUT_MUX05_CLB1_OUT5);
    XBAR_enableOutputMux(CLBOUTPUTXBAR_BASE, XBAR_OUTPUT5, XBAR_MUX05);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_8_CLB_OUTPUTXBAR5);

    // GPIO-8 is routed back to XBAR_INPUT-5 which is connected to EXTSYNCHIN1
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT5, 8);

    return;
}
#endif  // CLB_PWM_SYNC

//
// setup CMPSS
//
void HAL_setupCMPSS(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;

    //
    // Refer to the Figure 12-1 and Table 12-1 in Chapter 12 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B)
    //
    ASysCtl_selectCMPHPMux(M_IU_CMPHP_SEL, M_IU_CMPHP_MUX);   //CMP3P_H, C4
    ASysCtl_selectCMPLPMux(M_IU_CMPLP_SEL, M_IU_CMPLP_MUX);   //CMP3P_L, C4

    ASysCtl_selectCMPHPMux(M_IV_CMPHP_SEL, M_IV_CMPHP_MUX);   //CMP1P_H, A11
    ASysCtl_selectCMPLPMux(M_IV_CMPLP_SEL, M_IV_CMPLP_MUX);   //CMP1P_L, A11

    ASysCtl_selectCMPHPMux(M_IW_CMPHP_SEL, M_IW_CMPHP_MUX);   //CMP4P_H, C1
    ASysCtl_selectCMPLPMux(M_IW_CMPLP_SEL, M_IW_CMPLP_MUX);   //CMP4P_L, C1

    DEVICE_DELAY_US(500);

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
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], (4096 - 512));

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], (0 + 512));

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
// setup FSI
//
void HAL_setupFSI(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t ni;

    FSI_disableRxInternalLoopback(obj->fsiRxHandle);

    // Could add logic to calculate PRESCALER_VAL based on user input FSI CLK
    // 50Mhz FSI clock frequency
    FSI_performRxInitialization(obj->fsiRxHandle);
    FSI_performTxInitialization(obj->fsiTxHandle, PRESCALER_VAL);

    // Clear FSI RX&TX data
    for(ni = 0; ni < 16; ni++)
    {
        HWREGH(obj->fsiRxHandle + FSI_O_RX_BUF_BASE(ni)) = 0x000;
        HWREGH(obj->fsiTxHandle + FSI_O_TX_BUF_BASE(ni)) = 0x000;
    }

    return;
}

//
// Setup interrupts
//
void HAL_setupFSIInterrupts(HAL_Handle handle)
{
    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file. Total 4; FSI Tx/Rx :: INT1/INT2
    //
    Interrupt_register(INT_FSITXA1, &fsiTxInt1ISR);
    Interrupt_register(INT_FSITXA2, &fsiTxInt2ISR);
    Interrupt_register(INT_FSIRXA1, &fsiRxInt1ISR);
    Interrupt_register(INT_FSIRXA2, &fsiRxInt2ISR);

    //
    // Enable FSI Tx/Rx interrupts
    //
    Interrupt_enable(INT_FSITXA1);
    Interrupt_enable(INT_FSITXA2);
    Interrupt_enable(INT_FSIRXA1);
    Interrupt_enable(INT_FSIRXA2);


    // clear pending INT event
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    return;
}



//
// Sets up the GPIO (General Purpose I/O) pins
//
void HAL_setupGPIOs(HAL_Handle handle)
{
    // GPIO0->EPWM1A->M_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M_WH*
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

    // GPIO6->FSITXA_D0
    GPIO_setPinConfig(GPIO_6_FSITXA_D0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->FSITXA_CLK
    GPIO_setPinConfig(GPIO_7_FSITXA_CLK);
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
    // GPIO10->SPIA_SOMI->M_DRV_SDO*
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
    // GPIO11->SPIA_SIMO->M_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO11->Reserve
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO12->FSIRXA_D0
    GPIO_setPinConfig(GPIO_12_FSIRXA_D0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->FSIRXA_CLK
    GPIO_setPinConfig(GPIO_13_FSIRXA_CLK);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    //GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // GPIO15->EPWM3B->M_WL*
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

    // GPIO23-> DRV_ENC (site 1)
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 1);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);

    // GPIO24->Reserve
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);

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
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // GPIO39->EN_GATE (site 2)
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 1);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);

    // GPIO40->Reserve
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->FSIRXA_D1
    GPIO_setPinConfig(GPIO_41_FSIRXA_D1);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->OT (Site 1)
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_PULLUP);

    // GPIO43->EQEP1_INDEX
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->OT (site 2)
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_PULLUP);

    // GPIO46->FSITXA_D1
    GPIO_setPinConfig(GPIO_46_FSITXA_D1);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL3PHGAN_REVA

    return;
}

// Sets up the PWMs (Pulse Width Modulators) for motor
void HAL_setupMotorPWMs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // *****************************************
    // Inverter PWM configuration - PWM 1, 2, 3, 5
    // *****************************************
    for(cnt = 0; cnt < 4; cnt++)
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

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_B, 0);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D,
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
        EPWM_enableSyncOutPulseSource(obj->pwmHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

        EPWM_setSyncInPulseSource(obj->pwmHandle[cnt],
                                  EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    }

    EPWM_setTimeBasePeriod(obj->pwmHandle[0], M_INV_PWM_TBPRD);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], M_INV_PWM_TBPRD);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], M_INV_PWM_TBPRD);
    EPWM_setTimeBasePeriod(obj->pwmHandle[3], M_INV_PWM_TBPRD);

    // Setting up link from EPWM to ADC
    // EPWM1 - Inverter currents at sampling frequency (@ PRD or @(PRD&ZRO))
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

    // set SOC for pwm sync
    EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                EPWM_COUNTER_COMPARE_C, EPWM_SYNC_SHIFT_VALUE_NODES);

    // Generate pulse on 15th event
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                                    EPWM_SYNC_EVENT_COUNT_NODES);

    // Select SOC from counter at ctr = CMPC
    EPWM_setADCTriggerSource(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPC);

    // clear flag
    EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

    // Enable SOC on B group
    EPWM_enableADCTrigger(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

#if defined(CLB_PWM_SYNC) && (BUILDLEVEL >= FCL_LEVEL7)
    // EXTSYNCHIN1 configured to be routed from CLB compensated output
    // GPIO-9 -> INPUTXBAR.INPUT5 -> EXTSYNCHIN1
    EPWM_setSyncInPulseSource(obj->pwmHandle[0],
                              EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5);

    EPWM_setSyncInPulseSource(obj->pwmHandle[1],
                              EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5);

    EPWM_setSyncInPulseSource(obj->pwmHandle[2],
                              EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5);


    EPWM_setSyncInPulseSource(obj->pwmHandle[3],
                              EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5);

    EPWM_setPhaseShift(obj->pwmHandle[0], EPWM_SYNC_SHIFT_U_NODES);     //EPWM1
    EPWM_setPhaseShift(obj->pwmHandle[1], EPWM_SYNC_SHIFT_V_NODES);     //EPWM2
    EPWM_setPhaseShift(obj->pwmHandle[2], EPWM_SYNC_SHIFT_W_NODES);     //EPWM3
    EPWM_setPhaseShift(obj->pwmHandle[3], EPWM_SYNC_SHIFT_T_NODES);     //EPWM4
#else
    EPWM_disablePhaseShiftLoad(obj->pwmHandle[0]);
    EPWM_setSyncInPulseSource(obj->pwmHandle[0],
                              EPWM_SYNC_IN_PULSE_SRC_DISABLE);

    EPWM_setPhaseShift(obj->pwmHandle[0], 0);     //EPWM1
    EPWM_setPhaseShift(obj->pwmHandle[1], 2);     //EPWM2
    EPWM_setPhaseShift(obj->pwmHandle[2], 4);     //EPWM3
#endif  // CLB_PWM_SYNC

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

    // High and Low Compare event trips
    uint16_t curHi = 0;
    uint16_t curLo = 0;

    curHi = 2048 + scaleCurrentValue(currentLimit);
    curLo = 2048 - scaleCurrentValue(currentLimit);

    // Configure TRIP 4 to OR the High and Low trips from both
    // comparator 1 & 3, clear everything first
    EALLOW;
    HWREG(XBAR_EPWM_CFG_REG_BASE + M_XBAR_TRIP_ADDRL) = 0;
    HWREG(XBAR_EPWM_CFG_REG_BASE + M_XBAR_TRIP_ADDRH) = 0;
    EDIS;

    //Select GPIOn as INPUTXBARn
    XBAR_setInputPin(INPUTXBAR_BASE, M_XBAR_INPUT_NUM, M_XBAR_INPUT_GPIO);
    XBAR_lockInput(INPUTXBAR_BASE, M_XBAR_INPUT_NUM);

    // Enable Muxes for ored input of CMPSS1H and 1L, mux for Mux0x
    //cmpss3 - tripH
    XBAR_setEPWMMuxConfig(M_XBAR_TRIP_NUM, M_IU_XBAR_EPWM_MUX);

    //cmpss1 - tripH or tripL
    XBAR_setEPWMMuxConfig(M_XBAR_TRIP_NUM, M_IV_XBAR_EPWM_MUX);

    //cmpss4 - tripL
    XBAR_setEPWMMuxConfig(M_XBAR_TRIP_NUM, M_IW_XBAR_EPWM_MUX);

    // Disable all the muxes first
    XBAR_disableEPWMMux(M_XBAR_TRIP_NUM, 0xFFFF);

    // Enable Mux 0  OR Mux 4 to generate TRIP4
    XBAR_enableEPWMMux(M_XBAR_TRIP_NUM,
                       M_IU_XBAR_MUX | M_IV_XBAR_MUX | M_IW_XBAR_MUX);

    //
    // Configure TRIP for motor inverter phases
    //
    for(cnt = 0; cnt < 3; cnt++)
    {

        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], curHi);
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], curLo);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], M_TZ_OSHT_NUM);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                             M_DC_TRIP_NUM, EPWM_DC_TYPE_DCAH);

        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                             M_DC_TRIP_NUM, EPWM_DC_TYPE_DCBH);

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
