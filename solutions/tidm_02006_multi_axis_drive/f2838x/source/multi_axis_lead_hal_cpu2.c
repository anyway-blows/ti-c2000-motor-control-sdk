//#############################################################################
//
// FILE:    multi_axis_lead_hal_cpu2.c
//
// TITLE:   define initialize the handle functions of device
//
// Group:   C2000
//
// Target Family: F2837x
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
#include "multi_axis_lead_drive_main.h"

//
// SD Trip Level - scope for additional work
//
uint16_t hlt = 0x7FFF;
uint16_t llt = 0x0;

//
// These are defined by the linker file
//
extern uint32_t Cla1funcsRunStart;
extern uint32_t Cla1funcsLoadStart;
extern uint32_t Cla1funcsLoadSize;

extern uint32_t Cla1ConstRunStart;
extern uint32_t Cla1ConstLoadStart;
extern uint32_t Cla1ConstLoadSize;

//
// interrupt routines for CPU
//
extern __interrupt void motorControlISR(void);

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
void HAL_enableInterrupts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // clear pending INT event
    //
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    //
    // Enable PWM1INT in PIE group 3
    //
    Interrupt_enable(M_INT_PWM);

    //
    // Enable group 3 interrupts - EPWM1 is here
    //
    Interrupt_enableInCPU(INTERRUPT_CPU_INT3);

    return;
}

//
// HAL_init()
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
    //
    obj->dacHandle[0] = DACA_BASE;

    //
    // initialize CLA handle
    //
    obj->claHandle = CLA1_BASE;

    //
    // initialize timer handles
    //
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    //
    // initialize ADC handle
    //
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCB_BASE;
    obj->adcHandle[2] = ADCC_BASE;
    obj->adcHandle[3] = ADCD_BASE;

    //
    // initialize the ADC results
    //
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCBRESULT_BASE;
    obj->adcResult[2] = ADCCRESULT_BASE;
    obj->adcResult[3] = ADCDRESULT_BASE;

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
    obj->cmpssHandle[3] = M_W_CMPSS_BASE;

    //
    // initialize QEP driver
    //
    obj->qepHandle = M_QEP_BASE;

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;              //!< the DMA handle

    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_CH5_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_CH6_BASE;     //!< the DMA Channel handle

     return(handle);
} // end of HAL_MTR_init() function

//
// sets the HAL parameters
//
void HAL_setParams(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // Make sure the LSPCLK divider is set to divide by 4
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4); // 50MHz for SFRA

    //
    // Sets up the CLA
    //
    HAL_setupCLA(handle);

    //
    // Timing sync for background loops
    //
    HAL_setupCpuTimer(obj->timerHandle[0], MICROSEC_50);    // A tasks
    HAL_setupCpuTimer(obj->timerHandle[1], MICROSEC_100);   // B tasks
    HAL_setupCpuTimer(obj->timerHandle[2], MICROSEC_150);   // C tasks

    // setup the GPIO
    HAL_setupGPIOs(handle);

#ifdef DACOUT_EN
    //
    // Sets up the DAC
    //
    HAL_setupDACs(handle);
#endif  //DACOUT_EN

    //
    // Sets up the ADC
    //
    HAL_setupADCs(handle);

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
// Configure ADC
//
void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t cnt;

    //
    // setup ADC modules A, B, C, D
    //
    for(cnt = 0; cnt < 4; cnt++)
    {
        //
        // Set 12-bit single ended conversion mode
        //
        ADC_setMode(obj->adcHandle[cnt],
                    ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

        // Set main clock scaling factor (100MHz max clock for the ADC module)
        // Set the ADC clock to 50MHz
        ADC_setPrescaler(obj->adcHandle[cnt], ADC_CLK_DIV_4_0);

        // set the ADC interrupt pulse generation to end of conversion
        ADC_setInterruptPulseMode(obj->adcHandle[cnt], ADC_PULSE_END_OF_CONV);

        // enable the ADC
        ADC_enableConverter(obj->adcHandle[cnt]);

        // set priority of SOCs
        ADC_setSOCPriority(obj->adcHandle[cnt], ADC_PRI_ALL_HIPRI);
    }

    // delay to allow ADCs to power up
    DEVICE_DELAY_US(1500U);

    //-------------------------------------------------------------------------
    // For motor 1
    //-------------------------------------------------------------------------
    // Shunt Motor Currents (M1-Iu) @ C2
    // SOC0 will convert pin C2, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M_IU_ADC_BASE, M_IU_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IU_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IU_ADC_BASE, M_IU_ADC_PPB_NUM, M_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IU_ADC_BASE, M_IU_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iv) @ B2
    // SOC0 will convert pin B2, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M_IV_ADC_BASE, M_IV_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IV_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IV_ADC_BASE, M_IV_ADC_PPB_NUM, M_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IV_ADC_BASE, M_IV_ADC_PPB_NUM, 0);

    // Shunt Motor Currents (M1-Iw) @ A2
    // SOC0 will convert pin A2, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M_IW_ADC_BASE, M_IW_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_IW_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_IW_ADC_BASE, M_IW_ADC_PPB_NUM, M_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_IW_ADC_BASE, M_IW_ADC_PPB_NUM, 0);

    // Phase Voltage (M1-Vfb-dc) @ D14
    // SOC1 will convert pin D14, sample window in SYSCLK cycles
    // trigger on ePWM1 SOCA/C
    ADC_setupSOC(M_VDC_ADC_BASE, M_VDC_ADC_SOC_NUM,
                 M_ADC_TRIGGER_SOC, M_VDC_ADC_CH_NUM, M_ADC_SAMPLE_WINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with SOC0
    ADC_setupPPB(M_VDC_ADC_BASE, M_VDC_ADC_PPB_NUM, M_VDC_ADC_SOC_NUM);

    // Write zero to this for now till offset ISR is run
    ADC_setPPBCalibrationOffset(M_VDC_ADC_BASE, M_VDC_ADC_PPB_NUM, 0);

    return;
}

//
// setup CLA
//
void HAL_setupCLA(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    EALLOW;

#ifdef _FLASH
    //
    // Copy CLA code from its load address (FLASH) to CLA program RAM
    //
    // Note: during debug the load and run addresses can be
    // the same as Code Composer Studio can load the CLA program
    // RAM directly.
    //
    // The ClafuncsLoadStart, ClafuncsLoadEnd, and ClafuncsRunStart
    // symbols are created by the linker.
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
            (uint32_t)&Cla1funcsLoadSize);

    memcpy((uint32_t *)&Cla1ConstRunStart, (uint32_t *)&Cla1ConstLoadStart,
            (uint32_t)&Cla1ConstLoadSize);
#endif //_FLASH

    // make sure QEP access is given to CLA as Secondary master
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL2_EQEP, 1, SYSCTL_CPUSEL_CPU2);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, SYSCTL_ACCESS_CLA1,
                                      SYSCTL_ACCESS_FULL);

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
    while(MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU) != 1);

    // Initialize and wait for CPUToCLA1MsgRAM

    MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
    while(MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1) != 1);

    // Select LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS5 and then
    // set the space to be a program block

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_PROGRAM);

    // Next configure LS2RAM and LS3RAM as data spaces for the CLA
    // First configure the CLA to be the master and then
    // set the spaces to be code blocks
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
#pragma diag_suppress = 770
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_1, (uint16_t)(&Cla1Task1));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_2, (uint16_t)(&Cla1Task2));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_3, (uint16_t)(&Cla1Task3));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_4, (uint16_t)(&Cla1Task4));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_5, (uint16_t)(&Cla1Task5));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_6, (uint16_t)(&Cla1Task6));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_7, (uint16_t)(&Cla1Task7));
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_8, (uint16_t)(&Cla1Task8));
#pragma diag_suppress = 770

    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    CLA_enableIACK(obj->claHandle);
    CLA_enableTasks(obj->claHandle, CLA_TASKFLAG_ALL);

    // Enable EPWM1 INT trigger for CLA TASK1
    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_EPWM1INT);

    // Enable EPWM2 INT trigger for CLA TASK5
    CLA_setTriggerSource(CLA_TASK_5, CLA_TRIGGER_EPWM2INT);

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
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], 2048 + 1024);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], 2048 - 1024);

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

    DEVICE_DELAY_US(500);

    return;
}

//
// Setup OCP limits and digital filter parameters of CMPSS
//
void HAL_setupCMPSS_DACValue(HAL_Handle handle, uint16_t curHi, uint16_t curLo)
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
    ADC_setInterruptSource(M_IW_ADC_BASE,
                           ADC_INT_NUMBER1, M_IW_ADC_SOC_NUM);
    ADC_enableContinuousMode(M_IW_ADC_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(M_IW_ADC_BASE, ADC_INT_NUMBER1);

    return;
}

//
// setup CPU Timer
//
void HAL_setupCpuTimer(uint32_t base, uint32_t periodCount)
{
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);  // divide by 1 (SYSCLKOUT)
    CPUTimer_setPeriod(base, periodCount);
    CPUTimer_stopTimer(base);                // Stop timer / reload / restart
    CPUTimer_setEmulationMode(base,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_reloadTimerCounter(base);       // Reload counter with period value
    CPUTimer_resumeTimer(base);

    return;
}

#ifdef DACOUT_EN
//
// setup DAC
//
void HAL_setupDACs(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // DAC-A  ---> Resolver carrier excitation

    // Set DAC voltage reference to VRefHi
    DAC_setReferenceVoltage(obj->dacHandle[0], DAC_REF_ADC_VREFHI);

    // Set DAC shadow value register
    DAC_setShadowValue(obj->dacHandle[0], 1024);

    //Enable DAC output
    DAC_enableOutput(obj->dacHandle[0]);  // enable DACA

    //
    // Resolver carrier excitation signal additional initialization
    //
    // enable value change only on sync signal
    DAC_setLoadMode(obj->dacHandle[0], DAC_LOAD_PWMSYNC);

    // sync sel 5 means sync from pwm 6
    DAC_setPWMSyncSignal(obj->dacHandle[0], 5);

    return;
}
#endif // DACOUT_EN

//
// Sets up the GPIO (General Purpose I/O) pins
//
void HAL_setupGPIOs(HAL_Handle handle)
{

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

        // configure 2 and 3 as slaves
        EPWM_setSyncInPulseSource(obj->pwmHandle[cnt],
                                  EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    }

    EPWM_disablePhaseShiftLoad(obj->pwmHandle[0]);
    EPWM_setSyncInPulseSource(obj->pwmHandle[0],
                              EPWM_SYNC_IN_PULSE_SRC_DISABLE);

    halfPeriod = M_INV_PWM_TICKS / 2;     // 100MHz EPWMCLK

    EPWM_setPhaseShift(obj->pwmHandle[0], 0);
    EPWM_setPhaseShift(obj->pwmHandle[1], 2);
    EPWM_setPhaseShift(obj->pwmHandle[2], 4);

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
void HAL_setupMotorFaultProtection(HAL_Handle handle,
                                   const float32_t currentLimit)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    EPWM_DigitalCompareTripInput tripInSet = EPWM_DC_TRIP_TRIPIN4;

    // High and Low Compare event trips
    uint16_t curHi = 0;
    uint16_t curLo = 0;

    curHi = 2048 + scaleCurrentValue(currentLimit);
    curLo = 2048 - scaleCurrentValue(currentLimit);

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

        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt], EPWM_DC_MODULE_A,
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
        // EPWMxA and EPWMxB will go low
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
