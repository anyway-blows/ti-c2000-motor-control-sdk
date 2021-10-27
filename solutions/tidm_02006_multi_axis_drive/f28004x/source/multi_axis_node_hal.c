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
    obj->adcHandle[1] = ADCB_BASE;
    obj->adcHandle[2] = ADCC_BASE;

    //
    // initialize the ADC results
    //
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCBRESULT_BASE;
    obj->adcResult[2] = ADCCRESULT_BASE;

    //
    // initialize the DAC handles
    //
    obj->dacHandle[0] = DACA_BASE;
    obj->dacHandle[1] = DACB_BASE;

    //
    // initialize CLA handle
    //
    obj->claHandle = CLA1_BASE;

    //
    // initialize CLB handle
    //
    obj->clbHandle = CLB1_BASE;

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

    obj->pwmHandle[3] = M_SYNC1_PWM_BASE;    // Options for check
    obj->pwmHandle[4] = M_SYNC2_PWM_BASE;    // Options for check
    obj->pwmHandle[5] = M_SYNC3_PWM_BASE;    // Options for check

    //
    // initialize CMPSS handle
    //
    obj->cmpssHandle[0] = M_U_CMPSS_BASE;
    obj->cmpssHandle[1] = M_V_CMPSS_BASE;
    obj->cmpssHandle[2] = M_W_CMPSS_BASE;
    obj->cmpssHandle[3] = M_VDC_CMPSS_BASE;

    // initialize PGA handle
    obj->pgaHandle[0] = M_U_PGA_BASE;        //!< the PGA handle
    obj->pgaHandle[1] = M_V_PGA_BASE;        //!< the PGA handle
    obj->pgaHandle[2] = M_W_PGA_BASE;        //!< the PGA handle

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

    // Sets up the GPIO (General Purpose I/O) pins
    HAL_setupGPIOs(handle);

    // Sets up the CLA
    HAL_setupCLA(handle);

#ifdef CLB_PWM_SYNC
    // Sets up the CLB
    HAL_setupCLB(handle);
#endif  // CLB_PWM_SYNC

    // Timing sync for background loops
    HAL_setupCpuTimer(obj->timerHandle[0], MICROSEC_50);    // A tasks
    HAL_setupCpuTimer(obj->timerHandle[1], MICROSEC_100);   // B tasks
    HAL_setupCpuTimer(obj->timerHandle[2], MICROSEC_150);   // C tasks

    // Sets up the ADC
    HAL_setupADCs(handle);

    // setup the PGAs
    HAL_setupPGAs(handle);

    // setup the CMPSS
    HAL_setupCMPSS(handle);

#ifdef DACOUT_EN
    // Sets up the DAC
    HAL_setupDACs(handle);
#endif

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
    ADC_setVREF(obj->adcHandle[2], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    SysCtl_delay(100U);

    // setup ADC modules A, B
    for(cnt = 0; cnt < 3; cnt++)
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

//
// setup CLA
//
void HAL_setupCLA(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

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
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1,
                                      SYSCTL_ACCESS_CLA1,
                                      SYSCTL_ACCESS_FULL);

    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2,
                                      SYSCTL_ACCESS_CLA1,
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

    // Enable EPWM6 INT trigger for CLA TASK1
    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_EPWM6INT);

    // Enable EPWM1 INT trigger for CLA TASK5
    CLA_setTriggerSource(CLA_TASK_5, CLA_TRIGGER_EPWM1INT);

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
//    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN0, CLB_GLOBAL_IN_MUX_FSIRXA_PING_PACKET_RCVD);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN0, CLB_GLOBAL_IN_MUX_FSIRXA_DATA_PACKET_RCVD);

    // Unused Inputs below
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN1, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(obj->clbHandle, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

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

    // CLB output 12 is a copy of CLB output 4,
    // this output is used to trigger the FSI Tx module
    // CLB output 13 is a copy of CLB output 5,
    // this output will have delayed EPWMSYNCIN signal
    // CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_12 | CLB_OUTPUT_13, true);
    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_04 | CLB_OUTPUT_05, true);

    // GPIO for monitoring SYNCH
    XBAR_setOutputMuxConfig(XBAR_OUTPUT1, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT1, XBAR_MUX01);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_2_OUTPUTXBAR1);

    // GPIO for monitoring SYNCH with delay
    XBAR_setOutputMuxConfig(XBAR_OUTPUT2, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT2, XBAR_MUX03);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_3_OUTPUTXBAR2);

    // GPIO-3 is routed back to XBAR_INPUT-5 which is connected to EXTSYNCHIN1
    XBAR_setInputPin(XBAR_INPUT5, 3);

    // XBAR Config, CLB to PWM XBAR (CLB1_OUT4 to XBAR_TRIP4)
    // PWM XBAR to FSI has been hard coded at the start of this code.
    // For details on the connection, refer to Ext Frame Trigger Mux
    // section of the FSI chapter in the device TRM.
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX01_CLB1_OUT4);
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX01);

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
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3, 0);   //CMP3P_H, B2/Ia
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3, 0);   //CMP3P_L, B2/Ia

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 1);   //CMP1P_H, C0/Ib
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 1);   //CMP1P_L, C0/Ib

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_6, 3);   //CMP6P_H, A9/Ic
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_6, 3);   //CMP6P_L, A9/Ic

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_2, 3);   //CMP2P_H, A5/Vdc
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_2, 3);   //CMP2P_L, A5/Vdc

    for(cnt = 0; cnt < 4; cnt++)
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

    // Set DAC-H to allowed maximum voltage
    CMPSS_setDACValueHigh(obj->cmpssHandle[3], (4096 - 1024));

    // Set DAC-L to allowed minimum voltage
    CMPSS_setDACValueLow(obj->cmpssHandle[3], (0 + 10));

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
// setup DAC
//
void HAL_setupDACs(HAL_Handle handle)
{

    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // DAC-A  ---> Resolver carrier excitation
    // DAC-B  ---> General purpose display

    uint16_t cnt;

    for(cnt = 0; cnt < 2; cnt++)
    {
        // Set the DAC gain to 2
        DAC_setGainMode(obj->dacHandle[cnt], DAC_GAIN_TWO);

        // Set DAC voltage reference to VRefHi
        DAC_setReferenceVoltage(obj->dacHandle[cnt], DAC_REF_ADC_VREFHI);

        // enable value change only on sync signal
        DAC_setLoadMode(obj->dacHandle[cnt], DAC_LOAD_SYSCLK);

        //Enable DAC output
        DAC_enableOutput(obj->dacHandle[cnt]);

        // Set the DAC Shadow Output Value
        // Set the initial value to half of ADC range for 1.65V output
        DAC_setShadowValue(obj->dacHandle[cnt], 2048U);
    }

    return;
}

void HAL_setupPGAs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    // sets up PGA
    for(cnt = 0; cnt < 3; cnt++)
    {
        // Set a gain of 12 to PGA1/3/5
        PGA_setGain(obj->pgaHandle[cnt], PGA_GAIN_3);

        // No filter resistor for output
        PGA_setFilterResistor(obj->pgaHandle[cnt], PGA_LOW_PASS_FILTER_DISABLED);

        // Enable PGA1/3/5
        PGA_enable(obj->pgaHandle[cnt]);
    }

    return;
} // end of HAL_setupPGAs() function

//
// setup FSI
//
void HAL_setupFSI(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    FSI_disableRxInternalLoopback(obj->fsiRxHandle);

    // 50Mhz FSI clock frequency
    FSI_performTxInitialization(obj->fsiTxHandle, PRESCALER_VAL);
    FSI_performRxInitialization(obj->fsiRxHandle);

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
    Interrupt_register(INT_FSITXA_INT1, &fsiTxInt1ISR);
    Interrupt_register(INT_FSITXA_INT2, &fsiTxInt2ISR);
    Interrupt_register(INT_FSIRXA_INT1, &fsiRxInt1ISR);
    Interrupt_register(INT_FSIRXA_INT2, &fsiRxInt2ISR);

    //
    // Enable FSI Tx/Rx interrupts
    //
    Interrupt_enable(INT_FSITXA_INT1);
    Interrupt_enable(INT_FSITXA_INT2);
    Interrupt_enable(INT_FSIRXA_INT1);
    Interrupt_enable(INT_FSIRXA_INT2);


    // clear pending INT event
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    return;
}


//
// Sets up the GPIO (General Purpose I/O) pins
//
void HAL_setupGPIOs(HAL_Handle handle)
{

#ifdef _LAUNCHXL_F280049C
    // GPIO0->GPIO0/PWM_SYNC_T1
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->GPIO1/PWM_SYNC_T2
    GPIO_setMasterCore(1, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->GPIO2/
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_GPIO2);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->GPIO3/Reserve
    GPIO_setMasterCore(3, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->WH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // EPWM3B->WL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(5, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->FSI_TX0
    GPIO_setMasterCore(6, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_6_FSITXA_D0);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->TXCLK
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_FSITXA_CLK);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // EPWM5A->VH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(8, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_8_EPWM5_A);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // EPWM5B->VL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_EPWM5_B);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // EPWM6A->UH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(10, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // EPWM6B->UL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_EPWM6_B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // GPIO12->FSI_RX0
    GPIO_setMasterCore(12, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_FSIRXA_D0);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->GPIO13/TEST_MLOOP
    GPIO_setMasterCore(13, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_writePin(13, 0);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->GPIO14/Reserve
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // GPIO15->GPIO15/Reserve
    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->GPIO16/TEST_FSIRX
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_writePin(16, 0);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->GPIO17/TEST_FSITX
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_writePin(17, 0);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->X2 (N/A for GPIO)
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO22->GPIO22/Reserve
    GPIO_setMasterCore(22, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->GPIO23->LaunchPad LED5
    GPIO_setMasterCore(23, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->GPIO24/Reserve
    GPIO_setMasterCore(24, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->GPIO25->TEST_MISR
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_writePin(25, 0);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // GPIO26->GPIO26/Reserve
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);

    // GPIO27->GPIO27/Reserve
    GPIO_setMasterCore(27, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIRXDA
    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_28_SCIRXDA);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->SCITXDA
    GPIO_setMasterCore(29, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_29_SCITXDA);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->GPIO30->TEST_RXCMD
    GPIO_setMasterCore(30, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_writePin(30, 0);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // GPIO31->GPIO31/Reserve
    GPIO_setMasterCore(31, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

    // GPIO32->GPIO32/Reserve
    GPIO_setMasterCore(32, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

    // GPIO33->GPIO33/Reserve
    GPIO_setMasterCore(33, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->GPIO34->LaunchPad LED5
    GPIO_setMasterCore(34, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 0);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->EQEP1A
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_35_EQEP1A);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_PULLUP);

    // GPIO37->EQEP1B
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_37_EQEP1B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_PULLUP);

    // GPIO39->GPIO39->nEN_uC/SITE_1
    GPIO_setMasterCore(39, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 0);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->GPIO40/TEST_RXCMD
    GPIO_setMasterCore(40, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_writePin(40, 0);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->GPIO41/Reserve
    GPIO_setMasterCore(41, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->GPIO42/Reserve
    GPIO_setMasterCore(42, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO43->GPIO43/Reserve
    GPIO_setMasterCore(43, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->GPIO44/Reserve
    GPIO_setMasterCore(44, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->GPIO45/Reserve
    GPIO_setMasterCore(45, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->GPIO46/Reserve
    GPIO_setMasterCore(46, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);

    // GPIO47->GPIO47/Reserve
    GPIO_setMasterCore(47, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_47_GPIO47);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(47, GPIO_PIN_TYPE_STD);

    // GPIO48->GPIO48/Reserve
    GPIO_setMasterCore(48, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_48_GPIO48);
    GPIO_setDirectionMode(48, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(48, GPIO_PIN_TYPE_STD);

    // GPIO49->GPIO49/Reserve
    GPIO_setMasterCore(49, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(49, GPIO_PIN_TYPE_STD);

    // GPIO50->GPIO50/Reserve
    GPIO_setMasterCore(50, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_50_GPIO50);
    GPIO_setDirectionMode(50, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(50, GPIO_PIN_TYPE_STD);

    // GPIO51->GPIO51/Reserve
    GPIO_setMasterCore(51, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_51_GPIO51);
    GPIO_setDirectionMode(51, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(51, GPIO_PIN_TYPE_STD);

    // GPIO52->GPIO52/Reserve
    GPIO_setMasterCore(52, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_52_GPIO52);
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);

    // GPIO53->GPIO53/Reserve
    GPIO_setMasterCore(53, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_53_GPIO53);
    GPIO_setDirectionMode(53, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(53, GPIO_PIN_TYPE_STD);

    // GPIO54->GPIO54/Reserve
    GPIO_setMasterCore(54, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_54_GPIO54);
    GPIO_setDirectionMode(54, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_STD);

    // GPIO55->GPIO55/Reserve
    GPIO_setMasterCore(55, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_55_GPIO55);
    GPIO_setDirectionMode(55, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_STD);

    // GPIO56->GPIO56->TEST_TXCMD
    GPIO_setMasterCore(56, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_56_GPIO56);
    GPIO_writePin(56, 0);
    GPIO_setDirectionMode(56, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_STD);

    // GPIO57->GPIO57->TEST_MOTOR
    GPIO_setMasterCore(57, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_57_GPIO57);
    GPIO_writePin(57, 0);
    GPIO_setDirectionMode(57, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_STD);

    // GPIO58->GPIO58->OT/SITE_1
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_GPIO58);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_INVERT);

    // GPIO59->EQEP1I
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_EQEP1I);
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
#else // _TMDSCNCD_F280049C
    // EPWM1A->UH for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // EPWM1B->UL for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(1, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // EPWM2A->WH for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // EPWM2B->WL for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(3, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->WH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // EPWM3B->WL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(5, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // EPWM4A->VH for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(6, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // EPWM4B->VL for J5-J7/J6-J8 Connection
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_EPWM4B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // EPWM5A->VH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(8, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // EPWM5B->VL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_EPWM5B);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // EPWM6A->UH for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(10, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // EPWM6B->UL for J1-J3/J2-J4 Connection
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_EPWM6B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // GPIO12->GPIO12 for TEST_PIN
    GPIO_setMasterCore(12, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->GPIO13/Reserve
    GPIO_setMasterCore(13, GPIO_CORE_CPU1);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->EQEP2A
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_EQEP2A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_PULLUP);

    // GPIO15->EQEP2B
    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_EQEP2B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);

    // GPIO16->GPIO16/Reserve
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->GPIO17/Reserve
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->X2 (N/A for GPIO)
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->GPIO19/Reserve
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22->GPIO22/Reserve
    GPIO_setMasterCore(22, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->GPIO23->LaunchPad LED5
    GPIO_setMasterCore(23, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->GPIO24/Reserve
    GPIO_setMasterCore(24, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->GPIO25->TEST_PIN
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_writePin(25, 0);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // GPIO26->EQEP2I
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_EQEP2I);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);

    // GPIO27->GPIO27/Reserve
    GPIO_setMasterCore(27, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIRXDA
    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_28_SCIRXDA);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->SCITXDA
    GPIO_setMasterCore(29, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_29_SCITXDA);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->GPIO30/Reserve
    GPIO_setMasterCore(30, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // GPIO31->GPIO31/Reserve
    GPIO_setMasterCore(31, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

    // GPIO32->GPIO32, Use a wire connect to J6-24(GPIO26_BP)->OT/SITE_1
    GPIO_setMasterCore(32, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

    // GPIO33->GPIO33->nEN_uC/SITE_2
    GPIO_setMasterCore(33, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_writePin(33, 0);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->GPIO34->LaunchPad LED5
    GPIO_setMasterCore(34, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 0);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->EQEP1A
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_35_EQEP1A);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_PULLUP);

    // GPIO37->EQEP1B
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_37_EQEP1B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_PULLUP);

    // GPIO39->GPIO39->nEN_uC/SITE_1
    GPIO_setMasterCore(39, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 0);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->GPIO40/Reserve
    GPIO_setMasterCore(40, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->GPIO41/Reserve
    GPIO_setMasterCore(41, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->GPIO42/Reserve
    GPIO_setMasterCore(42, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO43->GPIO43/Reserve
    GPIO_setMasterCore(43, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->GPIO44/Reserve
    GPIO_setMasterCore(44, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->GPIO45/Reserve
    GPIO_setMasterCore(45, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->GPIO46/Reserve
    GPIO_setMasterCore(46, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);

    // GPIO47->GPIO47/Reserve
    GPIO_setMasterCore(47, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_47_GPIO47);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(47, GPIO_PIN_TYPE_STD);

    // GPIO48->GPIO48/Reserve
    GPIO_setMasterCore(48, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_48_GPIO48);
    GPIO_setDirectionMode(48, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(48, GPIO_PIN_TYPE_STD);

    // GPIO49->GPIO49/Reserve
    GPIO_setMasterCore(49, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(49, GPIO_PIN_TYPE_STD);

    // GPIO50->GPIO50/Reserve
    GPIO_setMasterCore(50, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_50_GPIO50);
    GPIO_setDirectionMode(50, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(50, GPIO_PIN_TYPE_STD);

    // GPIO51->GPIO51/Reserve
    GPIO_setMasterCore(51, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_51_GPIO51);
    GPIO_setDirectionMode(51, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(51, GPIO_PIN_TYPE_STD);

    // GPIO52->GPIO52/Reserve
    GPIO_setMasterCore(52, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_52_GPIO52);
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);

    // GPIO53->GPIO53/Reserve
    GPIO_setMasterCore(53, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_53_GPIO53);
    GPIO_setDirectionMode(53, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(53, GPIO_PIN_TYPE_STD);

    // GPIO54->GPIO54/Reserve
    GPIO_setMasterCore(54, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_54_GPIO54);
    GPIO_setDirectionMode(54, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_STD);

    // GPIO55->GPIO55/Reserve
    GPIO_setMasterCore(55, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_55_GPIO55);
    GPIO_setDirectionMode(55, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_STD);

    // GPIO56->GPIO56/Reserve
    GPIO_setMasterCore(56, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_56_GPIO56);
    GPIO_setDirectionMode(56, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_STD);

    // GPIO57->GPIO57/Reserve
    GPIO_setMasterCore(57, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_57_GPIO57);
    GPIO_setDirectionMode(57, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_STD);

    // GPIO58->GPIO58->OT/SITE_1
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_GPIO58);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_INVERT);

    // GPIO59->EQEP1I
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_EQEP1I);
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_PULLUP);
#endif // _LAUNCHXL_F280049C/

    return;
}

// Sets up the PWMs (Pulse Width Modulators) for motor
void HAL_setupMotorPWMs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // *****************************************
    // Inverter PWM configuration - PWM 1, 2, 3
    // *****************************************
    for(cnt = 0; cnt < 6; cnt++)
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

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_C, 0);

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_D, 0);

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

        #if defined(CLB_PWM_SYNC) && (BUILDLEVEL >= FCL_LEVEL7)
                EPWM_enablePhaseShiftLoad(obj->pwmHandle[cnt]);
                EPWM_setSyncOutPulseMode(obj->pwmHandle[cnt],
                                         EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
        #else
            EPWM_disablePhaseShiftLoad(obj->pwmHandle[cnt]);

            EPWM_setSyncOutPulseMode(obj->pwmHandle[cnt],
                                     EPWM_SYNC_OUT_PULSE_DISABLED);
        #endif  // CLB_PWM_SYNC
    }

    // 100MHz EPWMCLK
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], M_INV_PWM_TBPRD);     //EPWM6
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], M_INV_PWM_TBPRD);     //EPWM5
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], M_INV_PWM_TBPRD);     //EPWM3
    EPWM_setTimeBasePeriod(obj->pwmHandle[3], M_INV_PWM_TBPRD);     //EPWM1
    EPWM_setTimeBasePeriod(obj->pwmHandle[4], M_INV_PWM_TBPRD);     //EPWM4
    EPWM_setTimeBasePeriod(obj->pwmHandle[5], M_INV_PWM_TBPRD);     //EPWM2

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

    // set SOC for pwm sync
    EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                EPWM_COUNTER_COMPARE_C, EPWM_SYNC_SHIFT_VALUE_NODES);

//    EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[M_SYNC_NUM],
//                                         EPWM_COUNTER_COMPARE_C);

    // Select SOC from counter at ctr = CMPC
    EPWM_setADCTriggerSource(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPC);

    // Generate pulse on 15th event
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                                    EPWM_SYNC_EVENT_COUNT_NODES);

    EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                EPWM_COUNTER_COMPARE_C, EPWM_SYNC_SHIFT_VALUE_NODES);

    // clear flag
    EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

    // Enable SOC on B group
    EPWM_enableADCTrigger(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

#if defined(CLB_PWM_SYNC) && (BUILDLEVEL >= FCL_LEVEL7)
    // setup the Timer-Based Phase Register (TBPHS)
    EPWM_setPhaseShift(obj->pwmHandle[0], EPWM_SYNC_SHIFT_U_NODES);
    EPWM_setPhaseShift(obj->pwmHandle[1], EPWM_SYNC_SHIFT_V_NODES);
    EPWM_setPhaseShift(obj->pwmHandle[2], EPWM_SYNC_SHIFT_W_NODES);

    // Set ePWMs for PWM Sync
    EPWM_setCounterCompareValue(obj->pwmHandle[3],
                                EPWM_COUNTER_COMPARE_A, M_INV_PWM_HALF_TBPRD);

    EPWM_setCounterCompareValue(obj->pwmHandle[4],
                                EPWM_COUNTER_COMPARE_A, M_INV_PWM_HALF_TBPRD);

    EPWM_setCounterCompareValue(obj->pwmHandle[5],
                                EPWM_COUNTER_COMPARE_A, M_INV_PWM_HALF_TBPRD);

    // Set GPIO and XBAR for PWM Sync
    // EXTSYNCHIN1 configured to be routed from CLB compensated output
    // GPIO-3 -> INPUTXBAR.INPUT5 -> EXTSYNCHIN1
    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM1,
                              SYSCTL_SYNC_IN_SRC_EXTSYNCIN1);

    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4,
                              SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
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

    EPWM_DigitalCompareTripInput tripInSet = EPWM_DC_TRIP_TRIPIN5;

    // High and Low Compare event trips
    uint16_t curHi = 0;
    uint16_t curLo = 0;

    curHi = 2048 + scaleCurrentValue(currentLimit);
    curLo = 2048 - scaleCurrentValue(currentLimit);

    // Configure TRIP 5 to OR the High and Low trips from both
    // comparator 1 & 3, clear everything first
    EALLOW;
    HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP5MUX0TO15CFG) = 0;
    HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP5MUX16TO31CFG) = 0;
    EDIS;

    //Select GPIO58 as INPUTXBAR3
    XBAR_setInputPin(XBAR_INPUT3, M_XBAR_INPUT_GPIO);
    XBAR_lockInput(XBAR_INPUT3);

    // Enable Muxes for ored input of CMPSS1H and 1L, mux for Mux0x
    //cmpss3 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L);

    //cmpss1 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);

    //cmpss6 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX10_CMPSS6_CTRIPH_OR_L);

    //cmpss2 - tripH or tripL
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L);

    //inputxbar1 trip
    XBAR_setEPWMMuxConfig(XBAR_TRIP5, XBAR_EPWM_MUX05_INPUTXBAR3);

    // Disable all the muxes first
    XBAR_disableEPWMMux(XBAR_TRIP5, 0xFFFF);

    // Enable Mux 0, Mux 1 or Mux 4 to generate TRIP4
#if(BUILDLEVEL == FCL_LEVEL7)
    XBAR_enableEPWMMux(XBAR_TRIP5, XBAR_MUX04 | XBAR_MUX00 | XBAR_MUX10 |
                                   XBAR_MUX05);
#else
    XBAR_enableEPWMMux(XBAR_TRIP5, XBAR_MUX04 | XBAR_MUX00 | XBAR_MUX10 |
                                   XBAR_MUX02 | XBAR_MUX05);
#endif  //(BUILDLEVEL != FCL_LEVEL7)

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

        // clear any spurious  OST & DCAEVT1 flags
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], (EPWM_TZ_FLAG_OST |
                                                     EPWM_TZ_FLAG_DCAEVT1 |
                                                     EPWM_TZ_FLAG_CBC ));
    }

    // clear EPWM trip flags
    DEVICE_DELAY_US(1L);

    for(cnt = 0; cnt < 4; cnt++)
    {
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
