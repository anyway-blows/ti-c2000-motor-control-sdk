//#############################################################################
//
// FILE:    nulti_axis_lead_hal_cpu1.c
//
// TITLE:   define initialize the handle functions of device
//
// Group:   C2000
//
// Target Family: F2838x
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
#include "multi_axis_lead_ctrl_main.h"

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
    Interrupt_enable(M_CISR_INT_PWM);

    //
    // Enable group 3 interrupts - EPWM6 is here
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
    obj->dacHandle[0] = DACB_BASE;
    obj->dacHandle[1] = DACC_BASE;

    //
    // initialize CLA handle
    //
    obj->claHandle = CLA1_BASE;

    //
    // initialize SCI handle
    //
    obj->sciHandle[0] = SCIA_BASE;
    obj->sciHandle[1] = SCIB_BASE;

    //
    // initialize timer handles
    //
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    //
    // initialize PWM handles for motor_1
    //
    obj->pwmHandle[0] = M_SYNC_PWM_BASE;
    obj->pwmHandle[1] = M_SYNT_PWM_BASE;
    obj->pwmHandle[2] = M_CISR_PWM_BASE;

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

    //
    // Sets up the GPIO (General Purpose I/O) pins
    //
    HAL_setupGPIOs(handle);

#ifdef DACOUT_EN
    //
    // Sets up the DAC
    //
    HAL_setupDACs(handle);
#endif  //DACOUT_EN

    //
    // setup the PWMs
    //
    HAL_setupCtrlPWMs(handle);

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

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU);
    while(MemCfg_getInitStatus(MEMCFG_SECT_MSGCLA1TOCPU) != 1);

    // Initialize and wait for CPUToCLA1MsgRAM

    MemCfg_initSections(MEMCFG_SECT_MSGCPUTOCLA1);
    while(MemCfg_getInitStatus(MEMCFG_SECT_MSGCPUTOCLA1) != 1);

    // Select LS0RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS0 and then
    // set the space to be a program block

    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_PROGRAM);

    // Next configure LS1RAM as data spaces for the CLA
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

    // Enable EPWM7 INT trigger for CLA TASK5
    CLA_setTriggerSource(CLA_TASK_5, CLA_TRIGGER_EPWM7INT);

    EDIS;

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
    EPWM_setInterruptSource(obj->pwmHandle[M_CISR_NUM],
                            EPWM_INT_TBCTR_ZERO);
    #elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
    // Select INT @ ctr = 0 or ctr = prd
    EPWM_setInterruptSource(obj->pwmHandle[M_CISR_NUM],
                            EPWM_INT_TBCTR_ZERO_OR_PERIOD);
    #endif

    // Enable Interrupt Generation from the PWM module
    EPWM_enableInterrupt(obj->pwmHandle[M_CISR_NUM]);

    // This needs to be 1 for the INTFRC to work
    EPWM_setInterruptEventCount(obj->pwmHandle[M_CISR_NUM], 1);

    // Clear ePWM Interrupt flag
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[M_CISR_NUM]);

    Interrupt_register(M_CISR_INT_PWM, &motorControlISR);

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

    //
    // NOTE: External loopback, Modify GPIO settings as per setup
    //
    GPIO_setPinConfig(GPIO_CFG_FSI_TXCLK);
    GPIO_setPinConfig(GPIO_CFG_FSI_TX0);

    GPIO_setPinConfig(GPIO_CFG_FSI_RXCLK);
    GPIO_setPinConfig(GPIO_CFG_FSI_RX0);

    //
    // Set RX GPIO to be asynchronous (pass through without delay)
    // Default setting is to have 2 SYS_CLK cycles delay
    //
    GPIO_setQualificationMode(GPIO_PIN_FSI_RX0, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(GPIO_PIN_FSI_RXCLK, GPIO_QUAL_ASYNC);

    //
    // Could add logic to calculate PRESCALER_VAL based on user input FSI CLK
    //
    FSI_performRxInitialization(obj->fsiRxHandle);
    FSI_performTxInitialization(obj->fsiTxHandle, PRESCALER_VAL);

    // Clear FSI RX&TX data
    for(ni = 0; ni < 16; ni++)
    {
        HWREGH(obj->fsiRxHandle + FSI_O_RX_BUF_BASE(ni)) = 0x000;
        HWREGH(obj->fsiTxHandle + FSI_O_TX_BUF_BASE(ni)) = 0x000;
    }

    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_FRAME_CTRL);

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

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

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
    // DAC-B  ---> General purpose display
    // DAC-C  ---> General purpose display
    //

    // Set DAC voltage reference to VRefHi
    DAC_setReferenceVoltage(obj->dacHandle[0], DAC_REF_ADC_VREFHI);

    // Set DAC shadow value register
    DAC_setShadowValue(obj->dacHandle[0], 1024);

    //Enable DAC output
    DAC_enableOutput(obj->dacHandle[0]);

    // Set DAC voltage reference to VRefHi
    DAC_setReferenceVoltage(obj->dacHandle[1], DAC_REF_ADC_VREFHI);

    // Set DAC shadow value register
    DAC_setShadowValue(obj->dacHandle[1], 1024);

    //Enable DAC output
    DAC_enableOutput(obj->dacHandle[1]);

    return;
}
#endif // DACOUT_EN

//
// Sets up the GPIO (General Purpose I/O) pins
//

void HAL_setupGPIOs(HAL_Handle handle)
{
    // GPIO0->EPWM1A->UH_M
    GPIO_setMasterCore(0, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->UL_M
    GPIO_setMasterCore(1, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->VH_M
    GPIO_setMasterCore(2, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->VL_M
    GPIO_setMasterCore(3, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->WH_M
    GPIO_setMasterCore(4, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->EPWM3B->WL_M
    GPIO_setMasterCore(5, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->GPIO6/PWM_SYNC_T1->CC_PIN54
    GPIO_setMasterCore(6, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->GPIO7/PWM_SYNC_T2->CC_PIN56
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_EPWM4B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->GPIO8->FSI_RXD0A
    GPIO_setMasterCore(8, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_8_FSIRXA_D0);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO9->GPIO9->FSI_RXDCLKA
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_FSIRXA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // GPIO10->GPIO10/Reserve
    GPIO_setMasterCore(10, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // GPIO11->GPIO11/Reserve
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // GPIO12->GPIO12/PWM_SYNC_T3->CC_PIN58
    GPIO_setMasterCore(12, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_EPWM7A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->GPIO13/PWM_SYNC_T4->CC_PIN60
    GPIO_setMasterCore(13, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_13_EPWM7B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->GPIO14/PWM_SYNC_T5->CC_PIN62
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_EPWM8A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // GPIO15->GPIO15/PWM_SYNC_T5->CC_PIN64
    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_EPWM8B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->GPIO16/Reserve
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->GPIO17/Reserve
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->GPIO18/Reserve
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->GPIO19/Reserve
    GPIO_setMasterCore(19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // Setup GPIO for QEP operation
    // GPIO20->QEP1A_M
    GPIO_setMasterCore(20, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_20_EQEP1_A);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(20, GPIO_QUAL_3SAMPLE);

    // GPIO21->QEP1B_M
    GPIO_setMasterCore(21, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_21_EQEP1_B);
    GPIO_setDirectionMode(21, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(21, GPIO_QUAL_3SAMPLE);

    // GPIO22->GPIO22
    GPIO_setMasterCore(22, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // GPIO23->QEP1I_M
    GPIO_setMasterCore(23, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_23_EQEP1_INDEX);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);

    // GPIO24->GPIO24/Reserve
    GPIO_setMasterCore(24, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->GPIO25/Reserve
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // GPIO26->GPIO26/Reserve
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);

    // GPIO27->GPIO27/Reserve
    GPIO_setMasterCore(27, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIRXDA
    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->SCITXDA
    GPIO_setMasterCore(29, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->GPIO30/Reserve
    GPIO_setMasterCore(30, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // GPIO31->LED2
    GPIO_setMasterCore(31, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_writePin(31, 1);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

    // GPIO32->GPIO32->TEST_TXCMD
    GPIO_setMasterCore(32, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 0);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

    // GPIO33->GPIO33
    GPIO_setMasterCore(33, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->LED1
    GPIO_setMasterCore(34, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 1);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->GPIO35
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

    // GPIO36->GPIO36
    GPIO_setMasterCore(36, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_36_GPIO36);
    GPIO_setPadConfig(36, GPIO_PIN_TYPE_STD);

    // GPIO37->GPIO37
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // GPIO38->GPIO38
    GPIO_setMasterCore(38, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_38_GPIO38);
    GPIO_setPadConfig(38, GPIO_PIN_TYPE_STD);

    // GPIO39->GPIO39->TEST_MOTORS
    GPIO_setMasterCore(39, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 0);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->GPIO40->Tz
    GPIO_setMasterCore(40, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_INVERT); // invert the input

    // GPIO41->GPIO41->Clr_Fault
    GPIO_setMasterCore(41, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_writePin(41, 0);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->GPIO42->
    GPIO_setMasterCore(42, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO43->GPIO43
    GPIO_setMasterCore(43, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->GPIO44
    GPIO_setMasterCore(44, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->GPIO45->TEST_MOTOR1
    GPIO_setMasterCore(45, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_writePin(45, 0);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->GPIO46
    GPIO_setMasterCore(46, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);

    // GPIO47->GPIO47
    GPIO_setMasterCore(47, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_47_GPIO47);
    GPIO_setPadConfig(47, GPIO_PIN_TYPE_STD);

    // GPIO48->GPIO48
    GPIO_setMasterCore(48, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_48_GPIO48);
    GPIO_setPadConfig(48, GPIO_PIN_TYPE_STD);

    // GPIO49->GPIO49
    GPIO_setMasterCore(49, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setPadConfig(49, GPIO_PIN_TYPE_STD);

    // GPIO50->GPIO50
    GPIO_setMasterCore(50, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_50_GPIO50);
    GPIO_setPadConfig(50, GPIO_PIN_TYPE_STD);

    // GPIO51->GPIO51
    GPIO_setMasterCore(51, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_51_GPIO51);
    GPIO_setPadConfig(51, GPIO_PIN_TYPE_STD);

    // GPIO52->GPIO52
    GPIO_setMasterCore(52, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_52_GPIO52);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);

    // GPIO53->GPIO53
    GPIO_setMasterCore(53, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_53_GPIO53);
    GPIO_setPadConfig(53, GPIO_PIN_TYPE_STD);

    // GPIO54->GPIO54/TEST_FSIRX
    GPIO_setMasterCore(54, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_54_GPIO54);
    GPIO_writePin(54, 0);
    GPIO_setDirectionMode(54, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_STD);

    // GPIO55->GPIO55/TEST_FSITX
    GPIO_setMasterCore(55, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_55_GPIO55);
    GPIO_writePin(55, 0);
    GPIO_setDirectionMode(55, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_STD);

    // GPIO56->GPIO56/TEST_MLOOP
    GPIO_setMasterCore(56, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_56_GPIO56);
    GPIO_writePin(56, 0);
    GPIO_setDirectionMode(56, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_STD);

    // GPIO57->GPIO57
    GPIO_setMasterCore(57, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_57_GPIO57);
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_STD);

    // GPIO58->GPIO58->TripCC1
    GPIO_setMasterCore(58, GPIO_CORE_CPU2);
    GPIO_setPinConfig(GPIO_58_GPIO58);
    GPIO_writePin(58, 0);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);

    // GPIO59->GPIO59
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_GPIO59);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_STD);

    // GPIO60->GPIO60
    GPIO_setMasterCore(60, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_60_GPIO60);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_STD);

    // GPIO61->GPIO61
    GPIO_setMasterCore(61, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_61_GPIO61);
    GPIO_setPadConfig(61, GPIO_PIN_TYPE_STD);

    // GPIO62->GPIO62
    GPIO_setMasterCore(62, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_62_GPIO62);
    GPIO_setPadConfig(62, GPIO_PIN_TYPE_STD);

    // GPIO63->GPIO63
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_GPIO63);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_STD);

    // GPIO64->GPIO64
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_64_GPIO64);
    GPIO_setPadConfig(64, GPIO_PIN_TYPE_STD);

    // GPIO65->GPIO64
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_65_GPIO65);
    GPIO_setPadConfig(65, GPIO_PIN_TYPE_STD);

    // GPIO66->GPIO66
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_66_GPIO66);
    GPIO_setPadConfig(66, GPIO_PIN_TYPE_STD);

    // GPIO94->GPIO94
    GPIO_setMasterCore(94, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_94_GPIO94);
    GPIO_setPadConfig(94, GPIO_PIN_TYPE_STD);

    // GPIO99->GPIO99
    GPIO_setMasterCore(99, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_99_GPIO99);
    GPIO_setPadConfig(99, GPIO_PIN_TYPE_STD);

    // GPIO111->GPIO111
    GPIO_setMasterCore(111, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_111_GPIO111);
    GPIO_setPadConfig(111, GPIO_PIN_TYPE_STD);

    // GPIO124->GPIO124
    GPIO_setMasterCore(124, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_124_GPIO124);
    GPIO_setPadConfig(124, GPIO_PIN_TYPE_STD);

    // GPIO125->GPIO125/TEST_MISR
    GPIO_setMasterCore(125, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_125_GPIO125);
    GPIO_writePin(125, 0);
    GPIO_setDirectionMode(125, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(125, GPIO_PIN_TYPE_STD);

    // GPIO139->GPIO139
    GPIO_setMasterCore(139, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_139_GPIO139);
    GPIO_setPadConfig(139, GPIO_PIN_TYPE_STD);

    // GPIO153->GPIO153
    GPIO_setMasterCore(153, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_153_GPIO153);
    GPIO_setPadConfig(153, GPIO_PIN_TYPE_STD);

    // GPIO154->GPIO154
    GPIO_setMasterCore(154, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_154_GPIO154);
    GPIO_setPadConfig(154, GPIO_PIN_TYPE_STD);

    // GPIO155->GPIO155
    GPIO_setMasterCore(155, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_155_GPIO155);
    GPIO_setPadConfig(155, GPIO_PIN_TYPE_STD);

    // GPIO156->GPIO156
    GPIO_setMasterCore(156, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_156_GPIO156);
    GPIO_setPadConfig(156, GPIO_PIN_TYPE_STD);

    // GPIO157->GPIO157
    GPIO_setMasterCore(157, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_157_GPIO157);
    GPIO_setPadConfig(157, GPIO_PIN_TYPE_STD);

    // GPIO158->GPIO158
    GPIO_setMasterCore(158, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_158_GPIO158);
    GPIO_setPadConfig(158, GPIO_PIN_TYPE_STD);

    // GPIO159->GPIO159
    GPIO_setMasterCore(159, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_159_GPIO159);
    GPIO_setPadConfig(159, GPIO_PIN_TYPE_STD);

    // GPIO160->GPIO160
    GPIO_setMasterCore(160, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_160_GPIO160);
    GPIO_setPadConfig(160, GPIO_PIN_TYPE_STD);

    return;
}

// Sets up the PWMs (Pulse Width Modulators)
void HAL_setupCtrlPWMs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

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
                                    EPWM_COUNTER_COMPARE_A, M_INV_PWM_HALF_TBPRD);

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_B, M_INV_PWM_HALF_TBPRD);

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_C, M_INV_PWM_HALF_TBPRD);

        EPWM_setCounterCompareValue(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_COMPARE_D, M_INV_PWM_HALF_TBPRD);

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

        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);
        // configure sync
        EPWM_enableSyncOutPulseSource(obj->pwmHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);

        // configure all ePWMs as slaves
        EPWM_setSyncInPulseSource(obj->pwmHandle[cnt],
                                  EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1);
    }

    EPWM_setTimeBasePeriod(obj->pwmHandle[M_SYNC_NUM], M_INV_PWM_TBPRD);
    EPWM_setTimeBasePeriod(obj->pwmHandle[M_SYNT_NUM], M_INV_PWM_TBPRD);

    EPWM_setTimeBasePeriod(obj->pwmHandle[M_CISR_NUM], M_CTRL_PWM_TBPRD);

    EPWM_setCounterCompareValue(obj->pwmHandle[M_CISR_NUM],
                                EPWM_COUNTER_COMPARE_A, M_CTRL_PWM_HALF_TBPRD);

    // Setting up link from EPWM to ADC
    // EPWM1/EPWM4 - Inverter currents at sampling frequency
    //               (@ PRD or @ (PRD&ZRO) )
#if(SAMPLING_METHOD == SINGLE_SAMPLING)
    // Select SOC from counter at ctr = prd
    EPWM_setADCTriggerSource(obj->pwmHandle[M_CISR_NUM],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
    // Select SOC from counter at ctr = 0 or ctr = prd
    EPWM_setADCTriggerSource(obj->pwmHandle[M_CISR_NUM], EPWM_SOC_A,
                             EPWM_SOC_TBCTR_ZERO_OR_PERIOD);
#endif

    // Generate pulse on 1st event
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_CISR_NUM], EPWM_SOC_A, 1);

    // Enable SOC on A group
    EPWM_enableADCTrigger(obj->pwmHandle[M_CISR_NUM], EPWM_SOC_A);

    // set SOC for pwm sync
    EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                EPWM_COUNTER_COMPARE_C, EPWM_SYNC_SHIFT_VALUE);

    // Select SOC from counter at ctr = prd
    EPWM_setADCTriggerSource(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_A,
                             EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerSource(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPC);

    // Generate pulse on 15th event
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_A,
                                    EPWM_SYNC_EVENT_COUNT);
    EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B,
                                    EPWM_SYNC_EVENT_COUNT);

    // clear flag
    EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

    // Enable SOC on A&B group
    EPWM_enableADCTrigger(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_A);
    EPWM_enableADCTrigger(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);


    return;
}

//
// Configure Motor Fault Protection Against Over Current
//
void HAL_setupMotorFaultXBAR(HAL_Handle handle)
{

    //Select GPIO40 as INPUTXBAR1
    XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT1, M_XBAR_INPUT_GPIO);
    XBAR_lockInput(INPUTXBAR_BASE, XBAR_INPUT1);

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

    //inputxbar2 trip
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX01_INPUTXBAR1);

    // Disable all the muxes first
    XBAR_disableEPWMMux(XBAR_TRIP4, 0xFFFF);


    // Enable Mux 0  OR Mux 4 to generate TRIP4
#if(BUILDLEVEL == FCL_LEVEL5)       // Enable FSI
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX01);
#else
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX00 | XBAR_MUX00 | XBAR_MUX04 |
                                   XBAR_MUX01);
#endif  //BUILDLEVEL != FCL_LEVEL5

    //
    // Transfer ownership of motor drive peripherals to CPU02
    //
    // EPWM1, EPWM2, EPWM3
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM, 1, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM, 2, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM, 3, SYSCTL_CPUSEL_CPU2);

    // EQEP1
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL2_EQEP, 1, SYSCTL_CPUSEL_CPU2);

    // SPIA
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL6_SPI, 1, SYSCTL_CPUSEL_CPU2);

    // ADCA, ADCB, ADCC, ADCD
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 1, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 2, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 3, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 4, SYSCTL_CPUSEL_CPU2);

    // CMPSS1, CMPSS3
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL12_CMPSS, 1, SYSCTL_CPUSEL_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL12_CMPSS, 3, SYSCTL_CPUSEL_CPU2);

    // DACA
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL14_DAC, 1, SYSCTL_CPUSEL_CPU2);

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
