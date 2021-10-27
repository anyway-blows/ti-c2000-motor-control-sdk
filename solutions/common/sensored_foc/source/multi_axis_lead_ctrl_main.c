//#############################################################################
//
// FILE:    multi_axis_lead_main_cpu1.c
//
// TITLE:   multi-axis motor drive on the related kits
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
#include "motor_ctrl_settings.h"
#include "multi_axis_lead_ctrl_main.h"

//
// Functions
//

//
// State Machine function prototypes
//
// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
void A3(void);  //state A3
void A4(void);  //state A4

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3
void B4(void);  //state B4

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch

uint16_t vTimer0[5] = {0};  // Virtual Timers slaved off CPU Timer 0 (A events)
uint16_t vTimer1[5] = {0};  // Virtual Timers slaved off CPU Timer 1 (B events)
uint16_t serialCommsTimer = 0;

//
// USER Variables
//

//
// flag variables
//
volatile uint16_t enableCtrlFlag = false;
uint16_t led1Cnt = 0;
uint32_t backTicker = 0;

//
// These are defined by the linker file
//
#ifdef _FLASH
#if(SPD_CNTLR == SPD_DCL_CNTLR)
extern uint16_t dclfuncsLoadStart;
extern uint16_t dclfuncsLoadSize;
extern uint16_t dclfuncsRunStart;
extern uint16_t dclfuncsRunSize;
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)
#endif  // _FLASH

#pragma DATA_SECTION(ipcCMToCPUDataBuffer, "MSGRAM_CM_TO_CPU_ECAT")
ECAT_IPC_GetDataBuffer ipcCMToCPUDataBuffer;

#pragma DATA_SECTION(ipcCPUToCMDataBuffer, "MSGRAM_CPU_TO_CM_ECAT")
ECAT_IPC_PutDataBuffer ipcCPUToCMDataBuffer;

ECAT_IPC_GetDataBuffer dataBufferFromCM;
ECAT_IPC_PutDataBuffer dataBufferToCM;

CtrlMode_e ctrlCmdNode[SYS_NODE_NUM];
CtrlMode_e ctrlCmdNodePrev[SYS_NODE_NUM];

// speed reference command
float32_t speedRefNode[SYS_NODE_NUM];

// position reference command
float32_t positionRefNode[SYS_NODE_NUM];

uint16_t delayNodeCnt[SYS_NODE_NUM];
uint16_t delayNodeSet[SYS_NODE_NUM];

CtrlSync_e ctrlSyn;
CtrlSync_e ctrlSynPrev;

CtrlMode_e ctrlCmdNodeM;
float32_t speedRefNodeM;
float32_t positionRefNodeM;

uint16_t delaySynCnt;
uint16_t delaySynSet;

#pragma DATA_SECTION(ipcCPU2ToCPU1Data, "MSGRAM_CPU2_TO_CPU1")
DRIVE_IPC_dataFromCPU2_t ipcCPU2ToCPU1Data;

#pragma DATA_SECTION(ipcCPU1ToCPU2Data, "MSGRAM_CPU1_TO_CPU2")
DRIVE_IPC_dataToCPU2_t   ipcCPU1ToCPU2Data;

DRIVE_IPC_dataFromCPU2_t dataFromCPU2;
DRIVE_IPC_dataToCPU2_t   dataToCPU2;

//
// CPU1 main() function enter
//
void main(void)
{
    // initialize device clock and peripherals
    Device_init();

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

    // Boot CPU2 core
#ifdef _FLASH
    Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
#endif

    // clear the memory for global uninitialization variables
    // !!BE CAREFUL TO DO THIS ACTION
    HAL_clearDataRAM((void *)RAMMS_START_ADDRESS, RAMMS_SIZE_LENGTH);
    HAL_clearDataRAM((void *)RAMLS_START_ADDRESS, RAMLS_SIZE_LENGTH);
    HAL_clearDataRAM((void *)RAMGS_START_ADDRESS, RAMGS_SIZE_LENGTH);

    // Give control of GS1, GS9 & to CPU2
    MemCfg_setGSRAMMasterSel(MEMCFG_SECT_GS1, MEMCFG_GSRAMMASTER_CPU2);
    MemCfg_setGSRAMMasterSel(MEMCFG_SECT_GS9, MEMCFG_GSRAMMASTER_CPU2);

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    HAL_setupMotorFaultXBAR(halHandle);

    #ifdef _FLASH
    #if(SPD_CNTLR == SPD_DCL_CNTLR)
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&dclfuncsRunStart, &dclfuncsLoadStart, (size_t)&dclfuncsLoadSize);
    #endif  // (SPD_CNTLR == SPD_DCL_CNTLR)
    #endif  // _FLASH

    // Send IPC to CPU2 telling it to proceed with configuring the Peripherals
    IPC_setFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG1);

    // PWM Clocks Enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // initialize system parameters
    initSysParameters(&sysVars);

    {
        uint16_t nodes;

        for(nodes = SYS_NODEM; nodes< SYS_NODE_NUM; nodes++)
        {
            // initialize controller parameters for each motor
            initCtrlParameters(&ctrlVars[nodes]);

            // reset some controller variables for each motor
            resetControllerVars(&ctrlVars[nodes]);
        }
    }

    // Send IPC to CPU2 telling it to proceed with configuring the Peripherals
    IPC_setFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG2);

#if(BUILDLEVEL >= FCL_LEVEL5)       // Control Over FSI
    fsiRxTimeOutCntr = 0;
    fsiTxTimeOutCntr = 0;

    // initialize FSI module and DMA
    FSI_initParams(halHandle);
#endif  // ((BUILDLEVEL >= FCL_LEVEL4)

    // Clear LED counter
    led1Cnt = 0;
    GPIO_writePin(CCARD_GPIO_LED1, 0);   // LED Light

    #if(((BUILDLEVEL == FCL_LEVEL3) || (BUILDLEVEL == FCL_LEVEL4)) ||         \
        ((BUILDLEVEL == FCL_LEVEL10) || (BUILDLEVEL == FCL_LEVEL11)))
    //
    // Setup GPIOs for EtherCAT
    //
    ECAT_configureESCGPIOs();

    //
    // Setup CM clocks and release from reset
    // (On RAM, boots to RAM; On Flash, boots to Flash)
    //
    ECAT_configureAndReleaseCM();
    #endif  //(BUILDLEVEL == FCL_LEVEL3, 4, 10 11) that enable EtherCAT


    // *************** SFRA & SFRA_GUI COMM INIT CODE START *******************
#if(BUILDLEVEL == FCL_LEVEL9)       // enables SFRA
    // ************************************************************************
    // NOTE:
    // =====
    // In configureSFRA() below, use 'SFRA_GUI_PLOT_GH_H' to get open loop and
    // plant Bode plots using SFRA_GUI and open loop and closed loop Bode plots
    // using SFRA_GUI_MC. 'SFRA_GUI_PLOT_GH_CL' gives same plots for both GUIs.
    // The CL plot inferences shown in SFRA_GUI is not according to
    // NEMA ICS16 or GBT-16439-2009, so it is not recommended for bandwidth
    // determination purposes in servo drive evaluations. Use SFRA_GUI_MC for
    // that. Recommended to use the default setting (SFRA_GUI_PLOT_GH_H).
    // ************************************************************************
    //
    // configure the SFRA module. SFRA module and settings found in
    // sfra_gui.c/.h
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    sfraCollectStart = 0;
    sfraTestLoop = SFRA_TEST_Q_AXIS;  //speedLoop;

    configureSFRA(SFRA_GUI_PLOT_GH_H, M_SAMPLING_FREQ);

#endif  // (BUILDLEVEL == FCL_LEVEL9)
    // **************** SFRA & SFRA_GUI COMM INIT CODE END ********************

    // Waiting for enable flag set
    while(enableCtrlFlag == false)
    {
        backTicker++;

    #ifdef _FLASH
        if(backTicker > POWER_ON_DELAY_MASTER)
        {
            enableCtrlFlag = true;
        }
    #else
        #if(((BUILDLEVEL == FCL_LEVEL3) || (BUILDLEVEL == FCL_LEVEL4)) ||      \
            ((BUILDLEVEL == FCL_LEVEL10) || (BUILDLEVEL == FCL_LEVEL11)))
        if(backTicker > 100000L)
        {
            enableCtrlFlag = true;
        }
        #endif  //(BUILDLEVEL == FCL_LEVEL3, 4, 10, 11) that enable EtherCAT
    #endif // _FLASH
    }

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

//
// Initialize Datalog module
//
#ifdef DLOG_ENABLE
    DLOG_6CH_F_init(&dlog_6ch);
    dlog_6ch.input_ptr[0] = &dlog6ChData[0];
    dlog_6ch.input_ptr[1] = &dlog6ChData[1];
    dlog_6ch.input_ptr[2] = &dlog6ChData[2];
    dlog_6ch.input_ptr[3] = &dlog6ChData[3];
    dlog_6ch.input_ptr[4] = &dlog6ChData[4];
    dlog_6ch.input_ptr[5] = &dlog6ChData[5];

    dlog_6ch.output_ptr[0] = &DBUFF_6CH[0][0];
    dlog_6ch.output_ptr[1] = &DBUFF_6CH[1][0];
    dlog_6ch.output_ptr[2] = &DBUFF_6CH[2][0];
    dlog_6ch.output_ptr[3] = &DBUFF_6CH[3][0];
    dlog_6ch.output_ptr[4] = &DBUFF_6CH[4][0];
    dlog_6ch.output_ptr[5] = &DBUFF_6CH[5][0];

    dlog_6ch.status = 0;
    dlog_6ch.preScalar = 1;
    dlog_6ch.size = DLOG_BUF_SIZE;
#endif  // DLOG_ENABLE

#ifdef DACOUT_EN
    dacDataView[0].dacOffset = 0.0;
    dacDataView[1].dacOffset = 0.0;

    dacDataView[0].dacGain = 4.0;
    dacDataView[1].dacGain = 4.0;

    dacDataView[0].dacPtr = &ctrlVars[0].ctrlSpeedRef;
    dacDataView[1].dacPtr = &ctrlVars[0].speedWe;
#endif   // DACOUT_EN

    // Configure interrupt for motor
    HAL_setupInterrupts(halHandle);

#if(BUILDLEVEL >= FCL_LEVEL5)   // enables FSI
    // enable global interrupt
    EINT;          // Enable Global interrupt INTM

    ERTM;          // Enable Global realtime interrupt DBGM

    // FSI handshake
    FSI_handshakeLead(halHandle);

    // FSI Tx/Rx frame data
    FSI_setupTRxFrameData(halHandle);

    // Configure interrupt for motor
    HAL_enableInterrupts(halHandle);
#else
    // Configure interrupt for motor
    HAL_enableInterrupts(halHandle);

    // enable global interrupt
    EINT;          // Enable Global interrupt INTM

    ERTM;          // Enable Global realtime interrupt DBGM
#endif  // (BUILDLEVEL >= FCL_LEVEL5)

    GPIO_writePin(CCARD_GPIO_LED1, 1);   // LED Dark

    //
    // Initializations COMPLETE
    //  - IDLE loop. Just loop forever
    //
    for(;;)  //infinite loop
    {

        // State machine entry & exit point
        //===========================================================
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)
        //===========================================================

        #if(BUILDLEVEL >= FCL_LEVEL5)       // Enable FSI
        FSI_runTRxData(halHandle);
        #endif  // (BUILDLEVEL >= FCL_LEVEL4)

        #if(((BUILDLEVEL == FCL_LEVEL3) || (BUILDLEVEL == FCL_LEVEL4)) ||      \
            ((BUILDLEVEL == FCL_LEVEL10) || (BUILDLEVEL == FCL_LEVEL11)))
        ECAT_exchangeDataCPUandCM();
        #endif  //(BUILDLEVEL == FCL_LEVEL3, 4, 10, 11) that enable EtherCAT

        IPC_exchangeDataCPU1andCPU2();

        // run controllers for all nodes
        runController(SYS_NODEM);
        runController(SYS_NODE1);
        runController(SYS_NODE2);
        runController(SYS_NODE3);
        runController(SYS_NODE4);

    }
} //END MAIN CODE

//=============================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=============================================================================

//--------------------------------- FRAMEWORK ---------------------------------
void A0(void)
{
    // loop rate synchronizer for A-tasks
    if(CPUTimer_getTimerOverflowStatus(CPUTIMER0_BASE))
    {
        CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------

        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)
        serialCommsTimer++;
    }

    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    // loop rate synchronizer for B-tasks
    if(CPUTimer_getTimerOverflowStatus(CPUTIMER1_BASE))
    {
        CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);  // clear flag

        //-----------------------------------------------------------
        (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
        //-----------------------------------------------------------
        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;      // Allow A state tasks
}

//==============================================================================
//  A - TASKS (executed in every 50 usec)
//==============================================================================

//--------------------------------------------------------
void A1(void) // SPARE (not used)
//--------------------------------------------------------
{
    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    A_Task_Ptr = &A2;
    //-------------------
}

//-----------------------------------------------------------------
void A2(void) // SPARE (not used)
//-----------------------------------------------------------------
{

    //-------------------
    //the next time CpuTimer0 'counter' reaches Period value go to A3
    A_Task_Ptr = &A3;
    //-------------------
}

//-----------------------------------------
void A3(void) // SPARE (not used)
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A4;
    //-----------------
}

//-----------------------------------------
void A4(void) // SPARE (not used)
//-----------------------------------------
{

    //-----------------
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    A_Task_Ptr = &A1;
    //-----------------
}

//==============================================================================
//  B - TASKS (executed in every 100 usec)
//==============================================================================

//----------------------------------- USER -------------------------------------

//----------------------------------------
void B1(void) // Toggle GPIO-00
//----------------------------------------
{
    #if(BUILDLEVEL == FCL_LEVEL9)       // enables SFRA
    //
    // SFRA test
    //
    SFRA_F32_runBackgroundTask(&sfra1);
    SFRA_GUI_runSerialHostComms(&sfra1);
    #endif  // (BUILDLEVEL == FCL_LEVEL9)

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) // SPARE
//----------------------------------------
{
#if(BUILDLEVEL >= FCL_LEVEL5)
    if(fsiHandShakeState == FSI_HANDSHAKE_DONE)
    {
        led1Cnt++;

        if(led1Cnt >= CCARD_LED1_WAIT_TIME)
        {
            led1Cnt = 0;

            GPIO_togglePin(CCARD_GPIO_LED1);   // LED1
        }
    }
    else
    {
        GPIO_writePin(CCARD_GPIO_LED1, 1);   // LED2 Dark
    }
#else
    led1Cnt++;

    if(led1Cnt >= CCARD_LED1_WAIT_TIME)
    {
        led1Cnt = 0;

        GPIO_togglePin(CCARD_GPIO_LED1);   // LED1
    }
#endif

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) // SPARE
//----------------------------------------
{

    #if(((BUILDLEVEL == FCL_LEVEL3) || (BUILDLEVEL == FCL_LEVEL4)) ||          \
        ((BUILDLEVEL == FCL_LEVEL10) || (BUILDLEVEL == FCL_LEVEL11)))
    ECAT_runControl(SYS_NODEM);
    ECAT_runControl(SYS_NODE1);
    ECAT_runControl(SYS_NODE2);
    ECAT_runControl(SYS_NODE3);
    ECAT_runControl(SYS_NODE4);
    #endif  //(BUILDLEVEL == FCL_LEVEL3, 4, 10, 11) that enable EtherCAT

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B4;
    //-----------------
}

//----------------------------------------
void B4(void) // SPARE
//----------------------------------------
{

    #if(((BUILDLEVEL == FCL_LEVEL3) || (BUILDLEVEL == FCL_LEVEL4)) ||          \
        ((BUILDLEVEL == FCL_LEVEL10) || (BUILDLEVEL == FCL_LEVEL11)))

    #endif  //(BUILDLEVEL == FCL_LEVEL3, 4, 10, 11) that enable EtherCAT

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}


//*****************************************************************************
// EtherCAT support functions
//*****************************************************************************
//
// configureESCGPIOs - Setup EtherCAT GPIOs
//    Note: These are configured for the F2838x controlCARD. For
//          more pin mapping details refer to the GPIO chapter of the
//          F2838x Technical Reference Manual.
//
void ECAT_configureESCGPIOs(void)
{
    // Setup EtherCAT GPIOs
    // Note: These are configured for the F2838x controlCARD. For
    //       more pin mapping details refer to the GPIO chapter of the
    //       F2838x Technical Reference Manual.

    // PHY CLK
    //
    GPIO_setPinConfig(GPIO_154_ESC_PHY_CLK);

    //
    // PHY Reset
    //
    GPIO_setPinConfig(GPIO_155_ESC_PHY_RESETN);

    //
    // I2C for EEPROM
    //
    GPIO_setPinConfig(GPIO_150_ESC_I2C_SDA);
    GPIO_setQualificationMode(150, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(150, GPIO_PIN_TYPE_PULLUP);

    GPIO_setPinConfig(GPIO_151_ESC_I2C_SCL);
    GPIO_setQualificationMode(151, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(151, GPIO_PIN_TYPE_PULLUP);

    //
    // P0 TX and RX DATA
    //
    GPIO_setPinConfig(GPIO_158_ESC_TX0_DATA0);
    GPIO_setQualificationMode(158, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_159_ESC_TX0_DATA1);
    GPIO_setQualificationMode(159, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_160_ESC_TX0_DATA2);
    GPIO_setQualificationMode(160, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_161_ESC_TX0_DATA3);
    GPIO_setQualificationMode(161, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(GPIO_165_ESC_RX0_DATA0);
    GPIO_setQualificationMode(165, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_166_ESC_RX0_DATA1);
    GPIO_setQualificationMode(166, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_167_ESC_RX0_DATA2);
    GPIO_setQualificationMode(167, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_168_ESC_RX0_DATA3);
    GPIO_setQualificationMode(168, GPIO_QUAL_ASYNC);

    //
    // P0 TX Enable, RX DV, RX ERR
    //
    GPIO_setPinConfig(GPIO_156_ESC_TX0_ENA);
    GPIO_setQualificationMode(156, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_162_ESC_RX0_DV);
    GPIO_setQualificationMode(162, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_164_ESC_RX0_ERR);
    GPIO_setPadConfig(164, GPIO_PIN_TYPE_STD);

    //
    // P0 TX and RX Clk
    //
    GPIO_setPinConfig(GPIO_157_ESC_TX0_CLK);
    GPIO_setQualificationMode(157, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_163_ESC_RX0_CLK);
    GPIO_setQualificationMode(163, GPIO_QUAL_ASYNC);

    //
    // P0 Linkstatus and Link Active LED
    //
    GPIO_setPinConfig(GPIO_148_ESC_PHY0_LINKSTATUS);
    GPIO_setPadConfig(148, GPIO_PIN_TYPE_STD);

    GPIO_setPinConfig(GPIO_143_ESC_LED_LINK0_ACTIVE);
    GPIO_setQualificationMode(143, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(143, GPIO_PIN_TYPE_INVERT);

    //
    // P0+P1 MDIO CLK and Data
    //
    GPIO_setPinConfig(GPIO_152_ESC_MDIO_CLK);
    GPIO_setPinConfig(GPIO_153_ESC_MDIO_DATA);

    //
    // P1 TX and RX DATA
    //
    GPIO_setPinConfig(GPIO_131_ESC_TX1_DATA0);
    GPIO_setQualificationMode(131, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_132_ESC_TX1_DATA1);
    GPIO_setQualificationMode(132, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_134_ESC_TX1_DATA2);
    GPIO_setQualificationMode(134, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_135_ESC_TX1_DATA3);
    GPIO_setQualificationMode(135, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_139_ESC_RX1_DATA0);
    GPIO_setQualificationMode(139, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_140_ESC_RX1_DATA1);
    GPIO_setQualificationMode(140, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_141_ESC_RX1_DATA2);
    GPIO_setQualificationMode(141, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_142_ESC_RX1_DATA3);
    GPIO_setQualificationMode(142, GPIO_QUAL_ASYNC);

    //
    // P1 TX Enable, RX DV, RX ERR
    //
    GPIO_setPinConfig(GPIO_129_ESC_TX1_ENA);
    GPIO_setQualificationMode(129, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_136_ESC_RX1_DV);
    GPIO_setQualificationMode(136, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_138_ESC_RX1_ERR);
    GPIO_setPadConfig(138, GPIO_PIN_TYPE_STD);

    //
    // P1 TX and RX Clk
    //
    GPIO_setPinConfig(GPIO_130_ESC_TX1_CLK);
    GPIO_setQualificationMode(130, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_137_ESC_RX1_CLK);
    GPIO_setQualificationMode(137, GPIO_QUAL_ASYNC);

    //
    // P1 Linkstatus and Link Active LED
    //
    GPIO_setPinConfig(GPIO_149_ESC_PHY1_LINKSTATUS);
    GPIO_setPadConfig(149, GPIO_PIN_TYPE_PULLUP);

    GPIO_setPinConfig(GPIO_144_ESC_LED_LINK1_ACTIVE);
    GPIO_setQualificationMode(144, GPIO_QUAL_ASYNC);
    GPIO_setPadConfig(144, GPIO_PIN_TYPE_INVERT);

    //
    // Sync and Latch
    //
    GPIO_setPinConfig(GPIO_125_ESC_LATCH0);
    GPIO_setPinConfig(GPIO_126_ESC_LATCH1);
    GPIO_setPinConfig(GPIO_127_ESC_SYNC0);
    GPIO_setPinConfig(GPIO_128_ESC_SYNC1);
    GPIO_setDirectionMode(127, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(128, GPIO_DIR_MODE_OUT);

    //
    // Allocate controlCARD LEDS to CM
    //
    GPIO_setDirectionMode(CCARD_ECAT_RUN_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(CCARD_ECAT_ERR_LED_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(CCARD_ECAT_RUN_LED_GPIO, GPIO_CORE_CM);
    GPIO_setMasterCore(CCARD_ECAT_ERR_LED_GPIO, GPIO_CORE_CM);

    return;
}

//
// configure M-core and IPC for EtherCAT
//
void ECAT_configureAndReleaseCM(void)
{
#ifdef USE_20MHZ_XTAL        // The external crystal is 20MHz
    //
    // Setup AUX Clock for ECAT and CM
    // Configured to 500MHz raw ((20 * 25 IMULT) /1)
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_ENABLE | SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(25) | SYSCTL_AUXPLL_FMULT_0 |
                       SYSCTL_AUXPLL_DIV_1);
#else                         // The external crystal is 25MHz
    //
    // Setup AUX Clock for ECAT and CM
    // Configured to 500MHz raw ((25 * 20 IMULT) /1)
    //
    SysCtl_setAuxClock(SYSCTL_AUXPLL_ENABLE | SYSCTL_AUXPLL_OSCSRC_XTAL |
                       SYSCTL_AUXPLL_IMULT(20) | SYSCTL_AUXPLL_FMULT_0 |
                       SYSCTL_AUXPLL_DIV_1);
#endif

    //
    // Setup EtherCAT Clocks
    //
    // Aux = 500MHz and use /5 to get 100MHz for ECAT IP
    // (There is a built in /4 to get 25MHz for PHY when using
    //  internal clocking for PHY)
    //
    SysCtl_setECatClk(SYSCTL_ECATCLKOUT_DIV_5, SYSCTL_SOURCE_AUXPLL,
                      ESC_USE_INT_PHY_CLK);

    //
    // Allocate Ethercat to CM
    //
    SysCtl_allocateSharedPeripheral(SYSCTL_PALLOCATE_ETHERCAT,
                                    ALLOCATE_TO_CM);

    //
    // Configuring CM to run at 125MHz (AUX Raw = 500MHz)
    //
    SysCtl_setCMClk(SYSCTL_CMCLKOUT_DIV_4, SYSCTL_SOURCE_AUXPLL);

    //
    // Configure CM boot up configurations and boot mode
    //
#ifdef _FLASH
    IPC_setBootMode(IPC_CPU1_L_CM_R,
                    (BOOT_KEY | CM_BOOT_FREQ_125MHZ | BOOTMODE_BOOT_TO_FLASH));
#else
    IPC_setBootMode(IPC_CPU1_L_CM_R,
                    (BOOT_KEY | CM_BOOT_FREQ_125MHZ | BOOTMODE_BOOT_TO_RAM));
#endif

    //
    // Set IPC flag (required for boot)
    //
    IPC_setFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG0);

    //
    // Release CM from reset
    //
    SysCtl_controlCMReset(SYSCTL_CORE_DEACTIVE);

    return;
}

//
// exchange the data between CPU and CM
//
void ECAT_exchangeDataCPUandCM(void)
{
    uint16_t nodes;

    // CM to CPU data
    dataBufferFromCM.ctrlSynchron =
            ipcCMToCPUDataBuffer.ctrlSynchron;

    for(nodes = 0; nodes < SYS_NODE_NUM; nodes++)
    {
        // CPU to CM data
        ipcCPUToCMDataBuffer.statusNode[nodes].state =
                dataBufferToCM.statusNode[nodes].state;

        ipcCPUToCMDataBuffer.statusNode[nodes].speed =
                dataBufferToCM.statusNode[nodes].speed;

        ipcCPUToCMDataBuffer.statusNode[nodes].position =
                dataBufferToCM.statusNode[nodes].position;

        ipcCPUToCMDataBuffer.statusNode[nodes].IqRef =
                dataBufferToCM.statusNode[nodes].IqRef;     // n/a

        ipcCPUToCMDataBuffer.statusNode[nodes].IdRef =
                dataBufferToCM.statusNode[nodes].IdRef;     // n/a

        ipcCPUToCMDataBuffer.statusNode[nodes].torque =
                dataBufferToCM.statusNode[nodes].torque;     // n/a

        // CM to CPU data
        dataBufferFromCM.ctrlNode[nodes].command =
                ipcCMToCPUDataBuffer.ctrlNode[nodes].command;

        dataBufferFromCM.ctrlNode[nodes].speedRef =
                ipcCMToCPUDataBuffer.ctrlNode[nodes].speedRef;

        dataBufferFromCM.ctrlNode[nodes].positionRef =
                ipcCMToCPUDataBuffer.ctrlNode[nodes].positionRef;
    }

    if(dataBufferFromCM.ctrlSynchron == CTRL_SYN_DISABLE)
    {
        if(ctrlSynPrev != CTRL_SYN_DISABLE)
        {
            delaySynCnt = delaySynSet;
            ctrlSynPrev = CTRL_SYN_DISABLE;
        }
        else if(delaySynCnt == 0)
        {
            ctrlSyn = CTRL_SYN_DISABLE;
        }
    }
    else if(dataBufferFromCM.ctrlSynchron == CTRL_SYN_ENABLE)
    {
        if(ctrlSynPrev != CTRL_SYN_ENABLE)
        {
            delaySynCnt = delaySynSet;
            ctrlSynPrev = CTRL_SYN_ENABLE;
        }
        else if(delaySynCnt == 0)
        {
            ctrlSyn = CTRL_SYN_ENABLE;
        }
    }

    ctrlCmdNode[0] = dataBufferFromCM.ctrlNode[0].command;

    speedRefNode[0] =
            ECAT_convertPUToFloat(dataBufferFromCM.ctrlNode[0].speedRef);

    positionRefNode[0] =
            ECAT_convertPUToFloat(dataBufferFromCM.ctrlNode[0].positionRef);

    if(ctrlSyn == CTRL_SYN_ENABLE)
    {
        for(nodes = 1; nodes < SYS_NODE_NUM; nodes++)
        {
            ctrlCmdNode[nodes] = ctrlCmdNode[0];
            speedRefNode[nodes] = speedRefNode[0];
            positionRefNode[nodes] = positionRefNode[0];
        }
    }
    else
    {
        for(nodes = 1; nodes < SYS_NODE_NUM; nodes++)
        {
            ctrlCmdNode[nodes] =
               dataBufferFromCM.ctrlNode[nodes].command;

            speedRefNode[nodes] =
               ECAT_convertPUToFloat(dataBufferFromCM.ctrlNode[nodes].speedRef);

            positionRefNode[nodes] =
            ECAT_convertPUToFloat(dataBufferFromCM.ctrlNode[nodes].positionRef);
        }
    }

    if(delaySynCnt > 0)
    {
        delaySynCnt--;
    }

    return;
}

//
// run control via EtherCAT
void ECAT_runControl(uint16_t nodes)
{
    uint16_t ctrlNode;
    ctrlNode = nodes;

    if(sysVars.ecatCtrlSet == ECAT_CTRL_ENABLE)
    {
        switch (ctrlCmdNode[ctrlNode])
        {
            case CTRL_MODE_STOP:
            default :
                if(ctrlCmdNodePrev[ctrlNode] != CTRL_MODE_STOP)
                {
                    delayNodeCnt[ctrlNode] = delayNodeSet[ctrlNode];
                }

                ctrlCmdNodePrev[ctrlNode] = CTRL_MODE_STOP;

                ctrlVars[ctrlNode].ctrlModeSet = CTRL_MODE_STOP;
                ctrlVars[ctrlNode].ctrlStateSet = CTRL_STOP;
                break;

            case CTRL_MODE_SPEED:
                if(delayNodeCnt[ctrlNode] == 0)
                {
                    ctrlVars[ctrlNode].ctrlModeSet = CTRL_MODE_SPEED;

                    ctrlVars[ctrlNode].ctrlStateSet = CTRL_RUN;
                    ctrlVars[ctrlNode].speedRef = speedRefNode[ctrlNode];

                    if(ctrlCmdNodePrev[ctrlNode] != CTRL_MODE_SPEED)
                    {
                        delayNodeCnt[ctrlNode] = delayNodeSet[ctrlNode];
                        ctrlVars[ctrlNode].ctrlStateSet = CTRL_STOP;
                    }
                }

                ctrlCmdNodePrev[ctrlNode] = ctrlCmdNode[ctrlNode];

                break;

            case CTRL_MODE_POSITION:
                if(delayNodeCnt[ctrlNode] == 0)
                {
                    ctrlVars[ctrlNode].ctrlModeSet = CTRL_MODE_POSITION;

                    ctrlVars[ctrlNode].ctrlStateSet = CTRL_RUN;
                    ctrlVars[ctrlNode].positionRef = positionRefNode[ctrlNode];

                    if(ctrlCmdNodePrev[ctrlNode] != CTRL_MODE_POSITION)
                     {
                        delayNodeCnt[ctrlNode] = delayNodeSet[ctrlNode];
                        ctrlVars[ctrlNode].ctrlStateSet = CTRL_STOP;
                     }
                }

                ctrlCmdNodePrev[ctrlNode] = ctrlCmdNode[ctrlNode];

                break;
            case CTRL_MODE_RESET:
                if(ctrlCmdNodePrev[ctrlNode] != ctrlCmdNode[ctrlNode])
                {
                    delayNodeCnt[ctrlNode] = delayNodeSet[ctrlNode];
                }

                ctrlCmdNodePrev[ctrlNode] = ctrlCmdNode[ctrlNode];

                if(delayNodeCnt[ctrlNode] == 0)
                {
                    ctrlVars[ctrlNode].ctrlModeSet = CTRL_MODE_RESET;
                    ctrlVars[ctrlNode].ctrlStateSet = CTRL_RESET;
                }

                break;

            case CTRL_MODE_FAULT:

                break;
        }

        if(delayNodeCnt[ctrlNode] > 0)
        {
            delayNodeCnt[ctrlNode]--;
        }
    }
    else
    {
        if(sysVars.ctrlSynSet == CTRL_SYN_ENABLE)
        {
            ctrlVars[ctrlNode].ctrlStateSet = sysVars.ctrlStateSet;
            ctrlVars[ctrlNode].ctrlModeSet = sysVars.ctrlModeSet;
            ctrlVars[ctrlNode].speedSet = sysVars.speedSet;
            ctrlVars[ctrlNode].speedRef = sysVars.speedSet;
            ctrlVars[ctrlNode].positionSet = sysVars.positionSet;
        }
        else
        {
            ctrlVars[ctrlNode].speedRef = ctrlVars[ctrlNode].speedSet;
            ctrlVars[ctrlNode].positionSet = ctrlVars[ctrlNode].positionSet;
        }
    }

    #if((BUILDLEVEL != FCL_LEVEL3) && (BUILDLEVEL != FCL_LEVEL10))
    // Send Back Data
    dataBufferToCM.statusNode[ctrlNode].state = ctrlVars[ctrlNode].ctrlModeCom;

    dataBufferToCM.statusNode[ctrlNode].speed =
            ECAT_convertFloatToPU(ctrlVars[ctrlNode].speedWe);

    dataBufferToCM.statusNode[ctrlNode].position =
            ECAT_convertFloatToPU(ctrlVars[ctrlNode].posMechTheta);
    #endif  // (BUILDLEVEL != FCL_LEVEL2)

    dataBufferToCM.statusNode[ctrlNode].IqRef =
            ECAT_convertFloatToPU(ctrlVars[ctrlNode].IqRef);

    dataBufferToCM.statusNode[ctrlNode].IdRef =
            ECAT_convertFloatToPU(ctrlVars[ctrlNode].IdRef);

    dataBufferToCM.statusNode[ctrlNode].torque =
            ECAT_convertFloatToPU(ctrlVars[ctrlNode].torque);

    return;
}

// exchange data between CPU1 and CPU2 over IPC
void IPC_exchangeDataCPU1andCPU2(void)
{
    dataToCPU2.ctrlStateCom = ctrlVars[SYS_NODEM].ctrlStateCom;
    dataToCPU2.speedRef = ctrlVars[SYS_NODEM].speedRef;
    dataToCPU2.positionRef = ctrlVars[SYS_NODEM].positionRef;
    dataToCPU2.IdRef = ctrlVars[SYS_NODEM].IdRef;
    dataToCPU2.IqRef = ctrlVars[SYS_NODEM].IqRef;

    ipcCPU1ToCPU2Data.ctrlStateCom = dataToCPU2.ctrlStateCom;
    ipcCPU1ToCPU2Data.speedRef     = dataToCPU2.speedRef;
    ipcCPU1ToCPU2Data.positionRef  = dataToCPU2.positionRef;
    ipcCPU1ToCPU2Data.IdRef        = dataToCPU2.IdRef;
    ipcCPU1ToCPU2Data.IqRef        = dataToCPU2.IqRef;

    dataFromCPU2.ctrlStateFdb = ipcCPU2ToCPU1Data.ctrlStateFdb;
    dataFromCPU2.faultFlag    = ipcCPU2ToCPU1Data.faultFlag;
    dataFromCPU2.speed        = ipcCPU2ToCPU1Data.speed;
    dataFromCPU2.posMechTheta = ipcCPU2ToCPU1Data.posMechTheta;
    dataFromCPU2.Id           = ipcCPU2ToCPU1Data.Id;
    dataFromCPU2.Iq           = ipcCPU2ToCPU1Data.Iq;
    dataFromCPU2.torque       = ipcCPU2ToCPU1Data.torque;

    ctrlVars[SYS_NODEM].ctrlStateFdb = dataFromCPU2.ctrlStateFdb;
    ctrlVars[SYS_NODEM].speedWe = dataFromCPU2.speed;
    ctrlVars[SYS_NODEM].posMechTheta = dataFromCPU2.posMechTheta;

    return;
}

//
// End of Code
//
