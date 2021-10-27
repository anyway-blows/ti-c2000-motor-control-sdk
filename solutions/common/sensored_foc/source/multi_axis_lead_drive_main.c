//#############################################################################
//
// FILE:    multi_axis_lead_main_drive.c
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
#include "motor_drive_settings.h"
#include "multi_axis_lead_drive_main.h"

//
// State Machine function prototypes
//

// Alpha states
void A0(void);  //state A0
void B0(void);  //state B0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
void A3(void);  //state A3

// B branch states
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch

uint16_t vTimer0[4];    // Virtual Timers slaved off CPU Timer 0 (A events)
uint16_t vTimer1[4];    // Virtual Timers slaved off CPU Timer 1 (B events)
uint16_t serialCommsTimer;

//
// flag variables
//
volatile uint16_t enableDriveFlag = false;
uint16_t led2Cnt = 0;

uint32_t backTicker = 0;

//
// These are defined by the linker file
//
#if(SPD_CNTLR == SPD_DCL_CNTLR)
#ifdef _FLASH
extern uint16_t dclfuncsLoadStart;
extern uint16_t dclfuncsLoadSize;
extern uint16_t dclfuncsRunStart;
extern uint16_t dclfuncsRunSize;
#endif  // _FLASH
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

#pragma DATA_SECTION(ipcCPU1ToCPU2Data, "MSGRAM_CPU1_TO_CPU2")
DRIVE_IPC_dataFromCPU1_t ipcCPU1ToCPU2Data;

#pragma DATA_SECTION(ipcCPU2ToCPU1Data, "MSGRAM_CPU2_TO_CPU1")
DRIVE_IPC_dataToCPU1_t   ipcCPU2ToCPU1Data;

DRIVE_IPC_dataFromCPU1_t dataFromCPU1;
DRIVE_IPC_dataToCPU1_t   dataToCPU1;

//
// CPU2 main() function enter
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    Interrupt_initVectorTable();

    // Wait for verification from CPU1 that CPU2 has been given control of GS
    IPC_waitForFlag(IPC_CPU2_L_CPU1_R, IPC_FLAG1);

    // clear the memory for global uninitialization variables
    // !!BE CAREFUL TO DO THIS ACTION
    HAL_clearDataRAM((void *)RAMMS_START_ADDRESS, RAMMS_SIZE_LENGTH);
    HAL_clearDataRAM((void *)RAMLS_START_ADDRESS, RAMLS_SIZE_LENGTH);

    // Wait for verification from CPU1 that CPU2 has been given control of GS
    IPC_waitForFlag(IPC_CPU2_L_CPU1_R, IPC_FLAG2);

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    #ifdef _FLASH
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&dclfuncsRunStart, &dclfuncsLoadStart, (size_t)&dclfuncsLoadSize);
    #endif
#endif

    // PWM Clocks Enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // initialize motor parameters for motor_1
    initMotorParameters(&motorVars, halHandle);

// Note that the vectorial sum of d-q PI outputs should be less than 1.0 which
// refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
// is current sense through shunt resistors which depends on hardware/software
// implementation. Depending on the application requirements 3,2 or a single
// shunt resistor can be used for current waveform reconstruction. The higher
// number of shunt resistors allow the higher duty cycle operation and better
// dc bus utilization. The users should adjust the PI saturation levels
// carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
// in project manuals. Violation of this procedure yields distorted  current
// waveforms and unstable closed loop operations that may damage the inverter.
    // reset some control variables for motor
    resetControlVars(&motorVars);

    // setup faults protection for motor
    HAL_setupMotorFaultProtection(halHandle, motorVars.currentLimit);

    // clear any spurious OST & DCAEVT1 flags for motor
    HAL_clearTZFlag(halHandle);

    // Clear LED counter
    led2Cnt = 0;
    GPIO_writePin(CCARD_GPIO_LED2, 0);   // LED Light

    // Waiting for enable flag set
    while(enableDriveFlag == false)
    {
        backTicker++;

        #ifdef _FLASH
        if(backTicker > POWER_ON_DELAY_DRIVE)
        {
            enableDriveFlag = true;
        }
        #else
        #if(BUILDLEVEL >= FCL_LEVEL7)
        if(backTicker > 300000)
        {
            enableDriveFlag = true;
        }
        #endif  // (BUILDLEVEL >= FCL_LEVEL7)
        #endif // _FLASH
    }

    //find out the FCL SW version information
    while(FCL_getSwVersion() != 0x00000201)
    {
        backTicker++;
    }

    // *************** SFRA & SFRA_GUI COMM INIT CODE START *******************
#if(BUILDLEVEL == FCL_LEVEL6)
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
    //

    sfraCollectStart = 0;
    sfraTestLoop = SFRA_TEST_Q_AXIS;  //speedLoop;

    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, M_SAMPLING_FREQ);
#endif
    // **************** SFRA & SFRA_GUI COMM INIT CODE END ********************

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    // Set up the initialization value for some variables
    motorVars.posPtrMax = posPtrMax;
    motorVars.fclClrCntr = 1;
//
// Initialize Datalog module for motor 1 or motor 2
//
    DLOG_4CH_F_init(&dlog_4ch1);
    dlog_4ch1.input_ptr1 = &dlogCh1;    //data value
    dlog_4ch1.input_ptr2 = &dlogCh2;
    dlog_4ch1.input_ptr3 = &dlogCh3;
    dlog_4ch1.input_ptr4 = &dlogCh4;
    dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
    dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
    dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
    dlog_4ch1.size = 200;
    dlog_4ch1.pre_scalar = 5;
    dlog_4ch1.trig_value = 0.01;
    dlog_4ch1.status = 2;

    // Configure interrupt for motor
    HAL_setupInterrupts(halHandle);

    // Configure interrupt for motor
    HAL_enableInterrupts(halHandle);

    // current feedback offset calibration for motor_1
    runOffsetsCalculation(&motorVars);

    // enable global interrupt
    EINT;          // Enable Global interrupt INTM

    ERTM;          // Enable Global realtime interrupt DBGM

    // Clear the latch flag
    motorVars.clearFaultFlag = 1;
    GPIO_writePin(CCARD_GPIO_LED2, 1);   // LED Dark

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

        backTicker++;

        IPC_exchangeDataCPU2andCPU1();

        // motor running logic control
        runMotorControl(&motorVars, halHandle);
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
#if(BUILDLEVEL == FCL_LEVEL6)
    //
    // SFRA test
    //
    SFRA_F32_runBackgroundTask(&sfra1);
    SFRA_GUI_runSerialHostComms(&sfra1);
#endif

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    B_Task_Ptr = &B2;
    //-----------------
}

//----------------------------------------
void B2(void) // SPARE
//----------------------------------------
{

    led2Cnt++;

    if(led2Cnt >= CCARD_LED2_WAIT_TIME)
    {
        led2Cnt = 0;

        GPIO_togglePin(CCARD_GPIO_LED2);   // LED
    }

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B3
    B_Task_Ptr = &B3;
    //-----------------
}

//----------------------------------------
void B3(void) // SPARE
//----------------------------------------
{

    //-----------------
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    B_Task_Ptr = &B1;
    //-----------------
}

void IPC_exchangeDataCPU2andCPU1(void)
{
    dataFromCPU1.ctrlStateCom = ipcCPU1ToCPU2Data.ctrlStateCom;
    dataFromCPU1.speedRef     = ipcCPU1ToCPU2Data.speedRef;
    dataFromCPU1.positionRef  = ipcCPU1ToCPU2Data.positionRef;
    dataFromCPU1.IdRef        = ipcCPU1ToCPU2Data.IdRef;
    dataFromCPU1.IqRef        = ipcCPU1ToCPU2Data.IqRef;

    ipcCPU2ToCPU1Data.ctrlStateFdb = dataToCPU1.ctrlStateFdb;
    ipcCPU2ToCPU1Data.faultFlag    = dataToCPU1.faultFlag;
    ipcCPU2ToCPU1Data.speed        = dataToCPU1.speed;
    ipcCPU2ToCPU1Data.posMechTheta = dataToCPU1.posMechTheta;
    ipcCPU2ToCPU1Data.Id           = dataToCPU1.Id;
    ipcCPU2ToCPU1Data.Iq           = dataToCPU1.Iq;
    ipcCPU2ToCPU1Data.torque       = dataToCPU1.torque;

    #if(BUILDLEVEL >= FCL_LEVEL7)
    motorVars.ctrlStateCom = dataFromCPU1.ctrlStateCom;
    motorVars.IdRefSet = dataFromCPU1.IdRef;
    motorVars.IqRefSet = dataFromCPU1.IqRef;

    dataToCPU1.ctrlStateFdb = motorVars.ctrlStateFdb;
    dataToCPU1.speed = motorVars.speedWe;
    dataToCPU1.posMechTheta = motorVars.posMechTheta;
    #endif  // (BUILDLEVEL >= FCL_LEVEL7)

    return;
}

//
// End of Code
//