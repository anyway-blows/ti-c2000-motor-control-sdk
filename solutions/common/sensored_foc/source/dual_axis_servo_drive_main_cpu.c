//#############################################################################
//
// FILE:    multi_axis_slave_main.c
//
// TITLE:   multi-axis servo drive over FSI on the related kits
//
// Group:   C2000
//
// Target Family: F28004x/F28002x
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
#include "dual_axis_servo_drive_user.h"
#include "dual_axis_servo_drive_hal.h"
#include "dual_axis_servo_drive_cpu.h"

#include "sfra_settings.h"

//
// Functions
//


//
//  Prototype statements for Local Functions
//


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

//
// Global variables used in this system
//

// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch

uint16_t vTimer0[4];        // Virtual Timers slaved off CPU Timer 0 (A events)
uint16_t vTimer1[4];        // Virtual Timers slaved off CPU Timer 1 (B events)
uint16_t serialCommsTimer;

// flag variables
//
// Flag variables
//
volatile uint16_t enableFlag;
uint16_t backTicker;
uint16_t led1Cnt;
uint16_t led2Cnt;

#ifdef _FLASH
#if((SPD_CNTLR == SPD_DCL_CNTLR) || (SPD_CNTLR == SPD_NLPID_CNTLR))
extern uint16_t dclfuncsLoadStart;
extern uint16_t dclfuncsLoadSize;
extern uint16_t dclfuncsRunStart;
extern uint16_t dclfuncsRunSize;
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)
#endif  // _FLASH

// Variables for Datalog module
#ifdef DLOG_ENABLE

#ifdef F28004x_DEVICE
float32_t DBUFF_4CH1[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH2[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH3[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH4[DBUFF_SIZE_NUM];
float32_t dlogCh1;
float32_t dlogCh2;
float32_t dlogCh3;
float32_t dlogCh4;

// Create an instance of DATALOG Module
DLOG_4CH_F dlog_4ch1;
#endif  // F28004x_DEVICE

#if defined (F28002x_DEVICE)
float32_t DBUFF_4CH1[DBUFF_SIZE_NUM];
float32_t DBUFF_4CH2[DBUFF_SIZE_NUM];

float32_t dlogCh1;
float32_t dlogCh2;

// Create an instance of DATALOG Module
DLOG_2CH_F dlog_2ch1;
#pragma DATA_SECTION(DBUFF_4CH1,"datalog_data");
#pragma DATA_SECTION(DBUFF_4CH2,"datalog_data");
#pragma DATA_SECTION(dlogCh1,"datalog_data");
#pragma DATA_SECTION(dlogCh2,"datalog_data");
#pragma DATA_SECTION(dlog_2ch1,"datalog_data");

#endif  // F28002x_DEVICE

#endif  // DLOG_ENABLE

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");
#endif  // DAC128S_ENABLE

//*****************************************************************************
//
// main() function enter
//
void main(void)
{
    // initialize device clock and peripherals
    Device_init();

    // clear the memory for global uninitialization variables
    // !!BE CAREFUL TO DO THIS ACTION
    HAL_clearDataRAM((void *)RAMMS_START_ADDRESS, RAMMS_SIZE_LENGTH);
    HAL_clearDataRAM((void *)RAMLS_START_ADDRESS, RAMLS_SIZE_LENGTH);
    HAL_clearDataRAM((void *)RAMGS_START_ADDRESS, RAMGS_SIZE_LENGTH);

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // initialize the driver for motor 1
    halMtrHandle[MTR_1] =
            HAL_MTR_init(&halMtr[MTR_1], sizeof(halMtr[MTR_1]));

    // initialize the driver for motor 1
    halMtrHandle[MTR_2] =
            HAL_MTR_init(&halMtr[MTR_2], sizeof(halMtr[MTR_2]));

    // set the driver parameters
    HAL_setParams(halHandle);

    // set the driver parameters for motor 1
    HAL_setMotorParams(halMtrHandle[MTR_1]);

    // set the driver parameters for motor 2
    HAL_setMotorParams(halMtrHandle[MTR_2]);

    // PWM Clocks Enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // PWM Clocks Enable
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // initialize motor parameters for motor_1
    initMotorParameters(&motorVars[MTR_1], halMtrHandle[MTR_1]);

    // initialize motor parameters for motor_2
    initMotorParameters(&motorVars[MTR_2], halMtrHandle[MTR_2]);

    // initialize motor control variables for motor_1
    initControlVars(&motorVars[MTR_1]);

    // initialize motor control variables for motor_2
    initControlVars(&motorVars[MTR_2]);

    // setup faults protection for motor_1
    HAL_setupMotorFaultProtection(halMtrHandle[MTR_1]);

    // setup faults protection for motor_2
    HAL_setupMotorFaultProtection(halMtrHandle[MTR_2]);

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
    // reset some control variables for motor_1
    resetControlVars(&motorVars[MTR_1]);

    // reset some control variables for motor_2
    resetControlVars(&motorVars[MTR_2]);

    // clear any spurious OST & DCAEVT1 flags for motor_1
    HAL_clearTZFlag(halMtrHandle[MTR_1]);

    // clear any spurious OST & DCAEVT1 flags for motor_2
    HAL_clearTZFlag(halMtrHandle[MTR_2]);

    // Clear LED counter
    led1Cnt = 0;
    led2Cnt = 0;

    GPIO_writePin(HAL_GPIO_LED2, 0);   // LED2 Light

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
#if(SFRA_MOTOR == MOTOR_1)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, M1_SAMPLING_FREQ);

    motorVars[MTR_1].sfraEnableFlag = true;
    motorVars[MTR_2].sfraEnableFlag = false;
#elif(SFRA_MOTOR == MOTOR_2)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, M2_SAMPLING_FREQ);

    motorVars[MTR_1].sfraEnableFlag = false;
    motorVars[MTR_2].sfraEnableFlag = true;
#else  // SFRA_MOTOR != MOTOR_1 && SFRA_MOTOR != MOTOR_2
    motorVars[MTR_1].sfraEnableFlag = false;
    motorVars[MTR_2].sfraEnableFlag = false;
#endif  // SFRA_MOTOR = MOTOR_1 || SFRA_MOTOR = MOTOR_2

#endif  // BUILDLEVEL = FCL_LEVEL6

    //find out the FCL SW version information
    while(FCL_getSwVersion() != 0x00000201)
    {
        backTicker++;
    }

    // Waiting for enable flag set
    while(enableFlag == false)
    {
        #ifdef _FLASH
        if(backTicker == 0)
        {
            enableFlag = true;

            flagSyncRun = false;        // true
            ctrlState = CTRL_STOP;      // CTRL_RUN
        }
        else
        {
            backTicker--;
        }
        #else
        backTicker++;


        #endif  // _FLASH
    }

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    //
    // Initialize Datalog module
    //
#if defined(DLOG_ENABLE)
#if defined (F28004x_DEVICE)
    DLOG_4CH_F_init(&dlog_4ch1);

    dlog_4ch1.input_ptr1 = &dlogCh1;    //data value
    dlog_4ch1.input_ptr2 = &dlogCh2;
    dlog_4ch1.input_ptr3 = &dlogCh3;
    dlog_4ch1.input_ptr4 = &dlogCh4;

    dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
    dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
    dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];

    dlog_4ch1.size = DBUFF_SIZE_NUM;
    dlog_4ch1.pre_scalar = 5;
    dlog_4ch1.trig_value = 0.01;
    dlog_4ch1.status = 2;
#endif  // F28004x_DEVICE

#if defined (F28002x_DEVICE)
    DLOG_2CH_F_init(&dlog_2ch1);

    dlog_2ch1.input_ptr1 = &dlogCh1;    //data value
    dlog_2ch1.input_ptr2 = &dlogCh2;

    dlog_2ch1.output_ptr1 = &DBUFF_4CH1[0];
    dlog_2ch1.output_ptr2 = &DBUFF_4CH2[0];

    dlog_2ch1.size = DBUFF_SIZE_NUM;
    dlog_2ch1.pre_scalar = 5;
    dlog_2ch1.trig_value = 0.01;
    dlog_2ch1.status = 2;
#endif  // F28002x_DEVICE

#endif  // DLOG_ENABLE

#if defined(DAC128S_ENABLE)
    // initialize the DAC128S
    dac128sHandle = DAC128S_init(&dac128s);

    // setup SPI for DAC128S
    DAC128S_setupSPI(dac128sHandle);

    dac128s.ptrData[0] = &motorVars[0].pangle;                  // CH_A
    dac128s.ptrData[1] = &motorVars[0].posElecTheta;            // CH_B
    dac128s.ptrData[2] = &motorVars[1].pangle;                  // CH_C
    dac128s.ptrData[3] = &motorVars[1].posElecTheta;            // CH_D

    dac128s.ptrData[4] = &motorVars[0].rg.Out;                  // CH_E
    dac128s.ptrData[5] = &motorVars[0].svgen.Ta;                // CH_F
    dac128s.ptrData[6] = &motorVars[1].rg.Out;                  // CH_G
    dac128s.ptrData[7] = &motorVars[1].svgen.Ta;                // CH_H

    dac128s.gain[0] = 1.0f * 4096.0f;
    dac128s.gain[1] = 1.0f * 4096.0f;
    dac128s.gain[2] = 1.0f * 4096.0f;
    dac128s.gain[3] = 1.0f * 4096.0f;
    dac128s.gain[4] = 1.0f * 4096.0f;
    dac128s.gain[5] = 1.0f * 4096.0f;
    dac128s.gain[6] = 1.0f * 4096.0f;
    dac128s.gain[7] = 1.0f * 4096.0f;

    dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[4] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[5] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[6] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[7] = (uint16_t)(0.5f * 4096.0f);

    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE

    // Configure interrupt for motor_1
    HAL_setupInterrupts(halMtrHandle[MTR_1]);

    // Configure interrupt for motor_2
    HAL_setupInterrupts(halMtrHandle[MTR_2]);

    // current feedback offset calibration for motor_1
    runOffsetsCalculation(&motorVars[MTR_1]);

    // current feedback offset calibration for motor_1
    runOffsetsCalculation(&motorVars[MTR_2]);

    // Configure interrupt for motor_1
    HAL_enableInterrupts(halMtrHandle[MTR_1]);

    // Configure interrupt for motor_2
    HAL_enableInterrupts(halMtrHandle[MTR_2]);

    //Clear the latch flag
    motorVars[MTR_1].clearTripFlagDMC = 1;
    motorVars[MTR_2].clearTripFlagDMC = 1;

    // enable global interrupt
    EINT;          // Enable Global interrupt INTM

    ERTM;          // Enable Global realtime interrupt DBGM

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
        runSyncControl();

        // motor_1 running logic control
        runMotorControl(&motorVars[MTR_1], halMtrHandle[MTR_1]);

        // motor_2 running logic control
        runMotorControl(&motorVars[MTR_2], halMtrHandle[MTR_2]);

        backTicker++;

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
    led2Cnt++;

    if(led2Cnt >= CCARD_LED2_WAIT_TIME)
    {
        led2Cnt = 0;

        GPIO_togglePin(HAL_GPIO_LED2);   // LED
    }

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

//
// End of Code
//
