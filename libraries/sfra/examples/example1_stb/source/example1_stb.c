//###########################################################################
//
// FILE:   example1_stb.c
//
// AUTHOR: Manish Bhardwaj (C2000 Systems Solutions, Houston , TX)
//
//#############################################################################
// $TI Release: C2000 Software Frequency Response Analyzer Library v1.40.00.00 $
// $Release Date: Tue Sep 21 16:33:07 CDT 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################



//
// the includes
//

#include "example1_stb.h"

//
// globals
//

int16_t vTimer0[4];         // Virtual Timers slaved off CPU Timer 0 (A events)
int16_t vTimer1[4];         // Virtual Timers slaved off CPU Timer 1 (B events)
int16_t vTimer2[4];         // Virtual Timers slaved off CPU Timer 2 (C events)

//
// Variable declarations for state machine
//
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

//
// State Machine function prototypes
//

//
// Alpha states
//
void A0(void);  //state A0
void B0(void);  //state B0

//
// A branch states
//
void A1(void);  //state A1

//
// B branch states
//
void B1(void);  //state B1
void B2(void);  //state B2
void B3(void);  //state B3

//
//--- System Related ---
//
#pragma SET_DATA_SECTION("controlVariables")
DCL_PI gi;
DCL_DF22 gi2;

volatile float32_t gi_out;
volatile float32_t gi_out_prev;
//
// Reference variables
// current set point
//
volatile float32_t ac_cur_ref;
//
//flag to close the loop
//
volatile int32_t closeGiLoop;
#pragma SET_DATA_SECTION()

//
// SFRA Related Variables
//
SFRA_F32 sfra1;

float32_t plantMagVect[SFRA_FREQ_LENGTH];
float32_t plantPhaseVect[SFRA_FREQ_LENGTH];
float32_t olMagVect[SFRA_FREQ_LENGTH];
float32_t olPhaseVect[SFRA_FREQ_LENGTH];
float32_t clMagVect[SFRA_FREQ_LENGTH];
float32_t clPhaseVect[SFRA_FREQ_LENGTH];
float32_t freqVect[SFRA_FREQ_LENGTH];

void main(void)
{
    //
    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    // this routine will also initialize the CPU timers that are used in
    // the background task for this system
    //
    setupDevice();


    //
    // Tasks State-machine init
    //
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    //
    // Stop all PWM mode clock
    //
    disablePWMCLKCounting();

    setupUpDwnCountPWM(PWM_BASE, PFC_PWM_PERIOD);

    //
    // power up ADC on the device
    //
    setupADC();

    //
    //Profiling GPIO
    //
    setupProfilingGPIO();

    //
    // Initialize global variables generic to the board like ones used to
    // read current values and others
    //
    globalVariablesInit();

    //
    // setup SFRA
    //
    setupSFRA();

    //
    // Enable PWM Clocks
    //
    enablePWMCLKCounting();

    //
    // safe to setup PWM pins
    //
    setPinsAsPWM();

    //
    // ISR Mapping
    //
    setupInterrupt();

    //
    // IDLE loop. Just sit and loop forever,
    // periodically will branch into A0-A3, B0-B3, C0-C3 tasks
    // Frequency of this branching is set in setupDevice routine:
    //
    for(;;)  //infinite loop
    {
        //
        // State machine entry & exit point
        //
        (*Alpha_State_Ptr)(); // jump to an Alpha state (A0,B0,...)
    }
}

//
// control ISR Code
//
#if CONTROL_RUNNING_ON == C28x_CORE
interrupt void controlISR(void)
{
    controlCode();


    clearInterrupt(C28x_CONTROLISR_INTERRUPT_PIE_GROUP_NO);
}
#endif

//
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//

void A0(void)
{
    //
    // loop rate synchronizer for A-tasks
    //
    if(GET_TASK_A_TIMER_OVERFLOW_STATUS == 1)
    {
        CLEAR_TASK_A_TIMER_OVERFLOW_FLAG;   // clear flag

        (*A_Task_Ptr)();        // jump to an A Task (A1,A2,A3,...)
        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)

    }

    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    //
    // loop rate synchronizer for B-tasks
    //
    if(GET_TASK_B_TIMER_OVERFLOW_STATUS  == 1)
    {
        CLEAR_TASK_B_TIMER_OVERFLOW_FLAG;               // clear flag

        (*B_Task_Ptr)();        // jump to a B Task (B1,B2,B3,...)
        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    Alpha_State_Ptr = &A0;      // Allow C state tasks
}

//
//  A - TASKS (executed in every 1 msec)
//
void A1(void)
{

    SFRA_F32_runBackgroundTask(&sfra1);
    SFRA_GUI_runSerialHostComms(&sfra1);


    //
    //the next time CpuTimer0 'counter' reaches Period value go to A2
    //
    A_Task_Ptr = &A1;
}

//
//  B - TASKS (executed in every 5 msec)
//
void B1(void)
{
    //
    // the next time CpuTimer1 'counter' reaches Period value go to B2
    //
    B_Task_Ptr = &B2;
}

void B2(void)
{
    B_Task_Ptr = &B3;
}

void B3(void) //  SPARE
{
    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B1;
}

//
// setupSFRA
//
void setupSFRA(void)
{
    //
    //Resets the internal data of sfra module to zero
    //
    SFRA_F32_reset(&sfra1);

    //
    //Configures the SFRA module
    //
    SFRA_F32_config(&sfra1,
                    SFRA_ISR_FREQ,
                    SFRA_AMPLITUDE,
                    SFRA_FREQ_LENGTH,
                    SFRA_FREQ_START,
                    SFRA_FREQ_STEP_MULTIPLY,
                    plantMagVect,
                    plantPhaseVect,
                    olMagVect,
                    olPhaseVect,
                    clMagVect,
                    clPhaseVect,
                    freqVect,
                    1);

    //
    //Resets the response arrays to all zeroes
    //
    SFRA_F32_resetFreqRespArray(&sfra1);

    //
    //Initializes the frequency response array ,
    //The first element is SFRA_FREQ_START
    //The subsequent elements are freqVect[n-1]*SFRA_FREQ_STEP_MULTIPLY
    //This enables placing a fixed number of frequency points
    //between a decade of frequency.
    // The below routine can be substituted by a routine that sets
    // the frequency points arbitrarily as needed.
    //
    SFRA_F32_initFreqArrayWithLogSteps(&sfra1,
                                       SFRA_FREQ_START,
                                       SFRA_FREQ_STEP_MULTIPLY);

    //
    //configures the SCI channel for communication with SFRA host GUI
    //to change SCI channel change #define in the sfra_gui_scicomms_driverlib.c
    //the GUI also changes a LED status, this can also be changed with #define
    //in the file pointed to above
    //
    SFRA_GUI_config(SFRA_GUI_SCI_BASE,
                    SCI_VBUS_CLK,
                    SFRA_GUI_SCI_BAUDRATE,
                    SFRA_GUI_SCIRX_GPIO,
                    SFRA_GUI_SCIRX_GPIO_PIN_CONFIG,
                    SFRA_GUI_SCITX_GPIO,
                    SFRA_GUI_SCITX_GPIO_PIN_CONFIG,
                    SFRA_GUI_LED_INDICATOR,
                    SFRA_GUI_LED_GPIO,
                    SFRA_GUI_LED_GPIO_PIN_CONFIG,
                    &sfra1,
                    SFRA_GUI_PLOT_GH_H);
}

void globalVariablesInit(void)
{

    gi.Kp = GI_PI_KP;
    gi.Ki = GI_PI_KI;
    gi.Umax = GI_PI_MAX;
    gi.Umin = GI_PI_MIN;
    gi.i10 = 0;
    gi.i6 = 0;

    ac_cur_ref = 0.03;
    closeGiLoop = 1;

}


//
// End
//
