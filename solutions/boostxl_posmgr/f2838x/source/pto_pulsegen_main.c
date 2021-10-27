//#############################################################################
//
// FILE:           pto_pulsegen_main.c
//
// Description:    Example project for using PM pto_pulsegen Library.
//                 Includes pto_pulsegen library and corresponding
//                 include files.
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
// Test Usage:
// Refer to the PM PTO PulseGen Library and related documentation in
// C2000Ware_MotorControl_SDK for library functions and usage details
// pto_pulsegen.c has detials of pto function initialization and ISR used for
// updated the PTO pto_setOptions function
// Test case by default uses the Interrupt generated by
// CLB (PTO library function) for controlling PTO parameters
// Users can optionally change this to the CPU timer based update provided in
// this file
// Make sure the pto interrupt in pto_pulsegen.c is disabled
// if cputimer interrupt is used for updates
//

//
// Included Files
//
#include "pulsegen.h"        // Include file for pto_pulsegen interface
#include "device.h"
#include "driverlib.h"

//
// Function Prototypes
//
__interrupt void cpu_timer0_isr(void);
void configPtoTimerInt(void);

//
// Globals
//
extern uint32_t remVal;

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Turn on the module clock.
    //
    Interrupt_disableMaster();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Uncomment the line below to enable cputimer based updates for
    // PTO function. If using cputimer based updates, disable the
    // pto interrupts in pto_pulsegen.c.
    // Make sure the timer updates give enough time for
    // PTO library function calls. If there are conflicts in cputimer updates
    // vs. pto function updates, there can be unpredictable pulse
    //generation behavior
    //
    //configPTOTimerInt();

    //
    // Initialization routine for pto_pulsegen operation - defined in
    // pto_pulsegen.c
    // Configures the peripherals and enables clocks for required modules
    // Configures GPIO and XBar as needed for pto_pulsegen operation
    //
    pto_initPulsegen();
    SysCtl_delay(1600L);

    pto_pulsegen_startOperation();
    
    //
    // Run pulsegen with specified respective values for low pulse width,
    // high pulse width, Active Period, Full Period, Interrupt Time, 
    // ptoDirection output, and run status (1-run and 0-stop)
    //
    remVal = pto_pulsegen_runPulseGen(50, 50, 1000, 1000, 500, 1, 1);

    Interrupt_enableMaster();

    //
    // Infinite loop - ptoISR() updates the pulsegen options - in pulsegen.c
    //
    while(1)
    {
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
    }
}

//
// cpu_timer0_isr -
//
__interrupt void
cpu_timer0_isr(void)
{
    //
    // can be optionally used to accumulate reminder for the next period
    //
    //uint32_t  retval1;
    static uint16_t j = 0;
    
    if(j == 0)
    {
        remVal = pto_setOptions(4, 1000, 500, 1,  1);
        j = 1;
    }
    else
    {
        remVal = pto_setOptions(25, 1000, 500, 0, 1);
        j = 0;
    }

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// configPTOTimerInt -
//
void
configPtoTimerInt(void)
{
    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    Interrupt_register(INT_TIMER0, &cpu_timer0_isr);

    //
    // Configure CPU-Timer 0, 200MHz CPU Freq, 100us second Prd (in uSeconds)
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 20000);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h),
    // the below settings must also be updated.
    //
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    //
    // Enable CPU int1 which is connected to CPU-Timer 0 to CPU-Timer 2
    //
    Interrupt_enable(INT_TIMER0);

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    Interrupt_enableInCPU(INTERRUPT_CPU_INT7);

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    CPUTimer_startTimer(CPUTIMER0_BASE);
}

//
// End of file
//
