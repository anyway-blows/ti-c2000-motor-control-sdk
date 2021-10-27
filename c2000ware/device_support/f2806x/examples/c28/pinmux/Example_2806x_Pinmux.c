//###########################################################################
//
// FILE:   Example_2806x_Pinmux.c
//
// TITLE:  Empty Pinmux Project
//
//!  \addtogroup f2806x_example_list
//!  <h1>Empty Pinmux</h1>
//		
//#############################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//#############################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//
// Main
//
void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    //
    InitSysCtrl();

    //
    // Step 2. Initalize GPIO: 
    //
    //InitGpio();  // Skipped for this example  
    

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts 
    //
    DINT;

    //
    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.  
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt 
    // Service Routines (ISR).  
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    //
    InitPieVectTable();

                            
    for(;;)
    {
        
    }
}   

//
// End of File
//

