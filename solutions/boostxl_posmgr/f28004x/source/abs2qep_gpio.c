//#############################################################################
//
// FILE:           abs2qep_gpio.c
//
// Description:    Example project for using pto_abs2qep Library.
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
// Included Files
//
#include <stdlib.h>
#include <time.h>
#include "abs2qep.h"
#include "device.h"
#include "driverlib.h"

//
// Configure GPIOs to monitor signals
// Specific to the F28004x Example
//
//
void
pto_setupGPIO(void)
{
    //
    // PTO-QEP-A
    // Configure GPIO24 for OUTPUTXBAR1
    //
    GPIO_setPinConfig(GPIO_24_OUTPUTXBAR1);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    //
    // PTO-QEP-B
    // Configure GPIO3 for OUTPUTXBAR2
    //
    GPIO_setPinConfig(GPIO_3_OUTPUTXBAR2);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    //
    // PTO-QEP-I
    // Configure GPIO1 as EPWM1B
    // Enable CLB Tile 1 OUT2 to replace EPWM1B
    //
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    //
    // RUN/HALT - Test Pin
    // View the state of the RUN/HALT signal in Abs2Qep.
    // Configure GPIO0 as EPWM1A
    // Enable CLB Tile 1 OUT0 to replace EPWM1A
    //
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    //
    // GPIO13 - Test Pin
    // Toggles when entering the ePWM ISR as
    // a waveform visual marker
    // Remains high if the direction is forward
    // Remains low if the direction is reverse
    //
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);
    GPIO_writePin(13, 0);

    //
    // eQEP peripheral
    // Connect to the output from Abs2Qep
    // for testing
    //
    GPIO_setPinConfig(GPIO_10_EQEP1A);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(10, GPIO_QUAL_SYNC);

    GPIO_setPinConfig(GPIO_11_EQEP1B);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(11, GPIO_QUAL_SYNC);

    GPIO_setPinConfig(GPIO_9_EQEP1I);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(9, GPIO_QUAL_SYNC);

}

//
// End of File
//
