//#############################################################################
//
// FILE:           abs2qep.c
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
// Globals
//

//
// Initialize peripherals used by the abs2qep and the example
//
void
pto_initAbs2QEP(void)
{
    //
    // Setup the GPIO for this example
    //
    pto_setupGPIO();

    //
    // Setup the CLB
    //
    pto_abs2qep_setupPeriph();

    //
    // The PWM is used as a timer for sampling the position.
    // and its output is overridden by the CLB for signal
    // output.
    //
    pto_configEPWM();

    //
    // The QEP module is only used for testing.
    // The incremental position indicated by POSCOUNT will be
    // compared to the absolute position.
    //
    pto_configEQEP();
}

//
// The ePWM3 will be used as the position sample timer
//
void
pto_configEPWM(void)
{

    //
    // Enable the PWM module
    // Disable sync (Freeze clock to PWM)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPWM3_TIMER_TBPRD);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setClockPrescaler(EPWM3_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    //
    // Select INT on Time base counter zero event,
    //
    EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM3_BASE);
    EPWM_setInterruptEventCount(EPWM3_BASE, 1U);

    //
    // Assign the interrupt service routines to ePWM interrupts
    // Enable ePWM interrupts
    // The clock will be started right before the main while loop
    //
    Interrupt_register(INT_EPWM3, &pto_EPWM3ISR);
    Interrupt_enable(INT_EPWM3);
}

//
// pto_initEQEP - Initialize EQEP module
//
void
pto_configEQEP(void)
{
    //
    // Configure the decoder for quadrature count mode
    //
    EQEP_setDecoderConfig(EQEP1_BASE, (EQEP_CONFIG_2X_RESOLUTION |
                                       EQEP_CONFIG_QUADRATURE |
                                       EQEP_CONFIG_NO_SWAP));

    //
    // Sets the initial value for the encoder position counter.
    //
    EQEP_setInitialPosition(EQEP1_BASE, 0);

    //
    // Configures the mode in which the position counter is initialized.
    //
    EQEP_setPositionInitMode(EQEP1_BASE, EQEP_INIT_RISING_INDEX);

    //
    // Configure the position counter to reset on an index event
    //
    EQEP_setPositionCounterConfig(EQEP1_BASE, EQEP_POSITION_RESET_IDX,
                                  0xFFFFFFFF);

    //
    // Enable the eQEP1 module
    //
    EQEP_enableModule(EQEP1_BASE);

    EQEP_setCaptureConfig(EQEP1_BASE, EQEP_CAPTURE_CLK_DIV_64,
                          EQEP_UNIT_POS_EVNT_DIV_32);

    EQEP_enableCapture(EQEP1_BASE);
}

//
// Test the PTO
//
// Compare the absolute encoder angle to the incremental encoder angle
// The output of the PTO should be connected to an eQEP module as
// described in the documentation.
//
// Note: when sampling the QEP POSCNTR, make sure the previous PTO is
// complete, and the next PTO has not yet been started.
//
void
pto_checkPosition(int32_t incrementalPosition,                                 \
                  uint32_t absolutePositionPrevious)
{
    float32_t absoluteDegree;
    float32_t incrementalDegree;
    float32_t deltaNormalized;
    float32_t deltaDegree;
    static int32_t passCount = 0;
    static int32_t failCount = 0;
    static float32_t deltaMax = 0.0f;

    //
    // Convert both the absolute and incremental position to a degree
    // on the unit circle.  Compare these two values against the
    // threshold defined in the example header file.
    //
    absoluteDegree = 360.0f * ((float32_t) absolutePositionPrevious /          \
            (float32_t) PTO_ABS2QEP_ABS_MAX_POSITION);
    incrementalDegree = 360.0f * (( float32_t) incrementalPosition /           \
            ((float32_t) PTO_ABS2QEP_QCLK_PER_REV));
    deltaDegree = absoluteDegree > incrementalDegree ?                         \
                  absoluteDegree - incrementalDegree :                         \
                  incrementalDegree - absoluteDegree;
    deltaDegree = fmodf(deltaDegree, 360.0f);
    deltaNormalized = deltaDegree > 180.0f ? 360.0f - deltaDegree : deltaDegree;
    if(deltaMax < deltaNormalized)
    {
       deltaMax = deltaNormalized;
    }
    if(deltaNormalized > DELTA_THRESHOLD )
    {
       failCount++;
    }
    else
    {
       passCount++;
    }
}

//
// Simulate a new absolute position to test.
// Use a fraction of the max delta per sample
// Periodically switch directions and update
// the fraction.  The fractions were chosen to
// somewhat randomize the PTOs and where the
// index signal occurs.  Otherwise there is nothing
// special about the values selected.
//
//
uint32_t pto_generateTestAbsPosition(uint32_t currentPosition)
{
    uint32_t deltaPosition = 0ul;
    float32_t static fraction = 0.0f;
    uint16_t static setDirection = PTO_ABS2QEP_CLOCKWISE_PTO;
    uint16_t static sample = 0u;

    sample++;
    if(fraction > 1.0f)
    {
        fraction -= 1.0f;
        setDirection = setDirection == PTO_ABS2QEP_CLOCKWISE_PTO ?             \
                PTO_ABS2QEP_COUNTERCLOCKWISE_PTO : PTO_ABS2QEP_CLOCKWISE_PTO;
    }
    if(sample == 5u)
    {
        setDirection = setDirection == PTO_ABS2QEP_CLOCKWISE_PTO ?
                PTO_ABS2QEP_COUNTERCLOCKWISE_PTO : PTO_ABS2QEP_CLOCKWISE_PTO;  \
    }
    else if(sample == 10u)
    {
        fraction += .33f;
    }
    else if(sample == 15u)
    {
        fraction += .55f;
    }
    else if(sample == 20u)
    {
        sample = 0u;
    }
    //
    // test back-to-back zero
    //
    if(sample == 16u || sample == 17u || sample == 18u)
    {
        deltaPosition = 0ul;
    }
    else
    {
        deltaPosition = (uint32_t)                                             \
                ((float32_t)PTO_ABS2QEP_ABS_MAX_DELTA_PER_SAMPLE * fraction);
    }
    if(deltaPosition > (uint16_t)PTO_ABS2QEP_ABS_MAX_DELTA_PER_SAMPLE)
    {
        deltaPosition = (uint16_t)PTO_ABS2QEP_ABS_MAX_DELTA_PER_SAMPLE;
    }
    if(setDirection == PTO_ABS2QEP_CLOCKWISE_PTO)
    {
        //
        // Move absolute position clockwise
        //
        currentPosition = currentPosition + (uint32_t)deltaPosition;
        currentPosition = currentPosition > PTO_ABS2QEP_ABS_MAX_POSITION ?     \
                currentPosition - PTO_ABS2QEP_ABS_MAX_POSITION :               \
                currentPosition;
    }
    else
    {
        //
        // Move absolute position counter-clockwise
        //
        currentPosition = (uint32_t)deltaPosition > currentPosition ?          \
                PTO_ABS2QEP_ABS_MAX_POSITION + currentPosition -               \
                (uint32_t)deltaPosition :                                      \
                currentPosition - (uint32_t)deltaPosition;
    }
    return(currentPosition);
}
//
// End of File
//

