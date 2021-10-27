//#############################################################################
//
// FILE:   pto_abs2qep.c
//
// Description: C2000 Absolute Position to QEP Pulse Train Out (PTO) Library
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//*****************************************************************************
//
//! \addtogroup PTO_ABS2QEP
//! @{
//
//*****************************************************************************

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "stdint.h"
#include "stdbool.h"
#include "pto_abs2qep.h"

#include "clb_config.h"
#include "clb.h"

//
// Globals
//


//*****************************************************************************
//
//! Initialize the CLB XBAR-connect
//!
//! \param None
//!
//! This function configures Output-XBars to route CLB outputs to GPIO pins
//!
//! \return None
//
//*****************************************************************************
void
pto_abs2qep_initCLBXBAR(void)
{
#if defined(_F2837x)
    //
    // PTO-QEP-A output
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT7, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT7, XBAR_MUX01);

    //
    // PTO-QEP-B output
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT2, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT2, XBAR_MUX03);



#elif defined(_F28004x)
    //
    // PTO-QEP-A
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT1, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT1, XBAR_MUX01);

    //
    // PTO-QEP-B
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT2, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT2, XBAR_MUX03);

#elif defined(_F2838x)
    //
    // PTO-QEP-A
    //
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT7,                     \
                            XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT7, XBAR_MUX01);

    //
    // PTO-QEP-B
    //
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT2,                     \
                            XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT2, XBAR_MUX03);

#else
    #error Device is not specified.  Add device to predefined variables.
#endif
}

//*****************************************************************************
//
//! Setup CLB tiles
//!
//! \param None
//!
//! Setup the CLB and other interconnect XBARs used in this application.
//! This is called during the CLB initialization.
//!
//! \return None
//
//*****************************************************************************
void
pto_abs2qep_setupPeriph(void)
{
#if !defined(_F2837x)
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_CLB1);
#endif
    CLB_enableCLB(CLB1_BASE);
    initTILE1(CLB1_BASE);

    //
    // Select Global input instead of local input for all CLB IN
    //
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

    //
    // Unused Inputs below
    //
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

    //
    // Inputs set to GP register
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_GP_REG);


    //
    // Unused inputs to GP register
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN2, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN3, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN4, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN5, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN6, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

    //
    // Initialize the CLB XBAR
    //
    pto_abs2qep_initCLBXBAR();

    //
    // OUT2 (QEP-I) overrides PWM1B
    // OUT0 (RUN/HALT) overrides PWM1A
    //
    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_02, true);
    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_00, true);

    //
    // Clear the FIFO
    //
    CLB_clearFIFOs(CLB1_BASE);

    //
    // Sending a position 0 will setup the CLB
    // modules and set the interrupt tag to indicate
    // a PTO is not running.
    //
    pto_abs2qep_translatePosition(0);
    pto_abs2qep_runPulseGen(PTO_ABS2QEP_CLOCKWISE_PTO);
}

//*****************************************************************************
//
//! Start the PTO Generation
//!
//! \param pto_direction is either forward (PTO_ABS2QEP_CLOCKWISE_PTO) 
//!        or reverse (PTO_ABS2QEP_COUNTERCLOCKWISE_PTO)
//!
//! This function confirms that the previous PTO has completed and then
//! triggers the load event to the HLC
//!
//! pto_abs2qep_translatePostion() must be called before this function
//! to load the HLC pullData FIFO
//!
//!
//! \return None
//
//*****************************************************************************
__attribute__((ramfunc))
void
pto_abs2qep_runPulseGen(uint16_t pto_direction)
{

    // Confirm that the previous PTO has completed
    // Start the load of a new configuration
    // Set the direction appropriately
    //
    // The GPIO is a visual marker for the start and
    // direction of the QEP-A/B signals and can be 
    // removed in an application
    //

    while(CLB_getInterruptTag(CLB1_BASE) != PTO_ABS2QEP_PTO_DONE_TAG )
    {
    }
    CLB_setGPREG(CLB1_BASE, 0);
    CLB_clearInterruptTag(CLB1_BASE);
    if(pto_direction == PTO_ABS2QEP_CLOCKWISE_PTO)
    {
        CLB_setGPREG(CLB1_BASE, PTO_ABS2QEP_SET_CLOCKWISE |                    \
                     PTO_ABS2QEP_SET_LOAD);
        GPIO_writePin(PTO_ABSQEP_TEST_PIN_1, 1);
    }
    else
    {
        CLB_setGPREG(CLB1_BASE, PTO_ABS2QEP_SET_COUNTERCLOCKWISE |             \
                     PTO_ABS2QEP_SET_LOAD);
        GPIO_writePin(PTO_ABSQEP_TEST_PIN_1, 0);
    }
}


//*****************************************************************************
//
//! Translate a change in absolute position to QEP pulses
//!
//! \param  PositionNew is the new absolute position
//!
//! This function calculates the delta from the previous position and
//! translates it to an equivalent number of QCLKs needed to generate
//! QEP-A, QEP-B and QEP-I.
//!
//! The HLC FIFO is loaded with the configuration.
//!
//! \return None
//
//*****************************************************************************
__attribute__((ramfunc))
uint16_t
pto_abs2qep_translatePosition(uint32_t positionNew)
{
    uint16_t numQclkSend;
    uint16_t ptoDirection;
    int32_t  positionDelta;
    uint32_t indexHighEdge;
    uint16_t pulseWidth;
    float32_t numQclkFloat;
    float32_t numQclkFrac;
    int16_t numQclkAdjust;
    static float32_t numQclkTotalFrac = 0.0f;
    static float32_t numQclkCarry = 0.0f;
    static uint32_t positionPrevious = 0ul;
    static uint32_t positionCurrent = 0ul;
    float32_t numQclkInt;
    bool positionZeroCross;

    //
    // Uncomment the following to halt the CLB on a CPU halt.
    // EALLOW;
    // HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_STOP;
    // EDIS;
    //
    //
    // Archive old absolute position
    // Find the delta between old and current position
    // Determine if change in position crosses the absolute zero position
    //
    positionPrevious = positionCurrent;
    positionCurrent = positionNew;
    positionDelta = positionCurrent - positionPrevious;

    //
    // Check if absolute zero is crossed over during the position delta
    // If so, adjust the calculations for the delta position value to
    // account for crossover.
    // Calculate the delta in two parts as described in the
    // user's guide.
    // Calculate the match 1 and match2 values that will
    // be used in counter 2 to generate the QEP index signal
    //
    positionZeroCross = labs(positionDelta) >
                   (PTO_ABS2QEP_ABS_MAX_DELTA_PER_SAMPLE) ? true : false;

    if(positionZeroCross)
    {
        //
        // Crossed zero in the reverse direction
        // Change sign to indicate reverse
        //
        if(positionDelta > 0)
        {
            positionDelta = -1 *
                    (int32_t)((positionPrevious - 0) +
                         (PTO_ABS2QEP_ABS_MAX_POSITION - positionCurrent));
            indexHighEdge =
                    (int32_t)(positionPrevious * PTO_ABS2QEP_ABS_TO_INCR);
        }
        else
        {
            positionDelta =
                    (int32_t)((PTO_ABS2QEP_ABS_MAX_POSITION -
                                positionPrevious) +
                                positionCurrent);
            indexHighEdge =
                    (int32_t)((PTO_ABS2QEP_ABS_MAX_POSITION -
                                positionPrevious) *
                                PTO_ABS2QEP_ABS_TO_INCR);
        }
    }
    //
    // If zero was not crossed, use a large value so the match never occurs.
    //
    else
    {
        indexHighEdge = 0xFFFFFFFE;
    }
    //
    // Check to see if the QEP index signal occurs at 0
    // If it does, offset by 1 so it is not missed.
    //

    if(indexHighEdge == 0)
    {
        indexHighEdge += 1;
    }
    //
    // Calculate the number of QCLKs based on the delta
    // between the old and new absolute encoder positions.
    //
    // Determine the integer and fractional number of pulses.
    // If the accumulated fractional portion is > 1 or < -1,
    // increase the integer number of pulses by 1.
    //
    numQclkFloat = positionDelta * PTO_ABS2QEP_ABS_TO_INCR;
    numQclkFrac = modff(numQclkFloat, &numQclkInt);

    numQclkTotalFrac = numQclkCarry + numQclkFrac;

    if(numQclkTotalFrac >= 1.0f)
    {
        numQclkAdjust = 1;
    }
    else if(numQclkTotalFrac <= -1.0f)
    {
        numQclkAdjust = -1;
    }
    else
    {
        numQclkAdjust = 0;
    }
    numQclkCarry = numQclkTotalFrac - numQclkAdjust;
    numQclkSend = abs( (int16_t)numQclkInt + numQclkAdjust);

    //
    // Calculate the PTO pulse width in CLB clocks (width of each pulse)
    //
    // For the zero pulse case, set the width to a non-zero value. This
    // will prevent the pulse width generation CLB counter from issuing
    // a match1 signal before the CLB is halted.
    //

    if(numQclkSend != 0)
    {
        pulseWidth = (PTO_ABS2QEP_SAMPLING_RATIO / numQclkSend);
    }
    else
    {
        pulseWidth = 0xFFFF;
    }

    //
    // Write the updated counter match values to the FIFO
    //
    uint32_t bufferFIFO[4];
    bufferFIFO[0] = (uint32_t)numQclkSend;            // COUNTER 1 MATCH 2
    bufferFIFO[1] = (uint32_t)pulseWidth;             // COUNTER 0 MATCH 1
    bufferFIFO[2] = indexHighEdge;                    // COUNTER 2 MATCH 1
    bufferFIFO[3] = indexHighEdge + 1U;                 // COUNTER 2 MATCH 2
    CLB_writeFIFOs(CLB1_BASE, &bufferFIFO[0]);

    //
    // Log the direction information for when the PTO is
    // started by the application
    //

    if(positionDelta > 0)
    {
        ptoDirection = PTO_ABS2QEP_CLOCKWISE_PTO;
    }
    else
    {
        ptoDirection = PTO_ABS2QEP_COUNTERCLOCKWISE_PTO;

    }
    return(ptoDirection);
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//
// End of File
//

