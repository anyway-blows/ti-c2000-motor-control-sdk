//#############################################################################
//
// FILE:   pto_qepdiv_Source.c
//
// Description:Example project for using PM pto_qepdiv Library.
//              Includes pto_qepdiv library and corresponding include
//              files. Initializes the encoders and performs delay compensation.
//              Runs pto_qepdiv command set.
//              Continuously Read position value in an infinite loop
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "stdint.h"
#include "pto_qepdiv.h"

#include "clb_config.h"
#include "clb.h"

//*****************************************************************************
//
//! Setup CLB tiles
//!
//! \param None
//!
//! Setup the CLB and other interconnect XBARs used in this application.
//! This is called during the CLB initialization. This function needs to be
//! called after every system reset.
//!
//! \return None
//
//*****************************************************************************
void
pto_qepdiv_setupPeriph(void)
{
    pto_qepdiv_resetCLB();
    CLB_enableCLB(CLB2_BASE);
    initTILE2(CLB2_BASE);
    CLB_enableCLB(CLB1_BASE);
    initTILE1(CLB1_BASE);

    //
    // Normal CLB configuration
    //
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

    CLB_configLocalInputMux(CLB2_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB2_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_CLB_AUXSIG2);

    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_CLB_AUXSIG1);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB2_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_CLB_AUXSIG3);

    //
    // SW-controlled input select (GP_REG)
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN6, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN7, CLB_GP_IN_MUX_EXTERNAL);

    //
    // SW-controlled input select (GP_REG)
    //
    CLB_configGPInputMux(CLB2_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN6, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB2_BASE, CLB_IN7, CLB_GP_IN_MUX_EXTERNAL);

    //
    // Clear interface FIFOs (Optional)
    //
    CLB_clearFIFOs(CLB1_BASE);
    CLB_clearFIFOs(CLB2_BASE);

    //
    // Set the various counter loads and match values.
    //
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_0_LOAD,
                       TILE1_COUNTER_0_LOAD_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_LOAD,
                       TILE1_COUNTER_1_LOAD_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_LOAD,
                       TILE1_COUNTER_2_LOAD_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_0_MATCH1,
                       TILE1_COUNTER_0_MATCH1_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_0_MATCH2,
                       TILE1_COUNTER_0_MATCH2_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_MATCH1,
                       TILE1_COUNTER_1_MATCH1_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_MATCH2,
                       TILE1_COUNTER_1_MATCH2_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_MATCH1,
                       TILE1_COUNTER_2_MATCH1_VAL);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_MATCH2,
                       TILE1_COUNTER_2_MATCH2_VAL);

    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_LOAD,
                       TILE2_COUNTER_0_LOAD_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_1_LOAD,
                       TILE2_COUNTER_1_LOAD_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_2_LOAD,
                       TILE2_COUNTER_2_LOAD_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_MATCH1,
                       TILE2_COUNTER_0_MATCH1_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_MATCH2,
                       TILE2_COUNTER_0_MATCH2_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_1_MATCH1,
                       TILE2_COUNTER_1_MATCH1_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_1_MATCH2,
                       TILE2_COUNTER_1_MATCH2_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_2_MATCH1,
                       TILE2_COUNTER_2_MATCH1_VAL);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_2_MATCH2,
                       TILE2_COUNTER_2_MATCH2_VAL);

    //
    // Sync Settings (ON for most cases unless async input explicitly needed)
    //
    CLB_enableSynchronization(CLB1_BASE, CLB_IN0);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN1);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN2);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN3);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN4);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN5);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN6);
    CLB_enableSynchronization(CLB1_BASE, CLB_IN7);

    CLB_enableSynchronization(CLB2_BASE, CLB_IN0);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN1);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN2);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN3);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN4);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN5);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN6);
    CLB_enableSynchronization(CLB2_BASE, CLB_IN7);

    //
    // Input Filtering (Edge detects, or passthru)
    //
    CLB_selectInputFilter(CLB1_BASE, CLB_IN0, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN2, CLB_FILTER_ANY_EDGE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN3, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN4, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN5, CLB_FILTER_ANY_EDGE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN6, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN7, CLB_FILTER_RISING_EDGE);

    CLB_selectInputFilter(CLB2_BASE, CLB_IN0, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN1, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN2, CLB_FILTER_ANY_EDGE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN3, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN4, CLB_FILTER_ANY_EDGE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN5, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN6, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB2_BASE, CLB_IN7, CLB_FILTER_NONE);

    pto_qepdiv_initCLBXBAR();   // Initialize the CLB XBAR
}

//*****************************************************************************
//
//! Configure the divider
//!
//! \param divider is value of the divider
//! \param indexWidth is the number of cycles for which the index pulse output
//!        is kept on
//!
//! This function configures the divider. The divider value cannot be changed
//! dynamically. Users must reset the module using pto_qepdiv_reset before
//! reconfiguring the functionality.
//!
//! \return divider ptoFullPeriod is the return value, if the function is
//! executed successfully
//
//*****************************************************************************

uint16_t
pto_qepdiv_config(uint16_t divider, uint16_t indexWidth)
{
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_MATCH2, divider * 4);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_MATCH1, divider * 2);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_MATCH1, indexWidth - 1);
    return(divider);
}

//*****************************************************************************
//
//! This function initiates the pulse generation.
//!
//! \param run parameters to be passed to start(1) or stop(0) the function
//!
//! This function initiates the pulse generation. This function must only be called after
//! pto_qepdiv_setupPeriph. Hence, the pto_qepdiv_startOperation function kick starts the pulse generation
//! that was set up earlier.
//!
//! \return None.
//
//*****************************************************************************
void
pto_qepdiv_startOperation(uint16_t run)
{
    EALLOW;
    // HWREG(CLB2_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0x6;
    HWREG(CLB2_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = (CLB_LOAD_EN_GLOBAL_EN | CLB_LOAD_EN_STOP);
    __asm(" RPT #10 || NOP");
    // CLB_setOutputMask(CLB2_BASE, 0xF, true);
    CLB_setOutputMask(CLB2_BASE, CLB_OUTPUT_00 | CLB_OUTPUT_01 | CLB_OUTPUT_02 | CLB_OUTPUT_03, true);
    __asm(" RPT #10 || NOP");
    // HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0x6;
    HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = (CLB_LOAD_EN_GLOBAL_EN | CLB_LOAD_EN_STOP);
    __asm(" RPT #10 || NOP");
    EDIS;
    //CLB_setOutputMask(CLB1_BASE, 0xF, true);
    //__asm(" RPT #10 || NOP");

    CLB_setGPREG(CLB2_BASE, run);
    CLB_setGPREG(CLB1_BASE, run);
}

//*****************************************************************************
//
//! Reset the qepdiv parameters
//!
//! \param None
//!
//! This function resets the qepdiv parameters set by earlier configurations
//! and to begin a fresh setup. This function must be called in case the pulse
//! generation must be reset and started again at a later stage.
//!
//! \return None
//
//*****************************************************************************
void
pto_qepdiv_reset(void)
{
    pto_qepdiv_resetCLB();
    CLB_setGPREG(CLB1_BASE, 0);
    CLB_setGPREG(CLB2_BASE, 0);
}

//*****************************************************************************
//
//! Reset the CLB
//!
//! \param None
//!
//! This function configures Output-XBars to route CLB outputs to GPIO pins
//!
//! \return None
//
//*****************************************************************************

void
pto_qepdiv_resetCLB(void)
{
    //
    // Turn OFF the CLB functionality
    //
    CLB_setGPREG(CLB2_BASE, 0);

    EALLOW;
    HWREG(CLB2_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0;
    EDIS;

    //
    // Clear Counters - this will clear counter REGs.
    // Add_shift_on_even_en should be Zero for this to take effect.
    //
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_0_LOAD, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_LOAD, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_LOAD, 0x0);

    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_0_LOAD, 0x0);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_1_LOAD, 0x0);
    CLB_writeInterface(CLB2_BASE, CLB_ADDR_COUNTER_2_LOAD, 0x0);

    //
    // Clear and reload the HLC registers
    //
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R0, 0x9);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R1, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_MATCH1, 0x2);

    CLB_setOutputMask(CLB2_BASE, 0xF, true);
}

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
pto_qepdiv_initCLBXBAR(void) // see configuration for controlCARD
{
    //
    // QEPA - InputXbar INPUT4
    //
    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX07_INPUTXBAR4);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX07);

    //
    // QEPB - InputXbar INPUT5
    //
    XBAR_setCLBMuxConfig(XBAR_AUXSIG1, XBAR_CLB_MUX09_INPUTXBAR5);
    XBAR_enableCLBMux(XBAR_AUXSIG1, XBAR_MUX09);

    //
    // QEPI - InputXbar INPUT6
    //
    XBAR_setCLBMuxConfig(XBAR_AUXSIG2, XBAR_CLB_MUX11_INPUTXBAR6);
    XBAR_enableCLBMux(XBAR_AUXSIG2, XBAR_MUX11);

    //
    // DIR - CLB1_4.1
    //
    XBAR_setCLBMuxConfig(XBAR_AUXSIG3, XBAR_CLB_MUX01_CLB1_OUT4);
    XBAR_enableCLBMux(XBAR_AUXSIG3, XBAR_MUX01);

    //
    // Configure CLB1 o/p 5 to OUTPUT XBAR to OUTPUT3
    //
#ifdef F2837x_F28004x
    XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX03);
#else
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT3, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT3, XBAR_MUX03);
#endif
    //
    // for testing
    //
    //XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX05_CLB2_OUT4);
    //XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX05);
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
