//#############################################################################
//
// FILE:   pto_pulsegen.c
//
// Description: Example project for using PM pto_pulsegen Library.
//              Includes pto_pulsegen library and corresponding include
//              files. Initializes the encoders and performs delay compensation.
//              Runs pto_pulsegen command set.
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
#include "pto_pulsegen.h"

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
pto_pulsegen_setupPeriph(void)
{
    pto_pulsegen_resetCLB();
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

    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

    //
    // SW-controlled input select (GP_REG)
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN2, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN3, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN4, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN6, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN7, CLB_GP_IN_MUX_EXTERNAL);

    //
    // Clear interface FIFOs (Optional)
    //
    CLB_clearFIFOs(CLB1_BASE);

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

    //
    // Sync Settings (ON for most cases unless async input explicitly needed)
    //
    CLB_disableSynchronization(CLB1_BASE, CLB_IN0);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN1);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN2);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN3);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN4);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN5);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN6);
    CLB_disableSynchronization(CLB1_BASE, CLB_IN7);

    //
    // Input Filtering (Edge detects, or passthru)
    //
    CLB_selectInputFilter(CLB1_BASE, CLB_IN0, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN1, CLB_FILTER_RISING_EDGE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN2, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN3, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN4, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN5, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN6, CLB_FILTER_NONE);
    CLB_selectInputFilter(CLB1_BASE, CLB_IN7, CLB_FILTER_NONE);

    pto_pulsegen_initCLBXBAR();   // Initialize the CLB XBAR
}

//*****************************************************************************
//
//! Reset pulsegen
//!
//! \param None
//!
//! This function resets the pulsegen parameters set by earlier
//! configuration and start a fresh setup. The reset is done by calling the
//! reset CLB function.
//!
//! \return None
//
//*****************************************************************************
void
pto_pulsegen_reset(void)
{
    pto_pulsegen_resetCLB();
    CLB_setGPREG(CLB1_BASE, 0);
}

//*****************************************************************************
//
//! Run pulse generator by loading PTO options (pulses, period, duration
//! into HLC registers
//!
//! \param None
//! \param pulseLo is low pulse width
//! \param pulseHi is high pulse width
//! \param ptoActivePeriod is the period the pulses are sent out
//!        less than ptoFullPeriod
//! \param ptoFullPeriod is full PTO period
//! \param ptoInterruptTime is the time when the interrupt is generated to
//!        the CPU
//! \param ptoDirection is the output direction output. The output direction is
//!        latched at the beginning of new period
//! \param run is the run status. 1 indicates run and 0 is stop.
//!        This value is sampled at the beginning of the new period to determine
//!         to continue or halt the pulse generation
//!
//! This function is a runtime function to be called periodically
//! for dynamically configuring and changing the pulse generation requirements
//! as needed by the application
//!
//! \return ptoFullPeriod return the value, if the function is
//! executed successfully
//
//*****************************************************************************
uint16_t
pto_pulsegen_runPulseGen(uint32_t pulseLo, uint32_t pulseHi,
                            uint32_t ptoActivePeriod, uint32_t ptoFullPeriod,
                            uint32_t ptoInterruptTime, uint16_t ptoDirection,
                            uint16_t run)
{
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R0, pulseLo - 1);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R1, pulseLo + pulseHi - 1);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R2, ptoActivePeriod - 1);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R3, ptoFullPeriod - 1);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_MATCH1,
                       ptoInterruptTime - 1);
    CLB_setGPREG(CLB1_BASE,(run*6) + (ptoDirection*16) +1);
    return(ptoFullPeriod);
}

//*****************************************************************************
//
//! This function initiates the pulse generation.
//!
//! \param None
//!
//! This function must be called after pto_pulsegen_setupPeriph.
//! Hence, the pto_pulsegen_startOperation kick starts the pulse generation that
//! was set up earlier.
//!
//! \return None.
//
//*****************************************************************************
void
pto_pulsegen_startOperation(void)
{
    EALLOW;
    // HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0x6;
    HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = (CLB_LOAD_EN_GLOBAL_EN | CLB_LOAD_EN_STOP);
    EDIS;
    __asm(" RPT #3 || NOP");
    // CLB_setOutputMask(CLB1_BASE, 0x30, true);
    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_05 | CLB_OUTPUT_04, true);
    __asm(" RPT #3 || NOP");
    CLB_setGPREG(CLB1_BASE, 0x1);
    __asm(" RPT #3 || NOP");
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
pto_pulsegen_resetCLB(void)
{
    //
    // Turn OFF the CLB functionality
    //
    CLB_setGPREG(CLB1_BASE, 0);

    EALLOW;
    HWREG(CLB1_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0;
    EDIS;

    //
    // Clear Counters - this will clear counter REGs.
    // Add_shift_on_even_en should be Zero for this to take effect.
    //
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_0_LOAD, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_LOAD, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_2_LOAD, 0x0);

    //
    // Clear and reload the HLC registers
    //
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R0, 0x9);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_HLC_R1, 0x0);
    CLB_writeInterface(CLB1_BASE, CLB_ADDR_COUNTER_1_MATCH1, 0x2);

    CLB_setOutputMask(CLB1_BASE, 0xF, false);
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
pto_pulsegen_initCLBXBAR(void) // see configuration for controlCARD
{
    //
    // Configure CLB1 o/p 4 and 5 to OUTPUT XBAR to OUTPUT3 and OUTPUT4
    // respectively
    //
#ifdef F2837x_F28004x
    XBAR_setOutputMuxConfig(XBAR_OUTPUT3, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT3, XBAR_MUX01);

    XBAR_setOutputMuxConfig(XBAR_OUTPUT4, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT4, XBAR_MUX03);
#else
    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT3,
                            XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT3, XBAR_MUX01);

    XBAR_setOutputMuxConfig(OUTPUTXBAR_BASE, XBAR_OUTPUT4,
                            XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(OUTPUTXBAR_BASE, XBAR_OUTPUT4, XBAR_MUX03);
#endif
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
