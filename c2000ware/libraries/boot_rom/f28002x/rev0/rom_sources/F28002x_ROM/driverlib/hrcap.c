//###########################################################################
//
// FILE:   hrcap.c
//
// TITLE:  C28x HRCAP driver.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#include "hrcap.h"

//*****************************************************************************
//
//! \internal
//! Gets the HRCAP Calibration interrupt number.
//!
//! \param base is the base address of the HRCAP instance used.
//!
//! Given an HRCAP base address, this function returns the corresponding
//! Calibration interrupt number.
//!
//! \return Returns an HRCAP Calibration interrupt number, or 0 if \e base is
//!         invalid.
//
//*****************************************************************************
static uint32_t HRCAP_getCalibrationIntNumber(uint32_t base)
{
    uint32_t intNumber;

    ASSERT(HRCAP_isBaseValid(base));

       intNumber = INT_ECAP3_2;

    return(intNumber);
}

//*****************************************************************************
//
// HRCAP_registerCalibrationInterrupt
//
//*****************************************************************************
void HRCAP_registerCalibrationInterrupt(uint32_t base, void (*handler)(void))
{
    uint32_t intNumber;

    ASSERT(HRCAP_isBaseValid(base));

    intNumber = HRCAP_getCalibrationIntNumber(base);

    ASSERT(intNumber != 0);

    //
    // Register the interrupt handler.
    //
    Interrupt_register(intNumber, handler);

    //
    // Enable the HRCAP interrupt.
    //
    Interrupt_enable(intNumber);
}

//*****************************************************************************
//
// HRCAP_unregisterCalibrationInterrupt
//
//*****************************************************************************
void HRCAP_unregisterCalibrationInterrupt(uint32_t base)
{
    uint32_t intNumber;

    ASSERT(HRCAP_isBaseValid(base));

    //
    // Determine the interrupt number
    //
    intNumber = HRCAP_getCalibrationIntNumber(base);

    ASSERT(intNumber != 0);

    //
    // Disable the HRCAP interrupt.
    //
    Interrupt_disable(intNumber);

    //
    // Register the interrupt handler.
    //
    Interrupt_unregister(intNumber);
}

