//###########################################################################
//
// FILE:   adc.c
//
// TITLE:  C28x ADC driver.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#include "adc.h"

//*****************************************************************************
//
// Defines for locations of ADC calibration functions in OTP for use in
// ADC_setVREF(). Not intended for use by application code.
//
//*****************************************************************************
#define ADC_OFFSET_TRIM_OTP    0x70594U

//*****************************************************************************
//
// A mapping of ADC base address to interrupt number. Not intended for use by
// application code.
//
//*****************************************************************************
static const uint32_t ADC_intMap[3][ADC_NUM_INTERRUPTS + 1U] =
{
    {ADCA_BASE, INT_ADCA1, INT_ADCA2, INT_ADCA3, INT_ADCA4},
    {ADCC_BASE, INT_ADCC1, INT_ADCC2, INT_ADCC3, INT_ADCC4}
};

//*****************************************************************************
//
//! \internal
//! Returns the analog-to-digital converter interrupt number.
//!
//! \param base is the base address of the ADC module.
//! \param adcIntNum is interrupt number within the ADC wrapper.
//!
//! This function returns the interrupt number for the ADC interrupt that
//! corresponds to the base address passed in \e base and the number passed in
//! \e adcIntNum.  \e adcIntNum can take the value \b ADC_INT_NUMBER1,
//! \b ADC_INT_NUMBER2, \b ADC_INT_NUMBER3, \b or ADC_INT_NUMBER4 to express
//! which of the four interrupts of the ADC module should be used.
//!
//! \return Returns an ADC interrupt number or 0 if the interrupt does not
//! exist.
//
//*****************************************************************************
static uint32_t
ADC_getIntNumber(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint16_t index;
    uint16_t intMapRows = sizeof(ADC_intMap) / sizeof(ADC_intMap[0]);

    //
    // Loop through the table that maps ADC base addresses to interrupt
    // numbers.
    //
    for(index = 0U; index < intMapRows; index++)
    {
        //
        // See if this base address matches.
        //
        if(ADC_intMap[index][0U] == base)
        {
            //
            // Return the corresponding interrupt number.
            //
            return(ADC_intMap[index][(uint16_t)adcIntNum + 1U]);
        }
    }

    //
    // The base address could not be found, so return an error.
    //
    return(0U);
}

//*****************************************************************************
//
//! \internal
//! Returns the analog-to-digital converter event interrupt number.
//!
//! \param base base is the base address of the ADC module.
//!
//! This function returns the event interrupt number for the ADC with the base
//! address passed in the \e base parameter.
//!
//! \return Returns an ADC event interrupt number or 0 if the interrupt does not
//! exist.
//
//*****************************************************************************
static uint32_t
ADC_getPPBEventIntNumber(uint32_t base)
{
    uint32_t intNumber;

    //
    // Find the valid interrupt number for this ADC.
    //
    switch (base)
    {
        case ADCA_BASE:
            intNumber = INT_ADCA_EVT;
            break;


        case ADCC_BASE:
            intNumber = INT_ADCC_EVT;
            break;

        default:
            // Return an error.
            intNumber = 0U;
            break;
    }

    return(intNumber);
}

//*****************************************************************************
//
// ADC_setVREF
//
//*****************************************************************************
void
ADC_setVREF(uint32_t base, ADC_ReferenceMode refMode,
            ADC_ReferenceVoltage refVoltage)
{
    uint16_t *offset;
    uint16_t moduleShiftVal;
    uint16_t offsetShiftVal;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Assign a shift amount corresponding to which ADC module is being
    // configured.
    //
    switch(base)
    {
        case ADCA_BASE:
            moduleShiftVal = 0U;
            break;
        case ADCC_BASE:
            moduleShiftVal = 2U;
            break;
        default:
            //
            // Invalid base address!!
            //
            moduleShiftVal = 0U;
            break;
    }

    //
    // Offset trim for internal VREF 3.3V is unique and stored in upper byte.
    //
    if((refMode == ADC_REFERENCE_INTERNAL) &&
       (refVoltage == ADC_REFERENCE_3_3V))
    {
        offsetShiftVal = 8U;
    }
    else
    {
        offsetShiftVal = 0U;
    }

    //
    // Set up pointer to offset trim in OTP.
    //
    offset = (uint16_t *)(ADC_OFFSET_TRIM_OTP + ((uint32_t)6U *
                                                 (uint32_t)moduleShiftVal));

    //
    // Get offset trim from OTP and write it to the register.
    //
    EALLOW;
    HWREGH(base + ADC_O_OFFTRIM) = (*offset >> offsetShiftVal) & 0xFFU;

    //
    // Configure the reference mode (internal or external).
    //
    if(refMode == ADC_REFERENCE_INTERNAL)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
            ~(ASYSCTL_ANAREFCTL_ANAREFASEL << moduleShiftVal);
    }
    else
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) |=
            ASYSCTL_ANAREFCTL_ANAREFASEL << moduleShiftVal;
    }

    //
    // Configure the reference voltage (3.3V or 2.5V).
    //
    if(refVoltage == ADC_REFERENCE_3_3V)
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) &=
            ~(ASYSCTL_ANAREFCTL_ANAREFA2P5SEL << moduleShiftVal);
    }
    else
    {
        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFCTL) |=
            ASYSCTL_ANAREFCTL_ANAREFA2P5SEL << moduleShiftVal;
    }

    EDIS;
}

//*****************************************************************************
//
// ADC_registerInterrupt
//
//*****************************************************************************
void
ADC_registerInterrupt(uint32_t base, ADC_IntNumber adcIntNum,
                      void (*handler)(void))
{
    uint32_t intNumber;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Determine the interrupt number based on the ADC port.
    //
    intNumber = ADC_getIntNumber(base, adcIntNum);

    ASSERT(intNumber != 0);

    //
    // Register the interrupt handler.
    //
    Interrupt_register(intNumber, handler);

    //
    // Enable the ADC interrupt.
    //
    Interrupt_enable(intNumber);
}

//*****************************************************************************
//
// ADC_unregisterInterrupt
//
//*****************************************************************************
void
ADC_unregisterInterrupt(uint32_t base, ADC_IntNumber adcIntNum)
{
    uint32_t intNumber;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Determine the interrupt number based on the ADC port.
    //
    intNumber = ADC_getIntNumber(base, adcIntNum);

    ASSERT(intNumber != 0);

    //
    // Disable the ADC interrupt.
    //
    Interrupt_disable(intNumber);

    //
    // Register the interrupt handler.
    //
    Interrupt_unregister(intNumber);
}

//*****************************************************************************
//
// ADC_registerPPBEventInterrupt
//
//*****************************************************************************
void
ADC_registerPPBEventInterrupt(uint32_t base, void (*handler)(void))
{
    uint32_t intNumber;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Determine the interrupt number based on the ADC port.
    //
    intNumber = ADC_getPPBEventIntNumber(base);

    ASSERT(intNumber != 0);

    //
    // Register the interrupt handler.
    //
    Interrupt_register(intNumber, handler);

    //
    // Enable the ADC interrupt.
    //
    Interrupt_enable(intNumber);
}

//*****************************************************************************
//
// ADC_unregisterPPBEventInterrupt
//
//*****************************************************************************
void
ADC_unregisterPPBEventInterrupt(uint32_t base)
{
    uint32_t intNumber;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));

    //
    // Determine the interrupt number based on the ADC port.
    //
    intNumber = ADC_getPPBEventIntNumber(base);

    ASSERT(intNumber != 0);

    //
    // Disable the ADC interrupt.
    //
    Interrupt_disable(intNumber);

    //
    // Register the interrupt handler.
    //
    Interrupt_unregister(intNumber);
}

//*****************************************************************************
//
// ADC_setPPBTripLimits
//
//*****************************************************************************
void
ADC_setPPBTripLimits(uint32_t base, ADC_PPBNumber ppbNumber,
                     int32_t tripHiLimit, int32_t tripLoLimit)
{
    uint32_t ppbHiOffset;
    uint32_t ppbLoOffset;

    //
    // Check the arguments.
    //
    ASSERT(ADC_isBaseValid(base));
    ASSERT((tripHiLimit <= 65535) && (tripHiLimit >= -65536));
    ASSERT((tripLoLimit <= 65535) && (tripLoLimit >= -65536));

    //
    // Get the offset to the appropriate trip limit registers.
    //
    ppbHiOffset = (ADC_PPBxTRIPHI_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPHI;
    ppbLoOffset = (ADC_PPBxTRIPLO_STEP * (uint32_t)ppbNumber) +
                  ADC_O_PPB1TRIPLO;

    EALLOW;

    //
    // Set the trip high limit.
    //
    HWREG(base + ppbHiOffset) =
        (HWREG(base + ppbHiOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripHiLimit & ADC_PPBTRIP_MASK);

    //
    // Set the trip low limit.
    //
    HWREG(base + ppbLoOffset) =
        (HWREG(base + ppbLoOffset) & ~ADC_PPBTRIP_MASK) |
        ((uint32_t)tripLoLimit & ADC_PPBTRIP_MASK);

    EDIS;
}
