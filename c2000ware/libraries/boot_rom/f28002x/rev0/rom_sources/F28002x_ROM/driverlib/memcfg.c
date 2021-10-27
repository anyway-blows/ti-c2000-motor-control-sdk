//###########################################################################
//
// FILE:   memcfg.c
//
// TITLE:  C28x RAM config driver.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#include "memcfg.h"


#define MEMCFG_O_LSXACCPROT0       (MEMCFG_O_LSXACCPROT1 - 2U)

//*****************************************************************************
//
// MemCfg_lockConfig
//
//*****************************************************************************
void
MemCfg_lockConfig(uint32_t ramSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS) ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Set the bit that blocks writes to the sections' configuration registers.
    //
    EALLOW;

    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)  |= MEMCFG_SECT_NUM_MASK &
                                                     ramSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     ramSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     ramSections;
            break;

        case MEMCFG_SECT_TYPE_MASK:
            // Lock configuration for all sections.
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)  |= MEMCFG_SECT_NUM_MASK &
                                                     MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) |= MEMCFG_SECT_NUM_MASK &
                                                     MEMCFG_SECT_GSX_ALL;
            break;

        default:
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_unlockConfig
//
//*****************************************************************************
void
MemCfg_unlockConfig(uint32_t ramSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS) ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Clear the bit that blocks writes to the sections' configuration
    // registers.
    //
    EALLOW;

    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK)  &= ~(MEMCFG_SECT_NUM_MASK &
                                                       ramSections);
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) &= ~(MEMCFG_SECT_NUM_MASK &
                                                       ramSections);
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) &= ~(MEMCFG_SECT_NUM_MASK &
                                                       ramSections);
            break;

        case MEMCFG_SECT_TYPE_MASK:
            // Unlock configuration for all sections.
            HWREG(MEMCFG_BASE + MEMCFG_O_DXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_DX_ALL));
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_LSX_ALL));
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXLOCK) &=
                ~((uint32_t)(MEMCFG_SECT_NUM_MASK & MEMCFG_SECT_GSX_ALL));
            break;

        default:
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_commitConfig
//
//*****************************************************************************
void
MemCfg_commitConfig(uint32_t ramSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS) ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Set the bit that permanently blocks writes to the sections'
    // configuration registers.
    //
    EALLOW;

    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXCOMMIT)  |= MEMCFG_SECT_NUM_MASK &
                                                       ramSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       ramSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       ramSections;
            break;

        case MEMCFG_SECT_TYPE_MASK:
            // Commit configuration for all sections.
            HWREG(MEMCFG_BASE + MEMCFG_O_DXCOMMIT)  |= MEMCFG_SECT_NUM_MASK &
                                                       MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXCOMMIT) |= MEMCFG_SECT_NUM_MASK &
                                                       MEMCFG_SECT_GSX_ALL;
            break;

        default:
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_setProtection
//
//*****************************************************************************
void
MemCfg_setProtection(uint32_t ramSection, uint32_t protectMode)
{
    uint32_t shiftVal = 0U;
    uint32_t maskVal;
    uint32_t regVal;
    uint32_t sectionNum;
    uint32_t regOffset;

    //
    // Check the arguments.
    //
    ASSERT(((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS) ||
           ((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS));

    //
    // Calculate how far the protect mode value needs to be shifted. Each
    // section number is represented by a bit in the lower word of ramSection
    // and 8 bits in the corresponding ACCPROT register.
    //
    sectionNum = ramSection & MEMCFG_SECT_NUM_MASK;

    while(sectionNum != 1U)
    {
        sectionNum = sectionNum >> 1U;
        shiftVal += 8U;
    }

    //
    // Calculate register offset. Also, make sure the shift value is no greater
    // than 31.
    //
    regOffset = (shiftVal & ~(0x1FU)) >> 4U;
    shiftVal &= 0x0001FU;
    maskVal = MEMCFG_XACCPROTX_M << shiftVal;
    regVal = protectMode << shiftVal;

    //
    // Write the access protection mode into the appropriate field
    //
    EALLOW;

    switch(ramSection & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXACCPROT0 + regOffset) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXACCPROT0 + regOffset) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXACCPROT0 + regOffset) |= regVal;
            break;

        default:
            // Do nothing. Invalid ramSection.
            break;
    }

    EDIS;
}


//*****************************************************************************
//
// MemCfg_setTestMode
//
//*****************************************************************************
void
MemCfg_setTestMode(uint32_t ramSection, MemCfg_TestMode testMode)
{
    uint32_t shiftVal = 0U;
    uint32_t maskVal;
    uint32_t regVal;
    uint32_t sectionNum;

    //
    // Check the arguments.
    //
    ASSERT(((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)  ||
           ((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS) ||
           ((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS) ||
           ((ramSection & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG));

    //
    // Calculate how far the protect mode value needs to be shifted. Each
    // section number is represented by a bit in the lower word of ramSection
    // and 2 bits in the corresponding TEST register.
    //
    sectionNum = ramSection & MEMCFG_SECT_NUM_MASK;

    while(sectionNum != 1U)
    {
        sectionNum = sectionNum >> 1U;
        shiftVal += 2U;
    }

    maskVal = (uint32_t)MEMCFG_XTEST_M << shiftVal;
    regVal = (uint32_t)testMode << shiftVal;

    //
    // Write the test mode into the appropriate field
    //
    EALLOW;

    switch(ramSection & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_DXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXTEST) |= regVal;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXTEST) &= ~maskVal;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXTEST) |= regVal;
            break;


        default:
            // Do nothing. Invalid ramSection.
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_initSections
//
//*****************************************************************************
void
MemCfg_initSections(uint32_t ramSections)
{
    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Set the bit in the various initialization registers that starts
    // initialization.
    //
    EALLOW;

    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)   |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_LS:
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;

        case MEMCFG_SECT_TYPE_GS:
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      ramSections;
            break;


        case MEMCFG_SECT_TYPE_MASK:
            // Initialize all sections.
            HWREG(MEMCFG_BASE + MEMCFG_O_DXINIT)   |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_DX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_LSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_LSX_ALL;
            HWREG(MEMCFG_BASE + MEMCFG_O_GSXINIT)  |= MEMCFG_SECT_NUM_MASK &
                                                      MEMCFG_SECT_GSX_ALL;
            break;

        default:
            // Do nothing. Invalid ramSections. Make sure you aren't OR-ing
            // values for two different types of RAM.
            break;
    }

    EDIS;
}

//*****************************************************************************
//
// MemCfg_getInitStatus
//
//*****************************************************************************
bool
MemCfg_getInitStatus(uint32_t ramSections)
{
    uint32_t status;

    //
    // Check the arguments.
    //
    ASSERT(((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_D)   ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_LS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_GS)  ||
           ((ramSections & MEMCFG_SECT_TYPE_MASK) == MEMCFG_SECT_TYPE_MSG) ||
           (ramSections == MEMCFG_SECT_ALL));

    //
    // Read registers containing the initialization complete status.
    //
    switch(ramSections & MEMCFG_SECT_TYPE_MASK)
    {
        case MEMCFG_SECT_TYPE_D:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_DXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_LS:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE);
            break;

        case MEMCFG_SECT_TYPE_GS:
            status = HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE);
            break;


        case MEMCFG_SECT_TYPE_MASK:
            // Return the overall status.
            if((HWREG(MEMCFG_BASE + MEMCFG_O_DXINITDONE) ==
                MEMCFG_SECT_DX_ALL) &&
               (HWREG(MEMCFG_BASE + MEMCFG_O_LSXINITDONE) ==
                MEMCFG_SECT_LSX_ALL) &&
               (HWREG(MEMCFG_BASE + MEMCFG_O_GSXINITDONE) ==
                MEMCFG_SECT_GSX_ALL)
				)
            {
                status = MEMCFG_SECT_NUM_MASK;
            }
            else
            {
                status = 0U;
            }
            break;

        default:
            // Invalid ramSections. Make sure you aren't OR-ing values for two
            // different types of RAM.
            status = 0U;
            break;
    }

    return((ramSections & status) == (ramSections & MEMCFG_SECT_NUM_MASK));
}

//*****************************************************************************
//
// MemCfg_registerViolationInterrupt
//
//*****************************************************************************
void
MemCfg_registerViolationInterrupt(void (*handler)(void))
{
    //
    // Register the interrupt handler, returning an error if an error occurs.
    //
    Interrupt_register(INT_RAM_ACC_VIOL, handler);

    //
    // Enable the RAM access violation interrupt.
    //
    Interrupt_enable(INT_RAM_ACC_VIOL);
}

//*****************************************************************************
//
// MemCfg_unregisterViolationInterrupt
//
//*****************************************************************************
void
MemCfg_unregisterViolationInterrupt(void)
{
    //
    // Disable the interrupt.
    //
    Interrupt_disable(INT_RAM_ACC_VIOL);

    //
    // Unregister the interrupt handler.
    //
    Interrupt_unregister(INT_RAM_ACC_VIOL);
}



//*****************************************************************************
//
// MemCfg_registerCorrErrorInterrupt
//
//*****************************************************************************
void
MemCfg_registerCorrErrorInterrupt(void (*handler)(void))
{
    //
    // Register the interrupt handler, returning an error if an error occurs.
    //
    Interrupt_register(INT_RAM_CORR_ERR, handler);

    //
    // Enable the RAM access violation interrupt.
    //
    Interrupt_enable(INT_RAM_CORR_ERR);
}

//*****************************************************************************
//
// MemCfg_unregisterCorrErrorInterrupt
//
//*****************************************************************************
void
MemCfg_unregisterCorrErrorInterrupt(void)
{
    //
    // Disable the interrupt.
    //
    Interrupt_disable(INT_RAM_CORR_ERR);

    //
    // Unregister the interrupt handler.
    //
    Interrupt_unregister(INT_RAM_CORR_ERR);
}

//*****************************************************************************
//
// MemCfg_getCorrErrorAddress
//
//*****************************************************************************
uint32_t
MemCfg_getCorrErrorAddress(uint32_t stsFlag)
{
    //
    // Check the arguments.
    //
    if(stsFlag != MEMCFG_CERR_CPUREAD)
    {
        //
        // Currently, the only correctable error address that can be read
        // from a register is one for a CPU read error (MEMCFG_CERR_CPUREAD).
        // For the sake of keeping this function portable to possible future
        // devices with other error types, it still takes a stsFlag parameter.
        //
        ASSERT(false);
    }

    //
    // Read and return the error address.
    //
    return(HWREG(MEMORYERROR_BASE + MEMCFG_O_CCPUREADDR));
}

//*****************************************************************************
//
// MemCfg_getUncorrErrorAddress
//
//*****************************************************************************
uint32_t
MemCfg_getUncorrErrorAddress(uint32_t stsFlag)
{
    uint32_t address;
    uint32_t temp;

    //
    // Calculate the the address of the desired error address register.
    //
    address = MEMORYERROR_BASE + MEMCFG_O_UCCPUREADDR;

    temp = stsFlag;

    while(temp > 1U)
    {
        temp = temp >> 1U;
        address += (uint32_t)(MEMCFG_O_UCDMAREADDR - MEMCFG_O_UCCPUREADDR);
    }

    //
    // Read and return the error address at the calculated location.
    //
    return(HWREG(address));
}
