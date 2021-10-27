//###########################################################################
//
// FILE:   pmbus_stack_config.c
//
// TITLE:  PMBUS Communications Stack Configuration
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Includes
//
#include <stdint.h>
#include <stdbool.h>
#include "pmbus_stack_config.h"

//
// Defines
//
PMBUS_STACK_FILENUM(1)

#define PMBUS_STACK_SLAVE_ADDRESS  0U
#define PMBUS_STACK_SLAVE_MASK     0x7FU

//
// Globals
//
PMBus_StackObject PMBusStackSlave;
PMBus_StackHandle PMBusStackSlaveHandle = &PMBusStackSlave;

//*****************************************************************************
//
// PMBusStack_initModule
//
//*****************************************************************************
bool PMBusStack_initModule(PMBus_StackHandle handle, const uint32_t moduleBase,
                           uint16_t *buffer)
{
    //
    // Locals
    //
    bool status = false;

    //
    // Set the module base address
    //
    PMBusStackObject_setModuleBase(handle, moduleBase);

    //
    // Assign the buffer pointer to the object, set the current pointer to
    // the head of the buffer
    //
    PMBusStackObject_setBufferPointer(handle, buffer);
    PMBusStackObject_setCurrentPositionPointer(handle, buffer);

    //
    // Reset the PMBUS Module
    //
    PMBus_disableModule(PMBusStackObject_getModuleBase(handle));

    //
    // Take the PMBUS out of reset
    //
    PMBus_enableModule(PMBusStackObject_getModuleBase(handle));

    //
    // Initialize depending upon whether in slave or master mode
    //
    if(PMBusStackObject_getMode(handle) == PMBUS_STACK_MODE_SLAVE)
    {
        //
        // Initialize the slave module
        //
        PMBus_initSlaveMode(PMBusStackObject_getModuleBase(handle),
                            PMBusStackObject_getSlaveAddress(handle),
                            PMBusStackObject_getSlaveAddressMask(handle));

        //
        // Configure the slave module (writes to the PMBSC register)
        //
        PMBus_configSlave(PMBUSA_BASE, (PMBUS_SLAVE_ENABLE_PEC_PROCESSING |
                                        PMBUS_SLAVE_AUTO_ACK_4_BYTES));

        //
        // Enable interrupts
        //
        PMBus_enableInterrupt(PMBusStackObject_getModuleBase(handle),
                              (PMBUS_INT_DATA_READY | PMBUS_INT_DATA_REQUEST |
                               PMBUS_INT_EOM));

        //
        // Reset the state machine
        //
        PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_IDLE);
        PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);

        status = true;
    }
    else // (PMBusStackObject_getMode(handle) == PMBUS_STACK_MODE_MASTER)
    {
        //
        // Zero out the slave address and set the mask to 0x7F
        // (the mask is N/A in master mode)
        //
        PMBusStackObject_setSlaveAddress(handle, PMBUS_STACK_SLAVE_ADDRESS);
        PMBusStackObject_setSlaveAddressMask(handle, PMBUS_STACK_SLAVE_MASK);

        //
        // Initialize the master module
        //
        PMBus_initMasterMode(PMBusStackObject_getModuleBase(handle));

        //
        // Enable interrupts
        //
        PMBus_enableInterrupt(PMBusStackObject_getModuleBase(handle),
                              (PMBUS_INT_DATA_READY | PMBUS_INT_DATA_REQUEST |
                               PMBUS_INT_EOM | PMBUS_INT_ALERT));

        //
        // Reset the state machine
        //
        PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_IDLE);
        PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);

        status = true;
    }

    return(status);
}

//*****************************************************************************
//
// PMBusStack_defaultTransactionHandler
//
//*****************************************************************************
int32_t PMBusStack_defaultTransactionHandler(PMBus_StackHandle handle)
{
    //
    // Replace with an actual handler. This will stop emulation
    //
    PMBUS_STACK_ASSERT(0);

    return(-1);
}

//
// End of File
//
