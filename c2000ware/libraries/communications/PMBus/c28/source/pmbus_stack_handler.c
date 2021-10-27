//###########################################################################
//
// FILE:   pmbus_stack_handler.c
//
// TITLE:  PMBUS Communications State Machine
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
#include "pmbus_stack_handler.h"

//
// Defines
//
PMBUS_STACK_FILENUM(2)


//*****************************************************************************
//
// PMBusStack_slaveStateHandler
//
//*****************************************************************************
void PMBusStack_slaveStateHandler(PMBus_StackHandle handle)
{
    //
    // Read the status bits once at the start of each state handler
    // The status bits are cleared on a read, therefore, they should be
    // read once at the start of each ISR; any subsequent flags will cause
    // another interrupt
    //
    PMBusStackObject_setModuleStatus(handle,
                                     PMBus_getStatus(handle->moduleBase));

    switch(PMBusStackObject_getNextState(handle))
    {
        case PMBUS_STACK_STATE_IDLE:
            PMBusStack_slaveIdleStateHandler(handle);
            break;
        case PMBUS_STACK_STATE_RECEIVE_BYTE_WAIT_FOR_EOM:
            PMBusStack_slaveReceiveByteWaitForEOMStateHandler(handle);
            break;
        case PMBUS_STACK_STATE_READ_BLOCK:
            PMBusStack_slaveReadBlockStateHandler(handle);
            break;
        case PMBUS_STACK_STATE_READ_WAIT_FOR_EOM:
            PMBusStack_slaveReadWaitForEOMStateHandler(handle);
            break;
        case PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL:
            PMBusStack_slaveBlockWriteOrProcessCallStateHandler(handle);
            break;
        case PMBUS_STACK_STATE_EXTENDED_COMMAND:
            PMBusStack_slaveExtendedCommandStateHandler(handle);
            break;
        default:
            PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_IDLE);
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
            PMBusStack_slaveIdleStateHandler(handle);
            break;
    }
}

//
// End of File
//
