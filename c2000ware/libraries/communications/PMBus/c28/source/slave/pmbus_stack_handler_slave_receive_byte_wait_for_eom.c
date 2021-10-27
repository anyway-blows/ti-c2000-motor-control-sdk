//###########################################################################
//
// FILE:   pmbus_stack_handler_slave_receive_byte_wait_for_eom.c
//
// TITLE:  PMBUS_STACK_STATE_RECEIVE_BYTE_WAIT_FOR_EOM handler
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
PMBUS_STACK_FILENUM(7)

//*****************************************************************************
//
// PMBusStack_slaveReceiveByteWaitForEOMStateHandler
//
//*****************************************************************************
void PMBusStack_slaveReceiveByteWaitForEOMStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);

    //
    // Set the current state to RECEIVE_BYTE_WAIT_FOR_EOM
    //
    PMBusStackObject_setCurrentState(handle,
                                   PMBUS_STACK_STATE_RECEIVE_BYTE_WAIT_FOR_EOM);

    if(((status & PMBUS_PMBSTS_EOM) != 0U) &&
       ((status & PMBUS_PMBSTS_NACK) != 0U))
    {
        //
        // DATA_READY = 0, EOM = 1, DATA_REQUEST = 0, NACK = 1
        // After a DATA_REQUEST for a receive byte is seen by the slave,
        // it places the requested byte (and PEC) on the line and transitions
        // from the idle to the recevie byte wait-for-eom state
        //
        // The only interrupt expected after the recevied byte is that from an
        // EOM. The quick command also has EOM=1 and DATA_READY=0 at the end.
        // In order to distinguish between the two, if an EOM is received while
        // in this state it was a receive byte, if an EOM was recevied in the
        // idle state it was a quick command
        //
        // NOTE: ACK is required to allow the slave to ack its address in
        // any subsequent transaction.
        //
        PMBus_ackTransaction(base);

        //
        // Return to the idle state
        //
        PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);

        return;
    }
    else
    {
        //
        // Unhandled Exception
        //
        PMBUS_STACK_ASSERT(0);
    }
}

//
// End of File
//
