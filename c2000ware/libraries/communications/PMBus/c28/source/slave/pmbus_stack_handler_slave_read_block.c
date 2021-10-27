//###########################################################################
//
// FILE:   pmbus_stack_handler_slave_read_block.c
//
// TITLE:  PMBUS_STACK_STATE_READ_BLOCK handler
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
PMBUS_STACK_FILENUM(4)

//*****************************************************************************
//
// PMBusStack_slaveReadBlockStateHandler
//
//*****************************************************************************
void PMBusStack_slaveReadBlockStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t nBytes = PMBusStackObject_getNumOfBytes(handle);
    uint16_t *currentPointer;
    uint16_t bytesTransmitted = 0U;
    uint16_t bytesToTransmit = 0U;

    //
    // Set the current state to READ_BLOCK
    //
    PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_READ_BLOCK);

    if(((status & PMBUS_PMBSTS_EOM) != 0U) &&
       ((status & PMBUS_PMBSTS_NACK) != 0U))
    {
        //
        // EOM = 1, NACK = 1
        // Master has prematurely terminated the block read,
        // or in the READ/WAIT_FOR_EOM state, only 4 bytes
        // were required to be sent, and the transaction was complete
        // NOTE: ACK is required to allow the slave to ack its address in
        // any subsequent transaction.
        //
        PMBus_ackTransaction(base);

        //
        // Return to the idle state
        //
        PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
    }
    else if((status & PMBUS_PMBSTS_DATA_REQUEST) != 0U)
    {
        //
        // DATA_REQUEST = 1, EOM = 0, NACK = 0
        //
        currentPointer = PMBusStackObject_getCurrentPositionPointer(handle);
        bytesTransmitted = (uint16_t)(currentPointer - buffer);
        bytesToTransmit = nBytes - bytesTransmitted;

        if(bytesToTransmit > 4U)
        {
            //
            // Put the data to the PMBUS Transmit buffer, without PEC
            //
            PMBus_putSlaveData(base, currentPointer, 4U, false);

            //
            // Update the current index into the buffer
            //
            PMBusStackObject_setCurrentPositionPointer(handle,
                                                       &currentPointer[4U]);
        }
        else //bytesToTransmit <= 4U (the last transmission)
        {
            //
            // Put the last few bytes (<=4) to the PMBUS Transmit buffer,
            // with PEC
            //
            PMBus_putSlaveData(base, currentPointer, bytesToTransmit, true);

            //
            // Return to the EOM State
            //
            PMBusStackObject_setNextState(handle,
                                          PMBUS_STACK_STATE_READ_WAIT_FOR_EOM);
        }
    }
    else
    {
        //
        // Unhandled exception
        //
        PMBUS_STACK_ASSERT(0);
    }
}

//
// End of File
//
