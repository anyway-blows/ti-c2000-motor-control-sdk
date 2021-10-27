//###########################################################################
//
// FILE:   pmbus_stack_handler_slave_block_write_process_call.c
//
// TITLE:  PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL handler
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
PMBUS_STACK_FILENUM(5)

//*****************************************************************************
//
// PMBusStack_slaveBlockWriteOrProcessCallStateHandler
//
//*****************************************************************************
void PMBusStack_slaveBlockWriteOrProcessCallStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getCurrentPositionPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t nBytes = 0U;

    //
    // Set the current state to block write/Process Call
    //
    PMBusStackObject_setCurrentState(handle,
                                 PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL);

    if((status & PMBUS_PMBSTS_EOM) != 0U)
    {
        //
        // EOM = 1
        //
        if((status & PMBUS_PMBSTS_DATA_READY) != 0U)
        {
            //
            // RD_BYTE_COUNT != 0, EOM = 1, DATA_READY = 1, DATA_REQUEST = 0
            // This must be the last few bytes of the block write command
            // or an extended write byte/word command
            //

            //
            // Start saving data in the buffer from the currPtr position
            // Read the data
            //
            nBytes = PMBus_getSlaveData(base, buffer, status);

            //
            // Set the pointer in the buffer to point to the next
            // available spot
            //
            PMBusStackObject_setCurrentPositionPointer(handle, &buffer[nBytes]);

            //
            // Set the object's nBytes field
            //
            PMBusStackObject_setNumOfBytes(handle, (nBytes +
                                       PMBusStackObject_getNumOfBytes(handle)));

            //
            // Set the object transaction type
            //
            PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_BLOCKWRITE);

            //
            // Call the handler for the BLOCKWRITE transaction
            //
            handle->transactionHandle[PMBUS_TRANSACTION_BLOCKWRITE](handle);

            //
            // ACK the entire transaction
            //
            PMBus_ackTransaction(base);

            //
            // Set the next state to PMBUS_STACK_STATE_IDLE
            //
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
        }
        else
        {
            //
            // RD_BYTE_COUNT = 0, EOM = 1, DATA_READY = 0, DATA_REQUEST = 0
            // End-of-message, no data ready, no data requested, no bytes
            // received. This must be a write word command
            //

            //
            //ACK the entire transaction
            //
            PMBus_ackTransaction(base);

            //
            // Set the object transaction type
            //
            PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_WRITEWORD);

            //
            // Call the handler for the WRITEWORD transaction
            //
            handle->transactionHandle[PMBUS_TRANSACTION_WRITEWORD](handle);

            //
            // Set the next state to PMBUS_STACK_STATE_IDLE
            //
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
        }
    }
    else
    {
        //
        // EOM = 0
        //
        if((status & PMBUS_PMBSTS_DATA_READY) != 0U)
        {
            //
            // EOM = 0, DATA_READY = 1 RD_BYTE_COUNT = ?
            // Start saving data in the buffer from the currPtr position
            // Read the data
            //
            nBytes = PMBus_getSlaveData(base, buffer, status);

            //
            // Set the pointer in the buffer to point to the next
            // available spot
            //
            PMBusStackObject_setCurrentPositionPointer(handle, &buffer[nBytes]);

            //
            // Set the object's nBytes field
            //
            PMBusStackObject_setNumOfBytes(handle, (nBytes +
                                       PMBusStackObject_getNumOfBytes(handle)));

            //
            // ACK the entire transaction
            //
            PMBus_ackTransaction(base);

            //
            // Stay in the same state, i.e.
            // PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL
            //
        }
        else if ((status & PMBUS_PMBSTS_DATA_REQUEST) != 0U)
        {
            //
            // EOM = 0, DATA_READY = 0 RD_BYTE_COUNT  = 0, DATA_REQUEST = 1
            // This is a Block Write, Read, Process Call
            // Set the object transaction type
            //
            PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_BLOCKWRPC);

            //
            // Call the handler for the BLOCKWRPC transaction
            //
            (void)handle->transactionHandle[PMBUS_TRANSACTION_BLOCKWRPC](handle);

            //
            // Get the number of bytes to transfer, buffer pointer
            //
            nBytes = PMBusStackObject_getNumOfBytes(handle);

            //
            // Rewind the buffer pointer
            //
            buffer = PMBusStackObject_getBufferPointer(handle);

            if(nBytes <= 4U)
            {
                //
                // Put the data to the PMBUS Transmit buffer, with PEC
                //
                PMBus_putSlaveData(base, buffer, nBytes, true);
            }
            else // nBytes > 4
            {
                //
                // Put the data to the PMBUS Transmit buffer, without PEC
                //
                PMBus_putSlaveData(base, buffer, 4U, false);

                //
                // Update the current index into the buffer
                //
                PMBusStackObject_setCurrentPositionPointer(handle, &buffer[4U]);
            }
            //
            // Proceed to the READ_BLOCK state
            //
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_READ_BLOCK);
        }
        else
        {
            //
            // EOM = 0, DATA_READY = ? RD_BYTE_COUNT = ?, DATA_REQUEST = 0
            // A Fault condition
            //
            PMBUS_STACK_ASSERT(0);
        }
    }
}

//
// End of File
//
