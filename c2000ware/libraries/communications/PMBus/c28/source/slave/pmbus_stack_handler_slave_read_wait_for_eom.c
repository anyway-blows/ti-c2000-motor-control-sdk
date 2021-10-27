//###########################################################################
//
// FILE:   pmbus_stack_handler_slave_read_wait_for_eom.c
//
// TITLE:  PMBUS_STACK_STATE_READ_WAIT_FOR_EOM handler
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
PMBUS_STACK_FILENUM(6)

//*****************************************************************************
//
// PMBusStack_slaveReadWaitForEOMStateHandler
//
//*****************************************************************************
void PMBusStack_slaveReadWaitForEOMStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t nBytes = 0U;
    uint16_t command = buffer[0U];

    //
    // Set the current state to Read/Wait for EOM
    //
    PMBusStackObject_setCurrentState(handle,
                                     PMBUS_STACK_STATE_READ_WAIT_FOR_EOM);

    if(((status & PMBUS_PMBSTS_EOM) != 0U) &&
       ((status & PMBUS_PMBSTS_NACK) != 0U))
    {
        //
        // EOM = 1, NACK = 1
        // NOTE: ACK is required to allow the slave to ack its address in
        // any subsequent transaction.
        //
        PMBus_ackTransaction(base);

        //
        // Read byte/word/block was successful, return to the
        // Idle state
        //
        PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
    }
    //
    // At this point the command is already in the buffer
    //
    else if(PMBusStackObject_isCommandAndTransactionValid(command,
                                                    PMBUS_TRANSACTION_READBYTE))
    {
        //
        // Set the object transaction type
        //
        PMBusStackObject_setTransactionType(handle, PMBUS_TRANSACTION_READBYTE);

        //
        // Call the handler for the READBYTE transaction
        //
        (void)handle->transactionHandle[PMBUS_TRANSACTION_READBYTE](handle);

        //
        // Get the number of bytes to transfer, buffer pointer
        //
        nBytes = PMBusStackObject_getNumOfBytes(handle);

        //
        // Put the data to the PMBUS Transmit buffer, with PEC
        //
        PMBus_putSlaveData(base, buffer, nBytes, true);
    }
    else if(PMBusStackObject_isCommandAndTransactionValid(command,
                                                    PMBUS_TRANSACTION_READWORD))
    {
        //
        // Set the object transaction type
        //
        PMBusStackObject_setTransactionType(handle, PMBUS_TRANSACTION_READWORD);

        //
        // Call the handler for the READWORD transaction
        //
        (void)handle->transactionHandle[PMBUS_TRANSACTION_READWORD](handle);

        //
        // Get the number of bytes to transfer, buffer pointer
        //
        nBytes = PMBusStackObject_getNumOfBytes(handle);

        //
        // Put the data to the PMBUS Transmit buffer, with PEC
        //
        PMBus_putSlaveData(base, buffer, nBytes, true);
    }
    else if(PMBusStackObject_isCommandAndTransactionValid(command,
                                                   PMBUS_TRANSACTION_BLOCKREAD))
    {
        //
        // Set the object transaction type
        //
        PMBusStackObject_setTransactionType(handle, PMBUS_TRANSACTION_BLOCKREAD);

        //
        // Call the handler for the READBLOCK transaction
        //
        (void)handle->transactionHandle[PMBUS_TRANSACTION_BLOCKREAD](handle);

        //
        // Get the number of bytes to transfer, buffer pointer
        //
        nBytes = PMBusStackObject_getNumOfBytes(handle);

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
        // Unhandled exception
        //
        PMBUS_STACK_ASSERT(0);
    }
}

//
// End of File
//
