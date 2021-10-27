//###########################################################################
//
// FILE:   pmbus_stack_handler_extended_command.c
//
// TITLE: PMBUS_STACK_STATE_EXTENDED_COMMAND handler
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
PMBUS_STACK_FILENUM(8)

//*****************************************************************************
//
// PMBusStack_slaveExtendedCommandStateHandler
//
//*****************************************************************************
void PMBusStack_slaveExtendedCommandStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t command = buffer[1U];
    uint16_t nBytes = 0U;

    //
    // Set the current state to PMBUS_STACK_STATE_EXTENDED_COMMAND
    //
    PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_EXTENDED_COMMAND);

    if((status & PMBUS_PMBSTS_DATA_REQUEST) != 0U)
    {
        //
        // DATA_REQUEST = 1 (an extended read)
        //
        if(PMBusStackObject_isCommandAndTransactionValid(command,
           PMBUS_TRANSACTION_READBYTE))
        {
            //
            // Set the object transaction type
            //
            PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_READBYTE);

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
            PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_READWORD);

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
        else
        {
            //
            // Unhandled exception
            //
            PMBUS_STACK_ASSERT(0);
        }
    }
    else if((status & PMBUS_PMBSTS_EOM) != 0U)
    {
        //
        // EOM = 1, DATA_REQUEST = 0
        //
        if(((status & PMBUS_PMBSTS_NACK) != 0U) &&
           ((status & PMBUS_PMBSTS_DATA_READY) == 0U))
        {
            //
            // EOM = 1, NACK = 1, DATA_REQUEST = 0, DATA_READY = 0
            // Master NACKs the extended read, ack and
            // change state to IDLE
            //
            PMBus_ackTransaction(base);
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
        }
        else if(((status & PMBUS_PMBSTS_NACK) == 0U) &&
                ((status & PMBUS_PMBSTS_DATA_READY) != 0U))
        {
            //
            // EOM = 1, DATA_READY = 1, NACK = 0, DATA_REQUEST = 0
            // Master completes either an ext write byte/word
            //

             //
            // Set buffer pointer to the current pointer
            //
            buffer = PMBusStackObject_getCurrentPositionPointer(handle);

            if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 2U)
            {
                //
                // RD_BYTE_COUNT = 2, EOM = 1, DATA_READY = 1
                // extended write Byte (Receive buffer has Byte & PEC)
                //

                //
                // Read the data
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Set the object's nBytes field
                // Set the pointer in the buffer to point to the next
                // available spot
                //
                PMBusStackObject_setCurrentPositionPointer(handle,
                                                           &buffer[nBytes]);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, (nBytes +
                                       PMBusStackObject_getNumOfBytes(handle)));

                //
                // Set the object transaction type
                //
                PMBusStackObject_setTransactionType(handle,
                                                   PMBUS_TRANSACTION_WRITEBYTE);

                //
                // Call the handler for the WRITEBYTE transaction
                //
                (void)handle->
                transactionHandle[PMBUS_TRANSACTION_WRITEBYTE](handle);

                //
                // ACK the entire transaction
                //
                PMBus_ackTransaction(base);
            }
            else if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                     PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 3U)
            {
                //
                // RD_BYTE_COUNT = 3, EOM = 1, DATA_READY = 1
                // Write word (Receive buffer has Byte #1, Byte #2, PEC)
                //

                //
                // Read the data
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Set the object's nBytes field
                // Set the pointer in the buffer to point to the next
                // available spot
                //
                PMBusStackObject_setCurrentPositionPointer(handle,
                                                           &buffer[nBytes]);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, (nBytes +
                                       PMBusStackObject_getNumOfBytes(handle)));

                //
                // Set the object transaction type
                //
                PMBusStackObject_setTransactionType(handle,
                                                   PMBUS_TRANSACTION_WRITEWORD);

                //
                // Call the handler for the WRITEWORD transaction
                //
                (void)handle->
                transactionHandle[PMBUS_TRANSACTION_WRITEWORD](handle);

                //
                // ACK the entire transaction
                //
                PMBus_ackTransaction(base);
            }
            else
            {
                //
                // Unhandled exception
                //
                PMBUS_STACK_ASSERT(0);
            }

            //
            // Set next state to IDLE
            //
            PMBusStackObject_setNextState(handle, PMBUS_STACK_STATE_IDLE);
        }
        else
        {
            //
            // Unhandled exception
            //
            PMBUS_STACK_ASSERT(0);
        }
    }
    else
    {
        //
        // EOM = 0, DATA_REQUEST = 0, DATA_READY = X, NACK = X
        // Unhandled exception
        //
        PMBUS_STACK_ASSERT(0);
    }
}

//
// End of File
//
