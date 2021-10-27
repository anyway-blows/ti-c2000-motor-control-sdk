//###########################################################################
//
// FILE:   pmbus_stack_handler_slave_idle.c
//
// TITLE:  PMBUS_STACK_STATE_IDLE handler
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
PMBUS_STACK_FILENUM(3)

//*****************************************************************************
//
// PMBusStack_slaveIdleStateHandler
//
//*****************************************************************************
void PMBusStack_slaveIdleStateHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t status = PMBusStackObject_getModuleStatus(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t nBytes = 0U;
    uint16_t *currentPointer;

    //
    // Set the current state to idle
    //
    PMBusStackObject_setCurrentState(handle, PMBUS_STACK_STATE_IDLE);

    //
    // Rewind the current pointer
    //
    PMBusStackObject_setCurrentPositionPointer(handle, buffer);

    //
    // Set the object transaction type
    //
    PMBusStackObject_setTransactionType(handle, PMBUS_TRANSACTION_NONE);

    //
    // Check if the unit is busy
    //
    if((status & PMBUS_PMBSTS_UNIT_BUSY) == 0U)
    {
        //
        // Neither PMBUS hardware nor firmware are stuck,
        //
        // The user must use the clock low timeout feature to
        // fire an interrupt off to the CPU, query the comms
        // stack object, take remedial measures and reset
        // the state machine prior to returning from the interrupt
        //
    }

    if((status & PMBUS_PMBSTS_DATA_READY) != 0U)
    {
        if((status & PMBUS_PMBSTS_EOM) != 0U)
        {
            //
            // EOM = 1, DATA_READY = 1
            //
            if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 2U)
            {
                //
                // RD_BYTE_COUNT = 2, EOM = 1, DATA_READY = 1
                // Send Byte (Receive buffer has Byte #0, PEC)
                //

                //
                // Read the data
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, nBytes);

                //
                // Set the object transaction type
                //
                PMBusStackObject_setTransactionType(handle,
                                                    PMBUS_TRANSACTION_SENDBYTE);

                //
                // Call the handler for the SENDBYTE transaction
                //
                (void)handle->
                transactionHandle[PMBUS_TRANSACTION_SENDBYTE](handle);

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
                // Write Byte (Receive buffer has Command, Byte #0, PEC)
                //

                //
                // Read the data
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, nBytes);

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
            else
            {
                //
                // RD_BYTE_COUNT = 1, EOM = 1, DATA_READY = 1
                // Unhandled exception
                //
                PMBUS_STACK_ASSERT(0);
            }
        }
        else
        {
            //
            // EOM = 0, DATA_READY = 1
            //
            if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                PMBUS_PMBSTS_RD_BYTE_COUNT_S) >= 3U)
            {
                //
                // RD_BYTE_COUNT = 3/4, EOM = 0, DATA_READY = 1
                // Read buffer is full, this must either be a write word,
                // block write or process call
                //
                // Read the RXBUF, ack the transaction and then proceed
                // to the BLOCK_WRITE_OR_PROCESS_CALL state
                //
                // Read the data
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Set the pointer in the buffer to point to the next
                // available spot
                //
                currentPointer = PMBusStackObject_getCurrentPositionPointer(handle);
                PMBusStackObject_setCurrentPositionPointer(handle,
                                                           &currentPointer[nBytes]);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, nBytes);

                //
                // ACK the entire transaction
                //
                PMBus_ackTransaction(base);

                //
                // Set the next state to
                // PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL
                //
                PMBusStackObject_setNextState(handle,
                                 PMBUS_STACK_STATE_BLOCK_WRITE_OR_PROCESS_CALL);
            }
            else if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                     PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 2U)
            {
                //
                // RD_BYTE_COUNT = 2, EOM = 0, DATA_READY = 1
                // This must be either an extended
                // - Read Byte
                // - Read Word
                // - Write Byte
                // - Write Word
                //

                //
                // Read the command
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // Check the first byte for the extended command byte
                //
                PMBUS_STACK_ASSERT((buffer[0] ==
                                    PMBUS_CMD_MFR_SPECIFIC_COMMAND) ||
                                   (buffer[0] == PMBUS_CMD_PMBUS_COMMAND_EXT));

                //
                // Set the pointer in the buffer to point to the next
                // available spot
                //
                PMBusStackObject_setCurrentPositionPointer(handle,
                                                           &buffer[nBytes]);

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, nBytes);

                //
                // ACK the command for the slave to be able to proceed
                //
                PMBus_ackTransaction(base);

                //
                // Transition to the state PMBUS_STACK_STATE_EXTENDED_COMMAND
                //
                PMBusStackObject_setNextState(handle,
                                            PMBUS_STACK_STATE_EXTENDED_COMMAND);
            }
            else if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                     PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 1U)
            {
                //
                // RD_BYTE_COUNT = 1, EOM = 0, DATA_READY = 1
                // This could be either a
                // - Read Byte
                // - Read Word
                // - Block Read
                // The command determines which read transaction is being
                // requested by the master. We need to wait for DATA_REQUEST
                // to come in
                //

                //
                // Read the command
                //
                nBytes = PMBus_getSlaveData(base, buffer, status);

                //
                // ACK the command, for the master to be able to issue
                // the repeated start, and therby the DATA_REQUEST interrupt
                //
                PMBus_ackTransaction(base);

                //
                // Transition to the state PMBUS_STATE_READ_WAIT_FOR_EOM
                //
                PMBusStackObject_setNextState(handle,
                                           PMBUS_STACK_STATE_READ_WAIT_FOR_EOM);
            }
            else
            {
                //
                // Unhandled exception
                //
                PMBUS_STACK_ASSERT(0);
            }
        }
    }
    else
    {
        //
        // DATA_READY = 0
        //
        if((status & PMBUS_PMBSTS_EOM) == 0U)
        {
            //
            // DATA_READY = 0, EOM = 0
            //
            if((status & PMBUS_PMBSTS_DATA_REQUEST) != 0U)
            {
                //
                // DATA_READY = 0, EOM = 0, DATA_REQUEST = 1
                // This is a receive byte request
                // Set the object transaction type
                //
                PMBusStackObject_setTransactionType(handle,
                                                 PMBUS_TRANSACTION_RECEIVEBYTE);

                //
                // Call the handler for the RECEIVEBYTE transaction
                //
                (void)handle->
                transactionHandle[PMBUS_TRANSACTION_RECEIVEBYTE](handle);

                //
                // Get the number of bytes to transfer, buffer pointer
                //
                nBytes = PMBusStackObject_getNumOfBytes(handle);

                //
                // Put the data to the PMBUS Transmit buffer, with PEC
                //
                PMBus_putSlaveData(base, buffer, nBytes, true);

                //
                // Transition to the RECEIVE_BYTE_WAIT_FOR_EOM
                //
                PMBusStackObject_setNextState(handle,
                                   PMBUS_STACK_STATE_RECEIVE_BYTE_WAIT_FOR_EOM);
            }
            else
            {
                //
                // DATA_READY = 0, EOM = 0, DATA_REQUEST = 0
                // Unhandled Exception
                //
                PMBUS_STACK_ASSERT(0);
            }
        }
        else
        {
            //
            // DATA_READY = 0 EOM = 1
            //
            if((status & PMBUS_PMBSTS_DATA_REQUEST) != 0U)
            {
                //
                // DATA_READY = 0, EOM = 1, DATA_REQUEST = 1
                // Unhandled Exception
                //
                PMBUS_STACK_ASSERT(0);

            }
            else if (((status & PMBUS_PMBSTS_NACK) != 0U) &&
                     ((status & PMBUS_PMBSTS_ALERT_EDGE) != 0U))
            {
                //
                // DATA_READY = 0, EOM = 1, NACK = 1, ALERT_EDGE = 1
                // This is an alert response
                // ACK the transaction
                //
                PMBus_ackTransaction(base);
            }
            else if(((status & PMBUS_PMBSTS_RD_BYTE_COUNT_M) >>
                       PMBUS_PMBSTS_RD_BYTE_COUNT_S) == 0U)
            {
                //
                // RD_BYTE_COUNT = 0, EOM = 1, DATA_READY = 0, NACK = 0
                // Quick Command
                //

                //
                // Set the object's nBytes field
                //
                PMBusStackObject_setNumOfBytes(handle, 0U);

                //
                // Set the object transaction type
                //
                PMBusStackObject_setTransactionType(handle,
                                                PMBUS_TRANSACTION_QUICKCOMMAND);

                //
                // Call the handler for the QUCIKCOMMAND transaction
                //
                (void)handle->
                transactionHandle[PMBUS_TRANSACTION_QUICKCOMMAND](handle);

                //
                // ACK the entire transaction
                //
                PMBus_ackTransaction(base);
            }
            else
            {
                //
                // DATA_READY = 0, EOM = 1, DATA_REQUEST = 0,
                // RD_BYTE_COUNT != 0
                // Unhandled Exception
                //
                PMBUS_STACK_ASSERT(0);
            }
        }
    }
}

//
// End of File
//
