//###########################################################################
//
// FILE:   handle_block_read.c
//
// TITLE:  Block Read (255 bytes)
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
#include "pmbus_slave_test.h"

//*****************************************************************************
//
// PMBusSlave_blockReadTestHandler
//
//*****************************************************************************
void PMBusSlave_blockReadTestHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint16_t command = buffer[0];
    PMBus_Transaction transaction = PMBusStackObject_getTransactionType(handle);
    uint16_t i;
    uint16_t preRunPass = pass;

    //
    // Set the number of bytes to send (including the byte count)
    //
    PMBusStackObject_setNumOfBytes(handle, 256U);

    //
    // Set the byte cont as the first byte to be transmitted
    //
    buffer[0] = 255U;

    //
    // Send message to master
    //
    for(i = 1U; i <= 255U; i++)
    {
        buffer[i] = command ^ i;
    }

    //
    // Check transaction type
    //
    (transaction == PMBUS_TRANSACTION_BLOCKREAD) ? pass++: fail++;

    //
    // Change the handler for the next test
    //
    PMBusStackObject_setTransactionHandler(handle, PMBUS_TRANSACTION_BLOCKREAD,
                       (void (*)(void *))PMBusSlave_blockRead3BytesTestHandler);

    //
    // Check that expected pass counter value is correct
    //
    if((pass - preRunPass) != 1U)
    {
        fail++;
    }

    //
    // Update number of completed tests
    //
    testsCompleted++;
}

//
// End of File
//
