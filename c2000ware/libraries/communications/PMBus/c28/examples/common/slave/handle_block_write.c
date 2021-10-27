//###########################################################################
//
// FILE:   handle_block_write.c
//
// TITLE:  Block Write (255 bytes)
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
// PMBusSlave_blockWriteTestHandler
//
//*****************************************************************************
void PMBusSlave_blockWriteTestHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint16_t nBytes = PMBusStackObject_getNumOfBytes(handle);
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    PMBus_Transaction transaction = PMBusStackObject_getTransactionType(handle);
    uint16_t i;
    uint16_t preRunPass = pass;

    //
    // Check received data, transaction type, and PEC
    //
    (nBytes == 258U) ? pass++ : fail++;
    (handle->bufferPointer[0U] == PMBUS_CMD_VIN_OV_FAULT_LIMIT) ? pass++ : 
                                                                  fail++;
    (handle->bufferPointer[1U] == DATA_BYTE_COUNT) ? pass++ : fail++;
    (transaction == PMBUS_TRANSACTION_BLOCKWRITE) ? pass++: fail++;

    for(i = 0U; i < DATA_BYTE_COUNT; i++)
    {
        (handle->bufferPointer[i + 2U] == i) ? pass++ : fail++;
    }

    if(PMBus_verifyPEC(base, buffer, (uint16_t *)PMBus_crc8Table, nBytes - 1,
                       buffer[257U]))
    {
        pass++;
    }
    else
    {
        fail++;
    }

    //
    // Change the block write handler for the next test
    //
    PMBusStackObject_setTransactionHandler(handle, PMBUS_TRANSACTION_BLOCKWRITE,
                      (void (*)(void *))PMBusSlave_blockWrite3BytesTestHandler);

    //
    // Check that expected pass counter value is correct
    //
    if((pass - preRunPass) != 260U)
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
