//###########################################################################
//
// FILE:   handle_extended_read_word.c
//
// TITLE:  Read Word (Extended)
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
// PMBusSlave_extendedReadWordTestHandler
//
//*****************************************************************************
void PMBusSlave_extendedReadWordTestHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    uint16_t command = buffer[1];
    PMBus_Transaction transaction = PMBusStackObject_getTransactionType(handle);
    uint16_t preRunPass = pass;

    //
    // Set the number of bytes to send
    //
    PMBusStackObject_setNumOfBytes(handle, 2U);

    //
    // Send message to master
    //
    buffer[0] = command ^ CMD_XOR_MASK_V1;
    buffer[1] = command ^ CMD_XOR_MASK_V2;

    //
    // Check transaction type
    //
    (transaction == PMBUS_TRANSACTION_READWORD) ? pass++: fail++;

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
