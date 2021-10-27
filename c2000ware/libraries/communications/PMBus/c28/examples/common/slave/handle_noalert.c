//###########################################################################
//
// FILE:    handle_noalert.c
//
// TITLE:   Alert (different slave on the line, this slave does not respond)
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
// PMBusSlave_noAlertTestHandler
//
//*****************************************************************************
void PMBusSlave_noAlertTestHandler(PMBus_StackHandle handle)
{
    //
    // Locals
    //
    uint32_t base = PMBusStackObject_getModuleBase(handle);
    uint16_t *buffer = PMBusStackObject_getBufferPointer(handle);
    PMBus_Transaction transaction = PMBusStackObject_getTransactionType(handle);
    uint16_t addressedAs = PMBus_getOwnAddress(base);
    PMBus_accessType aType = PMBus_getCurrentAccessType(base);
    uint16_t preRunPass = pass;

    //
    // Check transaction type
    //
    if(transaction == PMBUS_TRANSACTION_QUICKCOMMAND)
    {
        pass++;
    }
    else
    {
        fail++;
    }

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
