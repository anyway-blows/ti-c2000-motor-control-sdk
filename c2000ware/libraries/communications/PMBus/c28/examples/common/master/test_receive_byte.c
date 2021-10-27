//###########################################################################
//
// FILE:   test_receive_byte.c
//
// TITLE:  Receive Byte Test
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
#include "pmbus_master_test.h"

//*****************************************************************************
//
// PMBusMaster_initReceiveByteTest
//
//*****************************************************************************
void PMBusMaster_initReceiveByteTest(PMBus_TestHandle handle)
{
    //
    // Reset flags
    //
    PMBusMaster_resetGlobalFlags();

    //
    // Reset test object
    //
    PMBusMaster_resetTestObject(handle);

    //
    // Set count to 1
    //
    handle->count = 1;

    //
    // Enable the test
    //
#if ENABLE_TEST_5 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_5 == 1
}

//*****************************************************************************
//
// PMBusMaster_runReceiveByteTest
//
//*****************************************************************************
void PMBusMaster_runReceiveByteTest(PMBus_TestHandle handle)
{
    //
    // Clear the spot where the command will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Do a receive byte
    // Configure the master to enable PEC, enable Read
    // Expect 1 byte
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 1U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_READ));

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
    }

    //
    // Get received data
    //
    PMBus_getMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], pmbusStatus);

    //
    // Check the received byte and data
    //
    (bytesReceived == 1U) ? handle->pass++ : handle->fail++;
    (pmbusMasterBuffer[0] == TEST5_DATA_BYTE) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 2U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
