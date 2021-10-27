//###########################################################################
//
// FILE:   test_write_word.c
//
// TITLE:  Write Word Test
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
// PMBusMaster_initWriteWordTest
//
//*****************************************************************************
void PMBusMaster_initWriteWordTest(PMBus_TestHandle handle)
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
    // Enable the test
    //
#if ENABLE_TEST_3 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_3 == 1
}

//*****************************************************************************
//
// PMBusMaster_runWriteWordTest
//
//*****************************************************************************
void PMBusMaster_runWriteWordTest(PMBus_TestHandle handle)
{
    //
    // Add command (index 0)  and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_VIN_OV_FAULT_LIMIT;
    pmbusMasterBuffer[1] = TEST3_DATA_BYTE_1;
    pmbusMasterBuffer[2] = TEST3_DATA_BYTE_2;

    //
    // Do a write word
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable command,
    // 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 2U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 3U);

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
    }

    //
    // Once the bus is free, if the slave NACK'd its a failure
    //
    (slaveAckReceived == true) ? handle->pass++ : handle->fail++;

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 1U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
