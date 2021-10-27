//###########################################################################
//
// FILE:   test_write_byte.c
//
// TITLE:  Write Byte Test
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
// PMBusMaster_initWriteByteTest
//
//*****************************************************************************
void PMBusMaster_initWriteByteTest(PMBus_TestHandle handle)
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
#if ENABLE_TEST_2 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_2 == 1
}

//*****************************************************************************
//
// PMBusMaster_runWriteByteTest
//
//*****************************************************************************
void PMBusMaster_runWriteByteTest(PMBus_TestHandle handle)
{
    //
    // Put command (index 0) into buffer and 1 byte of data (index 1)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_VIN_OV_FAULT_LIMIT;
    pmbusMasterBuffer[1] = TEST2_DATA_BYTE;

    //
    // Do a write byte
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable command,
    // 1 byte
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 1U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], 2U);

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
