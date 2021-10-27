//###########################################################################
//
// FILE:   test_extended_write_byte.c
//
// TITLE:  Write Byte (Extended)
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
// PMBusMaster_initExtendedWriteByteTest
//
//*****************************************************************************
void PMBusMaster_initExtendedWriteByteTest(PMBus_TestHandle handle)
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
#if ENABLE_TEST_18 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_18 == 1
}

//*****************************************************************************
//
// PMBusMaster_runExtendedWriteByteTest
//
//*****************************************************************************
void PMBusMaster_runExtendedWriteByteTest(PMBus_TestHandle handle)
{
    //
    // Add extended command (index 0), command (index 1), and
    // byte data to buffer (index 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_SPECIFIC_COMMAND;
    pmbusMasterBuffer[1] = PMBUS_CMD_STORE_DEFAULT_CODE;
    pmbusMasterBuffer[2] = PMBUS_CMD_STORE_DEFAULT_CODE ^ CMD_XOR_MASK_V1;

    //
    // Do an extended write byte
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable extended command,
    // 1 byte
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, 1U,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_EXT_CMD |
                        PMBUS_MASTER_ENABLE_CMD));
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
