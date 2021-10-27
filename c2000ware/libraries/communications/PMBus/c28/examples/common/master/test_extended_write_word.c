//###########################################################################
//
// FILE:   test_extended_write_word.c
//
// TITLE:  Write Word (Extended) Test
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
// PMBusMaster_initExtendedWriteWordTest
//
//*****************************************************************************
void PMBusMaster_initExtendedWriteWordTest(PMBus_TestHandle handle)
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
#if ENABLE_TEST_19 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_19 == 1
}

//*****************************************************************************
//
// PMBusMaster_runExtendedWriteWordTest
//
//*****************************************************************************
void PMBusMaster_runExtendedWriteWordTest(PMBus_TestHandle handle)
{
    //
    // Add extended command (index 0), command (index 1), and
    // word data to buffer (index 2 and 3)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_SPECIFIC_COMMAND;
    pmbusMasterBuffer[1] = PMBUS_CMD_VOUT_COMMAND;
    pmbusMasterBuffer[2] = PMBUS_CMD_VOUT_COMMAND ^ CMD_XOR_MASK_V1;
    pmbusMasterBuffer[3] = PMBUS_CMD_VOUT_COMMAND ^ CMD_XOR_MASK_V2;

    //
    // Do an extended write word
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable extended command,
    // 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, TEST19_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_EXT_CMD |
                        PMBUS_MASTER_ENABLE_CMD));
                        
    //
    // Send extended command, command, and 1 word of data
    //    
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], TEST19_4BYTES);

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
