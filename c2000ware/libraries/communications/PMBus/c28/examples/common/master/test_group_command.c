//###########################################################################
//
// FILE:   test_group_command.c
//
// TEST:   Group Command Test
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
// PMBusMaster_initGroupCommandTest
//
//*****************************************************************************
void PMBusMaster_initGroupCommandTest(PMBus_TestHandle handle)
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
    // Set count to 3
    //
    handle->count = 3U;

    //
    // Enable the test
    //
#if ENABLE_TEST_17 == 1
    handle->enabled = true;
#else
    handle->enabled = false;
#endif //ENABLE_TEST_17 == 1
}

//*****************************************************************************
//
// PMBusMaster_runGroupCommandTest
//
//*****************************************************************************
void PMBusMaster_runGroupCommandTest(PMBus_TestHandle handle)
{
    //
    // Clear the spot where the alerting slave's address will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Add command (index 0) and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_VOUT_MIN;
    pmbusMasterBuffer[1] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V3;
    pmbusMasterBuffer[2] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V4;

    //
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable group command,
    // enable command, 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS, TEST17_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_GRP_CMD |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], handle->count);

    //
    // Wait on DATA_REQUEST
    //
    while(masterDataRequested == false)
    {
        //
        // Fail if EOM & NACK after the 1st message
        //
        if(endOfMessage == true && slaveAckReceived == false)
        {
            handle->fail++;
            break;
        }
    }

    //
    // Reset data requested flag
    //
    masterDataRequested = false;

    //
    // Check if slave ACK is received
    //
    (slaveAckReceived == true) ? handle->pass++ : handle->fail++;

    //
    // Clear the spot where the alerting slave's address will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Add command (index 0) and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_VOUT_MAX;
    pmbusMasterBuffer[1] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V3;
    pmbusMasterBuffer[2] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V4;

    //
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable group command,
    // enable command, 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS + 1U, TEST17_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_GRP_CMD |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], handle->count);

    //
    // Wait on DATA_REQUEST
    //
    while(masterDataRequested == false)
    {
        //
        // exit if EOM & NACK after the 2nd message
        // This slave doesnt exist on the bus and master will see
        // a NACK for this address, which generateds the EOM.
        //
        if((endOfMessage == true) && (slaveAckReceived == false))
        {
            handle->pass++;
            break;
        }
    }

    //
    // Reset flags
    //
    PMBusMaster_resetGlobalFlags();

    //
    // Clear the spot where the alerting slave's address will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Add command (index 0) and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_IOUT_MAX;
    pmbusMasterBuffer[1] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V3;
    pmbusMasterBuffer[2] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V4;

    //
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), enable group command,
    // enable command, 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS + 2U, TEST17_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_GRP_CMD |
                        PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], handle->count);

    //
    // Wait on DATA_REQUEST
    //
    while(masterDataRequested == false)
    {
        //
        // exit if EOM & NACK after the 3rd message
        // This slave doesnt exist on the bus and master will see
        // a NACK for this address, which generateds the EOM.
        //
        if((endOfMessage == true) && (slaveAckReceived == false))
        {
            handle->pass++;
            break;
        }
    }

    //
    // Reset flags
    //
    PMBusMaster_resetGlobalFlags();

    //
    // Clear the spot where the alerting slave's address will go
    //
    memset(&pmbusMasterBuffer[0], 0U, handle->count * sizeof(uint16_t));

    //
    // Add command (index 0) and
    // word data to buffer (index 1 and 2)
    //
    pmbusMasterBuffer[0] = PMBUS_CMD_MFR_POUT_MAX;
    pmbusMasterBuffer[1] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V3;
    pmbusMasterBuffer[2] = pmbusMasterBuffer[0] ^ CMD_XOR_MASK_V4;

    //
    // Configure the master to enable PEC, enable Write (by omitting the read
    // option from the configWord, you enable write), disable group command
    // (by ommitting it), enable command, 2 bytes
    //
    PMBus_configMaster(PMBUSA_BASE, SLAVE_ADDRESS + 3U, TEST17_2BYTES,
                       (PMBUS_MASTER_ENABLE_PEC | PMBUS_MASTER_ENABLE_CMD));

    //
    // Send data
    //
    PMBus_putMasterData(PMBUSA_BASE, &pmbusMasterBuffer[0], handle->count);

    //
    // Wait for the EOM, and slave to ack the address before
    // reading data -- done in the ISR
    //
    while(endOfMessage == false)
    {
    }

    //
    // if EOM & NACK after the 4th message
    // This slave doesnt exist on the bus and master will see
    // a NACK for this address, which generateds the EOM.
    //
    if((endOfMessage == true) && (slaveAckReceived == false))
    {
        handle->pass++;
    }

    //
    // Check that expected pass counter value is correct
    //
    if(handle->pass != 4U)
    {
        handle->fail++;
    }

    return;
}

//
// End of File
//
