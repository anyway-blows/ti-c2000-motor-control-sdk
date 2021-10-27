//###########################################################################
//
// FILE:   pmbus_master_test.c
//
// TITLE:  Various master transactions for testing slave state machine
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

//
// Globals
//
PMBus_TestObject PMBUS_TESTS[NTESTS];
PMBus_TestHandle handle;

void (*initTestList[NTESTS])(PMBus_TestHandle) =
{
        PMBusMaster_initSendByteTest,
        PMBusMaster_initWriteByteTest,
        PMBusMaster_initWriteWordTest,
        PMBusMaster_initBlockWriteTest,
        PMBusMaster_initReceiveByteTest,
        PMBusMaster_initReadByteTest,
        PMBusMaster_initReadWordTest,
        PMBusMaster_initBlockReadTest,
        PMBusMaster_initBlockWriteReadProcessCallTest,
        PMBusMaster_initQuickCommandTest,
        PMBusMaster_initBlockWrite2BytesTest,
        PMBusMaster_initBlockWrite3BytesTest,
        PMBusMaster_initBlockRead3BytesTest,
        PMBusMaster_initProcessCallTest,
        PMBusMaster_initAlertTest,
        PMBusMaster_initNoAlertTest,
        PMBusMaster_initGroupCommandTest,
        PMBusMaster_initExtendedWriteByteTest,
        PMBusMaster_initExtendedWriteWordTest,
        PMBusMaster_initExtendedReadByteTest,
        PMBusMaster_initExtendedReadWordTest,
};

void (*runTestList[NTESTS])(PMBus_TestHandle) =
{
        PMBusMaster_runSendByteTest,
        PMBusMaster_runWriteByteTest,
        PMBusMaster_runWriteWordTest,
        PMBusMaster_runBlockWriteTest,
        PMBusMaster_runReceiveByteTest,
        PMBusMaster_runReadByteTest,
        PMBusMaster_runReadWordTest,
        PMBusMaster_runBlockReadTest,
        PMBusMaster_runBlockWriteReadProcessCallTest,
        PMBusMaster_runQuickCommandTest,
        PMBusMaster_runBlockWrite2BytesTest,
        PMBusMaster_runBlockWrite3BytesTest,
        PMBusMaster_runBlockRead3BytesTest,
        PMBusMaster_runProcessCallTest,
        PMBusMaster_runAlertTest,
        PMBusMaster_runNoAlertTest,
        PMBusMaster_runGroupCommandTest,
        PMBusMaster_runExtendedWriteByteTest,
        PMBusMaster_runExtendedWriteWordTest,
        PMBusMaster_runExtendedReadByteTest,
        PMBusMaster_runExtendedReadWordTest,
};

//
// End of File
//
