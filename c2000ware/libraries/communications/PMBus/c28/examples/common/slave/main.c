//###########################################################################
//
// FILE:   main.c
//
// TITLE:  PMBUS Slave Demo Code
//
// Note: Requires connection to PMBus master running master demo
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
#include <string.h>

//
// Globals
//
PMBus_StackObject pmbusStackSlave;
PMBus_StackHandle pmbusStackSlaveHandle = &pmbusStackSlave;
uint16_t pmbusSlaveBuffer[300U];
volatile uint16_t testsCompleted = 0U;
volatile uint16_t pass = 0U, fail = 0U;

//
// Function Prototypes
//
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc")
#else
#pragma CODE_SECTION(pmbusIntHandler, ".TI.ramfunc")
#endif // __cplusplus
__interrupt void pmbusIntHandler(void);

//
// Main
//
int main(void)
{
    //
    // Locals
    //
    uint16_t i;
    uint32_t moduleFreq = 0U;

#if defined(_FLASH)
    //
    // Setup the Flash banks
    //
    PMBusExample_setupFlash();
#endif //defined(_FLASH)

    //
    // Setup the system clocking
    //
    PMBusExample_setupSysCtrl();

    //
    // Enable the PMBUS GPIOs
    //
    PMBusExample_setupGPIO();

    //
    // Setup the interrupt tables, register PMBUS interrupt handler
    //
    PMBusExample_setupInterrupts(pmbusIntHandler);

    //
    // Transaction Handlers
    //
    // Set the default handlers, user must overwrite
    //
    for(i = 0U; i < NTRANSACTIONS; i++)
    {
        PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                        (PMBus_Transaction)i, (void (*)(void *))
                                        PMBusStack_defaultTransactionHandler);
    }

    //
    // Overwrite the send byte transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_SENDBYTE,
                              (void (*)(void *))PMBusSlave_sendByteTestHandler);

    //
    // Overwrite the write byte transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_WRITEBYTE,
                             (void (*)(void *))PMBusSlave_writeByteTestHandler);

    //
    // Overwrite the write word transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_WRITEWORD,
                             (void (*)(void *))PMBusSlave_writeWordTestHandler);

    //
    // Overwrite the block write transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_BLOCKWRITE,
                            (void (*)(void *))PMBusSlave_blockWriteTestHandler);

    //
    // Overwrite the Receive Byte transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_RECEIVEBYTE,
                           (void (*)(void *))PMBusSlave_receiveByteTestHandler);

    //
    // Overwrite the Read Byte transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_READBYTE,
                              (void (*)(void *))PMBusSlave_readByteTestHandler);

    //
    // Overwrite the Read Word transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_READWORD,
                              (void (*)(void *))PMBusSlave_readWordTestHandler);

    //
    // Overwrite the Block Read transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                           PMBUS_TRANSACTION_BLOCKREAD,
                             (void (*)(void *))PMBusSlave_blockReadTestHandler);

    //
    // Overwrite the Block Write Read Process Call transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                 PMBUS_TRANSACTION_BLOCKWRPC, (void (*)(void *))
                               PMBusSlave_blockWriteReadProcessCallTestHandler);

    //
    // Overwrite the Quick Command transaction handler
    //
    PMBusStackObject_setTransactionHandler(pmbusStackSlaveHandle,
                                          PMBUS_TRANSACTION_QUICKCOMMAND,
                         (void (*)(void *))PMBusSlave_quickCommandTestHandler);

    //
    // Set the buffer to some default non-zero value
    //
    memset(pmbusSlaveBuffer, 0xBAADU, sizeof(pmbusSlaveBuffer));

    //
    // Setup the PMBUS Stack Object
    //
    PMBusStackObject_setMode(pmbusStackSlaveHandle, PMBUS_STACK_MODE_SLAVE);

    //
    // Set the slave address and mask
    //
    PMBusStackObject_setSlaveAddress(pmbusStackSlaveHandle, SLAVE_ADDRESS);
    PMBusStackObject_setSlaveAddressMask(pmbusStackSlaveHandle,
                                         SLAVE_ADDRESSMASK);

    //
    // Initialize the state machine handler
    //
    PMBusStack_initModule(pmbusStackSlaveHandle, PMBUSA_BASE,
                          &pmbusSlaveBuffer[0]);

    //
    // Configure the PMBUS module clock to be PMBUS_MODULE_FREQ_MAX
    //
    moduleFreq = PMBus_configModuleClock(PMBUSA_BASE, PMBUS_MODULE_FREQ_MAX,
                                         PMBUS_SYS_FREQ_MAX);

    //
    // Configure the PMBUS bus clock
    ///
    PMBus_configBusClock(PMBUSA_BASE, PMBUS_CLOCKMODE_STANDARD, moduleFreq);

    //
    // Loop until tests complete
    //
    while(testsCompleted != NENABLEDTESTS)
    {
        if(fail > 0U)
        {
            //
            // Upon fail, halt debugger
            //
            __asm(" ESTOP0");
        }
    };

    //
    // Tests complete - Enter wait loop
    //
    done();

    //
    // Code does not reach this point
    //
    return(1);
}

//
// pmbusIntHandler - This is the main PMBus interrupt handler. Calls slave
//                   stack state machine.
//
__interrupt void pmbusIntHandler(void)
{
    //
    // Call the library State Machine handler
    //
    PMBusStack_slaveStateHandler(pmbusStackSlaveHandle);

    //
    // ACK any pending interrupts (if any got set)
    //
    Interrupt_clearACKGroup(PMBUS_INT_ACK);
}

//
// End of File
//
