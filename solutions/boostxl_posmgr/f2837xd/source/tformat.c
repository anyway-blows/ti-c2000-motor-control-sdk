//-----------------------------------------------------------------------------
//  FILE:           tformat.c
//
//  Description:    Contains all the initialization, data declarations
//                  and setup for tformat encoder interface.
//                  This file serves are a template for using PM_tformat
//                  Library to interface and incorporates all the encoder
//                  and library specific initializations and other important
//                  aspects of usage.
//
//  Version:        1.0
//
//  Target:         TMS320F28377D,
//
//-----------------------------------------------------------------------------
//  Copyright Texas Instruments 锟� 2004-2015
//-----------------------------------------------------------------------------
//  Revision History:
//-----------------------------------------------------------------------------
//  Date      | Description / Status
//-----------------------------------------------------------------------------
// Feb 2019  - Example project for PM tformat Library Usage
//-----------------------------------------------------------------------------

#include "tformat.h"

uint16_t tformatCRCtable[PM_TFORMAT_CRCTABLE_SIZE];  //table - CRC calculations

PM_tformat_DataStruct tformatData;        //PM tformat data structure

//
// Function to initialize tformat operation
//
void tformat_init(void) {

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);

    //
    //Configure EPWM4 to drive default values on GPIO7
    //
    tformat_configEPWM4();

    //
    // Generate table for tformat Polynomial defied as POLY1
    //
    PM_tformat_generateCRCTable(PM_TFORMAT_NBITS_POLY1, PM_TFORMAT_POLY1, tformatCRCtable);

    //
    //GPIO configuration for tformat operation
    //
    tformat_setupGPIO();

    //
    //XBAR configuration for tformat operation
    //
    tformat_configXBAR();

    PM_tformat_setupPeriph(DEVICE_LSPCLK_FREQ);

    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIB_BASE);
    SPI_disableInterrupt(SPIB_BASE, SPI_INT_RXFF);

    //
    // FIFO and interrupt configuration
    //
    SPI_clearInterruptStatus(SPIB_BASE, SPI_INT_RXFF);
    SPI_enableInterrupt(SPIB_BASE, SPI_INT_RXFF);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIB_BASE);

    //
    // Power up tformat 5v supply through GPIO139
    //
    GPIO_writePin(139, 1);
    SysCtl_delay(2000000L);
    PM_tformat_setFreq(TFORMAT_FREQ_DIVIDER);
}

void tformat_setupGPIO(void) {

    //
    // GPIO7 is SPI Clk slave
    //
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_EPWM4B);

    //
    // GPIO63 is the SPISIMOB
    //
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_SPISIMOB);
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);

    //
    // GPIO64 is the SPISOMIB
    //
    GPIO_setMasterCore(64, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_64_SPISOMIB);
    GPIO_setQualificationMode(64, GPIO_QUAL_ASYNC);

    //
    // GPIO65 is the SPICLKB
    //
    GPIO_setMasterCore(65, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_65_SPICLKB);
    GPIO_setQualificationMode(65, GPIO_QUAL_ASYNC);

    //
    // GPIO66 is the SPISTEB
    //
    GPIO_setMasterCore(66, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_66_SPISTEB);
    GPIO_setQualificationMode(66, GPIO_QUAL_ASYNC);

    //
    // GPIO9 is tformat TxEN
    //
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_OUTPUTXBAR6);

    //
    // GPIO139 is PwrEN
    //
    GPIO_setMasterCore(139, GPIO_CORE_CPU1);
    GPIO_setDirectionMode(139, GPIO_DIR_MODE_OUT);
}

void tformat_configXBAR(void)
{
    //
    // Connect InputXbar-INPUT1 to GPIO63 - SPISIMO
    //

    XBAR_setInputPin(XBAR_INPUT1, 63);

    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX01_INPUTXBAR1);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX01);
    XBAR_setOutputMuxConfig(XBAR_OUTPUT6, XBAR_OUT_MUX13_CLB4_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT6, XBAR_MUX13);
}


void tformat_error(void) {
    ESTOP0;     //Test failed!! Stop!
    for (;;);
}

interrupt void spiRxFIFOISR(void)
{
    uint16_t i;
    for (i = 0;i <= tformatData.fifoLevel;i++)
    {
        tformatData.rdata[i] = SPI_readDataNonBlocking(PM_TFORMAT_SPI);
    }
    SPI_clearInterruptStatus(PM_TFORMAT_SPI, SPI_INT_RXFF_OVERFLOW);
    SPI_clearInterruptStatus(PM_TFORMAT_SPI, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
    tformatData.dataReady = 1;
}


void tformat_configEPWM4(void) {

    //
    // Set the PWMA and B high as default values of tformat clk.
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_HIGH);


    //
    // Action on TZ1
    //
    EPWM_setTripZoneAction(EPWM4_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_HIGH);

    //
    // Forces a Trip Zone event
    //
    EPWM_forceTripZoneEvent(EPWM4_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

uint16_t tformat_checkCRC (uint16_t expectCRC5, uint16_t receiveCRC5) {
    if(expectCRC5 == receiveCRC5) {
        return(1);
    }
    else {
        return(0);
    }
}

//
//This function executes all the tformat commands by calling the tformat
//library functions with different IDs.
//This is an optional function that can be used as a sanity checker
//This function is of no relevance during runtime
//At the end of each command execution CRC is also checked.
//

uint16_t tformat_exCommands(void)
{

    uint16_t retval1; //used for function return val storage and checks
    uint32_t address, address_tmp, data, data_tmp;
    uint32_t crcResult;

    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAID2, 0, 0, 0);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) ;//{;}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAID2);

    crcResult = PM_tformat_getCRC(0, 24, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 3);
    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }

    SysCtl_delay(40000L); //DELAY_US(400L);
    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAID0, 0, 0, 0);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) {}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAID0);

    crcResult = PM_tformat_getCRC(0, 40, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 5);

    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }

    SysCtl_delay(40000L); //DELAY_US(400L);
    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAID1, 0, 0, 0);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) {}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAID1);

    crcResult = PM_tformat_getCRC(0, 40, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 5);
    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }

    SysCtl_delay(40000L); //DELAY_US(400L);
    address = 1;
    address_tmp = (__flip32(address) >> 24) & 0xFE; // includes busy "0"
    tformatData.rxPkts[0] = ((((uint32_t) PM_TFORMAT_DATAIDD) | 0x40) << 8) |
                            (uint32_t) address_tmp;
    crcResult = PM_tformat_getCRC(0, 16, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 2);
    crcResult = (crcResult) ^ (0xFF);

    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAIDD, address, 0, crcResult);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) {}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAIDD);

    crcResult = PM_tformat_getCRC(0, 32, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 4);
    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }

    SysCtl_delay(40000L); //DELAY_US(400L);
    address = 1; data = 35;
    address_tmp = (__flip32(address) >> 24) & 0xFE; // includes busy "0"
    data_tmp = (__flip32(data) >> 24) & 0xFF;
    tformatData.rxPkts[0] = ((((uint32_t) PM_TFORMAT_DATAID6) | 0x40) << 16) |
                            (((uint32_t) address_tmp) << 8) |
                            ((uint32_t) data_tmp);
    crcResult = PM_tformat_getCRC(0, 24, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 3);
    crcResult = (crcResult) ^ (0xFF);

    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAID6, address, data, crcResult);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) {}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAID6);

    crcResult = PM_tformat_getCRC(0, 32, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 4);
    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }

    SysCtl_delay(40000L); //DELAY_US(400L);
    address = 1;
    address_tmp = (__flip32(address) >> 24) & 0xFE; // includes busy "0"
    tformatData.rxPkts[0] = ((((uint32_t) PM_TFORMAT_DATAIDD) | 0x40) << 8) |
                            (uint32_t) address_tmp;
    crcResult = PM_tformat_getCRC(0, 16, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 2);
    crcResult = (crcResult) ^ (0xFF);

    retval1 = PM_tformat_setupCommand (PM_TFORMAT_DATAIDD, address, 0, crcResult);
    PM_tformat_startOperation();
    while(tformatData.dataReady != 1) {}
    retval1 = PM_tformat_receiveData(PM_TFORMAT_DATAIDD);

    crcResult = PM_tformat_getCRC(0, 32, 8, (uint16_t *)&tformatData.rxPkts,
                                  tformatCRCtable, 4);
    crcResult = crcResult ^ (0xFF);
     if(!tformat_checkCRC(crcResult, tformatData.crc))
     {
             ESTOP0;
     }
     SysCtl_delay(4000L);
     return(retval1);

}

//
// End of file
//
