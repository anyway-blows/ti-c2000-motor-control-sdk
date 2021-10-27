//#############################################################################
// File Name        :   fcl_tformat_f28004x_config.c
//#############################################################################
//
// Contains all the initialization, data declarations and setup for tformat
// encoder interface. This file serves are a template for using PM_tformat
// Library to interface and incorporates all the encoder and library specific
// initializations and other important aspects of usage.
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// includes
//
#include "fcl_tformat_f28004x_config.h"

#ifdef _FLASH
#ifndef __cplusplus
#pragma CODE_SECTION(spiRxFIFOISR,".TI.ramfunc");
#pragma CODE_SECTION(readTformatEncPosition,".TI.ramfunc");
#pragma CODE_SECTION(tformat_checkCRC,".TI.ramfunc");
#endif
#endif

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
    PM_tformat_generateCRCTable(PM_TFORMAT_NBITS_POLY1,
                                PM_TFORMAT_POLY1,
                                tformatCRCtable);

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
    SPI_disableModule(ENCODER_SPI_BASE);
    SPI_disableInterrupt(ENCODER_SPI_BASE, SPI_INT_RXFF);

    //
    // FIFO and interrupt configuration
    //
    SPI_clearInterruptStatus(ENCODER_SPI_BASE, SPI_INT_RXFF);
    SPI_enableInterrupt(ENCODER_SPI_BASE, SPI_INT_RXFF);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(ENCODER_SPI_BASE);

    //
    // Power up tformat 5v supply through GPIO139
    //
    GPIO_writePin(ENC_PWREN_PIN, 1);
    SysCtl_delay(20000000L);
    PM_tformat_setFreq(TFORMAT_FREQ_DIVIDER);
}

void tformat_setupGPIO(void)
{

    //
    // GPIO7 is SPI Clk slave
    //
    GPIO_setMasterCore(ENC_CLK_PWM_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_CLK_PWM_CFG);

    //
    // GPIO63 is the SPISIMOB
    //
    GPIO_setMasterCore(ENC_SPI_SIMO_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_SPI_SIMO_CFG);
    GPIO_setQualificationMode(ENC_SPI_SIMO_PIN, GPIO_QUAL_ASYNC);

    //
    // GPIO64 is the SPISOMIB
    //
    GPIO_setMasterCore(ENC_SPI_SOMI_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_SPI_SOMI_CFG);
    GPIO_setQualificationMode(ENC_SPI_SOMI_PIN, GPIO_QUAL_ASYNC);

    //
    // GPIO65 is the SPICLKB
    //
    GPIO_setMasterCore(ENC_SPI_CLK_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_SPI_CLK_CFG);
    GPIO_setQualificationMode(ENC_SPI_CLK_PIN, GPIO_QUAL_ASYNC);

    //
    // GPIO66 is the SPISTEB
    //
    GPIO_setMasterCore(ENC_SPI_STE_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_SPI_STE_CFG);
    GPIO_setQualificationMode(ENC_SPI_STE_PIN, GPIO_QUAL_ASYNC);

    //
    // GPIO9 is tformat TxEN
    //
    GPIO_setMasterCore(ENC_TXEN_PIN, GPIO_CORE_CPU1);
    GPIO_setPinConfig(ENC_TXEN_CFG);  // out x bar

    //
    // GPIO139 is PwrEN
    //
    GPIO_setMasterCore(ENC_PWREN_PIN, GPIO_CORE_CPU1);
    GPIO_setDirectionMode(ENC_PWREN_PIN, GPIO_DIR_MODE_OUT);
}

void tformat_configXBAR(void)
{
    XBAR_setInputPin(XBAR_INPUT1, ENC_SPI_SIMO_PIN); // xbar in

    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX01_INPUTXBAR1);
    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX01);
    XBAR_setOutputMuxConfig(XBAR_OUTPUT1, XBAR_OUT_MUX13_CLB4_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT1, XBAR_MUX13);
}


void tformat_error(void)
{
    ESTOP0;     //Test failed!! Stop!
    for (;;);
}


interrupt void spiRxFIFOISR(void)
{
    uint16_t i;

    for(i = 0;i <= tformatData.fifoLevel;i++)
    {
        tformatData.rdata[i] = SPI_readDataNonBlocking(PM_TFORMAT_SPI);
    }

    readTformatEncPosition();

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

uint16_t tformat_checkCRC (uint16_t expectCRC5, uint16_t receiveCRC5)
{
    if(expectCRC5 == receiveCRC5)
    {
        return(1);
    }
    else
    {
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

    retval1 = PM_tformat_setupCommand(PM_TFORMAT_DATAIDD,
                                      address, 0, crcResult);
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

    retval1 = PM_tformat_setupCommand(PM_TFORMAT_DATAID6,
                                      address, data, crcResult);
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

    retval1 = PM_tformat_setupCommand(PM_TFORMAT_DATAIDD,
                                      address, 0, crcResult);
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
