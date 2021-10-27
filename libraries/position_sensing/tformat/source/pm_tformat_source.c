//#############################################################################
//
// FILE:   PM_tformat_Source.c
//
// Description:Example project for using PM tformat Library.
//              Includes PM_tformat_lib library and corresponding include files
//              Initializes the encoders and performs delay compensation.
//              Runs tformat command set.
//              Continuously Read position value in an infinite loop
//
//  Target:     TMS320F28379D
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2020 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################


#include "driverlib.h"
#include "stdint.h"
#include "PM_tformat_include.h"
#include "PM_tformat_internal_include.h"

#include "clb_config.h"
#include "clb.h"

uint16_t tformat_tile1HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE1_HLCINSTR_0,
    TILE1_HLCINSTR_1,
    TILE1_HLCINSTR_2,
    TILE1_HLCINSTR_3,
    TILE1_HLCINSTR_4,
    TILE1_HLCINSTR_5,
    TILE1_HLCINSTR_6,
    TILE1_HLCINSTR_7,
    TILE1_HLCINSTR_8,
    TILE1_HLCINSTR_9,
    TILE1_HLCINSTR_10,
    TILE1_HLCINSTR_11,
    TILE1_HLCINSTR_12,
    TILE1_HLCINSTR_13,
    TILE1_HLCINSTR_14,
    TILE1_HLCINSTR_15,
    TILE1_HLCINSTR_16,
    TILE1_HLCINSTR_17,
    TILE1_HLCINSTR_18,
    TILE1_HLCINSTR_19,
    TILE1_HLCINSTR_20,
    TILE1_HLCINSTR_21,
    TILE1_HLCINSTR_22,
    TILE1_HLCINSTR_23,
    TILE1_HLCINSTR_24,
    TILE1_HLCINSTR_25,
    TILE1_HLCINSTR_26,
    TILE1_HLCINSTR_27,
    TILE1_HLCINSTR_28,
    TILE1_HLCINSTR_29,
    TILE1_HLCINSTR_30,
    TILE1_HLCINSTR_31
};

void tformat_initCLB4(void);
void tformat_resetCLB(void);

void PM_tformat_setupPeriph(uint32_t devLSPCLKFreq)
{
    tformat_resetCLB();
    tformat_initCLB4();

    CLB_configLocalInputMux(CLB4_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB4_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);

    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN1,
                             CLB_GLOBAL_IN_MUX_CLB_AUXSIG0);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM1A);
    CLB_configGlobalInputMux(CLB4_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM1A);

    CLB_configGPInputMux(CLB4_BASE, CLB_IN0, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN6, CLB_GP_IN_MUX_EXTERNAL);
    CLB_configGPInputMux(CLB4_BASE, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

    CLB_enableSynchronization(CLB4_BASE, CLB_IN0);
    CLB_enableSynchronization(CLB4_BASE, CLB_IN1);

    CLB_selectInputFilter(CLB4_BASE, CLB_IN0, CLB_FILTER_RISING_EDGE);
    CLB_selectInputFilter(CLB4_BASE, CLB_IN1, CLB_FILTER_FALLING_EDGE);

    tformat_initSPIFIFO(devLSPCLKFreq);   // Initialize the SPI only
    tformat_initCLBXBAR();   // Initialize the CLB XBAR
}

uint16_t PM_tformat_setupCommand(uint16_t tformatDataID, uint16_t eepromAddr,
                                 uint16_t eepromData, uint16_t crcSend)
{
    uint16_t sendClks;
    uint16_t receiveClks;
    uint16_t dummySPIClks;
    uint16_t i;
    uint16_t word1, word2, word3, temp;
    uint16_t addrl, datal;

    tformat_resetCLB();
    tformatData.dataReady = 0;

    word1 = (tformatDataID << 6) | 0x903F;

    switch(tformatDataID)
    {
        case (PM_TFORMAT_DATAID0):
        case (PM_TFORMAT_DATAID1):
        case (PM_TFORMAT_DATAID7):
        case (PM_TFORMAT_DATAID8):
        case (PM_TFORMAT_DATAIDC):

            sendClks = ABIT + PACKET_BITS + IDLE_BITS;
            receiveClks = 6 * PACKET_BITS ;

            if(((sendClks + receiveClks) % 16) == 0)
            {
                tformatData.fifoLevel =  ((sendClks + receiveClks) / 16);
            }
            else
            {
                tformatData.fifoLevel = ((sendClks + receiveClks) / 16) + 1;
            }

            dummySPIClks = (tformatData.fifoLevel * 16) -
                            (sendClks + receiveClks);

            tformat_configureSPILen(15);

            //
            //void tformat_configureCLBLen(uint16_t CLB4_C1_M1,
            // uint16_t CLB4_C1_M2, uint16_t CLB4_R0)
            //
            tformat_configureCLBLen((PACKET_BITS + 2), sendClks,
                                    receiveClks + dummySPIClks);

            tformatData.sdata[0] = word1;
            tformatData.sdata[1] = 0xFFFF;
            tformatData.sdata[2] = 0xFFFF;
            tformatData.sdata[3] = 0xFFFF;
            for (i = 0;i < tformatData.fifoLevel;i++)
            {
                SPI_writeDataNonBlocking(PM_TFORMAT_SPI, tformatData.sdata[i]);
            }

        break;

        case (PM_TFORMAT_DATAID2):

            sendClks = ABIT + PACKET_BITS + IDLE_BITS;
            receiveClks = 4 * PACKET_BITS ;

            if(((sendClks + receiveClks) % 16) == 0)
            {
                tformatData.fifoLevel =  ((sendClks + receiveClks) / 16);
            }
            else
            {
                tformatData.fifoLevel = ((sendClks + receiveClks) / 16) + 1;
            }

            dummySPIClks = (tformatData.fifoLevel * 16) -
                           (sendClks + receiveClks);

            tformat_configureSPILen(15);

            tformat_configureCLBLen((PACKET_BITS + 2), sendClks,
                                    receiveClks + dummySPIClks);

            tformatData.sdata[0] = word1;
            tformatData.sdata[1] = 0xFFFF;
            tformatData.sdata[2] = 0xFFFF;
            tformatData.sdata[3] = 0xFFFF;
            for (i = 0;i < tformatData.fifoLevel;i++)
            {
                SPI_writeDataNonBlocking(PM_TFORMAT_SPI, tformatData.sdata[i]);
            }

        break;

        case (PM_TFORMAT_DATAID3):

            sendClks = ABIT + PACKET_BITS + IDLE_BITS;
            receiveClks = 11 * PACKET_BITS ;

            if(((sendClks + receiveClks) % 16) == 0)
            {
                tformatData.fifoLevel =  ((sendClks + receiveClks) / 16);
            }
            else
            {
                tformatData.fifoLevel = ((sendClks + receiveClks) / 16) + 1;
            }
            dummySPIClks = (tformatData.fifoLevel * 16) -
                            (sendClks + receiveClks);

            tformat_configureSPILen(15);

            //
            //void tformat_configureCLBLen(uint16_t CLB4_C1_M1,
            //uint16_t CLB4_C1_M2, uint16_t CLB4_R0)
            //
            tformat_configureCLBLen((PACKET_BITS + 2), sendClks,
                                    receiveClks + dummySPIClks);

            tformatData.sdata[0] = word1;
            tformatData.sdata[1] = 0xFFFF;
            tformatData.sdata[2] = 0xFFFF;
            tformatData.sdata[3] = 0xFFFF;
            for (i = 0;i < tformatData.fifoLevel;i++)
                {
                    SPI_writeDataNonBlocking(PM_TFORMAT_SPI,
                    tformatData.sdata[i]);
                }

        break;

        case (PM_TFORMAT_DATAIDD):
            sendClks = ABIT + 3 * PACKET_BITS + IDLE_BITS;
            receiveClks = 4 * PACKET_BITS ;

            addrl = (__flip32(eepromAddr) >> 24) & 0xFE;    // busy "0"
            temp = (addrl >> 4) & 0xF;
            word1 = (tformatDataID << 6) | 0x9020 | temp;
            temp = (addrl  & 0xF) << 12;
            word2 = temp | 0x803 | (crcSend << 2);

            if(((sendClks + receiveClks) % 16) == 0)
            {
                tformatData.fifoLevel =  ((sendClks + receiveClks) / 16);
            }
            else
            {
                tformatData.fifoLevel = ((sendClks + receiveClks) / 16) + 1;
            }
            dummySPIClks = (tformatData.fifoLevel * 16) -
                    (sendClks + receiveClks);

            tformat_configureSPILen(15);

            //
            //void tformat_configureCLBLen(uint16_t CLB4_C1_M1,
            //uint16_t CLB4_C1_M2, uint16_t CLB4_R0)
            //
            tformat_configureCLBLen((3 * PACKET_BITS + 2), sendClks,
                    receiveClks + dummySPIClks);

            tformatData.sdata[0] = word1;
            tformatData.sdata[1] = word2;
            tformatData.sdata[2] = 0xFFFF;
            tformatData.sdata[3] = 0xFFFF;
            for (i = 0;i < tformatData.fifoLevel;i++)
            {
                SPI_writeDataNonBlocking(PM_TFORMAT_SPI, tformatData.sdata[i]);
            }

        break;

        case (PM_TFORMAT_DATAID6):

            sendClks = ABIT + 4 * PACKET_BITS + IDLE_BITS;
            receiveClks = 4 * PACKET_BITS ;

            addrl = (__flip32(eepromAddr) >> 24) & 0xFE;    // busy "0"
            datal = (__flip32(eepromData) >> 24) & 0xFF;
            temp = (addrl >> 4) & 0xF;
            word1 = (tformatDataID << 6) | 0x9020 | temp;
            temp = (addrl  & 0xF) << 12;
            word2 = temp | 0x802 | (datal << 2);
            word3 = (crcSend << 8) | 0xFF;

            if(((sendClks + receiveClks) % 16) == 0)
            {
                tformatData.fifoLevel =  ((sendClks + receiveClks) / 16);
            }
            else
            {
                tformatData.fifoLevel = ((sendClks + receiveClks) / 16) + 1;
            }

            dummySPIClks = (tformatData.fifoLevel * 16) -
                    (sendClks + receiveClks);

            tformat_configureSPILen(15);

            tformat_configureCLBLen((4 * PACKET_BITS + 2), sendClks,
                                    receiveClks + dummySPIClks);

            tformatData.sdata[0] = word1;
            tformatData.sdata[1] = word2;
            tformatData.sdata[2] = word3;
            tformatData.sdata[3] = 0xFFFF;
            for (i = 0;i < tformatData.fifoLevel;i++)
            {
                SPI_writeDataNonBlocking(PM_TFORMAT_SPI, tformatData.sdata[i]);
            }

        break;

        default:
            word1 = 0;
        break;

    }
    return(word1);
}

uint16_t PM_tformat_receiveData(uint16_t tformatDataID)
{
    uint32_t len, bitIndex, wordNo;
    uint16_t word1;

    word1  = 1;

    switch(tformatDataID)
    {
        case (PM_TFORMAT_DATAID0):
        case (PM_TFORMAT_DATAID1):
        case (PM_TFORMAT_DATAID7):
        case (PM_TFORMAT_DATAID8):
        case (PM_TFORMAT_DATAIDC):

            len = 8;

            bitIndex = 13;
            wordNo = 1;
            tformatData.controlField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.statusField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField0 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField1 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField2 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.crc = tformat_getBits (len, wordNo, bitIndex);

            tformatData.rxPkts[0] = ((uint32_t) tformatData.controlField << 24)
                                    | ((uint32_t) tformatData.statusField << 16)
                                    | ((uint32_t) tformatData.dataField0 << 8)
                                    | ((uint32_t) tformatData.dataField1);

            tformatData.rxPkts[1] = ((uint32_t) tformatData.dataField2) ;

        break;

        case (PM_TFORMAT_DATAID2):
            len = 8;

            bitIndex = 13;
            wordNo = 1;
            tformatData.controlField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.statusField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField0 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.crc = tformat_getBits (len, wordNo, bitIndex);

            tformatData.rxPkts[0] = ((uint32_t) tformatData.controlField << 16)
                                    | ((uint32_t) tformatData.statusField << 8)
                                    | ((uint32_t) tformatData.dataField0);
        break;

        case (PM_TFORMAT_DATAID3):
            len = 8;
            bitIndex = 13;
            wordNo = 1;
            tformatData.controlField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.statusField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField0 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField1 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField2 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField3 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField4 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField5 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField6 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.dataField7 = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.crc = tformat_getBits (len, wordNo, bitIndex);

            tformatData.rxPkts[0] = ((uint32_t) tformatData.controlField << 24)
                                  | ((uint32_t) tformatData.statusField << 16)
                                  | ((uint32_t) tformatData.dataField0 << 8)
                                  | (uint32_t) tformatData.dataField1;

            tformatData.rxPkts[1] = ((uint32_t) tformatData.dataField2 << 24) |
                                    ((uint32_t) tformatData.dataField3 << 16) |
                                    ((uint32_t) tformatData.dataField4 << 8) |
                                    ((uint32_t) tformatData.dataField5);

            tformatData.rxPkts[2] = ((uint32_t) tformatData.dataField6 << 8) |
                                      ((uint32_t) tformatData.dataField7);

        break;

        case (PM_TFORMAT_DATAIDD):
            len = 8;

            bitIndex = 9;
            wordNo = 2;
            tformatData.controlField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.eepromAddress = tformat_getBits (len,
                            wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.eepromRdDtata = tformat_getBits (len,
                            wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.crc = tformat_getBits (len, wordNo, bitIndex);

            tformatData.rxPkts[0] = ((uint32_t) tformatData.controlField << 16)
                                  | ((uint32_t) tformatData.eepromAddress << 8)
                                  | ((uint32_t) tformatData.eepromRdDtata);
        break;

        case (PM_TFORMAT_DATAID6):

            len = 8;

            bitIndex = 15;
            wordNo = 3;
            tformatData.controlField = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.eepromAddress = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.eepromRdDtata = tformat_getBits (len, wordNo, bitIndex);

            wordNo = wordNo + (10 + (16 - bitIndex) ) / 16;
            bitIndex = 16 - ((10 + (16 - bitIndex)) % 16) ;
            tformatData.crc = tformat_getBits (len, wordNo, bitIndex);

            tformatData.rxPkts[0] = ((uint32_t) tformatData.controlField << 16)
                                  | ((uint32_t) tformatData.eepromAddress << 8)
                                  | ((uint32_t) tformatData.eepromRdDtata);
            break;

    }

    return(word1);
}

uint32_t tformat_getBits (uint16_t len, uint16_t wordNo, uint16_t bitIndex)
{
    uint32_t shift0, shift1, shift2, mask0, mask1, mask2;
    uint32_t temp32;

    shift2 = (len > (16 + bitIndex) ) ? (32 - len + bitIndex): 32;
    shift1 = (len > (16 + bitIndex) ) ? (16 - shift2) : (len > bitIndex ) ?
            (16 - len + bitIndex): 32;
    shift0 = (len > (16 + bitIndex) ) ? (16 + shift1) : (len > bitIndex ) ?
            (16 - shift1): (bitIndex - len);

    mask2 = (len > (16 + bitIndex) ) ? 0xFFFFFFFF : 0;
    mask1 = (len > bitIndex ) ? 0xFFFFFFFF : 0;
    mask0 = (len > bitIndex )? ((1 << bitIndex) - 1): ((1 << len) - 1) ;

    temp32 = (len > (16 + bitIndex) ) ? (((tformatData.rdata[wordNo + 2] &
            mask2) >> shift2) | ((tformatData.rdata[wordNo + 1] & mask1)
            << shift1) | ((tformatData.rdata[wordNo] & mask0) << shift0))
            : (len > bitIndex ) ? (((tformatData.rdata[wordNo + 1] & mask1)
            >> shift1) | ((tformatData.rdata[wordNo] & mask0) << shift0))
            : ((tformatData.rdata[wordNo] >> shift0) & mask0) ;

    return(temp32);
}


void PM_tformat_setFreq(uint32_t freq_us)
{
    EALLOW;
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_0_MATCH1, (freq_us - 1));
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_0_MATCH2,
            ((2 * freq_us) - 1));
    CLB_setGPREG(CLB4_BASE, 0x80);
    EDIS;
}

void PM_tformat_startOperation(void)
{
    EALLOW;
    HWREG(CLB4_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) |= CLB_LOAD_EN_GLOBAL_EN
            | CLB_LOAD_EN_STOP;

    __asm(" RPT #10 || NOP");
    CLB_setOutputMask(CLB4_BASE, 0x3C, true);
    __asm(" RPT #10 || NOP");
    CLB_setGPREG(CLB4_BASE, 0x81);
}


void tformat_putBits (uint32_t data, uint16_t wordNo, uint16_t bitIndex)
{

    tformatData.sdata[wordNo] = (data >> (25 - bitIndex) )
                                & ((1L << bitIndex) - 1);
    if((25 - bitIndex) > 16)
    {
        tformatData.sdata[wordNo + 1] = (data >> (25 - bitIndex - 16) )
                                        & ((1L << 16) - 1);
        tformatData.sdata[wordNo + 2] = data &
                                        ((1 << (25 - bitIndex - 16)) - 1);
    }
    else
    {
        tformatData.sdata[wordNo + 1] = (data & ((1 << (25 - bitIndex)) - 1))
                                        << (16 - (25 - bitIndex));
    }


}

void tformat_configureSPILen(uint16_t wordLen)
{
    SPI_disableModule(PM_TFORMAT_SPI);
    SPI_enableModule(PM_TFORMAT_SPI);

    HWREGH(PM_TFORMAT_SPI + SPI_O_CCR) |= wordLen;
    HWREGH(PM_TFORMAT_SPI + SPI_O_FFRX) = (HWREGH(PM_TFORMAT_SPI + SPI_O_FFRX)
                                          & (~SPI_FFRX_RXFFIL_M))
                                          | (uint16_t)tformatData.fifoLevel;

    HWREGH(PM_TFORMAT_SPI + SPI_O_FFTX) = (HWREGH(PM_TFORMAT_SPI + SPI_O_FFTX)
                                          & (~SPI_FFTX_TXFFIL_M))
                                          | (uint16_t)tformatData.fifoLevel;


    SPI_resetTxFIFO(PM_TFORMAT_SPI);
    SPI_disableModule(PM_TFORMAT_SPI);
    SPI_enableModule(PM_TFORMAT_SPI);
}

void tformat_resetCLB(void)
{

    CLB_setGPREG(CLB4_BASE, 0);

    //
    // Turn OFF the CLB functionality
    //
    EALLOW;
    HWREG(CLB4_BASE + CLB_LOGICCTL + CLB_O_LOAD_EN) = 0;
    EDIS;

    //
    // Clear Counters - this will clear counter REGs. Add_shift_on_even_en
    // should be Zero for this to take effect.
    //
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_0_LOAD, 0x0);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_1_LOAD, 0x0);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_2_LOAD, 0x0);

    //
    // Clear and reload the HLC registers
    //
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_HLC_R0, 0x9);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_HLC_R1, 0x0);
    CLB_writeInterface(CLB4_BASE, CLB_ADDR_COUNTER_1_MATCH1, 0x2);

    CLB_setOutputMask(CLB4_BASE, 0, false);

}


void tformat_initSPIFIFO(uint32_t devLSPCLKFreq) {

//
// Initialize SPI FIFO registers
//
    SPI_disableModule(PM_TFORMAT_SPI);
    SPI_disableLoopback(PM_TFORMAT_SPI);

    SPI_setConfig(PM_TFORMAT_SPI, devLSPCLKFreq, SPI_PROT_POL1PHA0,
                SPI_MODE_SLAVE, 500000, 9);

    SPI_clearInterruptStatus(PM_TFORMAT_SPI, SPI_INT_RX_OVERRUN |
                             SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF |
                             SPI_INT_RXFF_OVERFLOW | SPI_INT_TXFF);
    SPI_enableFIFO(PM_TFORMAT_SPI);
    SPI_setFIFOInterruptLevel(PM_TFORMAT_SPI, SPI_FIFO_TX3, SPI_FIFO_RX3);
    SPI_setEmulationMode(PM_TFORMAT_SPI, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(PM_TFORMAT_SPI);
    SPI_resetTxFIFO(PM_TFORMAT_SPI);
    SPI_resetRxFIFO(PM_TFORMAT_SPI);

    SPI_enableInterrupt(PM_TFORMAT_SPI, SPI_INT_RX_OVERRUN |
                        SPI_INT_RX_DATA_TX_EMPTY | SPI_INT_RXFF
                        | SPI_INT_RXFF_OVERFLOW);
}

void tformat_initCLBXBAR() {

//    XBAR_setCLBMuxConfig(XBAR_AUXSIG0, XBAR_CLB_MUX01_INPUTXBAR1);
//    XBAR_enableCLBMux(XBAR_AUXSIG0, XBAR_MUX01);
//    XBAR_setOutputMuxConfig(XBAR_OUTPUT6, XBAR_OUT_MUX13_CLB4_OUT4);
//    XBAR_enableOutputMux(XBAR_OUTPUT6, XBAR_MUX13);
}

void tformat_initCLB4(void)
{
    uint16_t i;

//
// Output LUT
//
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT0, TILE1_CFG_OUTLUT_0);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT1, TILE1_CFG_OUTLUT_1);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT2, TILE1_CFG_OUTLUT_2);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT3, TILE1_CFG_OUTLUT_3);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT4, TILE1_CFG_OUTLUT_4);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT5, TILE1_CFG_OUTLUT_5);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT6, TILE1_CFG_OUTLUT_6);
    CLB_configOutputLUT(CLB4_BASE, CLB_OUT7, TILE1_CFG_OUTLUT_7);

    //
    // LUT4
    //
    CLB_selectLUT4Inputs(CLB4_BASE, TILE1_CFG_LUT4_IN0, TILE1_CFG_LUT4_IN1,
                         TILE1_CFG_LUT4_IN2, TILE1_CFG_LUT4_IN3);
    CLB_configLUT4Function(CLB4_BASE, TILE1_CFG_LUT4_FN10, TILE1_CFG_LUT4_FN2);

    //
    // FSM
    //
    CLB_selectFSMInputs(CLB4_BASE, TILE1_CFG_FSM_EXT_IN0,
                        TILE1_CFG_FSM_EXT_IN1, TILE1_CFG_FSM_EXTRA_IN0,
                        TILE1_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(CLB4_BASE, TILE1_CFG_FSM_NEXT_STATE_0,
                           TILE1_CFG_FSM_NEXT_STATE_1,
                           TILE1_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(CLB4_BASE, TILE1_CFG_FSM_LUT_FN10,
                             TILE1_CFG_FSM_LUT_FN2);

    //
    // Counters
    //
    CLB_selectCounterInputs(CLB4_BASE, TILE1_CFG_COUNTER_RESET,
                            TILE1_CFG_COUNTER_EVENT, TILE1_CFG_COUNTER_MODE_0,
                            TILE1_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(CLB4_BASE, TILE1_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(CLB4_BASE, CLB_CTR0, TILE1_COUNTER_0_LOAD_VAL,
                               TILE1_COUNTER_0_MATCH1_VAL,
                               TILE1_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(CLB4_BASE, CLB_CTR1, TILE1_COUNTER_1_LOAD_VAL,
                               TILE1_COUNTER_1_MATCH1_VAL,
                               TILE1_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(CLB4_BASE, CLB_CTR2, TILE1_COUNTER_2_LOAD_VAL,
                               TILE1_COUNTER_2_MATCH1_VAL,
                               TILE1_COUNTER_2_MATCH2_VAL);

    //
    // HLC
    //
    CLB_configHLCEventSelect(CLB4_BASE, TILE1_HLC_EVENT_SEL);
    CLB_setHLCRegisters(CLB4_BASE, TILE1_HLC_R0_INIT, TILE1_HLC_R1_INIT,
                        TILE1_HLC_R2_INIT, TILE1_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(CLB4_BASE, i, tformat_tile1HLCInstr[i]);
    }
}
