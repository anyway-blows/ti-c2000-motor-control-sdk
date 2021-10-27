//#############################################################################
//
// FILE:    multi_axis_lead_comms.c
//
// TITLE:   multi-axis servo drive over FSI on the related kits
//
// Group:   C2000
//
// Target Family: F2838x
//
//! multi_axis_fsi_lead_comms.c is for the lead device in the daisy-chain
//!     loop, multi_axis_fsi_slave_comms.c for the other N-1 devices(N>=2).
//!
//! In the code, there are different settings provided:
//! [#ifndef FSI_DMA_ENABLE] represents FSI communication using CPU control.
//! [#ifdef  FSI_DMA_ENABLE] && [ifdef TX_DMA_TRIGGER_ENABLE] represents FSI
//!     communication using DMA control, and enabling FSITX to trigger DMA
//!     for rapid data transmission.
//! [#ifdef FSI_DMA_ENABLE] && [ifndef TX_DMA_TRIGGER_ENABLE] represents FSI
//!     communication using DMA control, and using software to trigger DMA
//!     for the transmitted data (manually).
//!
//! In a real scenario two separate devices may power up in arbitrary order and
//! there is a need to establish a clean communication link which ensures that
//! receiver side is flushed to properly interpret the start of a new valid
//! frame.
//!
//! The lead device in the daisy chain topology initiates and drives the
//! handshake sequence and subsequent data transmissions.
//!
//! After above synchronization steps, FSI Rx can be configured as per use case
//! i.e. nWords, lane width, enabling events, etc and start the infinite
//! transfers. More details on establishing the communication link can be found
//! in the device TRM.
//!
//! User can edit some of configuration parameters as per use case, similar to
//! other examples.
//!
//! \b nWords - Number of words per transfer may be from 1 -16
//! \b nLanes - Choice to select single or double lane for frame transfers
//! \b txUserData - User data to be sent with Data frame
//! \b txDataFrameTag - Frame tag used for Data transfers
//! \b txPingFrameTag - Frame tag used for Ping transfers
//! \b txPingTimeRefCntr - Tx Ping timer reference counter
//! \b rxWdTimeoutRefCntr - Rx Watchdog timeout reference counter
//!
//!\b External \b Connections \n
//!  For the FSI daisy-chain topology external connections are required to
//!  be made between the devices in the chain. Each devices FSI TX pins need to
//!  be connected to the FSI RX pins of the next device in the chain (or ring).
//!  See below for external connections to include and GPIOs used:
//!
//!  External Connections Required:
//!     - FSIRX_CLK(F28388D)  to  FSITX_CLK (F280049C/F280025C)
//!     - FSIRX_RX0(F28388D)  to  FSITX_TX0 (F280049C/F280025C)
//!     - FSITX_CLK(F28388D)  to  FSIRX_CLK (F280049C/F280025C)
//!     - FSITX_TX0(F28388D)  to  FSIRX_RX0 (F280049C/F280025C)
//!
//!  F28388D ControlCard FSI Header GPIOs:
//!     - GPIO_27  ->    FSITX_CLK
//!     - GPIO_26  ->    FSITX_TX0
//!     - GPIO_9   ->    FSIRX_CLK
//!     - GPIO_8   ->    FSIRX_RX0
//!
//! \b Watch \b Variables \n
//!  - \b dataFrameCntr  Number of Data frame transfered
//!  - \b error          Non zero for transmit/receive data mismatch
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
#include "multi_axis_lead_ctrl_main.h"

#include "fsi_optimal_delay.h"

#pragma INTERRUPT (fsiTxInt1ISR, HPI)
#pragma INTERRUPT (fsiTxInt2ISR, HPI)
#pragma INTERRUPT (fsiRxInt1ISR, HPI)
#pragma INTERRUPT (fsiRxInt2ISR, HPI)

#ifdef _FLASH
#pragma CODE_SECTION(fsiTxInt1ISR, ".TI.ramfunc");
#pragma CODE_SECTION(fsiTxInt2ISR, ".TI.ramfunc");
#pragma CODE_SECTION(fsiRxInt1ISR, ".TI.ramfunc");
#pragma CODE_SECTION(fsiRxInt2ISR, ".TI.ramfunc");
#endif  // _FLASH


//
// Global variables for FSI
//
uint16_t fsiTxStartFlag;
uint16_t epwmSyncCmpStop;
uint16_t epwmSyncCmpDelay;
uint16_t epwmSyncPreScale;

uint16_t fsiTxStatus;
uint16_t fsiRxStatus;

uint16_t fsiTxIndex;
uint16_t fsiRxIndex;
uint16_t fsiSlaveNodeFirst;
uint16_t fsiSlaveNodeLast;
uint16_t fsiSlaveNodeActive;
uint16_t fsiSlaveNodeRecived;
uint16_t fsiSlaveNodeNext;

uint16_t *fsiTxDataBufAddr;     // FSI TX data buffer address
uint16_t *fsiRxDataBufAddr;     // FSI RX data buffer address
uint16_t *fsiTxTagBufAddr;      // FSI TX Tag & user data buffer address
uint16_t *fsiRxTagBufAddr;      // FSI RX Tag & user data buffer address

uint16_t *fsiTxDataAddr;        // FSI TX frame data address
uint16_t *fsiRxDataAddr;        // FSI RX frame data address
uint16_t *fsiTxTagAddr;         // FSI TX frame tag and user data address
uint16_t *fsiRxTagAddr;         // FSI RX frame tag and user data address

uint16_t fsiTxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];      // FSI TX data array
uint16_t fsiRxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];      // FSI RX data array
uint16_t fsiTxTagUdataBuf[FSI_NODE_NUM];
uint16_t fsiRxTagUdataBuf[FSI_NODE_NUM];

// Frame tag used with Data/Ping transfers
FSI_FrameTag fsiFrameTag[FSI_NODE_NUM];     // FSI frame Tag array for ping
FSI_FrameTag fsiPingTag0;
FSI_FrameTag fsiPingTag1;

uint16_t fsiTxFrameTag;
uint16_t fsiRxFrameTag;

uint16_t fsiTxDataWords;
uint16_t fsiRxDataWords;
uint16_t fsiTRxNodeNum;

uint16_t fsiTxDataBufWords;
uint16_t fsiRxDataBufWords;

FSI_DataWidth fsiTxLanes;
FSI_DataWidth fsiRxLanes;

volatile uint16_t fsiTxUserData[FSI_NODE_NUM];
volatile uint16_t fsiRxUserData[FSI_NODE_NUM];

// User data to be sent with Data frame(for CPU control)
volatile uint16_t fsiTxUserDataTag;
volatile uint16_t fsiRxUserDataTag;

volatile FSI_HandShake_e fsiHandShakeState;

volatile FSI_TRxState_e fsiTxInt1Received;
volatile FSI_TRxState_e fsiRxInt1Received;


volatile uint32_t fsiTxInt2Received;
volatile uint32_t fsiRxInt2Received;

volatile uint32_t fsiError;

uint32_t fsiDelayTapRX0;
uint32_t fsiDelayTapRX1;
uint32_t fsiDelayTapCLK;

uint32_t fsiTxTimeOutCntr;
uint32_t fsiRxTimeOutCntr;
uint32_t fsiTxTimeOutSet;

uint16_t fsiTxTimeWaitCntr;
uint16_t fsiTxTimeWaitSet;


uint16_t frameDataRX[FSI_TSF_WORDS];
uint16_t frameDataTX[FSI_TSF_WORDS];

uint16_t fsienableDeltaChk;
uint16_t fsienableCrcChk;

uint16_t dataCrcCalc;
uint16_t dataCrcRX;
uint16_t dataCrcTX;

const FSI_FrameTag fsiFrameTagCon[8] = {FSI_FRAME_TAG1, FSI_FRAME_TAG2, \
                                        FSI_FRAME_TAG3, FSI_FRAME_TAG4, \
                                        FSI_FRAME_TAG5, FSI_FRAME_TAG6, \
                                        FSI_FRAME_TAG7, FSI_FRAME_TAG8 };

//-----------------------------------------------------------------------------
//
// Function Prototypes
//

//
// FSI initialize
//
void FSI_initParams(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t ni;

    fsiTxStartFlag = 0;
    epwmSyncCmpStop = EPWM_SYNC_STOP_VALUE;
    epwmSyncCmpDelay = EPWM_SYNC_SHIFT_VALUE;
    epwmSyncPreScale = EPWM_SYNC_EVENT_COUNT;

    fsiTxIndex = 0;
    fsiRxIndex = 0;

    fsiSlaveNodeFirst = FSI_NODE_FIRST;
    fsiSlaveNodeLast = FSI_NODE_LAST;
    fsiSlaveNodeActive = FSI_NODE_ACTIVE;

    fsiTxDataWords = FSI_TX_WORDS;
    fsiRxDataWords = FSI_RX_WORDS;
    fsiTRxNodeNum = FSI_NODE_NUM;

    fsiTxDataBufWords = fsiTxDataWords * fsiTRxNodeNum;
    fsiRxDataBufWords = fsiRxDataWords * fsiTRxNodeNum;

    fsiTxLanes = (FSI_DataWidth)FSI_TX_LANES;
    fsiRxLanes = (FSI_DataWidth)FSI_RX_LANES;

    fsiTxTimeOutCntr = FSI_TX_TIME_OUT_CNTR;
    fsiRxTimeOutCntr = FSI_PING_TIME_OUT_CNTR;
    fsiTxTimeOutSet = FSI_TX_TIME_OUT_SET;

    fsiTxTimeWaitSet = FSI_TX_TIME_WAIT_CNTR;

    fsiError = 0;
    fsiTxInt2Received = 0;
    fsiRxInt2Received = 0;

    fsiTxInt1Received = FSI_TRx_IDLE;
    fsiRxInt1Received = FSI_TRx_IDLE;
    fsiHandShakeState  = FSI_HANDSHAKE_NO;

    fsienableDeltaChk = 0;
    fsienableCrcChk = 0;



    fsiDelayTapRX0 = 0;
    fsiDelayTapRX1 = 0;
    fsiDelayTapCLK = 0;

    for(ni = 0; ni < FSI_NODE_NUM; ni++)
    {
        fsiFrameTag[ni] = fsiFrameTagCon[ni];
    }

    fsiPingTag0 = FSI_FRAME_TAG0;
    fsiPingTag1 = FSI_FRAME_TAG1;

    fsiRxFrameTag = fsiFrameTag[fsiSlaveNodeActive];
    fsiTxFrameTag = fsiFrameTag[fsiSlaveNodeActive];

    fsiTxUserDataTag = fsiTxUserData[fsiSlaveNodeActive] +
                       fsiFrameTag[fsiSlaveNodeActive];
    fsiRxUserDataTag = fsiRxUserData[fsiSlaveNodeActive] +
                       fsiFrameTag[fsiSlaveNodeActive];

    // setup FSI
    HAL_setupFSI(handle);

    fsiTxDataBufAddr = (uint16_t *)(&fsiTxDataBuf[0][0]);
    fsiRxDataBufAddr = (uint16_t *)(&fsiRxDataBuf[0][0]);
    fsiTxTagBufAddr  = (uint16_t *)(&fsiTxUserDataTag);
    fsiRxTagBufAddr  = (uint16_t *)(&fsiRxUserDataTag);

    fsiTxDataAddr = (uint16_t *)FSI_getTxBufferAddress(obj->fsiTxHandle);
    fsiRxDataAddr = (uint16_t *)FSI_getRxBufferAddress(obj->fsiRxHandle);
    fsiTxTagAddr  = (uint16_t *)(obj->fsiTxHandle + FSI_O_TX_FRAME_TAG_UDATA);
    fsiRxTagAddr  = (uint16_t *)(obj->fsiRxHandle + FSI_O_RX_FRAME_TAG_UDATA);


    HAL_setupFSIInterrupts(handle);

    return;
}


//
// setup Tx/Rx frame data
//
void FSI_setupTRxFrameData(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#ifdef FSI_SKEW_COMP_ENABLE
    // - Implement FSI RX delay line calibration here
    //   to compensate for channel-to-channel skew.
    //   See Application Report SPRACJ9 and respective
    //   software examples within C2000ware.

    // Set the delay taps
    FSI_configRxDelayLine_fix(obj->fsiRxHandle, FSI_RX_DELAY_D0, fsiDelayTapRX0);
    FSI_configRxDelayLine_fix(obj->fsiRxHandle, FSI_RX_DELAY_D1, fsiDelayTapRX1);
    FSI_configRxDelayLine_fix(obj->fsiRxHandle, FSI_RX_DELAY_CLK, fsiDelayTapCLK);
#endif

    //
    // Setting for requested nWords and nLanes with transfers
    //
    FSI_setTxFrameType(obj->fsiTxHandle, FSI_FRAME_TYPE_NWORD_DATA);

    FSI_setTxSoftwareFrameSize(obj->fsiTxHandle, fsiTxDataWords);
    FSI_setTxDataWidth(obj->fsiTxHandle, (FSI_DataWidth)FSI_TX_LANES);

    // RX setting part
    FSI_setRxSoftwareFrameSize(obj->fsiRxHandle, fsiRxDataWords);
    FSI_setRxDataWidth(obj->fsiRxHandle, (FSI_DataWidth)FSI_RX_LANES);

    //
    // Enable transmit/receive error events to be sent over INT2 line
    // Overrun and Underrun conditions in Rx are not enabled as buffer pointers
    // are always overwritten to first location for sending data frames.
    //
    FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT2, (FSI_RX_EVT_CRC_ERR |
                                                       FSI_RX_EVT_EOF_ERR |
                                                       FSI_RX_EVT_TYPE_ERR));

    DEVICE_DELAY_US(10);

    FSI_disableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_PING_FRAME);

    DEVICE_DELAY_US(10);

    // Set FSI TX circular buffer pointer back to beginning
    FSI_setTxBufferPtr(obj->fsiTxHandle, 0U);

    // Set FSI RX circular buffer pointer back to beginning
    FSI_setRxBufferPtr(obj->fsiRxHandle, 0U);

    // write User Data and Frame Tag
    FSI_writeTxTagUserData(handle, fsiTxUserDataTag);

    FSI_enableTxInterrupt(obj->fsiTxHandle, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
    FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_DATA_FRAME);

    DEVICE_DELAY_US(10);

    #ifndef CLB_PWM_SYNC
    if(fsiHandShakeState == FSI_HANDSHAKE_DONE)
    {
        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);

        FSI_startTxTransmit(obj->fsiTxHandle);
    }
    #else
    if(fsiHandShakeState == FSI_HANDSHAKE_DONE)
    {
        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);
        FSI_startTxTransmit(obj->fsiTxHandle);
    }

    DEVICE_DELAY_US(500);

    EPWM_clearADCTriggerFlag(M_SYNC_PWM_BASE, EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(M_SYNC_PWM_BASE, EPWM_SOC_B);

    // Set up some FSI ping frame fields and external trigger source based on
    // selected ePWM
    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_EXT_TRIG);
    FSI_enableTxExtPingTrigger(obj->fsiTxHandle, M_PWM_SNYC_SOC);
    FSI_setTxExtFrameTrigger(obj->fsiTxHandle, M_PWM_SNYC_SOC);
    #endif  // CLB_PWM_SYNC


    return;
}

//
// FSI handshake
//
void FSI_handshakeLead(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // Enable normal data transfer events to be sent over INT1 line
    //
    FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_PING_FRAME);

    // Wait till interrupt is received on FSIRX INT1 line, verify it's for FRAME
    // DONE event for PING Frame reception
    //
    while(1)
    {
        //
        // Send the flush sequence
        //
        FSI_executeTxFlushSequence(obj->fsiRxHandle, PRESCALER_VAL);

        //
        // Send a ping frame with frame tag 0000b
        //
        FSI_setTxFrameTag(obj->fsiTxHandle, fsiPingTag0);
        FSI_setTxFrameType(obj->fsiTxHandle, FSI_FRAME_TYPE_PING);
        FSI_startTxTransmit(obj->fsiTxHandle);

        while((fsiRxInt1Received != FSI_TRx_DONE) && (fsiRxTimeOutCntr != 0U))
        {
            DEVICE_DELAY_US(1);

            fsiRxTimeOutCntr--;
        }

        if(fsiRxTimeOutCntr == 0)
        {
            fsiRxTimeOutCntr = FSI_PING_TIME_OUT_CNTR;
            continue;
        }
        else
        {
            fsiError += FSI_compareData(fsiRxStatus,
                              (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));

            FSI_checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, fsiPingTag0);

            fsiRxInt1Received = FSI_TRx_IDLE;

            //
            // If received frame type and tag matches, exit this loop and
            // proceed to next step by sending flush sequence, otherwise clear
            // error and interrupt flag and continue looping.
            //
            if(fsiError == 0)
            {

                break;
            }

            fsiError = 0;
        }
    }

    while(1)
    {
        //
        // Send a ping frame with frame tag 0001b
        //
        FSI_setTxFrameTag(obj->fsiTxHandle, fsiPingTag1);
        FSI_setTxFrameType(obj->fsiTxHandle, FSI_FRAME_TYPE_PING);
        FSI_startTxTransmit(obj->fsiTxHandle);

        while((fsiRxInt1Received != FSI_TRx_DONE) && (fsiRxTimeOutCntr != 0U))
        {
            DEVICE_DELAY_US(1);
            fsiRxTimeOutCntr--;
        }

        if(fsiRxTimeOutCntr == 0)
        {
            fsiRxTimeOutCntr = FSI_PING_TIME_OUT_CNTR;
            continue;
        }
        else
        {
            fsiError += FSI_compareData(fsiRxStatus,
                         (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));

            FSI_checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, fsiPingTag1);

            fsiRxInt1Received = FSI_TRx_IDLE;

            //
            // If received frame type and tag matches, exit this loop and
            // proceed to next step by sending flush sequence, otherwise clear
            // error and interrupt flag and continue looping.
            //
            if(fsiError == 0)
            {

                fsiHandShakeState = FSI_HANDSHAKE_DONE;
                break;
            }

            fsiError = 0;
        }
    }

    DEVICE_DELAY_US(500);       // 100->500

    fsiRxInt1Received = FSI_TRx_IDLE;
    fsiTxInt1Received = FSI_TRx_IDLE;


    return;
}

//
// Starts TX/RX FSI data
//
void FSI_runTRxData(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    if(fsiTxTimeOutCntr > 0)
    {
        fsiTxTimeOutCntr--;
    }

    if(fsiRxTimeOutCntr > 0)
    {
        fsiRxTimeOutCntr--;
    }

    if(fsiTxTimeWaitCntr > 0)
    {
        fsiTxTimeWaitCntr--;
    }


    //
    // Wait for RX data frame received event
    //
    if(fsiRxInt1Received == FSI_TRx_DONE)
    {

        FSI_updateReceivedData();

        fsiRxInt1Received = FSI_TRx_IDLE;
        fsiRxTimeOutCntr = FSI_RX_TIME_OUT_CNTR;
    }
    else if(fsiRxTimeOutCntr == 0)
    {

        fsiRxTimeOutCntr = FSI_RX_TIME_OUT_CNTR;

        FSI_disableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_PING_FRAME);

        DEVICE_DELAY_US(10);

        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

        // Set FSI RX circular buffer pointer back to beginning
        FSI_setRxBufferPtr(obj->fsiRxHandle, 0U);

        FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_DATA_FRAME);
    }

    // Wait for TX frame done event
    if(((fsiTxInt1Received == FSI_TRx_DONE) || (fsiTxTimeOutCntr == 0))
        && (fsiTxTimeWaitCntr == 0))
    {
        fsiTxInt1Received = FSI_TRx_IDLE;
        fsiTxTimeOutCntr = FSI_TX_TIME_OUT_CNTR;
        fsiTxTimeWaitCntr = fsiTxTimeWaitSet;

        FSI_updateTransmissionData();

        #ifndef CLB_PWM_SYNC
        DINT;          // Disable Global interrupt INTM

        // Set FSI TX circular buffer pointer back to beginning
        FSI_setTxBufferPtr(obj->fsiTxHandle, 0U);

        // write User Data and Frame Tag
        FSI_writeTxTagUserData(handle, fsiTxUserDataTag);

        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);

        EINT;          // Enable Global interrupt INTM

        FSI_startTxTransmit(obj->fsiTxHandle);
        #else
        DINT;          // Disable Global interrupt INTM
        // Set FSI TX circular buffer pointer back to beginning
        FSI_setTxBufferPtr(obj->fsiTxHandle, 0U);

        // write User Data and Frame Tag
        FSI_writeTxTagUserData(handle, fsiTxUserDataTag);

        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);

        EINT;          // Enable Global interrupt INTM

        // clear flag
        EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

        EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                    EPWM_COUNTER_COMPARE_C, epwmSyncCmpDelay);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[M_SYNC_NUM],
                                        EPWM_SOC_B, epwmSyncPreScale);

        // Enable SOC on B group
        EPWM_enableADCTrigger(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);
        #endif  // CLB_PWM_SYNC


    }

}

//
// update the received data
//
void FSI_updateReceivedData(void)
{
    int32_t tempData;
    uint16_t fsiNode, ctrlNode;
    uint16_t ni;
    uint16_t dataType;

    DINT;          // Disable Global interrupt INTM

    fsiNode = fsiSlaveNodeRecived;
    ctrlNode = fsiNode + 1;
    dataCrcRX = fsiRxUserData[fsiNode];

    for(ni = 0; ni< FSI_RX_WORDS; ni++)
    {
        frameDataRX[ni] = fsiRxDataBuf[fsiNode][ni];
    }

    EINT;          // Enable Global interrupt INTM

    if(((frameDataRX[0]>>12) & 0x000F) != fsiFrameTag[fsiNode])
    {

        return;
    }

    if(fsienableCrcChk == 0)
    {
        dataCrcCalc = FSI_USERTAG_CHK - fsiFrameTag[fsiNode];
    }
    else
    {
        #ifdef SW_CRC_EN
        dataCrcCalc = getCRC8Value(&frameDataRX[0], FSI_RX_WORDS);
        #else
        dataCrcCalc = getChkSumValue(&frameDataRX[0], FSI_RX_WORDS);
        #endif
    }

    if(dataCrcRX != dataCrcCalc)
    {
        return;
    }

    dataType = (frameDataRX[0]>>8) & 0x000F;


    switch(dataType)
    {
        case FSI_UDATA_PS_SP_N:
            ctrlVars[ctrlNode].ctrlStateFdb =
                    (CtrlState_e)(frameDataRX[0] & 0x00FF);

            tempData = (int32_t)frameDataRX[1];
            tempData = (tempData<<16) + frameDataRX[2];
            ctrlVars[ctrlNode].speedWe = FSI_convertPUToFloat(tempData);

            tempData = (int32_t)frameDataRX[3];
            tempData = (tempData<<16) + frameDataRX[4];
            ctrlVars[ctrlNode].posMechTheta = FSI_convertPUToFloat(tempData);

            #if(BUILDLEVEL == FCL_LEVEL5)       // Verify FSI
            if(fabsf(ctrlVars[ctrlNode].speedWe -
                     ctrlVars[ctrlNode].speedWePrev) >
                                       ctrlVars[ctrlNode].speedWeDelta)
            {
                ctrlVars[ctrlNode].speedWeError =
                        ctrlVars[ctrlNode].speedWe;

            }

            if(fabsf(ctrlVars[ctrlNode].posMechTheta -
                     ctrlVars[ctrlNode].posMechThetaPrev) >
                             ctrlVars[ctrlNode].posMechThetaDelta)
            {
                ctrlVars[ctrlNode].posMechThetaError =
                        ctrlVars[ctrlNode].posMechTheta;

            }
            #else
            if(fabsf(ctrlVars[ctrlNode].speedWe -
                     ctrlVars[ctrlNode].speedWePrev) >
                             ctrlVars[ctrlNode].speedWeDelta)
            {
                ctrlVars[ctrlNode].speedWeError =
                        ctrlVars[ctrlNode].speedWe;

                ctrlVars[ctrlNode].speedWe = ctrlVars[ctrlNode].speedWe * 0.4 +
                        ctrlVars[ctrlNode].speedWePrev * 0.6;

            }

            ctrlVars[ctrlNode].speedWePrev = ctrlVars[ctrlNode].speedWe;

            if(fabsf(ctrlVars[ctrlNode].posMechTheta -
                     ctrlVars[ctrlNode].posMechThetaPrev) >
                              ctrlVars[ctrlNode].posMechThetaDelta)
            {
                ctrlVars[ctrlNode].posMechThetaError =
                        ctrlVars[ctrlNode].posMechTheta;
            }
            #endif  // (BUILDLEVEL != FCL_LEVEL5)

            break;

        case FSI_UDATA_PS_SP_F:

            break;

        case FSI_UDATA_IS_FDB:

            break;

        case FSI_UDATA_TQ_POW:

            break;

        case FSI_UDATA_DV_MDT:

            break;
    }

    return;
}

//
// update the transmission data
//
void FSI_updateTransmissionData(void)
{
    int32_t tempData;
    uint16_t fsiNode, ctrlNode;
    uint16_t ni;
    uint16_t dataType;

    fsiNode = fsiSlaveNodeNext;
    ctrlNode = fsiNode + 1;

    dataType = FSI_UDATA_IS_REF;
    frameDataTX[0]  = (fsiFrameTag[fsiNode]<<12) & 0xF000;
    frameDataTX[0] += (dataType<<8) &0x0F00;

    switch(dataType)
    {
        case FSI_UDATA_IS_REF:
            frameDataTX[0] += ctrlVars[ctrlNode].ctrlStateCom & 0x00FF;

            tempData = FSI_convertFloatToPU(ctrlVars[ctrlNode].IqRef);
            frameDataTX[1] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[2] = (uint16_t)(tempData & 0x0000FFFF);

            tempData = FSI_convertFloatToPU(ctrlVars[ctrlNode].IdRef);
            frameDataTX[3] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[4] = (uint16_t)(tempData & 0x0000FFFF);
            break;

        case FSI_UDATA_GAIN_ID:

            break;

        case FSI_UDATA_GAIN_IQ:

            break;

        case FSI_UDATA_UMN_ID:

            break;

        case FSI_UDATA_UMN_IQ:

            break;
    }

    if(fsienableCrcChk == 0)
    {
        dataCrcTX = FSI_USERTAG_CHK - fsiFrameTag[fsiNode];
    }
    else
    {
        #ifdef SW_CRC_EN
        dataCrcTX = getCRC8Value(&frameDataTX[0], FSI_TX_WORDS);
        #else
        dataCrcTX = getChkSumValue(&frameDataTX[0], FSI_TX_WORDS);
        #endif
    }

    DINT;          // Disable Global interrupt INTM
    fsiTxUserData[fsiNode] = dataCrcTX;

    for(ni = 0; ni< FSI_TX_WORDS; ni++)
    {
        fsiTxDataBuf[fsiNode][ni] = frameDataTX[ni];
    }
    EINT;          // Enable Global interrupt INTM

    return;
}

//
// disableAllFSIInterrupts - Disables all event interrupts in both FSI Tx/Rx,
//                           also clear them
//
void disableAllFSIInterrupts(void)
{
    FSI_disableTxInterrupt(hal.fsiTxHandle, FSI_INT1, FSI_TX_EVTMASK);
    FSI_disableTxInterrupt(hal.fsiTxHandle, FSI_INT2, FSI_TX_EVTMASK);
    FSI_disableRxInterrupt(hal.fsiRxHandle, FSI_INT1, FSI_RX_EVTMASK);
    FSI_disableRxInterrupt(hal.fsiRxHandle, FSI_INT2, FSI_RX_EVTMASK);

    FSI_clearTxEvents(hal.fsiTxHandle, FSI_TX_EVTMASK);
    FSI_clearRxEvents(hal.fsiRxHandle, FSI_RX_EVTMASK);
}

//
// Checks received frame type/tag and updates global error flag
//
void FSI_checkReceivedFrameTypeTag(FSI_FrameType type, FSI_FrameTag tag)
{
    fsiError += FSI_compareData((uint16_t)FSI_getRxFrameType(hal.fsiRxHandle),
                                (uint16_t)type);

    if(type == FSI_FRAME_TYPE_PING)
    {
        fsiError += FSI_compareData(FSI_getRxPingTag(hal.fsiRxHandle),
                                    (uint16_t)tag);
    }
    else
    {
        fsiError += FSI_compareData(FSI_getRxFrameTag(hal.fsiRxHandle),
                                    (uint16_t)tag);
    }
}

//
// fsiTxInt1ISR - FSI Tx Interrupt on INsT1 line
//
__interrupt void fsiTxInt1ISR(void)
{

    fsiTxStatus = FSI_getTxEventStatus(hal.fsiTxHandle);

    if(((fsiTxStatus & FSI_TX_EVT_FRAME_DONE) != 0)  &&
            (fsiHandShakeState == FSI_HANDSHAKE_DONE))
    {

        #ifndef CLB_PWM_SYNC
        fsiTxTimeWaitCntr = fsiTxTimeWaitSet;
        #else
        fsiTxTimeWaitCntr = fsiTxTimeWaitSet;

        // clear flag
        EPWM_clearADCTriggerFlag(hal.pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

        EPWM_setCounterCompareValue(hal.pwmHandle[M_SYNC_NUM],
                                    EPWM_COUNTER_COMPARE_C, epwmSyncCmpStop);
        // Enable SOC on B group
        EPWM_disableADCTrigger(hal.pwmHandle[M_SYNC_NUM], EPWM_SOC_B);
        #endif  // CLB_PWM_SYNC
    }

    FSI_setTxBufferPtr(hal.fsiTxHandle, 0U);
    fsiTxInt1Received = FSI_TRx_DONE;

    // Clear the interrupt flag and issue ACK
    FSI_clearTxEvents(hal.fsiTxHandle, FSI_TX_EVTMASK);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
}

//
// fsiTxInt2ISR - FSI Tx Interrupt on INT2 line
//
__interrupt void fsiTxInt2ISR(void)
{
    fsiTxStatus = FSI_getTxEventStatus(hal.fsiTxHandle);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearTxEvents(hal.fsiTxHandle, FSI_TX_EVTMASK);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

}

//
// fsiRxInt1ISR - FSI Rx Interrupt on INT1 line
//
__interrupt void fsiRxInt1ISR(void)
{

    fsiRxStatus = FSI_getRxEventStatus(hal.fsiRxHandle);

    if((fsiRxStatus & FSI_RX_EVT_DATA_FRAME) != 0)
    {

        FSI_readRxFrameData(halHandle);
    }

    fsiRxInt1Received = FSI_TRx_DONE;
    FSI_setRxBufferPtr(hal.fsiRxHandle, 0U);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(hal.fsiRxHandle, fsiRxStatus);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);
}

//
// fsiRxInt2ISR - FSI Rx Interrupt on INT2 line
//
__interrupt void fsiRxInt2ISR(void)
{
    fsiRxStatus = FSI_getRxEventStatus(hal.fsiRxHandle);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(hal.fsiRxHandle, fsiRxStatus);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

}


// End of Code
//
