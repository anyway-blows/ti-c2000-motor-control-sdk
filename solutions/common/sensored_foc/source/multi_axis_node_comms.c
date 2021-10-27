//#############################################################################
//
// FILE:    multi_axis_node_comms.c
//
// TITLE:   multi-axis servo drive over FSI on the related kits
//
// Group:   C2000
//
// Target Family: F28004x/F28002x
//
//
//! fsi_ex16_daisy_handshake_lead is for the lead device in the daisy-chain
//!     loop, fsi_ex16_daisy_handshake_node for the other N-1 devices(N>=2).
//!
//! In the code, there are different settings provided:
//! [#define FSI_DMA_ENABLE 0] represents FSI communication using CPU control.
//! [#define FSI_DMA_ENABLE 1] represents FSI communication using DMA control,
//!     enabling FSIRX to trigger a DMA event and move the RX FSI data to the
//!     TX FSI buffer
//!
//! In a real scenario two separate devices may power up in arbitrary order and
//! there is a need to establish a clean communication link which ensures that
//! receiver side is flushed to properly interpret the start of a new valid
//! frame.
//!
//! The node devices in the daisy chain topology respond to the handshake
//! sequence and forwards the information  to the next device in the chain.
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
//!  F28002x ControlCard FSI Header GPIOs:
//!     - GPIO_27  ->    FSITX_CLK
//!     - GPIO_26  ->    FSITX_TX0
//!     - GPIO_13  ->    FSIRX_CLK
//!     - GPIO_12  ->    FSIRX_RX0
//!
//! \b Watch \b Variables \n
//!  - \b dataFrameCntr  Number of Data frames received back
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
#include "multi_axis_node_main.h"

#ifdef CLB_PWM_SYNC
#include "clb_config.h"
#endif  // CLB_PWM_SYNC

#include "fsi_optimal_delay.h"

//#if(BUILDLEVEL >= FCL_LEVEL7)

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
uint16_t clbCounter0Match1Delay;
uint16_t epwmSyncCmpStop;
uint16_t epwmSyncCmpDelay;
uint16_t epwmSyncPreScale;

uint16_t phaseShiftValueU;
uint16_t phaseShiftValueV;
uint16_t phaseShiftValueW;
uint16_t phaseShiftValueT;

uint16_t fsiTxStatus;
uint16_t fsiRxStatus;

uint16_t fsiSlaveNodeUse;
uint16_t fsiSlaveNodeNext;

uint16_t *fsiTxDataBufAddr;     // FSI TX data buffer address
uint16_t *fsiRxDataBufAddr;     // FSI RX data buffer address
uint16_t *fsiTxTagBufAddr;      // FSI TX Tag & user data buffer address
uint16_t *fsiRxTagBufAddr;      // FSI RX Tag & user data buffer address

uint16_t *fsiTxDataAddr;        // FSI TX frame data address
uint16_t *fsiRxDataAddr;        // FSI RX frame data address
uint16_t *fsiTxTagAddr;         // FSI TX frame tag & user data address
uint16_t *fsiRxTagAddr;         // FSI RX frame tag & user data address

#ifdef FSI_DMA_ENABLE
uint16_t fsiTxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];    // FSI TX data array
uint16_t fsiRxDataBuf[FSI_NODE_NUM][FSI_DATA_NUM];    // FSI RX data array
uint16_t fsiTxTagUdataBuf[FSI_NODE_NUM];
uint16_t fsiRxTagUdataBuf[FSI_NODE_NUM];
#else
uint16_t fsiTxDataBuf[FSI_DATA_NUM];        // FSI TX data array
uint16_t fsiRxDataBuf[FSI_DATA_NUM];        // FSI RX data array
#endif  // FSI_DMA_ENABLE

uint16_t fsiTsfDataBuf[FSI_DATA_NUM];       // FSI Transfer data array

// Frame tag used with Data/Ping transfers
FSI_FrameTag fsiFrameTag[FSI_NODE_NUM];     // FSI frame Tag array for ping
FSI_FrameTag fsiPingTag0;
FSI_FrameTag fsiPingTag1;

uint16_t fsiTxFrameTag;
uint16_t fsiRxFrameTag;

uint16_t fsiTxDataWords;
uint16_t fsiRxDataWords;
uint16_t fsiTRxNodeNum;

FSI_DataWidth fsiTxLanes;
FSI_DataWidth fsiRxLanes;

volatile uint16_t fsiTxUserData;
volatile uint16_t fsiRxUserData;

// User data to be sent with Data frame(for CPU control)
volatile uint16_t fsiTxUserDataTag;
volatile uint16_t fsiRxUserDataTag;
volatile uint16_t fsiTsfUserDataTag;

volatile FSI_HandShake_e fsiHandShakeState;

volatile FSI_TRxState_e fsiRxNodeReceived;
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

    FSI_setGPIOandNodeNum(handle);

    fsiTxDataWords = FSI_TX_WORDS;
    fsiRxDataWords = FSI_RX_WORDS;
    fsiTRxNodeNum = FSI_NODE_NUM;

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

    // Initlize the frame tag & user data
    fsiTxFrameTag = fsiFrameTag[fsiSlaveNodeUse];

    // setup FSI
    HAL_setupFSI(handle);

#ifdef FSI_DMA_ENABLE
    fsiTxDataBufAddr  = (uint16_t *)(&fsiTxDataBuf[0][0]);
    fsiRxDataBufAddr  = (uint16_t *)(&fsiRxDataBuf[0][0]);
    fsiTxTagBufAddr   = (uint16_t *)(&fsiTxTagUdataBuf[0]);
    fsiRxTagBufAddr   = (uint16_t *)(&fsiRxTagUdataBuf[0]);
#else
    fsiTxDataBufAddr  = (uint16_t *)(&fsiTxDataBuf[0]);
    fsiRxDataBufAddr  = (uint16_t *)(&fsiRxDataBuf[0]);
    fsiTxTagBufAddr   = (uint16_t *)(&fsiTxUserDataTag);
    fsiRxTagBufAddr   = (uint16_t *)(&fsiRxUserDataTag);
#endif

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
    FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT2, FSI_RX_EVT_CRC_ERR  |
                                                 FSI_RX_EVT_EOF_ERR  |
                                                 FSI_RX_EVT_TYPE_ERR);

    DEVICE_DELAY_US(10);

    FSI_disableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_PING_FRAME);
    DEVICE_DELAY_US(10);

    FSI_enableTxInterrupt(obj->fsiTxHandle, FSI_INT1, FSI_TX_EVT_FRAME_DONE);
    FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_DATA_FRAME);

    DEVICE_DELAY_US(100);


    #if defined(CLB_PWM_SYNC) && defined(F28004x_DEVICE)
//    // Setup ping transmit on ping receive
//    // FSI TX ping frames will be triggered on RX_PING_FRAME flag
//    // ePWM will be synchronized on RX_PING_FRAME + configured CLB delay
//    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_EXT_TRIG);
//    FSI_setTxExtFrameTrigger(obj->fsiTxHandle, 0);      // XBAR

    // Set up some FSI ping frame fields and external trigger source based on
    // selected ePWM
    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_EXT_TRIG);
    FSI_setTxExtFrameTrigger(obj->fsiTxHandle, M_PWM_SNYC_SOC);
    #endif  // CLB_PWM_SYNC && F28002x_DEVICE

    #if defined(CLB_PWM_SYNC) && defined(F28002x_DEVICE)
//    // Setup ping transmit on ping receive
//    // FSI TX ping frames will be triggered on RX_PING_FRAME flag
//    // ePWM will be synchronized on RX_PING_FRAME + configured CLB delay
//    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_EXT_TRIG);
//    FSI_setTxExtFrameTrigger(obj->fsiTxHandle, FSI_EXT_TRIGSRC_CLB1_CLBOUT31);

    // Set up some FSI ping frame fields and external trigger source based on
    // selected ePWM
    FSI_setTxStartMode(obj->fsiTxHandle, FSI_TX_START_EXT_TRIG);
    FSI_setTxExtFrameTrigger(obj->fsiTxHandle, M_PWM_SNYC_SOC);
    #endif  // CLB_PWM_SYNC && F28002x_DEVICE

    DEVICE_DELAY_US(10);

    return;
}

//
// FSI handshake
//
void FSI_handshakeNode(HAL_Handle handle)
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
        while(fsiRxInt1Received != FSI_TRx_DONE)
        {

        }

        fsiError += FSI_compareData(fsiRxStatus,
                          (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));

        FSI_checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, fsiPingTag0);

        fsiRxInt1Received = FSI_TRx_IDLE;

        //
        // If received frame type and tag matches, exit this loop and proceed to
        // next step by sending flush sequence, otherwise clear error and
        // interrupt flag and continue looping.
        //
        if(fsiError == 0)
        {

            break;
        }

        fsiError = 0;
    }

    while(1)
    {
        //
        // Send the flush sequence
        //
        FSI_executeTxFlushSequence(obj->fsiTxHandle, PRESCALER_VAL);

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

            FSI_checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, fsiPingTag1);

            fsiRxInt1Received = FSI_TRx_IDLE;

            //
            // If received frame type and tag matches, exit this loop and
            // proceed to next step by sending flush sequence, otherwise clear
            // error and interrupt flag and continue looping.
            //
            if(fsiError == 0)
            {

                #ifdef FSI_DMA_ENABLE
                fsiHandShakeState = FSI_HANDSHAKE_DONE;
                fsiTxTimeOutCntr = 0;
                #endif  // FSI_DMA_ENABLE

                break;
            }

            fsiError = 0;
        }
    }

    fsiRxInt1Received = FSI_TRx_IDLE;
    fsiTxInt1Received = FSI_TRx_IDLE;

    //
    // Send a ping frame with frame tag 0001b
    //
    FSI_setTxFrameTag(obj->fsiTxHandle, fsiPingTag1);
    FSI_setTxFrameType(obj->fsiTxHandle, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(obj->fsiTxHandle);

    //
    DEVICE_DELAY_US(500);       // 10->500


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
        if(fsiRxNodeReceived == FSI_TRx_DONE)
        {
            FSI_updateReceivedData();
            fsiRxNodeReceived = FSI_TRx_IDLE;
        }

        fsiRxInt1Received = FSI_TRx_IDLE;
        fsiRxTimeOutCntr = FSI_RX_TIME_OUT_CNTR;

    }
    else if(fsiRxTimeOutCntr == 0)
    {

        fsiRxTimeOutCntr = FSI_RX_TIME_OUT_CNTR;

        FSI_disableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_PING_FRAME);

        DEVICE_DELAY_US(10);

        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

        // Set FSI RX circular buffer pointer back to beginning
        FSI_setRxBufferPtr(obj->fsiRxHandle, 0U);

        FSI_enableRxInterrupt(obj->fsiRxHandle, FSI_INT1, FSI_RX_EVT_DATA_FRAME);
    }

    if(((fsiTxInt1Received == FSI_TRx_DONE) || (fsiTxTimeOutCntr == 0)) &&
        ((fsiHandShakeState == FSI_HANDSHAKE_DONE) && (fsiTxTimeWaitCntr == 0)))
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
        FSI_writeTxTagUserData(handle, fsiTsfUserDataTag);

        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);

        EINT;          // Enable Global interrupt INTM

        FSI_startTxTransmit(obj->fsiTxHandle);
        #else
        DINT;          // Disable Global interrupt INTM

        // Set FSI TX circular buffer pointer back to beginning
        FSI_setTxBufferPtr(obj->fsiTxHandle, 0U);

        // write User Data and Frame Tag
        FSI_writeTxTagUserData(handle, fsiTsfUserDataTag);

        // write TX data buffer
        FSI_writeTxDataBuffer(handle, fsiTxDataBufAddr, fsiTxDataWords);

        EINT;          // Enable Global interrupt INTM

//        CLB_configCounterLoadMatch(obj->clbHandle, CLB_CTR0,
//                                   TILE1_COUNTER_0_LOAD_VAL,
//                                   clbCounter0Match1Delay,
//                                   TILE1_COUNTER_0_MATCH2_VAL);

        // clear flag
        EPWM_clearADCTriggerFlag(obj->pwmHandle[M_SYNC_NUM], EPWM_SOC_B);

        EPWM_setCounterCompareValue(obj->pwmHandle[M_SYNC_NUM],
                                    EPWM_COUNTER_COMPARE_C, epwmSyncCmpDelay);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmHandle[0], phaseShiftValueU);
        EPWM_setPhaseShift(obj->pwmHandle[1], phaseShiftValueV);
        EPWM_setPhaseShift(obj->pwmHandle[2], phaseShiftValueW);

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
    uint16_t ni;
    uint16_t dataType;

#ifdef FSI_DMA_ENABLE
    DINT;          // Disable Global interrupt INTM
    dataCrcRX = fsiRxUserData;

    for(ni = 0; ni< FSI_RX_WORDS; ni++)
    {
        frameDataRX[ni] = fsiRxDataBuf[fsiSlaveNodeUse][ni];
    }
    EINT;          // Enable Global interrupt INTM
#else
    DINT;          // Disable Global interrupt INTM
    dataCrcRX = fsiRxUserData;

    for(ni = 0; ni< FSI_RX_WORDS; ni++)
    {
        frameDataRX[ni] = fsiRxDataBuf[ni];
    }
    EINT;          // Enable Global interrupt INTM
#endif  // FSI_DMA_ENABLE

    if(((frameDataRX[0]>>12) & 0x000F) != fsiFrameTag[fsiSlaveNodeUse])
    {

        return;
    }

    if(fsienableCrcChk == 0)
    {
        dataCrcCalc = FSI_USERTAG_CHK - fsiFrameTag[fsiSlaveNodeUse];
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
    }

    dataType = (frameDataRX[0]>>8) & 0x000F;


    switch(dataType)
    {
        case FSI_UDATA_IS_REF:
            motorVars.ctrlStateCom =
                    (CtrlState_e)(frameDataRX[0] & 0x00FF);

            tempData = (int32_t)frameDataRX[1];
            tempData = (tempData<<16) + frameDataRX[2];
            motorVars.IqRefSet = FSI_convertPUToFloat(tempData);

            tempData = (int32_t)frameDataRX[3];
            tempData = (tempData<<16) + frameDataRX[4];
            motorVars.IdRefSet = FSI_convertPUToFloat(tempData);

            #if(BUILDLEVEL == FCL_LEVEL7)
            if(fabsf(motorVars.IdRefSet - motorVars.IdRefPrev) >
                                                    motorVars.IdRefDelta)
            {
                motorVars.IdRefError = motorVars.IdRefSet;

            }

            if(fabsf(motorVars.IqRefSet - motorVars.IqRefPrev) >
                                                    motorVars.IqRefDelta)
            {
                motorVars.IqRefError = motorVars.IqRefSet;

            }
            #else   // (BUILDLEVEL > FCL_LEVEL7)
            if(fabsf(motorVars.IdRefSet - motorVars.IdRefPrev) >
                                                    motorVars.IdRefDelta)
            {

                motorVars.IdRefError = motorVars.IdRefSet;
                motorVars.IdRefSet = motorVars.IdRefSet * 0.4 +
                        motorVars.IdRefPrev * 0.6;
            }

            motorVars.IdRefPrev = motorVars.IdRefSet;

            if(fabsf(motorVars.IqRefSet - motorVars.IqRefPrev) >
                                                    motorVars.IqRefDelta)
            {

                motorVars.IqRefError = motorVars.IqRefSet;
                motorVars.IqRefSet = motorVars.IqRefSet * 0.4 +
                        motorVars.IqRefPrev * 0.6;
            }

            motorVars.IqRefPrev = motorVars.IqRefSet;

            #endif  // (BUILDLEVEL == FCL_LEVEL7)

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

    return;
}

//
// update the transmission data
//
void FSI_updateTransmissionData(void)
{
    int32_t tempData;
    uint16_t ni;
    uint16_t dataType;

    dataType = FSI_UDATA_PS_SP_N;
    frameDataTX[0] = (fsiFrameTag[fsiSlaveNodeUse]<<12) & 0xF000;
    frameDataTX[0] += (dataType<<8) &0x0F00;

    switch(dataType)
    {
        case FSI_UDATA_PS_SP_N:

            frameDataTX[0] += (((uint16_t)motorVars.ctrlStateFdb) & 0x00FF);

            tempData = FSI_convertFloatToPU(motorVars.speedWe);
            frameDataTX[1] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[2] = (uint16_t)(tempData & 0x0000FFFF);

            tempData = FSI_convertFloatToPU(motorVars.posMechTheta);
            frameDataTX[3] = (uint16_t)((tempData>>16) & 0x0000FFFF);
            frameDataTX[4] = (uint16_t)(tempData & 0x0000FFFF);
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

    if(fsienableCrcChk == 0)

    {
        dataCrcTX = FSI_USERTAG_CHK - fsiFrameTag[fsiSlaveNodeUse];
    }
    else
    {
        #ifdef SW_CRC_EN
        dataCrcTX = getCRC8Value(&frameDataTX[0], FSI_TX_WORDS);
        #else
        dataCrcTX = getChkSumValue(&frameDataTX[0], FSI_TX_WORDS);
        #endif
    }

#ifdef FSI_DMA_ENABLE
    DINT;          // Disable Global interrupt INTM
    fsiTxUserData = dataCrcTX;

    for(ni = 0; ni< FSI_TX_WORDS; ni++)
    {
        fsiTxDataBuf[0][ni] = frameDataTX[ni];
    }

    EINT;          // Enable Global interrupt INTM
#else
    DINT;          // Disable Global interrupt INTM
    fsiTxUserData = dataCrcTX;

    for(ni = 0; ni< FSI_TX_WORDS; ni++)
    {
        fsiTxDataBuf[ni] = frameDataTX[ni];
    }

    EINT;          // Enable Global interrupt INTM
#endif  // FSI_DMA_ENABLE

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
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
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
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

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

        if(fsiHandShakeState == FSI_HANDSHAKE_NO)
        {
            fsiHandShakeState = FSI_HANDSHAKE_DONE;
            fsiTxTimeOutCntr = 0;
        }
    }

    fsiRxInt1Received = FSI_TRx_DONE;
    FSI_setRxBufferPtr(hal.fsiRxHandle, 0U);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(hal.fsiRxHandle, fsiRxStatus);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
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
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

}


//#endif // (BUILDLEVEL >= FCL_LEVEL7)

//
// End of Code
//
