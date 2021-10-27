//#############################################################################
//
// FILE:   fsi_daisy_epwmsync_lead.c
//
// TITLE:  FSI daisy chain topology with ePWM synchronization,
//         lead device example
//
//! fsi_daisy_epwmsync_lead is for the lead device in the daisy-chain
//! loop, fsi_daisy_epwmsync_node for the other N-1 devices(N>=2).
//!
//! In a real scenario two separate devices may power up in arbitrary order and
//! there is a need to establish a clean communication link which ensures that
//! receiver side is flushed to properly interpret the start of a new valid
//! frame.
//!
//! The lead device in the daisy chain topology initiates and drives the
//! handshake sequence and subsequent data transmissions.
//!
//! After above synchronization steps, FSI Tx/Rx can be configured as per use
//! case i.e. nWords, lane width, enabling events, etc and start the infinite
//! transfers. More details on establishing the communication link can be found
//! in the device TRM.
//!
//! For the ePWM SYNC use case, each lead and node device configures a 20 KHz,
//! 50% duty cycle PWM signal once the handshake sequence is complete.
//! The PWMs of all node devices in the chain are intended to be synchronized 
//! to the lead device's PWM using the FSI communication link alone.
//!
//! The lead device is configured to continuously trigger FSI ping frame
//! transmissions on ePWM compare events. The event is configured using
//! the ePWM compare value, EPWM_CMPC_VALUE, and
//! the event prescale value, EPWM_SYNC_EVENT_COUNT.
//!
//! The CLB has been used to ensure EPWM signal synchronism among the devices.
//! Input received by the CLB will be the FSI_RX_EVT_PING_FRAME signal.
//! Once the input is received, CLB counter starts counting up from 0.
//! When the count reaches "match1" value, a signal is generated which is used as
//! the EXTSYNCIN1 signal for EPWM of the node device. The "match1" value
//! acts as the EXTSYNCIN1 delay for the device. Last device in the link will
//! need no delay and the first device in the link will have the maximum delay
//! to ensure synchronism between devices.
//!
//! Also, when the input is received by CLB, it forwards FSI_RX_EVT_PING_FRAME 
//! signal to the next device in link without any delay.
//!
//! On receiving an FSI ping frame at the node, the EPWMs for multiple node
//! devices can be synchronized with lead device by the user, by
//! setting appropriate \b counter_match_value in the CLB configuration.
//! \b counter_match_value can be set by using the "match1" value under
//! counter0 tab in the syscfg file that is available with project.
//! The details on the value which can be kept for
//! \b counter_match_value to start the calibration for the PWM
//! synchronization can be checked as shown below.
//!
//! For a setup of 'N' node devices, the \b counter_match_value for
//! device1 will be 37*(N-1), device2 will have the value as 37*(N-2),
//! similarly deviceN will have 1 to be fed as the "match1" value.
//!
//! It has to be noted that these values will vary depending on the user setup.
//! The values provided can be taken as the reference to start the calibration.
//! User also needs to verify the serial number of the device in the
//! target configuration file provided with the project.
//!
//! User can edit some of configuration parameters as per use case, similar to
//! other examples.
//!
//! \b nLanes - Choice to select single or double lane for frame transfers
//! \b txPingFrameTag - Frame tag used for Ping transfers
//! \b counter_match_value - "match1" value for CLB which generates delay
//! \b epwm_cmpc_value_var - Used to align the EPWMSYNCIN signal
//!
//!\b External \b Connections \n
//!  For the FSI daisy-chain topology external connections are required to
//!  be made between the devices in the chain. Each devices FSI TX pins need to
//!  be connected to the FSI RX pins of the next device in the chain (or ring).
//!  See below for external connections to include and GPIOs used:
//!
//!  External Connections Required:
//!     - FSIRX_CLK  to  FSITX_CLK
//!     - FSIRX_RX0  to  FSITX_TX0
//!     - FSIRX_RX1  to  FSITX_TX1 [Only needed when nLanes = 2_LANE]
//!
//!  ControlCard FSI Header GPIOs:
//!     - GPIO_27  ->    FSITX_CLK
//!     - GPIO_26  ->    FSITX_TX0
//!     - GPIO_25  ->    FSITX_TX1
//!     - GPIO_13  ->    FSIRX_CLK
//!     - GPIO_12  ->    FSIRX_RX0
//!     - GPIO_11  ->    FSIRX_RX1
//!
//!  LaunchPad FSI Header GPIOs:
//!     - GPIO_7   ->    FSITX_CLK
//!     - GPIO_6   ->    FSITX_TX0
//!     - GPIO_25   ->    FSITX_TX1
//!     - GPIO_33  ->    FSIRX_CLK
//!     - GPIO_12  ->    FSIRX_RX0
//!     - GPIO_2  ->    FSIRX_RX1
//!
//! \b Watch \b Variables \n
//!  - \b error          Non zero for transmit/receive data mismatch
//!  - \b FSIRxintCount  Number of Data frame received
//!
//! GPIO pins where output can be monitored:
//!  - GPIO_0  for EPWM1A output
//!  - GPIO_58 for EPWMSYNCIN signal without delay
//!  - GPIO_3  for EPWMSYNCIN signal with delay from CLB
//!  - LED1 and LED2 for handshake process
//!
//#############################################################################
// $TI Release: v3.04.00.00 $
// $Release Date: 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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
// Included Files
//
#include "driverlib.h"
#include "device.h"

#define PRESCALER_VAL   FSI_PRESCALE_50MHZ

//
// Globals, User can modify these parameters as per usecase
//

//
// Transfer can be happen over single or double lane
//
FSI_DataWidth nLanes = FSI_DATA_WIDTH_1_LANE;

//
// Frame tag used with Data/Ping transfers
//
FSI_FrameTag txPingFrameTag = FSI_FRAME_TAG0;

//
// Globals, these are not config parameters, user are not required to edit them
//
uint16_t txEventSts = 0, rxEventSts = 0;
uint16_t *txBufAddr = 0, *rxBufAddr = 0;

volatile uint32_t fsiTxInt1Received = 0, fsiTxInt2Received = 0;
volatile uint32_t fsiRxInt1Received = 0, fsiRxInt2Received = 0;
uint32_t txTimeOutCntr = 0x100000, rxTimeOutCntr = 0x100000;
volatile uint32_t FSIRxintCount = 0;

volatile uint16_t i = 0;

uint32_t error = 0;

//
// Function Prototypes
//
static inline void compare16(uint16_t val1, uint16_t val2);
void disableAllFSIInterrupts(void);
void checkReceivedFrameTypeTag(FSI_FrameType type, FSI_FrameTag tag);
void initFSI(void);
void testGPIO(void);
__interrupt void fsiTxInt1ISR(void);
__interrupt void fsiTxInt2ISR(void);
__interrupt void fsiRxInt1ISR(void);
__interrupt void fsiRxInt2ISR(void);

void handshake_node(void);

void ConfigPWM();

//
// EPWM Configurations
// 20kHz PWM
//
#define EPWM_TIMER_TBPRD  2499U
#define EPWM_CMPA_VALUE   1249U
#define EPWM_CMPB_VALUE   1249U

//
// User can choose any of 16 ePWM SOC event triggers
//
FSI_ExtFrameTriggerSrc ePWMTrigSel = FSI_EXT_TRIGSRC_EPWM1_SOCB;

//
// ePWM base addresses to operate on selected module and also Sysctl Clock to
// enable/disable them.
// Need to change base address as per ePWM trigger selection in \b ePWMTrigSel
//
uint32_t ePWMBaseAddr = EPWM1_BASE;
SysCtl_PeripheralPCLOCKCR epwmSysCtlClock = SYSCTL_PERIPH_CLK_EPWM1;

//
//CLB includes
//
#include "clb_config.h"
#include "clb.h"

uint32_t counter_match_value = TILE1_COUNTER_0_MATCH1_VAL;

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();
    testGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Initialize basic settings for FSI
    //
    initFSI();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file. Total 4; FSI Tx/Rx :: INT1/INT2
    //
    Interrupt_register(INT_FSITXA_INT1, &fsiTxInt1ISR);
    Interrupt_register(INT_FSITXA_INT2, &fsiTxInt2ISR);
    Interrupt_register(INT_FSIRXA_INT1, &fsiRxInt1ISR);
    Interrupt_register(INT_FSIRXA_INT2, &fsiRxInt2ISR);

    //
    // Enable FSI Tx/Rx interrupts
    //
    Interrupt_enable(INT_FSITXA_INT1);
    Interrupt_enable(INT_FSITXA_INT2);
    Interrupt_enable(INT_FSIRXA_INT1);
    Interrupt_enable(INT_FSIRXA_INT2);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;


    //
    // Configure CLB
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);

    CLB_enableCLB(CLB1_BASE);
    initTILE1(CLB1_BASE);

    //
    // Select Global input instead of local input for all CLB IN
    //
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN6, CLB_LOCAL_IN_MUX_GLOBAL_IN);
    CLB_configLocalInputMux(CLB1_BASE, CLB_IN7, CLB_LOCAL_IN_MUX_GLOBAL_IN);


    //
    // Select FSI_PING_PKT_RCVD for CLB1, IN0
    //
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_FSIRXA_PING_PACKET_RCVD);

    //
    // Unused Inputs below
    //
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN6, CLB_GLOBAL_IN_MUX_EPWM2A);
    CLB_configGlobalInputMux(CLB1_BASE, CLB_IN7, CLB_GLOBAL_IN_MUX_EPWM2A);

    //
    // Select External for CLB1, IN0
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);

    //
    // Unused inputs to GP register
    //
    CLB_configGPInputMux(CLB1_BASE, CLB_IN1, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN2, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN3, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN4, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN5, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN6, CLB_GP_IN_MUX_GP_REG);
    CLB_configGPInputMux(CLB1_BASE, CLB_IN7, CLB_GP_IN_MUX_GP_REG);

    CLB_setOutputMask(CLB1_BASE, CLB_OUTPUT_12 | CLB_OUTPUT_13, true);

    //
    // Configure the ePWM for 20kHz
    //
    ConfigPWM();

    //
    // EXTSYNCHIN1 configured to be routed from CLB compensated output
    // GPIO-3 -> INPUTXBAR.INPUT5 -> EXTSYNCHIN1
    //
    SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM1,
                              SYSCTL_SYNC_IN_SRC_EXTSYNCIN1);

    //
    // Configure ePWM output signal
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_EPWM1A);

    //
    // Enable peripheral clk for ePWM
    //
    SysCtl_enablePeripheral(epwmSysCtlClock);

    //
    // Start ePWM
    //
    EPWM_setEmulationMode(ePWMBaseAddr, EPWM_EMULATION_FREE_RUN);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // GPIO for monitoring SYNCH
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT1, XBAR_OUT_MUX01_CLB1_OUT4);
    XBAR_enableOutputMux(XBAR_OUTPUT1, XBAR_MUX01);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_58_OUTPUTXBAR1);

    //
    // GPIO for monitoring SYNCH with delay
    //
    XBAR_setOutputMuxConfig(XBAR_OUTPUT2, XBAR_OUT_MUX03_CLB1_OUT5);
    XBAR_enableOutputMux(XBAR_OUTPUT2, XBAR_MUX03);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_3_OUTPUTXBAR2);

    //
    // GPIO-3 is routed back to XBAR_INPUT-5 which is connected to EXTSYNCHIN1
    //
    XBAR_setInputPin(XBAR_INPUT5, 3);

    //
    // XBAR Config
    // CLB to PWM XBAR (CLB1_OUT4 to XBAR_TRIP4)
    // PWM XBAR to FSI has been hard coded at the start of this code.
    // For details on the connection, refer to Ext Frame Trigger Mux
    // section of the FSI chapter in the device TRM. 
    //
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, XBAR_EPWM_MUX01_CLB1_OUT4);
    XBAR_enableEPWMMux(XBAR_TRIP4, XBAR_MUX01);

    //
    // Enable normal data transfer events to be sent over INT1 line
    //
    FSI_enableRxInterrupt(FSIRXA_BASE, FSI_INT1, FSI_RX_EVT_PING_FRAME);

    //
    // Turn LED 1 ON and LED 2 OFF
    //
    GPIO_writePin(DEVICE_GPIO_PIN_LED1, 0);
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);

    //
    // Wait till interrupt is received on FSIRX INT1 line, verify it's for FRAME
    // DONE event for PING Frame reception
    //
    handshake_node();

    //
    // Turn LED 2 ON
    // Visual confirmation for the completion of handshake process
    //
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 0);

    //
    // Setup ping transmit on ping receive
    //
    FSI_setTxFrameType(FSITXA_BASE, FSI_FRAME_TYPE_PING);
    FSI_setTxFrameTag(FSITXA_BASE, txPingFrameTag);
    FSI_setTxStartMode(FSITXA_BASE, FSI_TX_START_EXT_TRIG);
    FSI_setTxExtFrameTrigger(FSITXA_BASE, 0);

    //
    // Continuous loop here
    // - FSI TX ping frames will be triggered on RX_PING_FRAME flag
    // - ePWM will be synchronized on RX_PING_FRAME + configured CLB delay
    //
    while(1);

}

void handshake_node(void)
{

    while(1)
    {
        while(fsiRxInt1Received != 1);
        compare16(rxEventSts, (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));
        checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, FSI_FRAME_TAG0);
        //
        // If received frame type and tag matches, exit this loop and proceed to
        // next step by sending flush sequence, otherwise clear error and
        // interrupt flag and continue looping.
        //
        if(error == 0)
        {
            fsiRxInt1Received = 0;
            break;
        }

        fsiRxInt1Received = 0;
        error = 0;
    }

    while(1)
    {
        //
        // Send the flush sequence
        //
        FSI_executeTxFlushSequence(FSITXA_BASE, PRESCALER_VAL);

        //
        // Send a ping frame with frame tag 0000b
        //
        FSI_setTxFrameTag(FSITXA_BASE, FSI_FRAME_TAG0);
        FSI_setTxFrameType(FSITXA_BASE, FSI_FRAME_TYPE_PING);
        FSI_startTxTransmit(FSITXA_BASE);

        while(fsiRxInt1Received != 1U && rxTimeOutCntr != 0U)
        {
            DEVICE_DELAY_US(1);
            rxTimeOutCntr--;
        }

        if(rxTimeOutCntr == 0)
        {
            rxTimeOutCntr = 0x100000;
            continue;
        }
        else
        {
            compare16(rxEventSts, (FSI_RX_EVT_PING_FRAME | FSI_RX_EVT_FRAME_DONE));
            checkReceivedFrameTypeTag(FSI_FRAME_TYPE_PING, FSI_FRAME_TAG1);
            //
            // If received frame type and tag matches, exit this loop and
            // proceed to next step by sending flush sequence, otherwise clear
            // error and interrupt flag and continue looping.
            //
            if(error == 0)
            {
                fsiRxInt1Received = 0;
                break;
            }

            fsiRxInt1Received = 0;
            error = 0;
        }
    }
    //
    // Send a ping frame with frame tag 0001b
    //
    FSI_setTxFrameTag(FSITXA_BASE, FSI_FRAME_TAG1);
    FSI_setTxFrameType(FSITXA_BASE, FSI_FRAME_TYPE_PING);
    FSI_startTxTransmit(FSITXA_BASE);
    DEVICE_DELAY_US(10);
}

//
// initFSI - Initializes FSI Tx/Rx and also sends FLUSH sequence.
//
void initFSI(void)
{
    FSI_disableRxInternalLoopback(FSIRXA_BASE);

    //
    // NOTE: External loopback, Modify GPIO settings as per setup
    //
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TXCLK);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TX0);

    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RXCLK);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RX0);

    if(nLanes == FSI_DATA_WIDTH_2_LANE)
    {
        GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_TX1);
        GPIO_setPinConfig(DEVICE_GPIO_CFG_FSI_RX1);
    }

    //
    // Set RX GPIO to be asynchronous
    // (pass through without delay)
    // Default setting is to have 2 SYS_CLK cycles delay
    //
    if(nLanes == FSI_DATA_WIDTH_2_LANE)
    {
        GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RX1, GPIO_QUAL_ASYNC);
    }
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RX0, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_FSI_RXCLK, GPIO_QUAL_ASYNC);

    FSI_performTxInitialization(FSITXA_BASE, PRESCALER_VAL);
    FSI_performRxInitialization(FSIRXA_BASE);
    txBufAddr = (uint16_t *)FSI_getTxBufferAddress(FSITXA_BASE);
    rxBufAddr = (uint16_t *)FSI_getRxBufferAddress(FSIRXA_BASE);
}

//
// fsiTxInt1ISR - FSI Tx Interrupt on INsT1 line
//
__interrupt void fsiTxInt1ISR(void)
{

    fsiTxInt1Received = 1U;
    txEventSts = FSI_getTxEventStatus(FSITXA_BASE);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearTxEvents(FSITXA_BASE, FSI_TX_EVTMASK);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
}

//
// fsiTxInt2ISR - FSI Tx Interrupt on INT2 line
//
__interrupt void fsiTxInt2ISR(void)
{
    fsiTxInt2Received = 1U;

    txEventSts = FSI_getTxEventStatus(FSITXA_BASE);

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearTxEvents(FSITXA_BASE, FSI_TX_EVTMASK);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    disableAllFSIInterrupts();

    //
    // INT2 line is set to fire for error events, stop immediately. Actual Error
    // is captured in txEventSts for debug
    //
    ESTOP0;
}

//
// fsiRxInt1ISR - FSI Rx Interrupt on INT1 line
//
__interrupt void fsiRxInt1ISR(void)
{
    rxEventSts = FSI_getRxEventStatus(FSIRXA_BASE);

    fsiRxInt1Received = 1U;

    FSIRxintCount++;

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(FSIRXA_BASE, rxEventSts);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
}

//
// fsiRxInt2ISR - FSI Rx Interrupt on INT2 line
//
__interrupt void fsiRxInt2ISR(void)
{
    rxEventSts = FSI_getRxEventStatus(FSIRXA_BASE);

    fsiRxInt2Received = fsiRxInt2Received + 1U;

    //
    // Clear the interrupt flag and issue ACK
    //
    FSI_clearRxEvents(FSIRXA_BASE, rxEventSts);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    disableAllFSIInterrupts();

    //
    // INT2 line is set to fire for error events, stop immediately. Error
    // is captured in rxEventSts for debug
    //
    ESTOP0;
}

//
// disableAllFSIInterrupts - Disables all event interrupts in both FSI Tx/Rx,
//                           also clear them
//
void disableAllFSIInterrupts(void)
{
    FSI_disableTxInterrupt(FSITXA_BASE, FSI_INT1, FSI_TX_EVTMASK);
    FSI_disableTxInterrupt(FSITXA_BASE, FSI_INT2, FSI_TX_EVTMASK);
    FSI_disableRxInterrupt(FSIRXA_BASE, FSI_INT1, FSI_RX_EVTMASK);
    FSI_disableRxInterrupt(FSIRXA_BASE, FSI_INT2, FSI_RX_EVTMASK);

    FSI_clearTxEvents(FSITXA_BASE, FSI_TX_EVTMASK);
    FSI_clearRxEvents(FSIRXA_BASE, FSI_RX_EVTMASK);
}

//
// compare16 - Compares two 16 bit values and increments global error flag by 1
//             for mismatch
//
static inline void compare16(uint16_t val1, uint16_t val2)
{
    if(val1 != val2)
    {
        error++;
    }
}

//
// checkReceivedFrameTypeTag - Checks received frame type/tag and updates global
//                             error flag
//
void checkReceivedFrameTypeTag(FSI_FrameType type, FSI_FrameTag tag)
{
    compare16((uint16_t)FSI_getRxFrameType(FSIRXA_BASE), (uint16_t)type);

    if(type == FSI_FRAME_TYPE_PING)
    {
        compare16(FSI_getRxPingTag(FSIRXA_BASE), (uint16_t)tag);
    }
    else
    {
        compare16(FSI_getRxFrameTag(FSIRXA_BASE), (uint16_t)tag);
    }
}

void testGPIO(void)
{
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);
}

//
// ConfigPWM - Configures requested ePWM
//             TB counter is in up/down count mode for this example
//
void ConfigPWM()
{
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(ePWMBaseAddr, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(ePWMBaseAddr, 0U);
    EPWM_setTimeBaseCounter(ePWMBaseAddr, 0U);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(ePWMBaseAddr,
                                EPWM_COUNTER_COMPARE_A,
                                EPWM_CMPA_VALUE);

    EPWM_setCounterCompareValue(ePWMBaseAddr,
                                EPWM_COUNTER_COMPARE_B,
                                EPWM_CMPB_VALUE);

    EPWM_setTimeBaseCounterMode(ePWMBaseAddr, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_enablePhaseShiftLoad(ePWMBaseAddr);
    EPWM_setPhaseShift(ePWMBaseAddr, EPWM_CMPA_VALUE);

    EPWM_setClockPrescaler(ePWMBaseAddr,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(ePWMBaseAddr,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(ePWMBaseAddr,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(ePWMBaseAddr,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(ePWMBaseAddr,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(ePWMBaseAddr,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(ePWMBaseAddr,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


    //
    // Setup ePWM SOC trigger
    //
    EPWM_setADCTriggerEventPrescale(ePWMBaseAddr, EPWM_SOC_A, 1);
    EPWM_setADCTriggerEventPrescale(ePWMBaseAddr, EPWM_SOC_B, 1);
    EPWM_enableADCTrigger(ePWMBaseAddr, EPWM_SOC_A);
    EPWM_enableADCTrigger(ePWMBaseAddr, EPWM_SOC_B);
    EPWM_setADCTriggerSource(ePWMBaseAddr, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerSource(ePWMBaseAddr, EPWM_SOC_B, EPWM_SOC_TBCTR_PERIOD);
}

//
// End of File
//
