//#############################################################################
//
// FILE:   ethernet_ex9_revmii_example_mii_remote.c
//
// TITLE:  Ethernet RevMII example
//
//! \addtogroup driver_example_cm_list
//! <h1> Ethernet RevMII Example MII side </h1>
//!
//! This example demonstrates the steps to be followed in using the
//! Ethernet of the Communication Manager Subsystem to initialize
//! the Ethernet module and Configure the module in MII mode.
//! The other device is running a RevMII mode
//! This demonstrates the sequence for MII - RevMII communication
//! Before running this Communication Manager code the C28x cpu1 code has to be run
//! to configure the clocks to Communication manager
//! and required IO pads for Ethernet module
//! This Example has been validated on an TI internal board and will not run on
//! Control Card. This has been run with RevMII Example Remote MAC side.
//! Once the RevMII mode is configured this appears like a PHY to the external
//! MAC which is connected over MDIO. Even though there is no physical PHY
//! the RevMII mode lets the remote MAC see this side as a MAC.
//!
//! \b External \b Connections \n
//! TBD
//!
//! \b Watch \b Variables \n
//!  - None
//!
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
#include "driverlib_cm.h"
#include "cm.h"

//
// Defines
//
#define PACKET_LENGTH 132

#define ETHERNET_NO_OF_RX_PACKETS   1U
//
//Change this define for changing Packet buffer length
//
#define ETHERNET_MAX_PACKET_LENGTH 128U
uint8_t Ethernet_rxBuffer[ETHERNET_NO_OF_RX_PACKETS *
                          ETHERNET_MAX_PACKET_LENGTH];

#define REVMII_REMOTE_PHY_ADDRESS 0U
#define REVMII_LOCAL_PHY_ADDRESS 0xAU

//
// Globals
//
uint8_t pData[PACKET_LENGTH];

//
// Main
//
void main(void)
{
    Ethernet_InitInterfaceConfig initInterfaceConfig;
    Ethernet_InitConfig *pInitCfg;
    Ethernet_Pkt_Desc pktDesc;
    uint32_t i;
    Ethernet_Statistics stats;
    Ethernet_Handle emac_handle;
    uint16_t revmiiIntStatus;

    //
    // Initialize device clock and peripherals
    //
    CM_init();

    //
    //Select the MII interface of the module
    //
    initInterfaceConfig.ssbase = EMAC_SS_BASE;
    initInterfaceConfig.enet_base = EMAC_BASE;
    initInterfaceConfig.phyMode = ETHERNET_SS_PHY_INTF_SEL_MII;

    //
    //Assign SoC specific functions for Enabling,Disabling interrupts
    //and for enabling the Peripheral at system level
    //
    initInterfaceConfig.ptrPlatformInterruptDisable = &Platform_disableInterrupt;
    initInterfaceConfig.ptrPlatformInterruptEnable = &Platform_enableInterrupt;
    initInterfaceConfig.ptrPlatformPeripheralEnable = &Platform_enablePeripheral;
    initInterfaceConfig.ptrPlatformPeripheralReset = &Platform_resetPeripheral;
    //
    //Assign the peripheral number at the SoC
    //
    initInterfaceConfig.peripheralNum = SYSCTL_PERIPH_CLK_ENET;
    //
    //Assign the default SoC specific interrupt numbers of Ethernet interrupts
    //
    initInterfaceConfig.interruptNum[0] = INT_EMAC;
    initInterfaceConfig.interruptNum[1] = INT_EMAC_TX0;
    initInterfaceConfig.interruptNum[2] = INT_EMAC_TX1;
    initInterfaceConfig.interruptNum[3] = INT_EMAC_RX0;
    initInterfaceConfig.interruptNum[4] = INT_EMAC_RX1;

    pInitCfg = Ethernet_initInterface(initInterfaceConfig);

    //
    // Get an initial configuration of known good parameters
    //
    Ethernet_getInitConfig(pInitCfg);
    //
    //Assign the callbacks for Getting packet buffer when needed
    //Releasing the TxPacketBuffer on Transmit interrupt callbacks
    //Receive packet callback on Receive packet completion interrupt
    //
    pInitCfg->pfcbGetPacket = &Ethernet_getPacketBuffer;
    pInitCfg->pfcbFreePacket = &Ethernet_releaseTxPacketBuffer;
    pInitCfg->pfcbRxPacket = &Ethernet_receivePacketCallback;
    //
    //Assign the Buffer to be used by the Low level driver for receiving
    //Packets. This should be accessible by the Ethernet DMA
    //
    pInitCfg->rxBuffer = Ethernet_rxBuffer;
    //
    //The Application handle is not used by this application
    //Hence using a dummy value of 1
    //
    Ethernet_getHandle((Ethernet_Handle)1,pInitCfg , &emac_handle);


    //
    //Do global Interrupt Enable
    //
    (void)Interrupt_enableInProcessor();
    //
    //Assign default ISRs
    //
    Interrupt_registerHandler(INT_EMAC_TX0, Ethernet_transmitISR);
    Interrupt_registerHandler(INT_EMAC_RX0, Ethernet_receiveISR);
    //
    //Enable the default interrupt handlers
    //
    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);

    //
    //Form the Packet in Memory
    //
    for(i=0;i<PACKET_LENGTH/4;i++)
    {
        if(i == 0)
            *((uint32_t *)pData + i) = 0x01020304;
        else if(i == 1)
            *((uint32_t *)pData + i)  = 0xFFFF0506;
        else
            HWREG((uint32_t *)pData +i) = 0xFFFFFFFF;
    }
    //
    //Configure the PHY address for Remote PHY in the MDIO module
    //


    Ethernet_configurePHYAddress(EMAC_BASE,REVMII_REMOTE_PHY_ADDRESS);
    //
    //Rev MII PHY control register
    //Loopback and 100mbps mode
    //Ethernet_writePHYRegister(EMAC_BASE,0x0,0x6100);
    //No Loopback and 100mbps mode
    //
    Ethernet_readPHYRegister(EMAC_BASE,
                                                                  0x0);
    Ethernet_writePHYRegister(EMAC_BASE,0x0,0x2100);

    //Check for RevMII link status change interrupt
    //Read RevMII Interrupt Status Mask Register
    revmiiIntStatus =  Ethernet_readPHYRegister(EMAC_BASE,
                                                   0x10);
    if(revmiiIntStatus& 0x100)
    {
        //
        //Clear the interrupt by writing to LSI field of RevMII Interrupt
        //Status Mask Register
        //
        revmiiIntStatus &= 0x100;
        Ethernet_writePHYRegister(EMAC_BASE,
                                    0x10,
                                    revmiiIntStatus);
    }

    pktDesc.bufferLength = PACKET_LENGTH;
    pktDesc.dataOffset = 0;
    pktDesc.dataBuffer = pData;
    pktDesc.nextPacketDesc = 0;
    pktDesc.flags = ETHERNET_PKT_FLAG_SOP |ETHERNET_PKT_FLAG_EOP|ETHERNET_PKT_FLAG_SA_INS;
    pktDesc.pktChannel = ETHERNET_DMA_CHANNEL_NUM_0;
    pktDesc.pktLength = PACKET_LENGTH;
    pktDesc.validLength = PACKET_LENGTH;
    pktDesc.numPktFrags = 1;


    Ethernet_sendPacket(emac_handle,&pktDesc);

    //
    //We expect a packet to come from Remote side
    //Query Statistics to check if a packet has come
    //

    Ethernet_getStatistics(emac_handle, &stats);
    if(stats.rxUnicastPacketsGood)
        __asm("   bkpt #0");
    else
        __asm("   bkpt #0");

}