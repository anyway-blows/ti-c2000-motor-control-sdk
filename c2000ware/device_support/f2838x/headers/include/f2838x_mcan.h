//###########################################################################
//
// FILE:    f2838x_mcan.h
//
// TITLE:   Definitions for the MCAN registers.
//
//###########################################################################
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
//###########################################################################

#ifndef F2838x_MCAN_H
#define F2838x_MCAN_H

#ifdef __cplusplus 
extern "C" {
#endif 


//---------------------------------------------------------------------------
// MCAN Individual Register Bit Definitions:

struct MCANSS_PID_BITS {                // bits description
    Uint16 MINOR:6;                     // 5:0 Minor Revision
    Uint16 rsvd1:2;                     // 7:6 Reserved
    Uint16 MAJOR:3;                     // 10:8 Major Revision
    Uint16 rsvd2:5;                     // 15:11 Reserved
    Uint16 MODULE_ID:12;                // 27:16 Module Identification Number
    Uint16 rsvd3:2;                     // 29:28 Reserved
    Uint16 SCHEME:2;                    // 31:30 PID Register Scheme
};

union MCANSS_PID_REG {
    Uint32  all;
    struct  MCANSS_PID_BITS  bit;
};

struct MCANSS_CTRL_BITS {               // bits description
    Uint16 rsvd1:3;                     // 2:0 Reserved
    Uint16 DBGSUSP_FREE:1;              // 3 Debug Suspend Free
    Uint16 WAKEUPREQEN:1;               // 4 Wakeup Request Enable
    Uint16 AUTOWAKEUP:1;                // 5 Automatic Wakeup Enable
    Uint16 EXT_TS_CNTR_EN:1;            // 6 External Timestamp Counter Enable
    Uint16 rsvd2:9;                     // 15:7 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANSS_CTRL_REG {
    Uint32  all;
    struct  MCANSS_CTRL_BITS  bit;
};

struct MCANSS_STAT_BITS {               // bits description
    Uint16 RESET:1;                     // 0 Soft Reset Status
    Uint16 MEM_INIT_DONE:1;             // 1 Memory Initialization Done
    Uint16 ENABLE_FDOE:1;               // 2 Flexible Datarate Operation Enable
    Uint16 rsvd1:13;                    // 15:3 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_STAT_REG {
    Uint32  all;
    struct  MCANSS_STAT_BITS  bit;
};

struct MCANSS_ICS_BITS {                // bits description
    Uint16 EXT_TS_CNTR_OVFL:1;          // 0 External Timestamp Counter Overflow Interrupt Status Clear
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_ICS_REG {
    Uint32  all;
    struct  MCANSS_ICS_BITS  bit;
};

struct MCANSS_IRS_BITS {                // bits description
    Uint16 EXT_TS_CNTR_OVFL:1;          // 0 External Timestamp Counter Overflow Interrupt Status
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_IRS_REG {
    Uint32  all;
    struct  MCANSS_IRS_BITS  bit;
};

struct MCANSS_IECS_BITS {               // bits description
    Uint16 EXT_TS_CNTR_OVFL:1;          // 0 External Timestamp Counter Overflow Interrupt Enable Clear
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_IECS_REG {
    Uint32  all;
    struct  MCANSS_IECS_BITS  bit;
};

struct MCANSS_IE_BITS {                 // bits description
    Uint16 EXT_TS_CNTR_OVFL:1;          // 0 External Timestamp Counter Overflow Interrupt Enable
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_IE_REG {
    Uint32  all;
    struct  MCANSS_IE_BITS  bit;
};

struct MCANSS_IES_BITS {                // bits description
    Uint16 EXT_TS_CNTR_OVFL:1;          // 0 External Timestamp Counter Overflow Interrupt Enable Status
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_IES_REG {
    Uint32  all;
    struct  MCANSS_IES_BITS  bit;
};

struct MCANSS_EOI_BITS {                // bits description
    Uint16 EOI:8;                       // 7:0 External Timestamp Counter Overflow End of Interrupt
    Uint16 rsvd1:8;                     // 15:8 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_EOI_REG {
    Uint32  all;
    struct  MCANSS_EOI_BITS  bit;
};

struct MCANSS_EXT_TS_PRESCALER_BITS {   // bits description
    Uint32 PRESCALER:24;                // 23:0 External Timestamp Prescaler
    Uint16 rsvd1:8;                     // 31:24 Reserved
};

union MCANSS_EXT_TS_PRESCALER_REG {
    Uint32  all;
    struct  MCANSS_EXT_TS_PRESCALER_BITS  bit;
};

struct MCANSS_EXT_TS_UNSERVICED_INTR_CNTR_BITS {// bits description
    Uint16 EXT_TS_INTR_CNTR:5;          // 4:0 External Timestamp Counter Unserviced Rollover Interrupts
    Uint16 rsvd1:11;                    // 15:5 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANSS_EXT_TS_UNSERVICED_INTR_CNTR_REG {
    Uint32  all;
    struct  MCANSS_EXT_TS_UNSERVICED_INTR_CNTR_BITS  bit;
};

struct MCANSS_REGS {
    union   MCANSS_PID_REG                   MCANSS_PID;                   // MCAN Subsystem Revision Register
    union   MCANSS_CTRL_REG                  MCANSS_CTRL;                  // MCAN Subsystem Control Register
    union   MCANSS_STAT_REG                  MCANSS_STAT;                  // MCAN Subsystem Status Register
    union   MCANSS_ICS_REG                   MCANSS_ICS;                   // MCAN Subsystem Interrupt Clear Shadow Register
    union   MCANSS_IRS_REG                   MCANSS_IRS;                   // MCAN Subsystem Interrupt Raw Satus Register
    union   MCANSS_IECS_REG                  MCANSS_IECS;                  // MCAN Subsystem Interrupt Enable Clear Shadow Register
    union   MCANSS_IE_REG                    MCANSS_IE;                    // MCAN Subsystem Interrupt Enable Register
    union   MCANSS_IES_REG                   MCANSS_IES;                   // MCAN Subsystem Interrupt Enable Status
    union   MCANSS_EOI_REG                   MCANSS_EOI;                   // MCAN Subsystem End of Interrupt
    union   MCANSS_EXT_TS_PRESCALER_REG      MCANSS_EXT_TS_PRESCALER;      // MCAN Subsystem External Timestamp Prescaler 0
    union   MCANSS_EXT_TS_UNSERVICED_INTR_CNTR_REG     MCANSS_EXT_TS_UNSERVICED_INTR_CNTR;// MCAN Subsystem External Timestamp Unserviced Interrupts Counter
};

struct MCAN_CREL_BITS {                 // bits description
    Uint16 DAY:8;                       // 7:0 Time Stamp Day
    Uint16 MON:8;                       // 15:8 Time Stamp Month
    Uint16 YEAR:4;                      // 19:16 Time Stamp Year
    Uint16 SUBSTEP:4;                   // 23:20 Sub-Step of Core Release
    Uint16 STEP:4;                      // 27:24 Step of Core Release
    Uint16 REL:4;                       // 31:28 Core Release
};

union MCAN_CREL_REG {
    Uint32  all;
    struct  MCAN_CREL_BITS  bit;
};

struct MCAN_DBTP_BITS {                 // bits description
    Uint16 DSJW:4;                      // 3:0 Data Resynchronization Jump Width
    Uint16 DTSEG2:4;                    // 7:4 Data Time Segment After Sample Point
    Uint16 DTSEG1:5;                    // 12:8 Data Time Segment Before Sample Point
    Uint16 rsvd1:3;                     // 15:13 Reserved
    Uint16 DBRP:5;                      // 20:16 Data Bit Rate Prescaler
    Uint16 rsvd2:2;                     // 22:21 Reserved
    Uint16 TDC:1;                       // 23 Transmitter Delay Compensation
    Uint16 rsvd3:8;                     // 31:24 Reserved
};

union MCAN_DBTP_REG {
    Uint32  all;
    struct  MCAN_DBTP_BITS  bit;
};

struct MCAN_TEST_BITS {                 // bits description
    Uint16 rsvd1:4;                     // 3:0 Reserved
    Uint16 LBCK:1;                      // 4 Loop Back Mode
    Uint16 TX:2;                        // 6:5 Control of Transmit Pin
    Uint16 RX:1;                        // 7 Receive Pin
    Uint16 rsvd2:8;                     // 15:8 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCAN_TEST_REG {
    Uint32  all;
    struct  MCAN_TEST_BITS  bit;
};

struct MCAN_RWD_BITS {                  // bits description
    Uint16 WDC:8;                       // 7:0 Watchdog Configuration
    Uint16 WDV:8;                       // 15:8 Watchdog Value
    Uint16 rsvd1:16;                    // 31:16 Reserved
};

union MCAN_RWD_REG {
    Uint32  all;
    struct  MCAN_RWD_BITS  bit;
};

struct MCAN_CCCR_BITS {                 // bits description
    Uint16 INIT:1;                      // 0 Initialization
    Uint16 CCE:1;                       // 1 Configuration Change Enable
    Uint16 ASM:1;                       // 2 Restricted Operation Mode
    Uint16 CSA:1;                       // 3 Clock Stop Acknowledge
    Uint16 CSR:1;                       // 4 Clock Stop Request
    Uint16 MON:1;                       // 5 Bus Monitoring Mode
    Uint16 DAR:1;                       // 6 Disable Automatic Retransmission
    Uint16 TEST:1;                      // 7 Test Mode Enable
    Uint16 FDOE:1;                      // 8 Flexible Datarate Operation Enable
    Uint16 BRSE:1;                      // 9 Bit Rate Switch Enable
    Uint16 rsvd1:2;                     // 11:10 Reserved
    Uint16 PXHD:1;                      // 12 Protocol Exception Handling Disable
    Uint16 EFBI:1;                      // 13 Edge Filtering During Bus Integration
    Uint16 TXP:1;                       // 14 Transmit Pause
    Uint16 NISO:1;                      // 15 Non-ISO Operation
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_CCCR_REG {
    Uint32  all;
    struct  MCAN_CCCR_BITS  bit;
};

struct MCAN_NBTP_BITS {                 // bits description
    Uint16 NTSEG2:7;                    // 6:0 Nominal Time Segment After Sample Point
    Uint16 rsvd1:1;                     // 7 Reserved
    Uint16 NTSEG1:8;                    // 15:8 Nominal Time Segment Before Sample Point
    Uint16 NBRP:9;                      // 24:16 Nominal Bit Rate Prescaler
    Uint16 NSJW:7;                      // 31:25 Nominal (Re)Synchronization Jump Width
};

union MCAN_NBTP_REG {
    Uint32  all;
    struct  MCAN_NBTP_BITS  bit;
};

struct MCAN_TSCC_BITS {                 // bits description
    Uint16 TSS:2;                       // 1:0 Timestamp Select
    Uint16 rsvd1:14;                    // 15:2 Reserved
    Uint16 TCP:4;                       // 19:16 Timestamp Counter Prescaler
    Uint16 rsvd2:12;                    // 31:20 Reserved
};

union MCAN_TSCC_REG {
    Uint32  all;
    struct  MCAN_TSCC_BITS  bit;
};

struct MCAN_TSCV_BITS {                 // bits description
    Uint16 TSC:16;                      // 15:0 Timestamp Counter
    Uint16 rsvd1:16;                    // 31:16 Reserved
};

union MCAN_TSCV_REG {
    Uint32  all;
    struct  MCAN_TSCV_BITS  bit;
};

struct MCAN_TOCC_BITS {                 // bits description
    Uint16 ETOC:1;                      // 0 Enable Timeout Counter
    Uint16 TOS:2;                       // 2:1 Timeout Select
    Uint16 rsvd1:13;                    // 15:3 Reserved
    Uint16 TOP:16;                      // 31:16 Timeout Period
};

union MCAN_TOCC_REG {
    Uint32  all;
    struct  MCAN_TOCC_BITS  bit;
};

struct MCAN_TOCV_BITS {                 // bits description
    Uint16 TOC:16;                      // 15:0 Timeout Counter
    Uint16 rsvd1:16;                    // 31:16 Reserved
};

union MCAN_TOCV_REG {
    Uint32  all;
    struct  MCAN_TOCV_BITS  bit;
};

struct MCAN_ECR_BITS {                  // bits description
    Uint16 TEC:8;                       // 7:0 Transmit Error Counter
    Uint16 REC:7;                       // 14:8 Receive Error Counter
    Uint16 RP:1;                        // 15 Receive Error Passive
    Uint16 CEL:8;                       // 23:16 CAN Error Logging
    Uint16 rsvd1:8;                     // 31:24 Reserved
};

union MCAN_ECR_REG {
    Uint32  all;
    struct  MCAN_ECR_BITS  bit;
};

struct MCAN_PSR_BITS {                  // bits description
    Uint16 LEC:3;                       // 2:0 Last Error Code
    Uint16 ACT:2;                       // 4:3 Node Activity
    Uint16 EP:1;                        // 5 Error Passive
    Uint16 EW:1;                        // 6 Warning Status
    Uint16 BO:1;                        // 7 Bus_Off Status
    Uint16 DLEC:3;                      // 10:8 Data Phase Last Error Code
    Uint16 RESI:1;                      // 11 ESI Flag of Last Received CAN FD Message
    Uint16 RBRS:1;                      // 12 BRS Flag of Last Received CAN FD Message
    Uint16 RFDF:1;                      // 13 Received a CAN FD Message
    Uint16 PXE:1;                       // 14 Protocol Exception Event
    Uint16 rsvd1:1;                     // 15 Reserved
    Uint16 TDCV:7;                      // 22:16 Transmitter Delay Compensation Value
    Uint16 rsvd2:9;                     // 31:23 Reserved
};

union MCAN_PSR_REG {
    Uint32  all;
    struct  MCAN_PSR_BITS  bit;
};

struct MCAN_TDCR_BITS {                 // bits description
    Uint16 TDCF:7;                      // 6:0 Transmitter Delay Compensation Filter Window Length
    Uint16 rsvd1:1;                     // 7 Reserved
    Uint16 TDCO:7;                      // 14:8 Transmitter Delay Compensation Offset
    Uint16 rsvd2:1;                     // 15 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCAN_TDCR_REG {
    Uint32  all;
    struct  MCAN_TDCR_BITS  bit;
};

struct MCAN_IR_BITS {                   // bits description
    Uint16 RF0N:1;                      // 0 Rx FIFO 0 New Message
    Uint16 RF0W:1;                      // 1 Rx FIFO 0 Watermark Reached
    Uint16 RF0F:1;                      // 2 Rx FIFO 0 Full
    Uint16 RF0L:1;                      // 3 Rx FIFO 0 Message Lost
    Uint16 RF1N:1;                      // 4 Rx FIFO 1 New Message
    Uint16 RF1W:1;                      // 5 Rx FIFO 1 Watermark Reached
    Uint16 RF1F:1;                      // 6 Rx FIFO 1 Full
    Uint16 RF1L:1;                      // 7 Rx FIFO 1 Message Lost
    Uint16 HPM:1;                       // 8 High Priority Message
    Uint16 TC:1;                        // 9 Transmission Completed
    Uint16 TCF:1;                       // 10 Transmission Cancellation Finished
    Uint16 TFE:1;                       // 11 Tx FIFO Empty
    Uint16 TEFN:1;                      // 12 Tx Event FIFO New Entry
    Uint16 TEFW:1;                      // 13 Tx Event FIFO Watermark Reached
    Uint16 TEFF:1;                      // 14 Tx Event FIFO Full
    Uint16 TEFL:1;                      // 15 Tx Event FIFO Element Lost
    Uint16 TSW:1;                       // 16 Timestamp Wraparound
    Uint16 MRAF:1;                      // 17 Message RAM Access Failure
    Uint16 TOO:1;                       // 18 Timeout Occurred
    Uint16 DRX:1;                       // 19 Message Stored to Dedicated Rx Buffer
    Uint16 rsvd1:1;                     // 20 Reserved
    Uint16 BEU:1;                       // 21 Bit Error Uncorrected
    Uint16 ELO:1;                       // 22 Error Logging Overflow
    Uint16 EP:1;                        // 23 Error Passive
    Uint16 EW:1;                        // 24 Warning Status
    Uint16 BO:1;                        // 25 Bus_Off Status
    Uint16 WDI:1;                       // 26 Watchdog Interrupt
    Uint16 PEA:1;                       // 27 Protocol Error in Arbitration Phase
    Uint16 PED:1;                       // 28 Protocol Error in Data Phase
    Uint16 ARA:1;                       // 29 Access to Reserved Address
    Uint16 rsvd2:1;                     // 30 Reserved
    Uint16 rsvd3:1;                     // 31 Reserved
};

union MCAN_IR_REG {
    Uint32  all;
    struct  MCAN_IR_BITS  bit;
};

struct MCAN_IE_BITS {                   // bits description
    Uint16 RF0NE:1;                     // 0 Rx FIFO 0 New Message Enable
    Uint16 RF0WE:1;                     // 1 Rx FIFO 0 Watermark Reached Enable
    Uint16 RF0FE:1;                     // 2 Rx FIFO 0 Full Enable
    Uint16 RF0LE:1;                     // 3 Rx FIFO 0 Message Lost Enable
    Uint16 RF1NE:1;                     // 4 Rx FIFO 1 New Message Enable
    Uint16 RF1WE:1;                     // 5 Rx FIFO 1 Watermark Reached Enable
    Uint16 RF1FE:1;                     // 6 Rx FIFO 1 Full Enable
    Uint16 RF1LE:1;                     // 7 Rx FIFO 1 Message Lost Enable
    Uint16 HPME:1;                      // 8 High Priority Message Enable
    Uint16 TCE:1;                       // 9 Transmission Completed Enable
    Uint16 TCFE:1;                      // 10 Transmission Cancellation Finished Enable
    Uint16 TFEE:1;                      // 11 Tx FIFO Empty Enable
    Uint16 TEFNE:1;                     // 12 Tx Event FIFO New Entry Enable
    Uint16 TEFWE:1;                     // 13 Tx Event FIFO Watermark Reached Enable
    Uint16 TEFFE:1;                     // 14 Tx Event FIFO Full Enable
    Uint16 TEFLE:1;                     // 15 Tx Event FIFO Element Lost Enable
    Uint16 TSWE:1;                      // 16 Timestamp Wraparound Enable
    Uint16 MRAFE:1;                     // 17 Message RAM Access Failure Enable
    Uint16 TOOE:1;                      // 18 Timeout Occurred Enable
    Uint16 DRXE:1;                      // 19 Message Stored to Dedicated Rx Buffer Enable
    Uint16 BECE:1;                      // 20 Bit Error Corrected Enable
    Uint16 BEUE:1;                      // 21 Bit Error Uncorrected Enable
    Uint16 ELOE:1;                      // 22 Error Logging Overflow Enable
    Uint16 EPE:1;                       // 23 Error Passive Enable
    Uint16 EWE:1;                       // 24 Warning Status Enable
    Uint16 BOE:1;                       // 25 Bus_Off Status Enable
    Uint16 WDIE:1;                      // 26 Watchdog Interrupt Enable
    Uint16 PEAE:1;                      // 27 Protocol Error in Arbitration Phase Enable
    Uint16 PEDE:1;                      // 28 Protocol Error in Data Phase Enable
    Uint16 ARAE:1;                      // 29 Access to Reserved Address Enable
    Uint16 rsvd1:2;                     // 31:30 Reserved
};

union MCAN_IE_REG {
    Uint32  all;
    struct  MCAN_IE_BITS  bit;
};

struct MCAN_ILS_BITS {                  // bits description
    Uint16 RF0NL:1;                     // 0 Rx FIFO 0 New Message Line
    Uint16 RF0WL:1;                     // 1 Rx FIFO 0 Watermark Reached Line
    Uint16 RF0FL:1;                     // 2 Rx FIFO 0 Full Line
    Uint16 RF0LL:1;                     // 3 Rx FIFO 0 Message Lost Line
    Uint16 RF1NL:1;                     // 4 Rx FIFO 1 New Message Line
    Uint16 RF1WL:1;                     // 5 Rx FIFO 1 Watermark Reached Line
    Uint16 RF1FL:1;                     // 6 Rx FIFO 1 Full Line
    Uint16 RF1LL:1;                     // 7 Rx FIFO 1 Message Lost Line
    Uint16 HPML:1;                      // 8 High Priority Message Line
    Uint16 TCL:1;                       // 9 Transmission Completed Line
    Uint16 TCFL:1;                      // 10 Transmission Cancellation Finished Line
    Uint16 TFEL:1;                      // 11 Tx FIFO Empty Line
    Uint16 TEFNL:1;                     // 12 Tx Event FIFO New Entry Line
    Uint16 TEFWL:1;                     // 13 Tx Event FIFO Watermark Reached Line
    Uint16 TEFFL:1;                     // 14 Tx Event FIFO Full Line
    Uint16 TEFLL:1;                     // 15 Tx Event FIFO Element Lost Line
    Uint16 TSWL:1;                      // 16 Timestamp Wraparound Line
    Uint16 MRAFL:1;                     // 17 Message RAM Access Failure Line
    Uint16 TOOL:1;                      // 18 Timeout Occurred Line
    Uint16 DRXL:1;                      // 19 Message Stored to Dedicated Rx Buffer Line
    Uint16 BECL:1;                      // 20 Bit Error Corrected Line
    Uint16 BEUL:1;                      // 21 Bit Error Uncorrected Line
    Uint16 ELOL:1;                      // 22 Error Logging Overflow Line
    Uint16 EPL:1;                       // 23 Error Passive Line
    Uint16 EWL:1;                       // 24 Warning Status Line
    Uint16 BOL:1;                       // 25 Bus_Off Status Line
    Uint16 WDIL:1;                      // 26 Watchdog Interrupt Line
    Uint16 PEAL:1;                      // 27 Protocol Error in Arbitration Phase Line
    Uint16 PEDL:1;                      // 28 Protocol Error in Data Phase Line
    Uint16 ARAL:1;                      // 29 Access to Reserved Address Line
    Uint16 rsvd1:2;                     // 31:30 Reserved
};

union MCAN_ILS_REG {
    Uint32  all;
    struct  MCAN_ILS_BITS  bit;
};

struct MCAN_ILE_BITS {                  // bits description
    Uint16 EINT0:1;                     // 0 Enable Interrupt Line 0
    Uint16 EINT1:1;                     // 1 Enable Interrupt Line 1
    Uint16 rsvd1:14;                    // 15:2 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_ILE_REG {
    Uint32  all;
    struct  MCAN_ILE_BITS  bit;
};

struct MCAN_GFC_BITS {                  // bits description
    Uint16 RRFE:1;                      // 0 Reject Remote Frames Extended
    Uint16 RRFS:1;                      // 1 Reject Remote Frames Standard
    Uint16 ANFE:2;                      // 3:2 Accept Non-matching Frames Extended
    Uint16 ANFS:2;                      // 5:4 Accept Non-matching Frames Standard
    Uint16 rsvd1:10;                    // 15:6 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_GFC_REG {
    Uint32  all;
    struct  MCAN_GFC_BITS  bit;
};

struct MCAN_SIDFC_BITS {                // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 FLSSA:14;                    // 15:2 Filter List Standard Start Address
    Uint16 LSS:8;                       // 23:16 List Size Standard
    Uint16 rsvd2:8;                     // 31:24 Reserved
};

union MCAN_SIDFC_REG {
    Uint32  all;
    struct  MCAN_SIDFC_BITS  bit;
};

struct MCAN_XIDFC_BITS {                // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 FLESA:14;                    // 15:2 Filter List Extended Start Address
    Uint16 LSE:7;                       // 22:16 List Size Extended
    Uint16 rsvd2:9;                     // 31:23 Reserved
};

union MCAN_XIDFC_REG {
    Uint32  all;
    struct  MCAN_XIDFC_BITS  bit;
};

struct MCAN_XIDAM_BITS {                // bits description
    Uint32 EIDM:29;                     // 28:0 Extended ID Mask
    Uint16 rsvd1:3;                     // 31:29 Reserved
};

union MCAN_XIDAM_REG {
    Uint32  all;
    struct  MCAN_XIDAM_BITS  bit;
};

struct MCAN_HPMS_BITS {                 // bits description
    Uint16 BIDX:6;                      // 5:0 Buffer Index
    Uint16 MSI:2;                       // 7:6 Message Storage Indicator
    Uint16 FIDX:7;                      // 14:8 Filter Index
    Uint16 FLST:1;                      // 15 Filter List
    Uint16 rsvd1:16;                    // 31:16 Reserved
};

union MCAN_HPMS_REG {
    Uint32  all;
    struct  MCAN_HPMS_BITS  bit;
};

struct MCAN_NDAT1_BITS {                // bits description
    Uint16 ND0:1;                       // 0 New Data RX Buffer 0
    Uint16 ND1:1;                       // 1 New Data RX Buffer 1
    Uint16 ND2:1;                       // 2 New Data RX Buffer 2
    Uint16 ND3:1;                       // 3 New Data RX Buffer 3
    Uint16 ND4:1;                       // 4 New Data RX Buffer 4
    Uint16 ND5:1;                       // 5 New Data RX Buffer 5
    Uint16 ND6:1;                       // 6 New Data RX Buffer 6
    Uint16 ND7:1;                       // 7 New Data RX Buffer 7
    Uint16 ND8:1;                       // 8 New Data RX Buffer 8
    Uint16 ND9:1;                       // 9 New Data RX Buffer 9
    Uint16 ND10:1;                      // 10 New Data RX Buffer 10
    Uint16 ND11:1;                      // 11 New Data RX Buffer 11
    Uint16 ND12:1;                      // 12 New Data RX Buffer 12
    Uint16 ND13:1;                      // 13 New Data RX Buffer 13
    Uint16 ND14:1;                      // 14 New Data RX Buffer 14
    Uint16 ND15:1;                      // 15 New Data RX Buffer 15
    Uint16 ND16:1;                      // 16 New Data RX Buffer 16
    Uint16 ND17:1;                      // 17 New Data RX Buffer 17
    Uint16 ND18:1;                      // 18 New Data RX Buffer 18
    Uint16 ND19:1;                      // 19 New Data RX Buffer 19
    Uint16 ND20:1;                      // 20 New Data RX Buffer 20
    Uint16 ND21:1;                      // 21 New Data RX Buffer 21
    Uint16 ND22:1;                      // 22 New Data RX Buffer 22
    Uint16 ND23:1;                      // 23 New Data RX Buffer 23
    Uint16 ND24:1;                      // 24 New Data RX Buffer 24
    Uint16 ND25:1;                      // 25 New Data RX Buffer 25
    Uint16 ND26:1;                      // 26 New Data RX Buffer 26
    Uint16 ND27:1;                      // 27 New Data RX Buffer 27
    Uint16 ND28:1;                      // 28 New Data RX Buffer 28
    Uint16 ND29:1;                      // 29 New Data RX Buffer 29
    Uint16 ND30:1;                      // 30 New Data RX Buffer 30
    Uint16 ND31:1;                      // 31 New Data RX Buffer 31
};

union MCAN_NDAT1_REG {
    Uint32  all;
    struct  MCAN_NDAT1_BITS  bit;
};

struct MCAN_NDAT2_BITS {                // bits description
    Uint16 ND32:1;                      // 0 New Data RX Buffer 32
    Uint16 ND33:1;                      // 1 New Data RX Buffer 33
    Uint16 ND34:1;                      // 2 New Data RX Buffer 34
    Uint16 ND35:1;                      // 3 New Data RX Buffer 35
    Uint16 ND36:1;                      // 4 New Data RX Buffer 36
    Uint16 ND37:1;                      // 5 New Data RX Buffer 37
    Uint16 ND38:1;                      // 6 New Data RX Buffer 38
    Uint16 ND39:1;                      // 7 New Data RX Buffer 39
    Uint16 ND40:1;                      // 8 New Data RX Buffer 40
    Uint16 ND41:1;                      // 9 New Data RX Buffer 41
    Uint16 ND42:1;                      // 10 New Data RX Buffer 42
    Uint16 ND43:1;                      // 11 New Data RX Buffer 43
    Uint16 ND44:1;                      // 12 New Data RX Buffer 44
    Uint16 ND45:1;                      // 13 New Data RX Buffer 45
    Uint16 ND46:1;                      // 14 New Data RX Buffer 46
    Uint16 ND47:1;                      // 15 New Data RX Buffer 47
    Uint16 ND48:1;                      // 16 New Data RX Buffer 48
    Uint16 ND49:1;                      // 17 New Data RX Buffer 49
    Uint16 ND50:1;                      // 18 New Data RX Buffer 50
    Uint16 ND51:1;                      // 19 New Data RX Buffer 51
    Uint16 ND52:1;                      // 20 New Data RX Buffer 52
    Uint16 ND53:1;                      // 21 New Data RX Buffer 53
    Uint16 ND54:1;                      // 22 New Data RX Buffer 54
    Uint16 ND55:1;                      // 23 New Data RX Buffer 55
    Uint16 ND56:1;                      // 24 New Data RX Buffer 56
    Uint16 ND57:1;                      // 25 New Data RX Buffer 57
    Uint16 ND58:1;                      // 26 New Data RX Buffer 58
    Uint16 ND59:1;                      // 27 New Data RX Buffer 59
    Uint16 ND60:1;                      // 28 New Data RX Buffer 60
    Uint16 ND61:1;                      // 29 New Data RX Buffer 61
    Uint16 ND62:1;                      // 30 New Data RX Buffer 62
    Uint16 ND63:1;                      // 31 New Data RX Buffer 63
};

union MCAN_NDAT2_REG {
    Uint32  all;
    struct  MCAN_NDAT2_BITS  bit;
};

struct MCAN_RXF0C_BITS {                // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 F0SA:14;                     // 15:2 Rx FIFO 0 Start Address
    Uint16 F0S:7;                       // 22:16 Rx FIFO 0 Size
    Uint16 rsvd2:1;                     // 23 Reserved
    Uint16 F0WM:7;                      // 30:24 Rx FIFO 0 Watermark
    Uint16 F0OM:1;                      // 31 FIFO 0 Operation Mode
};

union MCAN_RXF0C_REG {
    Uint32  all;
    struct  MCAN_RXF0C_BITS  bit;
};

struct MCAN_RXF0S_BITS {                // bits description
    Uint16 F0FL:7;                      // 6:0 Rx FIFO 0 Fill Level
    Uint16 rsvd1:1;                     // 7 Reserved
    Uint16 F0GI:6;                      // 13:8 Rx FIFO 0 Get Index
    Uint16 rsvd2:2;                     // 15:14 Reserved
    Uint16 F0PI:6;                      // 21:16 Rx FIFO 0 Put Index
    Uint16 rsvd3:2;                     // 23:22 Reserved
    Uint16 F0F:1;                       // 24 Rx FIFO 0 Full
    Uint16 RF0L:1;                      // 25 Rx FIFO 0 Message Lost
    Uint16 rsvd4:6;                     // 31:26 Reserved
};

union MCAN_RXF0S_REG {
    Uint32  all;
    struct  MCAN_RXF0S_BITS  bit;
};

struct MCAN_RXF0A_BITS {                // bits description
    Uint16 F0AI:6;                      // 5:0 Rx FIFO 0 Acknowledge Index
    Uint16 rsvd1:10;                    // 15:6 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_RXF0A_REG {
    Uint32  all;
    struct  MCAN_RXF0A_BITS  bit;
};

struct MCAN_RXBC_BITS {                 // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 RBSA:14;                     // 15:2 Rx Buffer Start Address
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_RXBC_REG {
    Uint32  all;
    struct  MCAN_RXBC_BITS  bit;
};

struct MCAN_RXF1C_BITS {                // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 F1SA:14;                     // 15:2 Rx FIFO 1 Start Address
    Uint16 F1S:7;                       // 22:16 Rx FIFO 1 Size
    Uint16 rsvd2:1;                     // 23 Reserved
    Uint16 F1WM:7;                      // 30:24 Rx FIFO 1 Watermark
    Uint16 F1OM:1;                      // 31 FIFO 1 Operation Mode
};

union MCAN_RXF1C_REG {
    Uint32  all;
    struct  MCAN_RXF1C_BITS  bit;
};

struct MCAN_RXF1S_BITS {                // bits description
    Uint16 F1FL:7;                      // 6:0 Rx FIFO 1 Fill Level
    Uint16 rsvd1:1;                     // 7 Reserved
    Uint16 F1GI:6;                      // 13:8 Rx FIFO 1 Get Index
    Uint16 rsvd2:2;                     // 15:14 Reserved
    Uint16 F1PI:6;                      // 21:16 Rx FIFO 1 Put Index
    Uint16 rsvd3:2;                     // 23:22 Reserved
    Uint16 F1F:1;                       // 24 Rx FIFO 1 Full
    Uint16 RF1L:1;                      // 25 Rx FIFO 1 Message Lost
    Uint16 rsvd4:4;                     // 29:26 Reserved
    Uint16 DMS:2;                       // 31:30 Debug Message Status
};

union MCAN_RXF1S_REG {
    Uint32  all;
    struct  MCAN_RXF1S_BITS  bit;
};

struct MCAN_RXF1A_BITS {                // bits description
    Uint16 F1AI:6;                      // 5:0 Rx FIFO 1 Acknowledge Index
    Uint16 rsvd1:10;                    // 15:6 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_RXF1A_REG {
    Uint32  all;
    struct  MCAN_RXF1A_BITS  bit;
};

struct MCAN_RXESC_BITS {                // bits description
    Uint16 F0DS:3;                      // 2:0 Rx FIFO 0 Data Field Size
    Uint16 rsvd1:1;                     // 3 Reserved
    Uint16 F1DS:3;                      // 6:4 Rx FIFO 1 Data Field Size
    Uint16 rsvd2:1;                     // 7 Reserved
    Uint16 RBDS:3;                      // 10:8 Rx Buffer Data Field Size
    Uint16 rsvd3:5;                     // 15:11 Reserved
    Uint16 rsvd4:16;                    // 31:16 Reserved
};

union MCAN_RXESC_REG {
    Uint32  all;
    struct  MCAN_RXESC_BITS  bit;
};

struct MCAN_TXBC_BITS {                 // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 TBSA:14;                     // 15:2 Tx Buffers Start Address
    Uint16 NDTB:6;                      // 21:16 Number of Dedicated Transmit Buffers
    Uint16 rsvd2:2;                     // 23:22 Reserved
    Uint16 TFQS:6;                      // 29:24 Transmit FIFO/Queue Size
    Uint16 TFQM:1;                      // 30 Tx FIFO/Queue Mode
    Uint16 rsvd3:1;                     // 31 Reserved
};

union MCAN_TXBC_REG {
    Uint32  all;
    struct  MCAN_TXBC_BITS  bit;
};

struct MCAN_TXFQS_BITS {                // bits description
    Uint16 TFFL:6;                      // 5:0 Tx FIFO Free Level
    Uint16 rsvd1:2;                     // 7:6 Reserved
    Uint16 TFGI:5;                      // 12:8 Tx FIFO Get Index
    Uint16 rsvd2:3;                     // 15:13 Reserved
    Uint16 TFQP:5;                      // 20:16 Tx FIFO/Queue Put Index
    Uint16 TFQF:1;                      // 21 Tx FIFO/Queue Full
    Uint16 rsvd3:10;                    // 31:22 Reserved
};

union MCAN_TXFQS_REG {
    Uint32  all;
    struct  MCAN_TXFQS_BITS  bit;
};

struct MCAN_TXESC_BITS {                // bits description
    Uint16 TBDS:3;                      // 2:0 Tx Buffer Data Field Size
    Uint16 rsvd1:13;                    // 15:3 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_TXESC_REG {
    Uint32  all;
    struct  MCAN_TXESC_BITS  bit;
};

struct MCAN_TXBRP_BITS {                // bits description
    Uint16 TRP0:1;                      // 0 Transmission Request Pending 0
    Uint16 TRP1:1;                      // 1 Transmission Request Pending 1
    Uint16 TRP2:1;                      // 2 Transmission Request Pending 2
    Uint16 TRP3:1;                      // 3 Transmission Request Pending 3
    Uint16 TRP4:1;                      // 4 Transmission Request Pending 4
    Uint16 TRP5:1;                      // 5 Transmission Request Pending 5
    Uint16 TRP6:1;                      // 6 Transmission Request Pending 6
    Uint16 TRP7:1;                      // 7 Transmission Request Pending 7
    Uint16 TRP8:1;                      // 8 Transmission Request Pending 8
    Uint16 TRP9:1;                      // 9 Transmission Request Pending 9
    Uint16 TRP10:1;                     // 10 Transmission Request Pending 10
    Uint16 TRP11:1;                     // 11 Transmission Request Pending 11
    Uint16 TRP12:1;                     // 12 Transmission Request Pending 12
    Uint16 TRP13:1;                     // 13 Transmission Request Pending 13
    Uint16 TRP14:1;                     // 14 Transmission Request Pending 14
    Uint16 TRP15:1;                     // 15 Transmission Request Pending 15
    Uint16 TRP16:1;                     // 16 Transmission Request Pending 16
    Uint16 TRP17:1;                     // 17 Transmission Request Pending 17
    Uint16 TRP18:1;                     // 18 Transmission Request Pending 18
    Uint16 TRP19:1;                     // 19 Transmission Request Pending 19
    Uint16 TRP20:1;                     // 20 Transmission Request Pending 20
    Uint16 TRP21:1;                     // 21 Transmission Request Pending 21
    Uint16 TRP22:1;                     // 22 Transmission Request Pending 22
    Uint16 TRP23:1;                     // 23 Transmission Request Pending 23
    Uint16 TRP24:1;                     // 24 Transmission Request Pending 24
    Uint16 TRP25:1;                     // 25 Transmission Request Pending 25
    Uint16 TRP26:1;                     // 26 Transmission Request Pending 26
    Uint16 TRP27:1;                     // 27 Transmission Request Pending 27
    Uint16 TRP28:1;                     // 28 Transmission Request Pending 28
    Uint16 TRP29:1;                     // 29 Transmission Request Pending 29
    Uint16 TRP30:1;                     // 30 Transmission Request Pending 30
    Uint16 TRP31:1;                     // 31 Transmission Request Pending 31
};

union MCAN_TXBRP_REG {
    Uint32  all;
    struct  MCAN_TXBRP_BITS  bit;
};

struct MCAN_TXBAR_BITS {                // bits description
    Uint16 _AR0:1;                       // 0 Add Request 0
    Uint16 _AR1:1;                       // 1 Add Request 1
    Uint16 _AR2:1;                       // 2 Add Request 2
    Uint16 _AR3:1;                       // 3 Add Request 3
    Uint16 _AR4:1;                       // 4 Add Request 4
    Uint16 _AR5:1;                       // 5 Add Request 5
    Uint16 _AR6:1;                       // 6 Add Request 6
    Uint16 _AR7:1;                       // 7 Add Request 7
    Uint16 _AR8:1;                       // 8 Add Request 8
    Uint16 _AR9:1;                       // 9 Add Request 9
    Uint16 _AR10:1;                      // 10 Add Request 10
    Uint16 _AR11:1;                      // 11 Add Request 11
    Uint16 _AR12:1;                      // 12 Add Request 12
    Uint16 _AR13:1;                      // 13 Add Request 13
    Uint16 _AR14:1;                      // 14 Add Request 14
    Uint16 _AR15:1;                      // 15 Add Request 15
    Uint16 _AR16:1;                      // 16 Add Request 16
    Uint16 _AR17:1;                      // 17 Add Request 17
    Uint16 _AR18:1;                      // 18 Add Request 18
    Uint16 _AR19:1;                      // 19 Add Request 19
    Uint16 _AR20:1;                      // 20 Add Request 20
    Uint16 _AR21:1;                      // 21 Add Request 21
    Uint16 _AR22:1;                      // 22 Add Request 22
    Uint16 _AR23:1;                      // 23 Add Request 23
    Uint16 _AR24:1;                      // 24 Add Request 24
    Uint16 _AR25:1;                      // 25 Add Request 25
    Uint16 _AR26:1;                      // 26 Add Request 26
    Uint16 _AR27:1;                      // 27 Add Request 27
    Uint16 _AR28:1;                      // 28 Add Request 28
    Uint16 _AR29:1;                      // 29 Add Request 29
    Uint16 _AR30:1;                      // 30 Add Request 30
    Uint16 _AR31:1;                      // 31 Add Request 31
};

union MCAN_TXBAR_REG {
    Uint32  all;
    struct  MCAN_TXBAR_BITS  bit;
};

struct MCAN_TXBCR_BITS {                // bits description
    Uint16 CR0:1;                       // 0 Cancellation Request 0
    Uint16 CR1:1;                       // 1 Cancellation Request 1
    Uint16 CR2:1;                       // 2 Cancellation Request 2
    Uint16 CR3:1;                       // 3 Cancellation Request 3
    Uint16 CR4:1;                       // 4 Cancellation Request 4
    Uint16 CR5:1;                       // 5 Cancellation Request 5
    Uint16 CR6:1;                       // 6 Cancellation Request 6
    Uint16 CR7:1;                       // 7 Cancellation Request 7
    Uint16 CR8:1;                       // 8 Cancellation Request 8
    Uint16 CR9:1;                       // 9 Cancellation Request 9
    Uint16 CR10:1;                      // 10 Cancellation Request 10
    Uint16 CR11:1;                      // 11 Cancellation Request 11
    Uint16 CR12:1;                      // 12 Cancellation Request 12
    Uint16 CR13:1;                      // 13 Cancellation Request 13
    Uint16 CR14:1;                      // 14 Cancellation Request 14
    Uint16 CR15:1;                      // 15 Cancellation Request 15
    Uint16 CR16:1;                      // 16 Cancellation Request 16
    Uint16 CR17:1;                      // 17 Cancellation Request 17
    Uint16 CR18:1;                      // 18 Cancellation Request 18
    Uint16 CR19:1;                      // 19 Cancellation Request 19
    Uint16 CR20:1;                      // 20 Cancellation Request 20
    Uint16 CR21:1;                      // 21 Cancellation Request 21
    Uint16 CR22:1;                      // 22 Cancellation Request 22
    Uint16 CR23:1;                      // 23 Cancellation Request 23
    Uint16 CR24:1;                      // 24 Cancellation Request 24
    Uint16 CR25:1;                      // 25 Cancellation Request 25
    Uint16 CR26:1;                      // 26 Cancellation Request 26
    Uint16 CR27:1;                      // 27 Cancellation Request 27
    Uint16 CR28:1;                      // 28 Cancellation Request 28
    Uint16 CR29:1;                      // 29 Cancellation Request 29
    Uint16 CR30:1;                      // 30 Cancellation Request 30
    Uint16 CR31:1;                      // 31 Cancellation Request 31
};

union MCAN_TXBCR_REG {
    Uint32  all;
    struct  MCAN_TXBCR_BITS  bit;
};

struct MCAN_TXBTO_BITS {                // bits description
    Uint16 TO0:1;                       // 0 Transmission Occurred 0
    Uint16 TO1:1;                       // 1 Transmission Occurred 1
    Uint16 TO2:1;                       // 2 Transmission Occurred 2
    Uint16 TO3:1;                       // 3 Transmission Occurred 3
    Uint16 TO4:1;                       // 4 Transmission Occurred 4
    Uint16 TO5:1;                       // 5 Transmission Occurred 5
    Uint16 TO6:1;                       // 6 Transmission Occurred 6
    Uint16 TO7:1;                       // 7 Transmission Occurred 7
    Uint16 TO8:1;                       // 8 Transmission Occurred 8
    Uint16 TO9:1;                       // 9 Transmission Occurred 9
    Uint16 TO10:1;                      // 10 Transmission Occurred 10
    Uint16 TO11:1;                      // 11 Transmission Occurred 11
    Uint16 TO12:1;                      // 12 Transmission Occurred 12
    Uint16 TO13:1;                      // 13 Transmission Occurred 13
    Uint16 TO14:1;                      // 14 Transmission Occurred 14
    Uint16 TO15:1;                      // 15 Transmission Occurred 15
    Uint16 TO16:1;                      // 16 Transmission Occurred 16
    Uint16 TO17:1;                      // 17 Transmission Occurred 17
    Uint16 TO18:1;                      // 18 Transmission Occurred 18
    Uint16 TO19:1;                      // 19 Transmission Occurred 19
    Uint16 TO20:1;                      // 20 Transmission Occurred 20
    Uint16 TO21:1;                      // 21 Transmission Occurred 21
    Uint16 TO22:1;                      // 22 Transmission Occurred 22
    Uint16 TO23:1;                      // 23 Transmission Occurred 23
    Uint16 TO24:1;                      // 24 Transmission Occurred 24
    Uint16 TO25:1;                      // 25 Transmission Occurred 25
    Uint16 TO26:1;                      // 26 Transmission Occurred 26
    Uint16 TO27:1;                      // 27 Transmission Occurred 27
    Uint16 TO28:1;                      // 28 Transmission Occurred 28
    Uint16 TO29:1;                      // 29 Transmission Occurred 29
    Uint16 TO30:1;                      // 30 Transmission Occurred 30
    Uint16 TO31:1;                      // 31 Transmission Occurred 31
};

union MCAN_TXBTO_REG {
    Uint32  all;
    struct  MCAN_TXBTO_BITS  bit;
};

struct MCAN_TXBCF_BITS {                // bits description
    Uint16 CF0:1;                       // 0 Cancellation Finished 0
    Uint16 CF1:1;                       // 1 Cancellation Finished 1
    Uint16 CF2:1;                       // 2 Cancellation Finished 2
    Uint16 CF3:1;                       // 3 Cancellation Finished 3
    Uint16 CF4:1;                       // 4 Cancellation Finished 4
    Uint16 CF5:1;                       // 5 Cancellation Finished 5
    Uint16 CF6:1;                       // 6 Cancellation Finished 6
    Uint16 CF7:1;                       // 7 Cancellation Finished 7
    Uint16 CF8:1;                       // 8 Cancellation Finished 8
    Uint16 CF9:1;                       // 9 Cancellation Finished 9
    Uint16 CF10:1;                      // 10 Cancellation Finished 10
    Uint16 CF11:1;                      // 11 Cancellation Finished 11
    Uint16 CF12:1;                      // 12 Cancellation Finished 12
    Uint16 CF13:1;                      // 13 Cancellation Finished 13
    Uint16 CF14:1;                      // 14 Cancellation Finished 14
    Uint16 CF15:1;                      // 15 Cancellation Finished 15
    Uint16 CF16:1;                      // 16 Cancellation Finished 16
    Uint16 CF17:1;                      // 17 Cancellation Finished 17
    Uint16 CF18:1;                      // 18 Cancellation Finished 18
    Uint16 CF19:1;                      // 19 Cancellation Finished 19
    Uint16 CF20:1;                      // 20 Cancellation Finished 20
    Uint16 CF21:1;                      // 21 Cancellation Finished 21
    Uint16 CF22:1;                      // 22 Cancellation Finished 22
    Uint16 CF23:1;                      // 23 Cancellation Finished 23
    Uint16 CF24:1;                      // 24 Cancellation Finished 24
    Uint16 CF25:1;                      // 25 Cancellation Finished 25
    Uint16 CF26:1;                      // 26 Cancellation Finished 26
    Uint16 CF27:1;                      // 27 Cancellation Finished 27
    Uint16 CF28:1;                      // 28 Cancellation Finished 28
    Uint16 CF29:1;                      // 29 Cancellation Finished 29
    Uint16 CF30:1;                      // 30 Cancellation Finished 30
    Uint16 CF31:1;                      // 31 Cancellation Finished 31
};

union MCAN_TXBCF_REG {
    Uint32  all;
    struct  MCAN_TXBCF_BITS  bit;
};

struct MCAN_TXBTIE_BITS {               // bits description
    Uint16 TIE0:1;                      // 0 Transmission Interrupt Enable 0
    Uint16 TIE1:1;                      // 1 Transmission Interrupt Enable 1
    Uint16 TIE2:1;                      // 2 Transmission Interrupt Enable 2
    Uint16 TIE3:1;                      // 3 Transmission Interrupt Enable 3
    Uint16 TIE4:1;                      // 4 Transmission Interrupt Enable 4
    Uint16 TIE5:1;                      // 5 Transmission Interrupt Enable 5
    Uint16 TIE6:1;                      // 6 Transmission Interrupt Enable 6
    Uint16 TIE7:1;                      // 7 Transmission Interrupt Enable 7
    Uint16 TIE8:1;                      // 8 Transmission Interrupt Enable 8
    Uint16 TIE9:1;                      // 9 Transmission Interrupt Enable 9
    Uint16 TIE10:1;                     // 10 Transmission Interrupt Enable 10
    Uint16 TIE11:1;                     // 11 Transmission Interrupt Enable 11
    Uint16 TIE12:1;                     // 12 Transmission Interrupt Enable 12
    Uint16 TIE13:1;                     // 13 Transmission Interrupt Enable 13
    Uint16 TIE14:1;                     // 14 Transmission Interrupt Enable 14
    Uint16 TIE15:1;                     // 15 Transmission Interrupt Enable 15
    Uint16 TIE16:1;                     // 16 Transmission Interrupt Enable 16
    Uint16 TIE17:1;                     // 17 Transmission Interrupt Enable 17
    Uint16 TIE18:1;                     // 18 Transmission Interrupt Enable 18
    Uint16 TIE19:1;                     // 19 Transmission Interrupt Enable 19
    Uint16 TIE20:1;                     // 20 Transmission Interrupt Enable 20
    Uint16 TIE21:1;                     // 21 Transmission Interrupt Enable 21
    Uint16 TIE22:1;                     // 22 Transmission Interrupt Enable 22
    Uint16 TIE23:1;                     // 23 Transmission Interrupt Enable 23
    Uint16 TIE24:1;                     // 24 Transmission Interrupt Enable 24
    Uint16 TIE25:1;                     // 25 Transmission Interrupt Enable 25
    Uint16 TIE26:1;                     // 26 Transmission Interrupt Enable 26
    Uint16 TIE27:1;                     // 27 Transmission Interrupt Enable 27
    Uint16 TIE28:1;                     // 28 Transmission Interrupt Enable 28
    Uint16 TIE29:1;                     // 29 Transmission Interrupt Enable 29
    Uint16 TIE30:1;                     // 30 Transmission Interrupt Enable 30
    Uint16 TIE31:1;                     // 31 Transmission Interrupt Enable 31
};

union MCAN_TXBTIE_REG {
    Uint32  all;
    struct  MCAN_TXBTIE_BITS  bit;
};

struct MCAN_TXBCIE_BITS {               // bits description
    Uint16 CFIE0:1;                     // 0 Cancellation Finished Interrupt Enable 0
    Uint16 CFIE1:1;                     // 1 Cancellation Finished Interrupt Enable 1
    Uint16 CFIE2:1;                     // 2 Cancellation Finished Interrupt Enable 2
    Uint16 CFIE3:1;                     // 3 Cancellation Finished Interrupt Enable 3
    Uint16 CFIE4:1;                     // 4 Cancellation Finished Interrupt Enable 4
    Uint16 CFIE5:1;                     // 5 Cancellation Finished Interrupt Enable 5
    Uint16 CFIE6:1;                     // 6 Cancellation Finished Interrupt Enable 6
    Uint16 CFIE7:1;                     // 7 Cancellation Finished Interrupt Enable 7
    Uint16 CFIE8:1;                     // 8 Cancellation Finished Interrupt Enable 8
    Uint16 CFIE9:1;                     // 9 Cancellation Finished Interrupt Enable 9
    Uint16 CFIE10:1;                    // 10 Cancellation Finished Interrupt Enable 10
    Uint16 CFIE11:1;                    // 11 Cancellation Finished Interrupt Enable 11
    Uint16 CFIE12:1;                    // 12 Cancellation Finished Interrupt Enable 12
    Uint16 CFIE13:1;                    // 13 Cancellation Finished Interrupt Enable 13
    Uint16 CFIE14:1;                    // 14 Cancellation Finished Interrupt Enable 14
    Uint16 CFIE15:1;                    // 15 Cancellation Finished Interrupt Enable 15
    Uint16 CFIE16:1;                    // 16 Cancellation Finished Interrupt Enable 16
    Uint16 CFIE17:1;                    // 17 Cancellation Finished Interrupt Enable 17
    Uint16 CFIE18:1;                    // 18 Cancellation Finished Interrupt Enable 18
    Uint16 CFIE19:1;                    // 19 Cancellation Finished Interrupt Enable 19
    Uint16 CFIE20:1;                    // 20 Cancellation Finished Interrupt Enable 20
    Uint16 CFIE21:1;                    // 21 Cancellation Finished Interrupt Enable 21
    Uint16 CFIE22:1;                    // 22 Cancellation Finished Interrupt Enable 22
    Uint16 CFIE23:1;                    // 23 Cancellation Finished Interrupt Enable 23
    Uint16 CFIE24:1;                    // 24 Cancellation Finished Interrupt Enable 24
    Uint16 CFIE25:1;                    // 25 Cancellation Finished Interrupt Enable 25
    Uint16 CFIE26:1;                    // 26 Cancellation Finished Interrupt Enable 26
    Uint16 CFIE27:1;                    // 27 Cancellation Finished Interrupt Enable 27
    Uint16 CFIE28:1;                    // 28 Cancellation Finished Interrupt Enable 28
    Uint16 CFIE29:1;                    // 29 Cancellation Finished Interrupt Enable 29
    Uint16 CFIE30:1;                    // 30 Cancellation Finished Interrupt Enable 30
    Uint16 CFIE31:1;                    // 31 Cancellation Finished Interrupt Enable 31
};

union MCAN_TXBCIE_REG {
    Uint32  all;
    struct  MCAN_TXBCIE_BITS  bit;
};

struct MCAN_TXEFC_BITS {                // bits description
    Uint16 rsvd1:2;                     // 1:0 Reserved
    Uint16 EFSA:14;                     // 15:2 Event FIFO Start Address
    Uint16 EFS:6;                       // 21:16 Event FIFO Size
    Uint16 rsvd2:2;                     // 23:22 Reserved
    Uint16 EFWM:6;                      // 29:24 Event FIFO Watermark
    Uint16 rsvd3:2;                     // 31:30 Reserved
};

union MCAN_TXEFC_REG {
    Uint32  all;
    struct  MCAN_TXEFC_BITS  bit;
};

struct MCAN_TXEFS_BITS {                // bits description
    Uint16 EFFL:6;                      // 5:0 Event FIFO Fill Level
    Uint16 rsvd1:2;                     // 7:6 Reserved
    Uint16 EFGI:5;                      // 12:8 Event FIFO Get Index
    Uint16 rsvd2:3;                     // 15:13 Reserved
    Uint16 EFPI:5;                      // 20:16 Event FIFO Put Index
    Uint16 rsvd3:3;                     // 23:21 Reserved
    Uint16 EFF:1;                       // 24 Event FIFO Full
    Uint16 TEFL:1;                      // 25 Tx Event FIFO Element Lost
    Uint16 rsvd4:6;                     // 31:26 Reserved
};

union MCAN_TXEFS_REG {
    Uint32  all;
    struct  MCAN_TXEFS_BITS  bit;
};

struct MCAN_TXEFA_BITS {                // bits description
    Uint16 EFAI:5;                      // 4:0 Event FIFO Acknowledge Index
    Uint16 rsvd1:11;                    // 15:5 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCAN_TXEFA_REG {
    Uint32  all;
    struct  MCAN_TXEFA_BITS  bit;
};

struct MCAN_REGS {
    union   MCAN_CREL_REG                    MCAN_CREL;                    // MCAN Core Release Register
    Uint32                                   MCAN_ENDN;                    // MCAN Endian Register
    Uint16                                   rsvd1[2];                     // Reserved
    union   MCAN_DBTP_REG                    MCAN_DBTP;                    // MCAN Data Bit Timing and Prescaler Register
    union   MCAN_TEST_REG                    MCAN_TEST;                    // MCAN Test Register
    union   MCAN_RWD_REG                     MCAN_RWD;                     // MCAN RAM Watchdog
    union   MCAN_CCCR_REG                    MCAN_CCCR;                    // MCAN CC Control Register
    union   MCAN_NBTP_REG                    MCAN_NBTP;                    // MCAN Nominal Bit Timing and Prescaler Register
    union   MCAN_TSCC_REG                    MCAN_TSCC;                    // MCAN Timestamp Counter Configuration
    union   MCAN_TSCV_REG                    MCAN_TSCV;                    // MCAN Timestamp Counter Value
    union   MCAN_TOCC_REG                    MCAN_TOCC;                    // MCAN Timeout Counter Configuration
    union   MCAN_TOCV_REG                    MCAN_TOCV;                    // MCAN Timeout Counter Value
    Uint16                                   rsvd2[8];                     // Reserved
    union   MCAN_ECR_REG                     MCAN_ECR;                     // MCAN Error Counter Register
    union   MCAN_PSR_REG                     MCAN_PSR;                     // MCAN Protocol Status Register
    union   MCAN_TDCR_REG                    MCAN_TDCR;                    // MCAN Transmitter Delay Compensation Register
    Uint16                                   rsvd3[2];                     // Reserved
    union   MCAN_IR_REG                      MCAN_IR;                      // MCAN Interrupt Register
    union   MCAN_IE_REG                      MCAN_IE;                      // MCAN Interrupt Enable
    union   MCAN_ILS_REG                     MCAN_ILS;                     // MCAN Interrupt Line Select
    union   MCAN_ILE_REG                     MCAN_ILE;                     // MCAN Interrupt Line Enable
    Uint16                                   rsvd4[16];                    // Reserved
    union   MCAN_GFC_REG                     MCAN_GFC;                     // MCAN Global Filter Configuration
    union   MCAN_SIDFC_REG                   MCAN_SIDFC;                   // MCAN Standard ID Filter Configuration
    union   MCAN_XIDFC_REG                   MCAN_XIDFC;                   // MCAN Extended ID Filter Configuration
    Uint16                                   rsvd5[2];                     // Reserved
    union   MCAN_XIDAM_REG                   MCAN_XIDAM;                   // MCAN Extended ID and Mask
    union   MCAN_HPMS_REG                    MCAN_HPMS;                    // MCAN High Priority Message Status
    union   MCAN_NDAT1_REG                   MCAN_NDAT1;                   // MCAN New Data 1
    union   MCAN_NDAT2_REG                   MCAN_NDAT2;                   // MCAN New Data 2
    union   MCAN_RXF0C_REG                   MCAN_RXF0C;                   // MCAN Rx FIFO 0 Configuration
    union   MCAN_RXF0S_REG                   MCAN_RXF0S;                   // MCAN Rx FIFO 0 Status
    union   MCAN_RXF0A_REG                   MCAN_RXF0A;                   // MCAN Rx FIFO 0 Acknowledge
    union   MCAN_RXBC_REG                    MCAN_RXBC;                    // MCAN Rx Buffer Configuration
    union   MCAN_RXF1C_REG                   MCAN_RXF1C;                   // MCAN Rx FIFO 1 Configuration
    union   MCAN_RXF1S_REG                   MCAN_RXF1S;                   // MCAN Rx FIFO 1 Status
    union   MCAN_RXF1A_REG                   MCAN_RXF1A;                   // MCAN Rx FIFO 1 Acknowledge
    union   MCAN_RXESC_REG                   MCAN_RXESC;                   // MCAN Rx Buffer / FIFO Element Size Configuration
    union   MCAN_TXBC_REG                    MCAN_TXBC;                    // MCAN Tx Buffer Configuration
    union   MCAN_TXFQS_REG                   MCAN_TXFQS;                   // MCAN Tx FIFO / Queue Status
    union   MCAN_TXESC_REG                   MCAN_TXESC;                   // MCAN Tx Buffer Element Size Configuration
    union   MCAN_TXBRP_REG                   MCAN_TXBRP;                   // MCAN Tx Buffer Request Pending
    union   MCAN_TXBAR_REG                   MCAN_TXBAR;                   // MCAN Tx Buffer Add Request
    union   MCAN_TXBCR_REG                   MCAN_TXBCR;                   // MCAN Tx Buffer Cancellation Request
    union   MCAN_TXBTO_REG                   MCAN_TXBTO;                   // MCAN Tx Buffer Transmission Occurred
    union   MCAN_TXBCF_REG                   MCAN_TXBCF;                   // MCAN Tx Buffer Cancellation Finished
    union   MCAN_TXBTIE_REG                  MCAN_TXBTIE;                  // MCAN Tx Buffer Transmission Interrupt Enable
    union   MCAN_TXBCIE_REG                  MCAN_TXBCIE;                  // MCAN Tx Buffer Cancellation Finished Interrupt Enable
    Uint16                                   rsvd6[4];                     // Reserved
    union   MCAN_TXEFC_REG                   MCAN_TXEFC;                   // MCAN Tx Event FIFO Configuration
    union   MCAN_TXEFS_REG                   MCAN_TXEFS;                   // MCAN Tx Event FIFO Status
    union   MCAN_TXEFA_REG                   MCAN_TXEFA;                   // MCAN Tx Event FIFO Acknowledge
};

struct MCANERR_REV_BITS {               // bits description
    Uint16 REVMIN:6;                    // 5:0 Minor Revision
    Uint16 rsvd1:2;                     // 7:6 Reserved
    Uint16 REVMAJ:3;                    // 10:8 Major Revision
    Uint16 rsvd2:5;                     // 15:11 Reserved
    Uint16 MODULE_ID:12;                // 27:16 Module Identification Number
    Uint16 rsvd3:2;                     // 29:28 Reserved
    Uint16 SCHEME:2;                    // 31:30 PID Register Scheme
};

union MCANERR_REV_REG {
    Uint32  all;
    struct  MCANERR_REV_BITS  bit;
};

struct MCANERR_VECTOR_BITS {            // bits description
    Uint16 ECC_VECTOR:11;               // 10:0 ECC RAM ID
    Uint16 rsvd1:4;                     // 14:11 Reserved
    Uint16 RD_SVBUS:1;                  // 15 Read Trigger
    Uint16 RD_SVBUS_ADDRESS:8;          // 23:16 Read Address Offset
    Uint16 RD_SVBUS_DONE:1;             // 24 Read Completion Flag
    Uint16 rsvd2:7;                     // 31:25 Reserved
};

union MCANERR_VECTOR_REG {
    Uint32  all;
    struct  MCANERR_VECTOR_BITS  bit;
};

struct MCANERR_STAT_BITS {              // bits description
    Uint16 NUM_RAMS:11;                 // 10:0 Number of RAMs
    Uint16 rsvd1:5;                     // 15:11 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_STAT_REG {
    Uint32  all;
    struct  MCANERR_STAT_BITS  bit;
};

struct MCANERR_WRAP_REV_BITS {          // bits description
    Uint16 REVMIN:6;                    // 5:0 Minor Revision
    Uint16 rsvd1:2;                     // 7:6 Reserved
    Uint16 REVMAJ:3;                    // 10:8 Major Revision
    Uint16 rsvd2:5;                     // 15:11 Reserved
    Uint16 MODULE_ID:12;                // 27:16 Module Identification Number
    Uint16 rsvd3:2;                     // 29:28 Reserved
    Uint16 SCHEME:2;                    // 31:30 PID Register Scheme
};

union MCANERR_WRAP_REV_REG {
    Uint32  all;
    struct  MCANERR_WRAP_REV_BITS  bit;
};

struct MCANERR_CTRL_BITS {              // bits description
    Uint16 ECC_ENABLE:1;                // 0 Enable ECC Generation
    Uint16 ECC_CHECK:1;                 // 1 Enable ECC Check
    Uint16 ENABLE_RMW:1;                // 2 Enable Read-Modify-Write
    Uint16 FORCE_SEC:1;                 // 3 Force Single Bit Error Corrected Error
    Uint16 FORCE_DED:1;                 // 4 Force Double Bit Error Detected Error
    Uint16 FORCE_N_ROW:1;               // 5 Force Next Single/Double Bit Error
    Uint16 ERROR_ONCE:1;                // 6 Force Error Only Once Enable
    Uint16 rsvd1:1;                     // 7 Reserved
    Uint16 CHECK_SVBUS_TIMEOUT:1;       // 8 SVBUS Timeout Enable
    Uint16 rsvd2:7;                     // 15:9 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_CTRL_REG {
    Uint32  all;
    struct  MCANERR_CTRL_BITS  bit;
};

struct MCANERR_ERR_CTRL2_BITS {         // bits description
    Uint16 ECC_BIT1:16;                 // 15:0 Force Error Bit1 Column Index
    Uint16 ECC_BIT2:16;                 // 31:16 Force Error Bit2 Column Index
};

union MCANERR_ERR_CTRL2_REG {
    Uint32  all;
    struct  MCANERR_ERR_CTRL2_BITS  bit;
};

struct MCANERR_ERR_STAT1_BITS {         // bits description
    Uint16 ECC_SEC:2;                   // 1:0 Single Bit Error Corrected Status
    Uint16 ECC_DED:2;                   // 3:2 Double Bit Error Detected Status
    Uint16 ECC_OTHER:1;                 // 4 SEC While Writeback Error Status
    Uint16 rsvd1:2;                     // 6:5 Reserved
    Uint16 CTRL_REG_ERROR:1;            // 7 Control Register Error
    Uint16 CLR_ECC_SEC:2;               // 9:8 Clear ECC_SEC
    Uint16 CLR_ECC_DED:2;               // 11:10 Clear ECC_DED
    Uint16 CLR_ECC_OTHER:1;             // 12 Clear ECC_OTHER
    Uint16 rsvd2:2;                     // 14:13 Reserved
    Uint16 CLR_CTRL_REG_ERROR:1;        // 15 Clear Control Register Error
    Uint16 ECC_BIT1:16;                 // 31:16 ECC Error Bit Position
};

union MCANERR_ERR_STAT1_REG {
    Uint32  all;
    struct  MCANERR_ERR_STAT1_BITS  bit;
};

struct MCANERR_ERR_STAT3_BITS {         // bits description
    Uint16 WB_PEND:1;                   // 0 Delayed Write Back Pending Status
    Uint16 SVBUS_TIMEOUT:1;             // 1 Serial VBUS Timeout Flag
    Uint16 rsvd1:7;                     // 8:2 Reserved
    Uint16 CLR_SVBUS_TIMEOUT:1;         // 9 Clear Serial VBUS Timeout
    Uint16 rsvd2:6;                     // 15:10 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_ERR_STAT3_REG {
    Uint32  all;
    struct  MCANERR_ERR_STAT3_BITS  bit;
};

struct MCANERR_SEC_EOI_BITS {           // bits description
    Uint16 EOI_WR:1;                    // 0 End of Interrupt
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_SEC_EOI_REG {
    Uint32  all;
    struct  MCANERR_SEC_EOI_BITS  bit;
};

struct MCANERR_SEC_STATUS_BITS {        // bits description
    Uint16 MSGMEM_PEND:1;               // 0 Message RAM SEC Interrupt Pending
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_SEC_STATUS_REG {
    Uint32  all;
    struct  MCANERR_SEC_STATUS_BITS  bit;
};

struct MCANERR_SEC_ENABLE_SET_BITS {    // bits description
    Uint16 MSGMEM_ENABLE_SET:1;         // 0 Message RAM SEC Interrupt Pending Enable Set
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_SEC_ENABLE_SET_REG {
    Uint32  all;
    struct  MCANERR_SEC_ENABLE_SET_BITS  bit;
};

struct MCANERR_SEC_ENABLE_CLR_BITS {    // bits description
    Uint16 MSGMEM_ENABLE_CLR:1;         // 0 Message RAM SEC Interrupt Pending Enable Clear
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_SEC_ENABLE_CLR_REG {
    Uint32  all;
    struct  MCANERR_SEC_ENABLE_CLR_BITS  bit;
};

struct MCANERR_DED_EOI_BITS {           // bits description
    Uint16 EOI_WR:1;                    // 0 End of Interrupt
    Uint16 rsvd1:15;                    // 15:1 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_DED_EOI_REG {
    Uint32  all;
    struct  MCANERR_DED_EOI_BITS  bit;
};

struct MCANERR_DED_STATUS_BITS {        // bits description
    Uint16 MSGMEM_PEND:1;               // 0 Message RAM DED Interrupt Pending
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_DED_STATUS_REG {
    Uint32  all;
    struct  MCANERR_DED_STATUS_BITS  bit;
};

struct MCANERR_DED_ENABLE_SET_BITS {    // bits description
    Uint16 MSGMEM_ENABLE_SET:1;         // 0 Message RAM DED Interrupt Pending Enable Set
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_DED_ENABLE_SET_REG {
    Uint32  all;
    struct  MCANERR_DED_ENABLE_SET_BITS  bit;
};

struct MCANERR_DED_ENABLE_CLR_BITS {    // bits description
    Uint16 MSGMEM_ENABLE_CLR:1;         // 0 Message RAM DED Interrupt Pending Enable Clear
    Uint16 rsvd1:1;                     // 1 Reserved
    Uint16 rsvd2:14;                    // 15:2 Reserved
    Uint16 rsvd3:16;                    // 31:16 Reserved
};

union MCANERR_DED_ENABLE_CLR_REG {
    Uint32  all;
    struct  MCANERR_DED_ENABLE_CLR_BITS  bit;
};

struct MCANERR_AGGR_ENABLE_SET_BITS {   // bits description
    Uint16 ENABLE_PARITY_SET:1;         // 0 Enable Parity Errors Set
    Uint16 ENABLE_TIMEOUT_SET:1;        // 1 Enable Timeout Errors Set
    Uint16 rsvd1:14;                    // 15:2 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_AGGR_ENABLE_SET_REG {
    Uint32  all;
    struct  MCANERR_AGGR_ENABLE_SET_BITS  bit;
};

struct MCANERR_AGGR_ENABLE_CLR_BITS {   // bits description
    Uint16 ENABLE_PARITY_CLR:1;         // 0 Enable Parity Errors Clear
    Uint16 ENABLE_TIMEOUT_CLR:1;        // 1 Enable Timeout Errors Clear
    Uint16 rsvd1:14;                    // 15:2 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_AGGR_ENABLE_CLR_REG {
    Uint32  all;
    struct  MCANERR_AGGR_ENABLE_CLR_BITS  bit;
};

struct MCANERR_AGGR_STATUS_SET_BITS {   // bits description
    Uint16 AGGR_PARITY_ERR:2;           // 1:0 Aggregator Parity Error Status
    Uint16 SVBUS_TIMEOUT:2;             // 3:2 Aggregator Serial VBUS Timeout Error Status
    Uint16 rsvd1:12;                    // 15:4 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_AGGR_STATUS_SET_REG {
    Uint32  all;
    struct  MCANERR_AGGR_STATUS_SET_BITS  bit;
};

struct MCANERR_AGGR_STATUS_CLR_BITS {   // bits description
    Uint16 AGGR_PARITY_ERR:2;           // 1:0 Aggregator Parity Error Status
    Uint16 SVBUS_TIMEOUT:2;             // 3:2 Aggregator Serial VBUS Timeout Error Status
    Uint16 rsvd1:12;                    // 15:4 Reserved
    Uint16 rsvd2:16;                    // 31:16 Reserved
};

union MCANERR_AGGR_STATUS_CLR_REG {
    Uint32  all;
    struct  MCANERR_AGGR_STATUS_CLR_BITS  bit;
};

struct MCAN_ERROR_REGS {
    union   MCANERR_REV_REG                  MCANERR_REV;                  // MCAN Error Aggregator Revision Register
    Uint16                                   rsvd1[2];                     // Reserved
    union   MCANERR_VECTOR_REG               MCANERR_VECTOR;               // MCAN ECC Vector Register
    union   MCANERR_STAT_REG                 MCANERR_STAT;                 // MCAN Error Misc Status
    union   MCANERR_WRAP_REV_REG             MCANERR_WRAP_REV;             // MCAN ECC Wrapper Revision Register
    union   MCANERR_CTRL_REG                 MCANERR_CTRL;                 // MCAN ECC Control
    Uint32                                   MCANERR_ERR_CTRL1;            // MCAN ECC Error Control 1 Register
    union   MCANERR_ERR_CTRL2_REG            MCANERR_ERR_CTRL2;            // MCAN ECC Error Control 2 Register
    union   MCANERR_ERR_STAT1_REG            MCANERR_ERR_STAT1;            // MCAN ECC Error Status 1 Register
    Uint32                                   MCANERR_ERR_STAT2;            // MCAN ECC Error Status 2 Register
    union   MCANERR_ERR_STAT3_REG            MCANERR_ERR_STAT3;            // MCAN ECC Error Status 3 Register
    Uint16                                   rsvd2[8];                     // Reserved
    union   MCANERR_SEC_EOI_REG              MCANERR_SEC_EOI;              // MCAN Single Error Corrected End of Interrupt Register
    union   MCANERR_SEC_STATUS_REG           MCANERR_SEC_STATUS;           // MCAN Single Error Corrected Interrupt Status Register
    Uint16                                   rsvd3[30];                    // Reserved
    union   MCANERR_SEC_ENABLE_SET_REG       MCANERR_SEC_ENABLE_SET;       // MCAN Single Error Corrected Interrupt Enable Set Register
    Uint16                                   rsvd4[30];                    // Reserved
    union   MCANERR_SEC_ENABLE_CLR_REG       MCANERR_SEC_ENABLE_CLR;       // MCAN Single Error Corrected Interrupt Enable Clear Register
    Uint16                                   rsvd5[60];                    // Reserved
    union   MCANERR_DED_EOI_REG              MCANERR_DED_EOI;              // MCAN Double Error Detected End of Interrupt Register
    union   MCANERR_DED_STATUS_REG           MCANERR_DED_STATUS;           // MCAN Double Error Detected Interrupt Status Register
    Uint16                                   rsvd6[30];                    // Reserved
    union   MCANERR_DED_ENABLE_SET_REG       MCANERR_DED_ENABLE_SET;       // MCAN Double Error Detected Interrupt Enable Set Register
    Uint16                                   rsvd7[30];                    // Reserved
    union   MCANERR_DED_ENABLE_CLR_REG       MCANERR_DED_ENABLE_CLR;       // MCAN Double Error Detected Interrupt Enable Clear Register
    Uint16                                   rsvd8[30];                    // Reserved
    union   MCANERR_AGGR_ENABLE_SET_REG      MCANERR_AGGR_ENABLE_SET;      // MCAN Error Aggregator Enable Set Register
    union   MCANERR_AGGR_ENABLE_CLR_REG      MCANERR_AGGR_ENABLE_CLR;      // MCAN Error Aggregator Enable Clear Register
    union   MCANERR_AGGR_STATUS_SET_REG      MCANERR_AGGR_STATUS_SET;      // MCAN Error Aggregator Status Set Register
    union   MCANERR_AGGR_STATUS_CLR_REG      MCANERR_AGGR_STATUS_CLR;      // MCAN Error Aggregator Status Clear Register
};

//---------------------------------------------------------------------------
// MCAN External References & Function Declarations:
//
#ifdef CPU1
extern volatile struct MCAN_ERROR_REGS McanErrorRegs;
extern volatile struct MCAN_REGS McanRegs;
extern volatile struct MCANSS_REGS McanssRegs;
#endif

#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif

//===========================================================================
// End of file.
//===========================================================================
