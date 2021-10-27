//###########################################################################
//
// FILE:    hw_hrcap.h
//
// TITLE:   Definitions for the HRCAP registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_HRCAP_H
#define HW_HRCAP_H

//*****************************************************************************
//
// The following are defines for the HRCAP register offsets
//
//*****************************************************************************
#define HRCAP_O_TSCTR             0x0U         // Time-Stamp Counter
#define HRCAP_O_CTRPHS            0x2U         // Counter Phase Offset Value
                                               // Register
#define HRCAP_O_CAP1              0x4U         // Capture 1 Register
#define HRCAP_O_CAP2              0x6U         // Capture 2 Register
#define HRCAP_O_CAP3              0x8U         // Capture 3 Register
#define HRCAP_O_CAP4              0xAU         // Capture 4 Register
#define HRCAP_O_ECCTL0            0x12U        // Capture Control Register 0
#define HRCAP_O_ECCTL1            0x14U        // Capture Control Register 1
#define HRCAP_O_ECCTL2            0x15U        // Capture Control Register 2
#define HRCAP_O_ECEINT            0x16U        // Capture Interrupt Enable
                                               // Register
#define HRCAP_O_ECFLG             0x17U        // Capture Interrupt Flag
                                               // Register
#define HRCAP_O_ECCLR             0x18U        // Capture Interrupt Clear
                                               // Register
#define HRCAP_O_ECFRC             0x19U        // Capture Interrupt Force
                                               // Register
#define HRCAP_O_ECAPSYNCINSEL     0x1EU        // SYNC source select register
#define HRCAP_O_HRCTL             0x0U         // High-Res Control Register
#define HRCAP_O_HRINTEN           0x4U         // High-Res Calibration
                                               // Interrupt Enable Register
#define HRCAP_O_HRFLG             0x6U         // High-Res Calibration
                                               // Interrupt Flag Register
#define HRCAP_O_HRCLR             0x8U         // High-Res Calibration
                                               // Interrupt Clear Register
#define HRCAP_O_HRFRC             0xAU         // High-Res Calibration
                                               // Interrupt Force Register
#define HRCAP_O_HRCALPRD          0xCU         // High-Res Calibration Period
                                               // Register
#define HRCAP_O_HRSYSCLKCTR       0xEU         // High-Res Calibration SYSCLK
                                               // Counter Register
#define HRCAP_O_HRSYSCLKCAP       0x10U        // High-Res Calibration SYSCLK
                                               // Capture Register
#define HRCAP_O_HRCLKCTR          0x12U        // High-Res Calibration HRCLK
                                               // Counter Register
#define HRCAP_O_HRCLKCAP          0x14U        // High-Res Calibration HRCLK
                                               // Capture Register
#define HRCAP_O_HRDEBUGCTL        0x1AU        // High-Res Debug control
                                               // register
#define HRCAP_O_HRDEBUGOBSERVE1   0x1CU        // High-Res Raw output and
                                               // internal nodes of HRCLK capture
                                               // delay line
#define HRCAP_O_HRDEBUGOBSERVE2   0x1EU        // High-Res Raw output and
                                               // internal nodes of HRCLK capture
                                               // delay line

//*****************************************************************************
//
// The following are defines for the bit fields in the ECCTL0 register
//
//*****************************************************************************
#define HRCAP_ECCTL0_INPUTSEL_S   0U
#define HRCAP_ECCTL0_INPUTSEL_M   0x7FU        // INPUT source select

//*****************************************************************************
//
// The following are defines for the bit fields in the ECCTL1 register
//
//*****************************************************************************
#define HRCAP_ECCTL1_CAP1POL      0x1U         // Capture Event 1 Polarity
                                               // select
#define HRCAP_ECCTL1_CTRRST1      0x2U         // Counter Reset on Capture
                                               // Event 1
#define HRCAP_ECCTL1_CAP2POL      0x4U         // Capture Event 2 Polarity
                                               // select
#define HRCAP_ECCTL1_CTRRST2      0x8U         // Counter Reset on Capture
                                               // Event 2
#define HRCAP_ECCTL1_CAP3POL      0x10U        // Capture Event 3 Polarity
                                               // select
#define HRCAP_ECCTL1_CTRRST3      0x20U        // Counter Reset on Capture
                                               // Event 3
#define HRCAP_ECCTL1_CAP4POL      0x40U        // Capture Event 4 Polarity
                                               // select
#define HRCAP_ECCTL1_CTRRST4      0x80U        // Counter Reset on Capture
                                               // Event 4
#define HRCAP_ECCTL1_CAPLDEN      0x100U       // Enable Loading CAP1-4 regs on
                                               // a Cap Event
#define HRCAP_ECCTL1_PRESCALE_S   9U
#define HRCAP_ECCTL1_PRESCALE_M   0x3E00U      // Event Filter prescale select
#define HRCAP_ECCTL1_FREE_SOFT_S  14U
#define HRCAP_ECCTL1_FREE_SOFT_M  0xC000U      // Emulation mode

//*****************************************************************************
//
// The following are defines for the bit fields in the ECCTL2 register
//
//*****************************************************************************
#define HRCAP_ECCTL2_CONT_ONESHT  0x1U         // Continuous or one-shot
#define HRCAP_ECCTL2_STOP_WRAP_S  1U
#define HRCAP_ECCTL2_STOP_WRAP_M  0x6U         // Stop value for one-shot, Wrap
                                               // for continuous
#define HRCAP_ECCTL2_REARM        0x8U         // One-shot re-arm
#define HRCAP_ECCTL2_TSCTRSTOP    0x10U        // TSCNT counter stop
#define HRCAP_ECCTL2_SYNCI_EN     0x20U        // Counter sync-in select
#define HRCAP_ECCTL2_SYNCO_SEL_S  6U
#define HRCAP_ECCTL2_SYNCO_SEL_M  0xC0U        // Sync-out mode
#define HRCAP_ECCTL2_SWSYNC       0x100U       // SW forced counter sync
#define HRCAP_ECCTL2_CAP_APWM     0x200U       // CAP/APWM operating mode
                                               // select
#define HRCAP_ECCTL2_APWMPOL      0x400U       // APWM output polarity select
#define HRCAP_ECCTL2_CTRFILTRESET  0x800U       // Reset event filter, modulus
                                               // counter, and interrupt flags.
#define HRCAP_ECCTL2_DMAEVTSEL_S  12U
#define HRCAP_ECCTL2_DMAEVTSEL_M  0x3000U      // DMA event select
#define HRCAP_ECCTL2_MODCNTRSTS_S  14U
#define HRCAP_ECCTL2_MODCNTRSTS_M  0xC000U      // modulo counter status

//*****************************************************************************
//
// The following are defines for the bit fields in the ECEINT register
//
//*****************************************************************************
#define HRCAP_ECEINT_CEVT1        0x2U         // Capture Event 1 Interrupt
                                               // Enable
#define HRCAP_ECEINT_CEVT2        0x4U         // Capture Event 2 Interrupt
                                               // Enable
#define HRCAP_ECEINT_CEVT3        0x8U         // Capture Event 3 Interrupt
                                               // Enable
#define HRCAP_ECEINT_CEVT4        0x10U        // Capture Event 4 Interrupt
                                               // Enable
#define HRCAP_ECEINT_CTROVF       0x20U        // Counter Overflow Interrupt
                                               // Enable
#define HRCAP_ECEINT_CTR_EQ_PRD   0x40U        // Period Equal Interrupt Enable
#define HRCAP_ECEINT_CTR_EQ_CMP   0x80U        // Compare Equal Interrupt
                                               // Enable
#define HRCAP_ECEINT_HRERROR      0x100U       // High resolution error
                                               // interrupt enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ECFLG register
//
//*****************************************************************************
#define HRCAP_ECFLG_INT           0x1U         // Global Flag
#define HRCAP_ECFLG_CEVT1         0x2U         // Capture Event 1 Interrupt
                                               // Flag
#define HRCAP_ECFLG_CEVT2         0x4U         // Capture Event 2 Interrupt
                                               // Flag
#define HRCAP_ECFLG_CEVT3         0x8U         // Capture Event 3 Interrupt
                                               // Flag
#define HRCAP_ECFLG_CEVT4         0x10U        // Capture Event 4 Interrupt
                                               // Flag
#define HRCAP_ECFLG_CTROVF        0x20U        // Counter Overflow Interrupt
                                               // Flag
#define HRCAP_ECFLG_CTR_PRD       0x40U        // Period Equal Interrupt Flag
#define HRCAP_ECFLG_CTR_CMP       0x80U        // Compare Equal Interrupt Flag
#define HRCAP_ECFLG_HRERROR       0x100U       // High resolution error
                                               // interrupt Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the ECCLR register
//
//*****************************************************************************
#define HRCAP_ECCLR_INT           0x1U         // ECAP Global Interrupt Status
                                               // Clear
#define HRCAP_ECCLR_CEVT1         0x2U         // Capture Event 1 Status Clear
#define HRCAP_ECCLR_CEVT2         0x4U         // Capture Event 2 Status Clear
#define HRCAP_ECCLR_CEVT3         0x8U         // Capture Event 3 Status Clear
#define HRCAP_ECCLR_CEVT4         0x10U        // Capture Event 4 Status Clear
#define HRCAP_ECCLR_CTROVF        0x20U        // Counter Overflow Status Clear
#define HRCAP_ECCLR_CTR_PRD       0x40U        // Period Equal Status Clear
#define HRCAP_ECCLR_CTR_CMP       0x80U        // Compare Equal Status Clear
#define HRCAP_ECCLR_HRERROR       0x100U       // High resolution error status
                                               // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ECFRC register
//
//*****************************************************************************
#define HRCAP_ECFRC_CEVT1         0x2U         // Capture Event 1 Force
                                               // Interrupt
#define HRCAP_ECFRC_CEVT2         0x4U         // Capture Event 2 Force
                                               // Interrupt
#define HRCAP_ECFRC_CEVT3         0x8U         // Capture Event 3 Force
                                               // Interrupt
#define HRCAP_ECFRC_CEVT4         0x10U        // Capture Event 4 Force
                                               // Interrupt
#define HRCAP_ECFRC_CTROVF        0x20U        // Counter Overflow Force
                                               // Interrupt
#define HRCAP_ECFRC_CTR_PRD       0x40U        // Period Equal Force Interrupt
#define HRCAP_ECFRC_CTR_CMP       0x80U        // Compare Equal Force Interrupt
#define HRCAP_ECFRC_HRERROR       0x100U       // High resolution error Force
                                               // interrupt

//*****************************************************************************
//
// The following are defines for the bit fields in the ECAPSYNCINSEL register
//
//*****************************************************************************
#define HRCAP_ECAPSYNCINSEL_SEL_S  0U
#define HRCAP_ECAPSYNCINSEL_SEL_M  0x1FU        // SYNCIN source select

//*****************************************************************************
//
// The following are defines for the bit fields in the HRCTL register
//
//*****************************************************************************
#define HRCAP_HRCTL_HRE           0x1U         // High Resolution Enable
#define HRCAP_HRCTL_HRCLKE        0x2U         // High Resolution Clock Enable
#define HRCAP_HRCTL_PRDSEL        0x4U         // Calibration Period Match
#define HRCAP_HRCTL_CALIBSTART    0x8U         // Calibration start
#define HRCAP_HRCTL_CALIBSTS      0x10U        // Calibration status
#define HRCAP_HRCTL_CALIBCONT     0x20U        // Continuous mode Calibration
                                               // Select

//*****************************************************************************
//
// The following are defines for the bit fields in the HRINTEN register
//
//*****************************************************************************
#define HRCAP_HRINTEN_CALIBDONE   0x2U         // Calibration doe interrupt
                                               // enable
#define HRCAP_HRINTEN_CALPRDCHKSTS  0x4U         // Calibration period check
                                               // status enable

//*****************************************************************************
//
// The following are defines for the bit fields in the HRFLG register
//
//*****************************************************************************
#define HRCAP_HRFLG_CALIBINT      0x1U         // Global calibration Interrupt
                                               // Status Flag
#define HRCAP_HRFLG_CALIBDONE     0x2U         // Calibration Done Interrupt
                                               // Flag Bit
#define HRCAP_HRFLG_CALPRDCHKSTS  0x4U         // Calibration period check
                                               // status Flag Bi

//*****************************************************************************
//
// The following are defines for the bit fields in the HRCLR register
//
//*****************************************************************************
#define HRCAP_HRCLR_CALIBINT      0x1U         // Clear Global calibration
                                               // Interrupt Flag
#define HRCAP_HRCLR_CALIBDONE     0x2U         // Clear Calibration Done
                                               // Interrupt Flag Bit
#define HRCAP_HRCLR_CALPRDCHKSTS  0x4U         // Clear Calibration period
                                               // check status Flag Bit:

//*****************************************************************************
//
// The following are defines for the bit fields in the HRFRC register
//
//*****************************************************************************
#define HRCAP_HRFRC_CALIBDONE     0x2U         // Force Calibration Done
                                               // Interrupt Flag Bit
#define HRCAP_HRFRC_CALPRDCHKSTS  0x4U         // Force Calibration period
                                               // check status Flag Bit:

//*****************************************************************************
//
// The following are defines for the bit fields in the HRDEBUGCTL register
//
//*****************************************************************************
#define HRCAP_HRDEBUGCTL_DISABLEINVSEL  0x1U         // Disable INVSEL logic
#define HRCAP_HRDEBUGCTL_DELAYRESETDLINE  0x2U         // Delay RESET by one cycle
#define HRCAP_HRDEBUGCTL_CAPIN_MMAP_SOURCE  0x4U         // Memory mapped CAPIN source
#define HRCAP_HRDEBUGCTL_CALIB_INPUT_SEL_S  4U
#define HRCAP_HRDEBUGCTL_CALIB_INPUT_SEL_M  0x30U        // Calibration Input select
#define HRCAP_HRDEBUGCTL_OBSERVE_SRC_SEL_S  8U
#define HRCAP_HRDEBUGCTL_OBSERVE_SRC_SEL_M  0xF00U       // Source select for observe
                                               // registers
#endif