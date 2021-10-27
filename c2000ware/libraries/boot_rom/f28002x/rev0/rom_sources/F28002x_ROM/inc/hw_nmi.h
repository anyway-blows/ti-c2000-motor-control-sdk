//###########################################################################
//
// FILE:    hw_nmi.h
//
// TITLE:   Definitions for the NMI registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_NMI_H
#define HW_NMI_H

//*****************************************************************************
//
// The following are defines for the NMI register offsets
//
//*****************************************************************************
#define NMI_O_CFG                 0x0U         // NMI Configuration Register
#define NMI_O_FLG                 0x1U         // NMI Flag Register (SYSRsn
                                               // Clear)
#define NMI_O_FLGCLR              0x2U         // NMI Flag Clear Register
#define NMI_O_FLGFRC              0x3U         // NMI Flag Force Register
#define NMI_O_WDCNT               0x4U         // NMI Watchdog Counter Register
#define NMI_O_WDPRD               0x5U         // NMI Watchdog Period Register
#define NMI_O_SHDFLG              0x6U         // NMI Shadow Flag Register
#define NMI_O_ERRORSTS            0x7U         // Error pin status
#define NMI_O_ERRORSTSCLR         0x8U         // ERRORSTS clear register
#define NMI_O_ERRORSTSFRC         0x9U         // ERRORSTS force register
#define NMI_O_ERRORCTL            0xAU         // Error pin control register
#define NMI_O_ERRORLOCK           0xBU         // Lock register to Error pin
                                               // registers.

//*****************************************************************************
//
// The following are defines for the bit fields in the NMICFG register
//
//*****************************************************************************
#define NMI_CFG_NMIE              0x1U         // Global NMI Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the NMIFLG register
//
//*****************************************************************************
#define NMI_FLG_NMIINT            0x1U         // NMI Interrupt Flag
#define NMI_FLG_CLOCKFAIL         0x2U         // Clock Fail Interrupt Flag
#define NMI_FLG_RAMUNCERR         0x4U         // RAM Uncorrectable Error NMI
                                               // Flag
#define NMI_FLG_FLUNCERR          0x8U         // Flash Uncorrectable Error NMI
                                               // Flag
#define NMI_FLG_CPU1HWBISTERR     0x10U        // HW BIST Error NMI Flag
#define NMI_FLG_SYSDBGNMI         0x80U        // System Debug Module NMI Flag
#define NMI_FLG_RLNMI             0x100U       // Reconfigurable Logic NMI Flag
#define NMI_FLG_OVF               0x800U       // Over Voltage Fault Indication
                                               // Flag
#define NMI_FLG_SWERR             0x2000U      // SW Error Force NMI Flag
#define NMI_FLG_CRC_FAIL          0x4000U      // BGCRC calculation failed.

//*****************************************************************************
//
// The following are defines for the bit fields in the NMIFLGCLR register
//
//*****************************************************************************
#define NMI_FLGCLR_NMIINT         0x1U         // NMIINT Flag Clear
#define NMI_FLGCLR_CLOCKFAIL      0x2U         // CLOCKFAIL Flag Clear
#define NMI_FLGCLR_RAMUNCERR      0x4U         // RAMUNCERR Flag Clear
#define NMI_FLGCLR_FLUNCERR       0x8U         // FLUNCERR Flag Clear
#define NMI_FLGCLR_CPU1HWBISTERR  0x10U        // CPU1HWBISTERR Flag Clear
#define NMI_FLGCLR_SYSDBGNMI      0x80U        // SYSDBGNMI Flag Clear
#define NMI_FLGCLR_RLNMI          0x100U       // RLNMI Flag Clear
#define NMI_FLGCLR_OVF            0x800U       // OVF Flag Clear
#define NMI_FLGCLR_SWERR          0x2000U      // SW Error Flag Clear
#define NMI_FLGCLR_CRC_FAIL       0x4000U      // CRC_FAIL flag clear

//*****************************************************************************
//
// The following are defines for the bit fields in the NMIFLGFRC register
//
//*****************************************************************************
#define NMI_FLGFRC_CLOCKFAIL      0x2U         // CLOCKFAIL Flag Force
#define NMI_FLGFRC_RAMUNCERR      0x4U         // RAMUNCERR Flag Force
#define NMI_FLGFRC_FLUNCERR       0x8U         // FLUNCERR Flag Force
#define NMI_FLGFRC_CPU1HWBISTERR  0x10U        // CPU1HWBISTERR Flag Force
#define NMI_FLGFRC_SYSDBGNMI      0x80U        // SYSDBGNMI Flag Force
#define NMI_FLGFRC_RLNMI          0x100U       // RLNMI Flag Force
#define NMI_FLGFRC_OVF            0x800U       // OVF Flag Force
#define NMI_FLGFRC_SWERR          0x2000U      // SW Error Flag Force
#define NMI_FLGFRC_CRC_FAIL       0x4000U      // CRC_FAIL flag force

//*****************************************************************************
//
// The following are defines for the bit fields in the NMISHDFLG register
//
//*****************************************************************************
#define NMI_SHDFLG_CLOCKFAIL      0x2U         // Shadow CLOCKFAIL Flag
#define NMI_SHDFLG_RAMUNCERR      0x4U         // Shadow RAMUNCERR Flag
#define NMI_SHDFLG_FLUNCERR       0x8U         // Shadow FLUNCERR Flag
#define NMI_SHDFLG_CPU1HWBISTERR  0x10U        // Shadow CPU1HWBISTERR Flag
#define NMI_SHDFLG_SYSDBGNMI      0x80U        // Shadow SYSDBGNMI Flag
#define NMI_SHDFLG_RLNMI          0x100U       // Shadow RLNMI Flag
#define NMI_SHDFLG_OVF            0x800U       // Shadow OVF Flag
#define NMI_SHDFLG_SWERR          0x2000U      // Shadow SW Error Flag
#define NMI_SHDFLG_CRC_FAIL       0x4000U      // CRC_FAIL flag

//*****************************************************************************
//
// The following are defines for the bit fields in the ERRORSTS register
//
//*****************************************************************************
#define NMI_ERRORSTS_ERROR        0x1U         // Error flag.
#define NMI_ERRORSTS_PINSTS       0x2U         // Error pin status.

//*****************************************************************************
//
// The following are defines for the bit fields in the ERRORSTSCLR register
//
//*****************************************************************************
#define NMI_ERRORSTSCLR_ERROR     0x1U         // ERRORFLG.ERROR clear bit

//*****************************************************************************
//
// The following are defines for the bit fields in the ERRORSTSFRC register
//
//*****************************************************************************
#define NMI_ERRORSTSFRC_ERROR     0x1U         // ERRORSTS.ERROR pin force.

//*****************************************************************************
//
// The following are defines for the bit fields in the ERRORCTL register
//
//*****************************************************************************
#define NMI_ERRORCTL_ERRORPOLSEL  0x1U         // ERROR pin polarity select

//*****************************************************************************
//
// The following are defines for the bit fields in the ERRORLOCK register
//
//*****************************************************************************
#define NMI_ERRORLOCK_ERRORCTL    0x1U         // ERRORCTL Lock bit
#endif
