//###########################################################################
//
// FILE:    hw_memcfg.h
//
// TITLE:   Definitions for the MEMCFG registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_MEMCFG_H
#define HW_MEMCFG_H

//*****************************************************************************
//
// The following are defines for the MEMCFG register offsets
//
//*****************************************************************************
#define MEMCFG_O_DXLOCK           0x0U         // Dedicated RAM Config Lock
                                               // Register
#define MEMCFG_O_DXCOMMIT         0x2U         // Dedicated RAM Config Lock
                                               // Commit Register
#define MEMCFG_O_DXTEST           0x10U        // Dedicated RAM TEST Register
#define MEMCFG_O_DXINIT           0x12U        // Dedicated RAM Init Register
#define MEMCFG_O_DXINITDONE       0x14U        // Dedicated RAM InitDone Status
                                               // Register
#define MEMCFG_O_LSXLOCK          0x20U        // Local Shared RAM Config Lock
                                               // Register
#define MEMCFG_O_LSXCOMMIT        0x22U        // Local Shared RAM Config Lock
                                               // Commit Register
#define MEMCFG_O_LSXMSEL          0x24U        // Local Shared RAM Master Sel
                                               // Register
#define MEMCFG_O_LSXCLAPGM        0x26U        // Local Shared RAM Prog/Exe
                                               // control Register
#define MEMCFG_O_LSXACCPROT1      0x2AU        // Local Shared RAM Config
                                               // Register 1
#define MEMCFG_O_LSXTEST          0x30U        // Local Shared RAM TEST
                                               // Register
#define MEMCFG_O_LSXINIT          0x32U        // Local Shared RAM Init
                                               // Register
#define MEMCFG_O_LSXINITDONE      0x34U        // Local Shared RAM InitDone
                                               // Status Register
#define MEMCFG_O_GSXLOCK          0x40U        // Global Shared RAM Config Lock
                                               // Register
#define MEMCFG_O_GSXCOMMIT        0x42U        // Global Shared RAM Config Lock
                                               // Commit Register
#define MEMCFG_O_GSXACCPROT0      0x48U        // Global Shared RAM Config
                                               // Register 0
#define MEMCFG_O_GSXTEST          0x50U        // Global Shared RAM TEST
                                               // Register
#define MEMCFG_O_GSXINIT          0x52U        // Global Shared RAM Init
                                               // Register
#define MEMCFG_O_GSXINITDONE      0x54U        // Global Shared RAM InitDone
                                               // Status Register
#define MEMCFG_O_MAVFLG           0x20U        // Master Access Violation Flag
                                               // Register
#define MEMCFG_O_MAVSET           0x22U        // Master Access Violation Flag
                                               // Set Register
#define MEMCFG_O_MAVCLR           0x24U        // Master Access Violation Flag
                                               // Clear Register
#define MEMCFG_O_MAVINTEN         0x26U        // Master Access Violation
                                               // Interrupt Enable Register
#define MEMCFG_O_MCPUFAVADDR      0x28U        // Master CPU Fetch Access
                                               // Violation Address
#define MEMCFG_O_MCPUWRAVADDR     0x2AU        // Master CPU Write Access
                                               // Violation Address
#define MEMCFG_O_MDMAWRAVADDR     0x2CU        // Master  DMA Write Access
                                               // Violation Address
#define MEMCFG_O_MHICWRAVADDR(i)  (0x2EU + ((i) * 0x2U))   // (0 <= i < 2) Master  HIC Write Access Violation Address
#define MEMCFG_O_UCERRFLG         0x0U         // Uncorrectable Error Flag
                                               // Register
#define MEMCFG_O_UCERRSET         0x2U         // Uncorrectable Error Flag Set
                                               // Register
#define MEMCFG_O_UCERRCLR         0x4U         // Uncorrectable Error Flag
                                               // Clear Register
#define MEMCFG_O_UCCPUREADDR      0x6U         // Uncorrectable CPU Read Error
                                               // Address
#define MEMCFG_O_UCDMAREADDR      0x8U         // Uncorrectable DMA Read Error
                                               // Address
#define MEMCFG_O_UCHICAREADDR     0xEU         // Uncorrectable HICA Read Error
                                               // Address
#define MEMCFG_O_CERRFLG          0x20U        // Correctable Error Flag
                                               // Register
#define MEMCFG_O_CERRSET          0x22U        // Correctable Error Flag Set
                                               // Register
#define MEMCFG_O_CERRCLR          0x24U        // Correctable Error Flag Clear
                                               // Register
#define MEMCFG_O_CCPUREADDR       0x26U        // Correctable CPU Read Error
                                               // Address
#define MEMCFG_O_CERRCNT          0x2EU        // Correctable Error Count
                                               // Register
#define MEMCFG_O_CERRTHRES        0x30U        // Correctable Error Threshold
                                               // Value Register
#define MEMCFG_O_CEINTFLG         0x32U        // Correctable Error Interrupt
                                               // Flag Status Register
#define MEMCFG_O_CEINTCLR         0x34U        // Correctable Error Interrupt
                                               // Flag Clear Register
#define MEMCFG_O_CEINTSET         0x36U        // Correctable Error Interrupt
                                               // Flag Set Register
#define MEMCFG_O_CEINTEN          0x38U        // Correctable Error Interrupt
                                               // Enable Register
#define MEMCFG_O_ROMWAITSTATE     0x0U         // ROM Wait State Configuration
                                               // Register
#define MEMCFG_O_ROMPREFETCH      0x0U         // ROM Prefetch Configuration
                                               // Register

//*****************************************************************************
//
// The following are defines for the bit fields in the DXLOCK register
//
//*****************************************************************************
#define MEMCFG_DXLOCK_LOCK_M0     0x1U         // M0 RAM Lock bits
#define MEMCFG_DXLOCK_LOCK_M1     0x2U         // M1 RAM Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the DXCOMMIT register
//
//*****************************************************************************
#define MEMCFG_DXCOMMIT_COMMIT_M0  0x1U         // M0 RAM Permanent Lock bits
#define MEMCFG_DXCOMMIT_COMMIT_M1  0x2U         // M1 RAM Permanent Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the DXTEST register
//
//*****************************************************************************
#define MEMCFG_DXTEST_TEST_M0_S   0U
#define MEMCFG_DXTEST_TEST_M0_M   0x3U         // Selects the different modes
                                               // for M0 RAM
#define MEMCFG_DXTEST_TEST_M1_S   2U
#define MEMCFG_DXTEST_TEST_M1_M   0xCU         // Selects the different modes
                                               // for M1 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the DXINIT register
//
//*****************************************************************************
#define MEMCFG_DXINIT_INIT_M0     0x1U         // RAM Initialization control
                                               // for M0 RAM.
#define MEMCFG_DXINIT_INIT_M1     0x2U         // RAM Initialization control
                                               // for M1 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the DXINITDONE register
//
//*****************************************************************************
#define MEMCFG_DXINITDONE_INITDONE_M0  0x1U         // RAM Initialization status for
                                               // M0 RAM.
#define MEMCFG_DXINITDONE_INITDONE_M1  0x2U         // RAM Initialization status for
                                               // M1 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXLOCK register
//
//*****************************************************************************
#define MEMCFG_LSXLOCK_LOCK_LS4   0x10U        // LS4 RAM Lock bits
#define MEMCFG_LSXLOCK_LOCK_LS5   0x20U        // LS5 RAM Lock bits
#define MEMCFG_LSXLOCK_LOCK_LS6   0x40U        // LS6 RAM Lock bits
#define MEMCFG_LSXLOCK_LOCK_LS7   0x80U        // LS7 RAM Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXCOMMIT register
//
//*****************************************************************************
#define MEMCFG_LSXCOMMIT_COMMIT_LS4  0x10U        // LS4 RAM Permanent Lock bits
#define MEMCFG_LSXCOMMIT_COMMIT_LS5  0x20U        // LS5 RAM Permanent Lock bits
#define MEMCFG_LSXCOMMIT_COMMIT_LS6  0x40U        // LS6 RAM Permanent Lock bits
#define MEMCFG_LSXCOMMIT_COMMIT_LS7  0x80U        // LS7 RAM Permanent Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXMSEL register
//
//*****************************************************************************
#define MEMCFG_LSXMSEL_MSEL_LS4_S  8U
#define MEMCFG_LSXMSEL_MSEL_LS4_M  0x300U       // Master Select for LS4 RAM
#define MEMCFG_LSXMSEL_MSEL_LS5_S  10U
#define MEMCFG_LSXMSEL_MSEL_LS5_M  0xC00U       // Master Select for LS5 RAM
#define MEMCFG_LSXMSEL_MSEL_LS6_S  12U
#define MEMCFG_LSXMSEL_MSEL_LS6_M  0x3000U      // Master Select for LS6 RAM
#define MEMCFG_LSXMSEL_MSEL_LS7_S  14U
#define MEMCFG_LSXMSEL_MSEL_LS7_M  0xC000U      // Master Select for LS7 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXCLAPGM register
//
//*****************************************************************************
#define MEMCFG_LSXCLAPGM_CLAPGM_LS4  0x10U        // Selects LS4 RAM as program vs
                                               // data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS5  0x20U        // Selects LS5 RAM as program vs
                                               // data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS6  0x40U        // Selects LS6 RAM as program vs
                                               // data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS7  0x80U        // Selects LS7 RAM as program vs
                                               // data memory for CLA

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXACCPROT1 register
//
//*****************************************************************************
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS4  0x1U         // Fetch Protection For LS4 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS4  0x2U         // CPU WR Protection For LS4 RAM
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS5  0x100U       // Fetch Protection For LS5 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS5  0x200U       // CPU WR Protection For LS5 RAM
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS6  0x10000U     // Fetch Protection For LS6 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS6  0x20000U     // CPU WR Protection For LS6 RAM
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS7  0x1000000U   // Fetch Protection For LS7 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS7  0x2000000U   // CPU WR Protection For LS7 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXTEST register
//
//*****************************************************************************
#define MEMCFG_LSXTEST_TEST_LS4_S  8U
#define MEMCFG_LSXTEST_TEST_LS4_M  0x300U       // Selects the different modes
                                               // for LS4 RAM
#define MEMCFG_LSXTEST_TEST_LS5_S  10U
#define MEMCFG_LSXTEST_TEST_LS5_M  0xC00U       // Selects the different modes
                                               // for LS5 RAM
#define MEMCFG_LSXTEST_TEST_LS6_S  12U
#define MEMCFG_LSXTEST_TEST_LS6_M  0x3000U      // Selects the different modes
                                               // for LS6 RAM
#define MEMCFG_LSXTEST_TEST_LS7_S  14U
#define MEMCFG_LSXTEST_TEST_LS7_M  0xC000U      // Selects the different modes
                                               // for LS7 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXINIT register
//
//*****************************************************************************
#define MEMCFG_LSXINIT_INIT_LS4   0x10U        // RAM Initialization control
                                               // for LS4 RAM.
#define MEMCFG_LSXINIT_INIT_LS5   0x20U        // RAM Initialization control
                                               // for LS5 RAM.
#define MEMCFG_LSXINIT_INIT_LS6   0x40U        // RAM Initialization control
                                               // for LS6 RAM.
#define MEMCFG_LSXINIT_INIT_LS7   0x80U        // RAM Initialization control
                                               // for LS7 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the LSXINITDONE register
//
//*****************************************************************************
#define MEMCFG_LSXINITDONE_INITDONE_LS4  0x10U        // RAM Initialization status for
                                               // LS4 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS5  0x20U        // RAM Initialization status for
                                               // LS5 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS6  0x40U        // RAM Initialization status for
                                               // LS6 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS7  0x80U        // RAM Initialization status for
                                               // LS7 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXLOCK register
//
//*****************************************************************************
#define MEMCFG_GSXLOCK_LOCK_GS0   0x1U         // GS0 RAM Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXCOMMIT register
//
//*****************************************************************************
#define MEMCFG_GSXCOMMIT_COMMIT_GS0  0x1U         // GS0 RAM Permanent Lock bits

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXACCPROT0 register
//
//*****************************************************************************
#define MEMCFG_GSXACCPROT0_FETCHPROT_GS0  0x1U         // Fetch Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_CPUWRPROT_GS0  0x2U         // CPU WR Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_DMAWRPROT_GS0  0x4U         // DMA WR Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_HICWRPROT_GS0  0x8U         // HIC WR Protection For GS0 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXTEST register
//
//*****************************************************************************
#define MEMCFG_GSXTEST_TEST_GS0_S  0U
#define MEMCFG_GSXTEST_TEST_GS0_M  0x3U         // Selects the different modes
                                               // for GS0 RAM

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXINIT register
//
//*****************************************************************************
#define MEMCFG_GSXINIT_INIT_GS0   0x1U         // RAM Initialization control
                                               // for GS0 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the GSXINITDONE register
//
//*****************************************************************************
#define MEMCFG_GSXINITDONE_INITDONE_GS0  0x1U         // RAM Initialization status for
                                               // GS0 RAM.

//*****************************************************************************
//
// The following are defines for the bit fields in the MAVFLG register
//
//*****************************************************************************
#define MEMCFG_MAVFLG_CPUFETCH    0x1U         // Master CPU Fetch Access
                                               // Violation Flag
#define MEMCFG_MAVFLG_CPUWRITE    0x2U         // Master CPU Write Access
                                               // Violation Flag
#define MEMCFG_MAVFLG_DMAWRITE    0x4U         // Master DMA Write Access
                                               // Violation Flag
#define MEMCFG_MAVFLG_HICAWRITE   0x8U         // Master HICA Write Access
                                               // Violation Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the MAVSET register
//
//*****************************************************************************
#define MEMCFG_MAVSET_CPUFETCH    0x1U         // Master CPU Fetch Access
                                               // Violation Flag Set
#define MEMCFG_MAVSET_CPUWRITE    0x2U         // Master CPU Write Access
                                               // Violation Flag Set
#define MEMCFG_MAVSET_DMAWRITE    0x4U         // Master DMA Write Access
                                               // Violation Flag Set
#define MEMCFG_MAVSET_HICAWRITE   0x8U         // Master HICA Write Access
                                               // Violation Flag Set

//*****************************************************************************
//
// The following are defines for the bit fields in the MAVCLR register
//
//*****************************************************************************
#define MEMCFG_MAVCLR_CPUFETCH    0x1U         // Master CPU Fetch Access
                                               // Violation Flag Clear
#define MEMCFG_MAVCLR_CPUWRITE    0x2U         // Master CPU Write Access
                                               // Violation Flag Clear
#define MEMCFG_MAVCLR_DMAWRITE    0x4U         // Master DMA Write Access
                                               // Violation Flag Clear
#define MEMCFG_MAVCLR_HICAWRITE   0x8U         // Master HICA Write Access
                                               // Violation Flag Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the MAVINTEN register
//
//*****************************************************************************
#define MEMCFG_MAVINTEN_CPUFETCH  0x1U         // Master CPU Fetch Access
                                               // Violation Interrupt Enable
#define MEMCFG_MAVINTEN_CPUWRITE  0x2U         // Master CPU Write Access
                                               // Violation Interrupt Enable
#define MEMCFG_MAVINTEN_DMAWRITE  0x4U         // Master DMA Write Access
                                               // Violation Interrupt Enable
#define MEMCFG_MAVINTEN_HICAWRITE  0x8U         // Master HICA Write Access
                                               // Violation Interrupt Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UCERRFLG register
//
//*****************************************************************************
#define MEMCFG_UCERRFLG_CPURDERR  0x1U         // CPU Uncorrectable Read Error
                                               // Flag
#define MEMCFG_UCERRFLG_DMARDERR  0x2U         // DMA Uncorrectable Read Error
                                               // Flag
#define MEMCFG_UCERRFLG_HICARDERR  0x10U        // HICA Uncorrectable Read Error
                                               // Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the UCERRSET register
//
//*****************************************************************************
#define MEMCFG_UCERRSET_CPURDERR  0x1U         // CPU Uncorrectable Read Error
                                               // Flag Set
#define MEMCFG_UCERRSET_DMARDERR  0x2U         // DMA Uncorrectable Read Error
                                               // Flag Set
#define MEMCFG_UCERRSET_HICARDERR  0x10U        // HICA Uncorrectable Read Error
                                               // Flag Set

//*****************************************************************************
//
// The following are defines for the bit fields in the UCERRCLR register
//
//*****************************************************************************
#define MEMCFG_UCERRCLR_CPURDERR  0x1U         // CPU Uncorrectable Read Error
                                               // Flag Clear
#define MEMCFG_UCERRCLR_DMARDERR  0x2U         // DMA Uncorrectable Read Error
                                               // Flag Clear
#define MEMCFG_UCERRCLR_HICARDERR  0x10U        // HICA Uncorrectable Read Error
                                               // Flag Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the CERRFLG register
//
//*****************************************************************************
#define MEMCFG_CERRFLG_CPURDERR   0x1U         // CPU Correctable Read Error
                                               // Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the CERRSET register
//
//*****************************************************************************
#define MEMCFG_CERRSET_CPURDERR   0x1U         // CPU Correctable Read Error
                                               // Flag Set

//*****************************************************************************
//
// The following are defines for the bit fields in the CERRCLR register
//
//*****************************************************************************
#define MEMCFG_CERRCLR_CPURDERR   0x1U         // CPU Correctable Read Error
                                               // Flag Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the CEINTFLG register
//
//*****************************************************************************
#define MEMCFG_CEINTFLG_CEINTFLAG  0x1U         // Total corrected error count
                                               // exceeded threshold flag.

//*****************************************************************************
//
// The following are defines for the bit fields in the CEINTCLR register
//
//*****************************************************************************
#define MEMCFG_CEINTCLR_CEINTCLR  0x1U         // CPU Corrected Error Threshold
                                               // Exceeded Error Clear.

//*****************************************************************************
//
// The following are defines for the bit fields in the CEINTSET register
//
//*****************************************************************************
#define MEMCFG_CEINTSET_CEINTSET  0x1U         // Total corrected error count
                                               // exceeded flag set.

//*****************************************************************************
//
// The following are defines for the bit fields in the CEINTEN register
//
//*****************************************************************************
#define MEMCFG_CEINTEN_CEINTEN    0x1U         // CPU/DMA Correctable Error
                                               // Interrupt Enable.

//*****************************************************************************
//
// The following are defines for the bit fields in the ROMWAITSTATE register
//
//*****************************************************************************
#define MEMCFG_ROMWAITSTATE_WSDISABLE  0x1U         // ROM Wait State Enable/Disable
                                               // Control

//*****************************************************************************
//
// The following are defines for the bit fields in the ROMPREFETCH register
//
//*****************************************************************************
#define MEMCFG_ROMPREFETCH_PFENABLE  0x1U         // ROM Prefetch Enable/Disable
                                               // Control
#endif
