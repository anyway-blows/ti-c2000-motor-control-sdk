//###########################################################################
//
// FILE:    hw_emif.h
//
// TITLE:   Definitions for the EMIF registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_EMIF_H
#define HW_EMIF_H

//*****************************************************************************
//
// The following are defines for the EMIF register offsets
//
//*****************************************************************************
#define EMIF_O_RCSR               0x0U         // Revision Code and Status
                                               // Register
#define EMIF_O_ASYNC_WCCR         0x2U         // Async Wait Cycle Config
                                               // Register
#define EMIF_O_SDRAM_CR           0x4U         // SDRAM (EMxCS0n) Config
                                               // Register
#define EMIF_O_SDRAM_RCR          0x6U         // SDRAM Refresh Control
                                               // Register
#define EMIF_O_ASYNC_CS2_CR       0x8U         // Async 1 (EMxCS2n) Config
                                               // Register
#define EMIF_O_ASYNC_CS3_CR       0xAU         // Async 2 (EMxCS3n) Config
                                               // Register
#define EMIF_O_ASYNC_CS4_CR       0xCU         // Async 3 (EMxCS4n) Config
                                               // Register
#define EMIF_O_SDRAM_TR           0x10U        // SDRAM Timing Register
#define EMIF_O_TOTAL_SDRAM_AR     0x18U        // Total SDRAM Accesses Register
#define EMIF_O_TOTAL_SDRAM_ACTR   0x1AU        // Total SDRAM Activate Register
#define EMIF_O_SDR_EXT_TMNG       0x1EU        // SDRAM SR/PD Exit Timing
                                               // Register
#define EMIF_O_INT_RAW            0x20U        // Interrupt Raw Register
#define EMIF_O_INT_MSK            0x22U        // Interrupt Masked Register
#define EMIF_O_INT_MSK_SET        0x24U        // Interrupt Mask Set Register
#define EMIF_O_INT_MSK_CLR        0x26U        // Interrupt Mask Clear Register
#define EMIF_O_IO_CTRL            0x28U        // IO Control Register
#define EMIF_O_IO_STAT            0x2AU        // IO Status Register
#define EMIF_O_IODFT_TLECR        0x42U        // IODFT Test Logic Execution
                                               // Counter Register
#define EMIF_O_IODFT_TLGCR        0x44U        // IODFT Test Logic Global
                                               // Control Register
#define EMIF_O_IODFT_TLAMR        0x48U        // IODFT Test Logic Address MISR
                                               // Result Register
#define EMIF_O_IODFT_TLDMR        0x4AU        // IODFT Test Logic Data MISR
                                               // Result Register
#define EMIF_O_IODFT_TLDCMR       0x4CU        // IODFT Test Logic Data and
                                               // Control MISR Result Register
#define EMIF_O_MODEL_REL_NUM      0x56U        // Module Release Number
                                               // Register

//*****************************************************************************
//
// The following are defines for the bit fields in the RCSR register
//
//*****************************************************************************
#define EMIF_RCSR_MINOR_REVISION_S  0U
#define EMIF_RCSR_MINOR_REVISION_M  0xFFU        // Minor Revision.
#define EMIF_RCSR_MAJOR_REVISION_S  8U
#define EMIF_RCSR_MAJOR_REVISION_M  0xFF00U      // Major Revision.
#define EMIF_RCSR_MODULE_ID_S     16U
#define EMIF_RCSR_MODULE_ID_M     0x3FFF0000U  // EMIF module ID.
#define EMIF_RCSR_FR              0x40000000U  // EMIF is running in full rate
                                               // or half rate.
#define EMIF_RCSR_BE              0x80000000U  // EMIF endian mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the ASYNC_WCCR register
//
//*****************************************************************************
#define EMIF_ASYNC_WCCR_MAX_EXT_WAIT_S  0U
#define EMIF_ASYNC_WCCR_MAX_EXT_WAIT_M  0xFFU        // Maximum Extended Wait cycles.
#define EMIF_ASYNC_WCCR_WP0       0x10000000U  // Polarity for EMxWAIT.

//*****************************************************************************
//
// The following are defines for the bit fields in the SDRAM_CR register
//
//*****************************************************************************
#define EMIF_SDRAM_CR_PAGESIGE_S  0U
#define EMIF_SDRAM_CR_PAGESIGE_M  0x7U         // Page Size.
#define EMIF_SDRAM_CR_IBANK_S     4U
#define EMIF_SDRAM_CR_IBANK_M     0x70U        // Internal Bank setup of SDRAM
                                               // devices.
#define EMIF_SDRAM_CR_BIT_11_9_LOCK  0x100U       // Bits 11 to 9 are writable
                                               // only if this bit is set.
#define EMIF_SDRAM_CR_CL_S        9U
#define EMIF_SDRAM_CR_CL_M        0xE00U       // CAS Latency.
#define EMIF_SDRAM_CR_NM          0x4000U      // Narrow Mode.
#define EMIF_SDRAM_CR_PDWR        0x20000000U  // Perform refreshes during
                                               // Power Down.
#define EMIF_SDRAM_CR_PD          0x40000000U  // Power Down.
#define EMIF_SDRAM_CR_SR          0x80000000U  // Self Refresh.

//*****************************************************************************
//
// The following are defines for the bit fields in the SDRAM_RCR register
//
//*****************************************************************************
#define EMIF_SDRAM_RCR_REFRESH_RATE_S  0U
#define EMIF_SDRAM_RCR_REFRESH_RATE_M  0x1FFFU      // Refresh Rate.

//*****************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS2_CR register
//
//*****************************************************************************
#define EMIF_ASYNC_CS2_CR_ASIZE_S  0U
#define EMIF_ASYNC_CS2_CR_ASIZE_M  0x3U         // Asynchronous Memory Size.
#define EMIF_ASYNC_CS2_CR_TA_S    2U
#define EMIF_ASYNC_CS2_CR_TA_M    0xCU         // Turn Around cycles.
#define EMIF_ASYNC_CS2_CR_R_HOLD_S  4U
#define EMIF_ASYNC_CS2_CR_R_HOLD_M  0x70U        // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS2_CR_R_STROBE_S  7U
#define EMIF_ASYNC_CS2_CR_R_STROBE_M  0x1F80U      // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS2_CR_R_SETUP_S  13U
#define EMIF_ASYNC_CS2_CR_R_SETUP_M  0x1E000U     // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS2_CR_W_HOLD_S  17U
#define EMIF_ASYNC_CS2_CR_W_HOLD_M  0xE0000U     // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS2_CR_W_STROBE_S  20U
#define EMIF_ASYNC_CS2_CR_W_STROBE_M  0x3F00000U   // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS2_CR_W_SETUP_S  26U
#define EMIF_ASYNC_CS2_CR_W_SETUP_M  0x3C000000U  // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS2_CR_EW      0x40000000U  // Extend Wait mode.
#define EMIF_ASYNC_CS2_CR_SS      0x80000000U  // Select Strobe mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS3_CR register
//
//*****************************************************************************
#define EMIF_ASYNC_CS3_CR_ASIZE_S  0U
#define EMIF_ASYNC_CS3_CR_ASIZE_M  0x3U         // Asynchronous Memory Size.
#define EMIF_ASYNC_CS3_CR_TA_S    2U
#define EMIF_ASYNC_CS3_CR_TA_M    0xCU         // Turn Around cycles.
#define EMIF_ASYNC_CS3_CR_R_HOLD_S  4U
#define EMIF_ASYNC_CS3_CR_R_HOLD_M  0x70U        // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS3_CR_R_STROBE_S  7U
#define EMIF_ASYNC_CS3_CR_R_STROBE_M  0x1F80U      // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS3_CR_R_SETUP_S  13U
#define EMIF_ASYNC_CS3_CR_R_SETUP_M  0x1E000U     // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS3_CR_W_HOLD_S  17U
#define EMIF_ASYNC_CS3_CR_W_HOLD_M  0xE0000U     // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS3_CR_W_STROBE_S  20U
#define EMIF_ASYNC_CS3_CR_W_STROBE_M  0x3F00000U   // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS3_CR_W_SETUP_S  26U
#define EMIF_ASYNC_CS3_CR_W_SETUP_M  0x3C000000U  // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS3_CR_EW      0x40000000U  // Extend Wait mode.
#define EMIF_ASYNC_CS3_CR_SS      0x80000000U  // Select Strobe mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the ASYNC_CS4_CR register
//
//*****************************************************************************
#define EMIF_ASYNC_CS4_CR_ASIZE_S  0U
#define EMIF_ASYNC_CS4_CR_ASIZE_M  0x3U         // Asynchronous Memory Size.
#define EMIF_ASYNC_CS4_CR_TA_S    2U
#define EMIF_ASYNC_CS4_CR_TA_M    0xCU         // Turn Around cycles.
#define EMIF_ASYNC_CS4_CR_R_HOLD_S  4U
#define EMIF_ASYNC_CS4_CR_R_HOLD_M  0x70U        // Read Strobe Hold cycles.
#define EMIF_ASYNC_CS4_CR_R_STROBE_S  7U
#define EMIF_ASYNC_CS4_CR_R_STROBE_M  0x1F80U      // Read Strobe Duration cycles.
#define EMIF_ASYNC_CS4_CR_R_SETUP_S  13U
#define EMIF_ASYNC_CS4_CR_R_SETUP_M  0x1E000U     // Read Strobe Setup cycles.
#define EMIF_ASYNC_CS4_CR_W_HOLD_S  17U
#define EMIF_ASYNC_CS4_CR_W_HOLD_M  0xE0000U     // Write Strobe Hold cycles.
#define EMIF_ASYNC_CS4_CR_W_STROBE_S  20U
#define EMIF_ASYNC_CS4_CR_W_STROBE_M  0x3F00000U   // Write Strobe Duration cycles.
#define EMIF_ASYNC_CS4_CR_W_SETUP_S  26U
#define EMIF_ASYNC_CS4_CR_W_SETUP_M  0x3C000000U  // Write Strobe Setup cycles.
#define EMIF_ASYNC_CS4_CR_EW      0x40000000U  // Extend Wait mode.
#define EMIF_ASYNC_CS4_CR_SS      0x80000000U  // Select Strobe mode.

//*****************************************************************************
//
// The following are defines for the bit fields in the SDRAM_TR register
//
//*****************************************************************************
#define EMIF_SDRAM_TR_T_RRD_S     4U
#define EMIF_SDRAM_TR_T_RRD_M     0x70U        // Activate to Activate timing
                                               // for different bank.
#define EMIF_SDRAM_TR_T_RC_S      8U
#define EMIF_SDRAM_TR_T_RC_M      0xF00U       // Activate to Activate timing .
#define EMIF_SDRAM_TR_T_RAS_S     12U
#define EMIF_SDRAM_TR_T_RAS_M     0xF000U      // Activate to Precharge timing.
#define EMIF_SDRAM_TR_T_WR_S      16U
#define EMIF_SDRAM_TR_T_WR_M      0x70000U     // Last Write to Precharge
                                               // timing.
#define EMIF_SDRAM_TR_T_RCD_S     20U
#define EMIF_SDRAM_TR_T_RCD_M     0x700000U    // Activate to Read/Write
                                               // timing.
#define EMIF_SDRAM_TR_T_RP_S      24U
#define EMIF_SDRAM_TR_T_RP_M      0x7000000U   // Precharge to Activate/Refresh
                                               // timing.
#define EMIF_SDRAM_TR_T_RFC_S     27U
#define EMIF_SDRAM_TR_T_RFC_M     0xF8000000U  // Refresh/Load Mode to
                                               // Refresh/Activate timing

//*****************************************************************************
//
// The following are defines for the bit fields in the SDR_EXT_TMNG register
//
//*****************************************************************************
#define EMIF_SDR_EXT_TMNG_T_XS_S  0U
#define EMIF_SDR_EXT_TMNG_T_XS_M  0x1FU        // Self Refresh exit to new
                                               // command timing.

//*****************************************************************************
//
// The following are defines for the bit fields in the INT_RAW register
//
//*****************************************************************************
#define EMIF_INT_RAW_AT           0x1U         // Asynchronous Timeout.
#define EMIF_INT_RAW_LT           0x2U         // Line Trap.
#define EMIF_INT_RAW_WR_S         2U
#define EMIF_INT_RAW_WR_M         0x3CU        // Wait Rise.

//*****************************************************************************
//
// The following are defines for the bit fields in the INT_MSK register
//
//*****************************************************************************
#define EMIF_INT_MSK_AT_MASKED    0x1U         // Asynchronous Timeout.
#define EMIF_INT_MSK_LT_MASKED    0x2U         // Line Trap.
#define EMIF_INT_MSK_WR_MASKED_S  2U
#define EMIF_INT_MSK_WR_MASKED_M  0x3CU        // Wait Rise.

//*****************************************************************************
//
// The following are defines for the bit fields in the INT_MSK_SET register
//
//*****************************************************************************
#define EMIF_INT_MSK_SET_AT_MASK_SET  0x1U         // Asynchronous Timeout.
#define EMIF_INT_MSK_SET_LT_MASK_SET  0x2U         // Line Trap.
#define EMIF_INT_MSK_SET_WR_MASK_SET_S  2U
#define EMIF_INT_MSK_SET_WR_MASK_SET_M  0x3CU        // Wait Rise.

//*****************************************************************************
//
// The following are defines for the bit fields in the INT_MSK_CLR register
//
//*****************************************************************************
#define EMIF_INT_MSK_CLR_AT_MASK_CLR  0x1U         // Asynchronous Timeout.
#define EMIF_INT_MSK_CLR_LT_MASK_CLR  0x2U         // Line Trap.
#define EMIF_INT_MSK_CLR_WR_MASK_CLR_S  2U
#define EMIF_INT_MSK_CLR_WR_MASK_CLR_M  0x3CU        // Wait Rise.

//*****************************************************************************
//
// The following are defines for the bit fields in the IO_CTRL register
//
//*****************************************************************************
#define EMIF_IO_CTRL_IO_CTRL_S    0U
#define EMIF_IO_CTRL_IO_CTRL_M    0xFFFFU      // VTP calibration control for
                                               // the IOs

//*****************************************************************************
//
// The following are defines for the bit fields in the IO_STAT register
//
//*****************************************************************************
#define EMIF_IO_STAT_IO_STAT_S    0U
#define EMIF_IO_STAT_IO_STAT_M    0xFFFFU      // VTP calibration status of the
                                               // IOs

//*****************************************************************************
//
// The following are defines for the bit fields in the IODFT_TLECR register
//
//*****************************************************************************
#define EMIF_IODFT_TLECR_TLEC_S   0U
#define EMIF_IODFT_TLECR_TLEC_M   0xFFFFU      // Number of cycle that the MISR
                                               // signature will be
                                               // accumulated.

//*****************************************************************************
//
// The following are defines for the bit fields in the IODFT_TLGCR register
//
//*****************************************************************************
#define EMIF_IODFT_TLGCR_TM       0x1U         // Functional mode enable.
#define EMIF_IODFT_TLGCR_PATC_S   1U
#define EMIF_IODFT_TLGCR_PATC_M   0xEU         // Pattern Code.
#define EMIF_IODFT_TLGCR_MC_S     4U
#define EMIF_IODFT_TLGCR_MC_M     0x30U        // MISR State.
#define EMIF_IODFT_TLGCR_TOEN     0x40U        // Test output enable control.
#define EMIF_IODFT_TLGCR_ESEL     0x80U        // Output enable select.
#define EMIF_IODFT_TLGCR_MMS      0x100U       // Chooses the source of the
                                               // MISR input.
#define EMIF_IODFT_TLGCR_OPG_LD   0x1000U      // Load pattern generators
                                               // initial value.
#define EMIF_IODFT_TLGCR_ACT_CAP_EN  0x2000U      // Active cycles capture enable.
#define EMIF_IODFT_TLGCR_MT       0x4000U      // MISR on/off trigger command.

//*****************************************************************************
//
// The following are defines for the bit fields in the IODFT_TLAMR register
//
//*****************************************************************************
#define EMIF_IODFT_TLAMR_ADDR_TLMR_S  0U
#define EMIF_IODFT_TLAMR_ADDR_TLMR_M  0xFFFFFFFU   // MISR result for address.

//*****************************************************************************
//
// The following are defines for the bit fields in the IODFT_TLDCMR register
//
//*****************************************************************************
#define EMIF_IODFT_TLDCMR_DATA_TLMR_MSB_S  0U
#define EMIF_IODFT_TLDCMR_DATA_TLMR_MSB_M  0x7U         // MSB of MISR result for data
                                               // pins.
#define EMIF_IODFT_TLDCMR_DQM_TLMR_S  8U
#define EMIF_IODFT_TLDCMR_DQM_TLMR_M  0x3F00U      // MISR result for DQM pins.
#define EMIF_IODFT_TLDCMR_CTL_TLMR_S  16U
#define EMIF_IODFT_TLDCMR_CTL_TLMR_M  0x3FFF0000U  // MISR result for control pins.

//*****************************************************************************
//
// The following are defines for the bit fields in the MODEL_REL_NUM register
//
//*****************************************************************************
#define EMIF_MODEL_REL_NUM_RELEASE_NUM_S  0U
#define EMIF_MODEL_REL_NUM_RELEASE_NUM_M  0xFFU        // Release Number.
#endif
