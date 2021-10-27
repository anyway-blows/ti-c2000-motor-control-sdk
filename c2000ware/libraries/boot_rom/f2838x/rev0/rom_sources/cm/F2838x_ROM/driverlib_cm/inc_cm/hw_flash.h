//###########################################################################
//
// FILE:    hw_flash.h
//
// TITLE:   Definitions for the FLASH registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_FLASH_H
#define HW_FLASH_H

//*****************************************************************************
//
// The following are defines for the FLASH register offsets
//
//*****************************************************************************
#define FLASH_O_FRDCNTL           0x0U         // Flash Read Control Register
#define FLASH_O_FSPRD             0x4U         // Flash Read Margin Control
                                               // Register
#define FLASH_O_FEDACSTATUS       0x1CU        // Flash Error Status Register
#define FLASH_O_FBBUSY            0x38U        // Bank Busy Register
#define FLASH_O_FBAC              0x3CU        // Flash Bank Access Control
                                               // Register
#define FLASH_O_FBFALLBACK        0x40U        // Flash Bank Fallback Power
                                               // Register
#define FLASH_O_FBPRDY            0x44U        // Flash Bank Pump Ready
                                               // Register
#define FLASH_O_FPAC1             0x48U        // Flash Pump Access Control
                                               // Register 1
#define FLASH_O_FPAC2             0x4CU        // Flash Pump Access Control
                                               // Register 2
#define FLASH_O_FMSTAT            0x54U        // Flash Module Status Register
#define FLASH_O_FEMU_DMSW         0x58U        // EEPROM Emulation Data MSW
                                               // Register
#define FLASH_O_FEMU_DLSW         0x5CU        // EEPROM Emulation Data LSW
                                               // Register
#define FLASH_O_FEMU_ECC          0x60U        // EEPROM Emulation ECC Register
#define FLASH_O_FLOCK             0x64U        // Flash Lock Register
#define FLASH_O_FEMU_ADDR         0x68U        // EEPROM Emulation Address
#define FLASH_O_FRD_INTF_CTRL_LOCK  0x2FCU       // Lock register for
                                               // FLASH_CTRL_REGS (Not including
                                               // FRD_INTF_CTRL_LOCK ).
#define FLASH_O_FRD_INTF_CTRL     0x300U       // Flash Read Interface Control
                                               // Register
#define FLASH_O_ECC_ENABLE        0x0U         // ECC Enable
#define FLASH_O_SINGLE_ERR_ADDR_LOW  0x4U         // Single Error Address Low
#define FLASH_O_SINGLE_ERR_ADDR_HIGH  0x8U         // Single Error Address High
#define FLASH_O_UNC_ERR_ADDR_LOW  0xCU         // Uncorrectable Error Address
                                               // Low
#define FLASH_O_UNC_ERR_ADDR_HIGH  0x10U        // Uncorrectable Error Address
                                               // High
#define FLASH_O_ERR_STATUS        0x14U        // Error Status
#define FLASH_O_ERR_POS           0x18U        // Error Position
#define FLASH_O_ERR_STATUS_CLR    0x1CU        // Error Status Clear
#define FLASH_O_ERR_CNT           0x20U        // Error Control
#define FLASH_O_ERR_THRESHOLD     0x24U        // Error Threshold
#define FLASH_O_ERR_INTFLG        0x28U        // Error Interrupt Flag
#define FLASH_O_ERR_INTCLR        0x2CU        // Error Interrupt Flag Clear
#define FLASH_O_FDATAH_TEST       0x30U        // Data High Test
#define FLASH_O_FDATAL_TEST       0x34U        // Data Low Test
#define FLASH_O_FADDR_TEST        0x38U        // ECC Test Address
#define FLASH_O_FECC_TEST         0x3CU        // ECC Test Address
#define FLASH_O_FECC_CTRL         0x40U        // ECC Control
#define FLASH_O_FOUTH_TEST        0x44U        // Test Data Out High
#define FLASH_O_FOUTL_TEST        0x48U        // Test Data Out Low
#define FLASH_O_FECC_STATUS       0x4CU        // ECC Status
#define FLASH_O_ECC_REGS_LOCK     0x7CU        // Lock register for
                                               // FLASH_ECC_REGS (Not including
                                               // FLASH_ECC_REGS_LOCK ).

//*****************************************************************************
//
// The following are defines for the bit fields in the FRDCNTL register
//
//*****************************************************************************
#define FLASH_FRDCNTL_RWAIT_S     8U
#define FLASH_FRDCNTL_RWAIT_M     0xF00U       // Random Read Waitstate

//*****************************************************************************
//
// The following are defines for the bit fields in the FSPRD register
//
//*****************************************************************************
#define FLASH_FSPRD_RM0           0x1U         // ReadMargin0 Configuration
#define FLASH_FSPRD_RM1           0x2U         // Readmargin1 Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the FEDACSTATUS register
//
//*****************************************************************************
#define FLASH_FEDACSTATUS_FSM_DONE  0x1000000U   // FSM_DONE : FSM is finished
#define FLASH_FEDACSTATUS_RVF_EVT  0x2000000U   // FSM command Read_verify
                                               // failed error event

//*****************************************************************************
//
// The following are defines for the bit fields in the FBBUSY register
//
//*****************************************************************************
#define FLASH_FBBUSY_BUSY         0x1U         // Bank Busy

//*****************************************************************************
//
// The following are defines for the bit fields in the FBAC register
//
//*****************************************************************************
#define FLASH_FBAC_VREADST_S      0U
#define FLASH_FBAC_VREADST_M      0xFFU        // VREAD Setup Time Count
#define FLASH_FBAC_BAGP_S         8U
#define FLASH_FBAC_BAGP_M         0xFF00U      // Bank Active Grace Period

//*****************************************************************************
//
// The following are defines for the bit fields in the FBFALLBACK register
//
//*****************************************************************************
#define FLASH_FBFALLBACK_BNKPWR0_S  0U
#define FLASH_FBFALLBACK_BNKPWR0_M  0x3U         // Bank Power Mode of BANK0

//*****************************************************************************
//
// The following are defines for the bit fields in the FBPRDY register
//
//*****************************************************************************
#define FLASH_FBPRDY_BANKRDY      0x1U         // Flash Bank Active Power State
#define FLASH_FBPRDY_PUMPRDY      0x8000U      // Flash Pump Active Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the FPAC1 register
//
//*****************************************************************************
#define FLASH_FPAC1_PMPPWR        0x1U         // Charge Pump Fallback Power
                                               // Mode
#define FLASH_FPAC1_PSLEEP_S      16U
#define FLASH_FPAC1_PSLEEP_M      0xFFF0000U   // Pump Sleep Down Count

//*****************************************************************************
//
// The following are defines for the bit fields in the FPAC2 register
//
//*****************************************************************************
#define FLASH_FPAC2_PAGP_S        0U
#define FLASH_FPAC2_PAGP_M        0xFFFFU      // Pump Active Grace Period

//*****************************************************************************
//
// The following are defines for the bit fields in the FMSTAT register
//
//*****************************************************************************
#define FLASH_FMSTAT_SLOCK        0x1U         // Sector Lock Status.
#define FLASH_FMSTAT_PSUSP        0x2U         // Program Suspend.
#define FLASH_FMSTAT_ESUSP        0x4U         // Erase Suspend.
#define FLASH_FMSTAT_VOLTSTAT     0x8U         // Core Voltage Status.
#define FLASH_FMSTAT_CSTAT        0x10U        // Command Status.
#define FLASH_FMSTAT_INVDAT       0x20U        // Invalid Data.
#define FLASH_FMSTAT_PGM          0x40U        // Program Active.
#define FLASH_FMSTAT_ERS          0x80U        // Erase Active.
#define FLASH_FMSTAT_BUSY         0x100U       // Busy Bit.
#define FLASH_FMSTAT_CV           0x200U       // Compact Verify
#define FLASH_FMSTAT_EV           0x400U       // Erase verify
#define FLASH_FMSTAT_PCV          0x800U       // Precondition verify
#define FLASH_FMSTAT_PGV          0x1000U      // Program verify
#define FLASH_FMSTAT_DBF          0x2000U      // Disturbance Test Fail
#define FLASH_FMSTAT_ILA          0x4000U      // Illegal Address
#define FLASH_FMSTAT_RVF          0x8000U      // Read Verify Failure
#define FLASH_FMSTAT_RDVER        0x10000U     // Read verify command currently
                                               // underway.
#define FLASH_FMSTAT_RVSUSP       0x20000U     // Read Verify Suspend.

//*****************************************************************************
//
// The following are defines for the bit fields in the FEMU_ECC register
//
//*****************************************************************************
#define FLASH_FEMU_ECC_EMU_ECC_S  0U
#define FLASH_FEMU_ECC_EMU_ECC_M  0xFFU        // EEPROM Emulation ECC Register

//*****************************************************************************
//
// The following are defines for the bit fields in the FLOCK register
//
//*****************************************************************************
#define FLASH_FLOCK_ENCOM_S       16U
#define FLASH_FLOCK_ENCOM_M       0xFFFF0000U  // Enable Command Registers

//*****************************************************************************
//
// The following are defines for the bit fields in the FEMU_ADDR register
//
//*****************************************************************************
#define FLASH_FEMU_ADDR_EMU_ADDR_S  3U
#define FLASH_FEMU_ADDR_EMU_ADDR_M  0x3FFFF8U    // EEPROM Emulation Address

//*****************************************************************************
//
// The following are defines for the bit fields in the FRD_INTF_CTRL register
//
//*****************************************************************************
#define FLASH_FRD_INTF_CTRL_PROG_CACHE_EN  0x1U         // Program Cache Enable
#define FLASH_FRD_INTF_CTRL_DATA_CACHE_EN  0x2U         // Data Cache Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ECC_ENABLE register
//
//*****************************************************************************
#define FLASH_ECC_ENABLE_ENABLE_S  0U
#define FLASH_ECC_ENABLE_ENABLE_M  0xFU         // Enable ECC

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_STATUS register
//
//*****************************************************************************
#define FLASH_ERR_STATUS_FAIL_0_L  0x1U         // Lower 64bits Single Bit Error
                                               // Corrected Value 0
#define FLASH_ERR_STATUS_FAIL_1_L  0x2U         // Lower 64bits Single Bit Error
                                               // Corrected Value 1
#define FLASH_ERR_STATUS_UNC_ERR_L  0x4U         // Lower 64 bits Uncorrectable
                                               // error occurred
#define FLASH_ERR_STATUS_FAIL_0_H  0x10000U     // Upper 64bits Single Bit Error
                                               // Corrected Value 0
#define FLASH_ERR_STATUS_FAIL_1_H  0x20000U     // Upper 64bits Single Bit Error
                                               // Corrected Value 1
#define FLASH_ERR_STATUS_UNC_ERR_H  0x40000U     // Upper 64 bits Uncorrectable
                                               // error occurred

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_POS register
//
//*****************************************************************************
#define FLASH_ERR_POS_ERR_POS_L_S  0U
#define FLASH_ERR_POS_ERR_POS_L_M  0x3FU        // Bit Position of Single bit
                                               // Error in lower 64 bits
#define FLASH_ERR_POS_ERR_TYPE_L  0x100U       // Error Type in lower 64 bits
#define FLASH_ERR_POS_ERR_POS_H_S  16U
#define FLASH_ERR_POS_ERR_POS_H_M  0x3F0000U    // Bit Position of Single bit
                                               // Error in upper 64 bits
#define FLASH_ERR_POS_ERR_TYPE_H  0x1000000U   // Error Type in upper 64 bits

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_STATUS_CLR register
//
//*****************************************************************************
#define FLASH_ERR_STATUS_CLR_FAIL_0_L_CLR  0x1U         // Lower 64bits Single Bit Error
                                               // Corrected Value 0 Clear
#define FLASH_ERR_STATUS_CLR_FAIL_1_L_CLR  0x2U         // Lower 64bits Single Bit Error
                                               // Corrected Value 1 Clear
#define FLASH_ERR_STATUS_CLR_UNC_ERR_L_CLR  0x4U         // Lower 64 bits Uncorrectable
                                               // error occurred Clear
#define FLASH_ERR_STATUS_CLR_FAIL_0_H_CLR  0x10000U     // Upper 64bits Single Bit Error
                                               // Corrected Value 0 Clear
#define FLASH_ERR_STATUS_CLR_FAIL_1_H_CLR  0x20000U     // Upper 64bits Single Bit Error
                                               // Corrected Value 1 Clear
#define FLASH_ERR_STATUS_CLR_UNC_ERR_H_CLR  0x40000U     // Upper 64 bits Uncorrectable
                                               // error occurred Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_CNT register
//
//*****************************************************************************
#define FLASH_ERR_CNT_ERR_CNT_S   0U
#define FLASH_ERR_CNT_ERR_CNT_M   0xFFFFU      // Error counter

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_THRESHOLD register
//
//*****************************************************************************
#define FLASH_ERR_THRESHOLD_ERR_THRESHOLD_S  0U
#define FLASH_ERR_THRESHOLD_ERR_THRESHOLD_M  0xFFFFU      // Error Threshold

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_INTFLG register
//
//*****************************************************************************
#define FLASH_ERR_INTFLG_SINGLE_ERR_INTFLG  0x1U         // Single Error Interrupt Flag
#define FLASH_ERR_INTFLG_UNC_ERR_INTFLG  0x2U         // Uncorrectable Interrupt Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the ERR_INTCLR register
//
//*****************************************************************************
#define FLASH_ERR_INTCLR_SINGLE_ERR_INTCLR  0x1U         // Single Error Interrupt Flag
                                               // Clear
#define FLASH_ERR_INTCLR_UNC_ERR_INTCLR  0x2U         // Uncorrectable Interrupt Flag
                                               // Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the FADDR_TEST register
//
//*****************************************************************************
#define FLASH_FADDR_TEST_ADDRL_S  3U
#define FLASH_FADDR_TEST_ADDRL_M  0xFFF8U      // ECC Address Low
#define FLASH_FADDR_TEST_ADDRH_S  16U
#define FLASH_FADDR_TEST_ADDRH_M  0x3F0000U    // ECC Address High

//*****************************************************************************
//
// The following are defines for the bit fields in the FECC_TEST register
//
//*****************************************************************************
#define FLASH_FECC_TEST_ECC_S     0U
#define FLASH_FECC_TEST_ECC_M     0xFFU        // ECC Control Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the FECC_CTRL register
//
//*****************************************************************************
#define FLASH_FECC_CTRL_ECC_TEST_EN  0x1U         // Enable ECC Test Logic
#define FLASH_FECC_CTRL_ECC_SELECT  0x2U         // ECC Bit Select
#define FLASH_FECC_CTRL_DO_ECC_CALC  0x4U         // Enable ECC Calculation

//*****************************************************************************
//
// The following are defines for the bit fields in the FECC_STATUS register
//
//*****************************************************************************
#define FLASH_FECC_STATUS_SINGLE_ERR  0x1U         // Test Result is Single Bit
                                               // Error
#define FLASH_FECC_STATUS_UNC_ERR  0x2U         // Test Result is Uncorrectable
                                               // Error
#define FLASH_FECC_STATUS_DATA_ERR_POS_S  2U
#define FLASH_FECC_STATUS_DATA_ERR_POS_M  0xFCU        // Holds Bit Position of Error
#define FLASH_FECC_STATUS_ERR_TYPE  0x100U       // Holds Bit Position of 8 Check
                                               // Bits of Error
#endif
