//###########################################################################
//
// FILE:    hw_memmap.h
//
// TITLE:   Macros defining the memory map of the C28x.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_MEMMAP_H
#define HW_MEMMAP_H

//*****************************************************************************
//
// The following are defines for the base address of the memories and
// peripherals.
//
//*****************************************************************************
#define ADCARESULT_BASE             0x00000B00U // ADCA Result Registers
#define ADCCRESULT_BASE             0x00000B40U // ADCC Result Registers
#define CPUTIMER0_BASE              0x00000C00U // CPU Timer 0 Registers
#define CPUTIMER1_BASE              0x00000C08U // CPU Timer 1 Registers
#define CPUTIMER2_BASE              0x00000C10U // CPU Timer 2 Registers
#define PIECTRL_BASE                0x00000CE0U // PIE Registers
#define PIEVECTTABLE_BASE           0x00000D00U // PIE Vector Table
#define DMA_BASE                    0x00001000U // DMA Control Registers
#define DMA_CH1_BASE                0x00001020U // DMA Channel Registers
#define DMA_CH2_BASE                0x00001040U // DMA Channel Registers
#define DMA_CH3_BASE                0x00001060U // DMA Channel Registers
#define DMA_CH4_BASE                0x00001080U // DMA Channel Registers
#define DMA_CH5_BASE                0x000010A0U // DMA Channel Registers
#define DMA_CH6_BASE                0x000010C0U // DMA Channel Registers
#define EPWM1_BASE                  0x00004000U // EPWM1
#define EPWM2_BASE                  0x00004100U // EPWM2
#define EPWM3_BASE                  0x00004200U // EPWM3
#define EPWM4_BASE                  0x00004300U // EPWM4
#define EPWM5_BASE                  0x00004400U // EPWM5
#define EPWM6_BASE                  0x00004500U // EPWM6
#define EPWM7_BASE                  0x00004600U // EPWM7
#define EQEP1_BASE                  0x00005100U // EQEP1
#define EQEP2_BASE                  0x00005140U // EQEP2
#define ECAP1_BASE                  0x00005200U // ECAP1
#define ECAP2_BASE                  0x00005240U // ECAP2
#define ECAP3_BASE                  0x00005280U // ECAP3
#define HRCAP3_BASE                 0x000052A0U // HRCAP3
#define CMPSS1_BASE                 0x00005C80U // CMPSS1
#define CMPSS2_BASE                 0x00005CA0U // CMPSS2
#define CMPSS3_BASE                 0x00005CC0U // CMPSS3
#define CMPSS4_BASE                 0x00005CE0U // CMPSS4
#define SPIA_BASE                   0x00006100U // SPI A Registers
#define SPIB_BASE                   0x00006110U // SPI B Registers
#define BGCRC_BASE                  0x00006340U // BGCRC Registers
#define PMBUSA_BASE                 0x00006400U // PMBUS A Registers
#define HICAA_BASE                  0x00006500U // HICA-A Registers
#define FSITXA_BASE                 0x00006600U // FSITX Configuration Registers
#define FSIRXA_BASE                 0x00006680U // FSIRX Configuration Registers
#define LINA_BASE                   0x00006A00U // LIN A Registers
#define LINB_BASE                   0x00006B00U // LIN B Registers
#define WD_BASE                     0x00007000U // Watchdog Registers
#define NMI_BASE                    0x00007060U // NMI Registers
#define XINT_BASE                   0x00007070U // Interrupt Control Counter Registers
#define SCIA_BASE                   0x00007200U // SCI A Registers
#define I2CA_BASE                   0x00007300U // I2C A Registers
#define I2CB_BASE                   0x00007340U // I2C B Registers
#define ADCA_BASE                   0x00007400U // ADCA Configuration Registers
#define ADCC_BASE                   0x00007500U // ADCC Configuration Registers
#define INPUTXBAR_BASE              0x00007900U // Input XBAR1 (GPTRIP) Input Select Registers
#define XBAR_BASE                   0x00007920U // X-Bar Registers
#define SYNCSOC_BASE                0x00007940U // SYNC SOC registers
#define INPUTXBAR2_BASE             0x00007960U // Input XBAR2 (CLB), Configuration registers
#define DMACLASRCSEL_BASE           0x00007980U // DMA CLA Triggers Source Select Registers
#define EPWMXBAR_BASE               0x00007A00U // EPWM XBAR Configuration Registers
#define CLBXBAR_BASE                0x00007A40U // CLB XBAR Configuration Registers
#define OUTPUTXBAR_BASE             0x00007A80U // Output X-BAR1 Configuration Registers
#define OUTPUTXBAR2_BASE            0x00007BC0U // Output X-BAR2 (CLB) Configuration Registers
#define GPIOCTRL_BASE               0x00007C00U // GPIO 0 to 31 Mux A Configuration Registers
#define GPIODATA_BASE               0x00007F00U // GPIO 0 to 31 Mux A Data Registers
#define GPIODATAREAD_BASE           0x00007F80U // GPIO Data read registers
#define CANA_BASE                   0x00048000U // CAN-A Registers
#define CANA_MSG_RAM_BASE           0x00049000U // CAN-A Message RAM
#define DEVCFG_BASE                 0x0005D000U // Device Configuration Registers
#define CLKCFG_BASE                 0x0005D200U // Clock Configuration Registers
#define CPUSYS_BASE                 0x0005D300U // CPU System Configuration Registers
#define PERIPHAC_BASE               0x0005D500U // Peripheral Master Access Registers
#define ANALOGSUBSYS_BASE           0x0005D700U // Analog System Control Registers
#define HWBIST_BASE                 0x0005E000U // Self Test (HWBIST) Registers
#define PBIST_BASE                  0x0005E200U // PBIST Registers
#define USBTEST_BASE                0x0005E400U // USB Test Registers
#define VMT_SPARE_CTL_BASE          0x0005E600U // VMT and Spare Control Registers
#define DCC0_BASE                   0x0005E700U // DCC0 Registers
#define DCC1_BASE                   0x0005E740U // DCC1 Registers
#define ENHANCED_DEBUG_BASE         0x0005E800U // Enhanced Debug Registers
#define DCSMBANK0_Z1_BASE           0x0005F000U // Zone 1 DCSM Registers
#define DCSMBANK0_Z2_BASE           0x0005F040U // Zone 2 DCSM Registers
#define DCSMCOMMON_BASE             0x0005F070U // Security Registers
#define DCSMCOMMON2_BASE            0x0005F080U // Security Registers
#define MEMCFG_BASE                 0x0005F400U // Memory config registers
#define ACCESSPROTECTION_BASE       0x0005F500U // Access protection registers
#define MEMORYERROR_BASE            0x0005F540U // Memory error registers
#define FLASH0CTRL_BASE             0x0005F800U // Flash control registers
#define FLASH0ECC_BASE              0x0005FB00U // Flash ECC error log registers
#define DCSMBANK0_Z1OTP_BASE        0x00078000U // Zone 1 DCSM OTP
#define DCSMBANK0_Z2OTP_BASE        0x00078200U // Zone 2 DCSM OTP

#endif // HW_MEMMAP_H

