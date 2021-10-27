//###########################################################################
//
// FILE:    hw_asysctl.h
//
// TITLE:   Definitions for the ASYSCTL registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_ASYSCTL_H
#define HW_ASYSCTL_H

//*****************************************************************************
//
// The following are defines for the ASYSCTL register offsets
//
//*****************************************************************************
#define ASYSCTL_O_INTOSC1TRIM     0x0U         // Internal Oscillator 1 Trim
                                               // Register
#define ASYSCTL_O_INTOSC2TRIM     0x2U         // Internal Oscillator 2 Trim
                                               // Register
#define ASYSCTL_O_PUMPOSCTRIM     0x4U         // Internal Oscillator 3 Trim
                                               // Register
#define ASYSCTL_O_OSCREFTRIM      0x6U         // Internal Oscillator Reference
                                               // Trim Register
#define ASYSCTL_O_ANAREFTRIMA     0x8U         // Analog Reference Trim A
                                               // Register
#define ASYSCTL_O_PMMVMONTRIM     0xEU         // Power Management Module VMON
                                               // Trim Register
#define ASYSCTL_O_PMMVMONTRIM2    0x10U        // Power Management Module VMON
                                               // Trim Register
#define ASYSCTL_O_PMMREFTRIM      0x12U        // Power Management Module
                                               // Reference Trim Register
#define ASYSCTL_O_PMMVREGTRIM     0x14U        // Power Management Module VREG
                                               // Trim Register
#define ASYSCTL_O_LPPUMPTRIM      0x1CU        // PUMP trim register
#define ASYSCTL_O_REFSYSTRIM      0x1DU        // REFSYS module reference trim
#define ASYSCTL_O_TRIMLOCK        0x2EU        // Lock Register for all the
                                               // TRIM Registers
#define ASYSCTL_O_INTOSCCSR1      0x30U        // Internal Oscillator  Control
                                               // and Status Register 1
#define ASYSCTL_O_INTOSCCSR2      0x31U        // Internal Oscillator  Control
                                               // and Status Register 2
#define ASYSCTL_O_INTOSCCSR3      0x32U        // Internal Oscillator Control
                                               // and Status Register 3
#define ASYSCTL_O_TSNSCFG         0x33U        // Temperature Sensor Config
                                               // Register
#define ASYSCTL_O_REFCONFIGA      0x34U        // Config register for analog
                                               // reference A.
#define ASYSCTL_O_REFCONFIGC      0x38U        // Config register for analog
                                               // reference C.
#define ASYSCTL_O_PMMCSR          0x3AU        // Power Management Module
                                               // Control and Status Register
#define ASYSCTL_O_PMMCONFIGDEBUG0  0x3CU        // PMM Config Register 0 used
                                               // for debug.
#define ASYSCTL_O_PMMCONFIGDFT    0x44U        // PMM Config Register  used for
                                               // test.
#define ASYSCTL_O_REFSYSCSR       0x48U        // REFSYS CSR register.
#define ASYSCTL_O_INTERNALTESTCTL  0x4AU        // INTERNALTEST Node Control
                                               // Register
#define ASYSCTL_O_PUMPVREADCT     0x4CU        // Pump VREADCT register
#define ASYSCTL_O_PUMPVCG2P5CT    0x4DU        // Pump VCG2P5CT register
#define ASYSCTL_O_CONFIGLOCK      0x5EU        // Lock Register for all the
                                               // config registers.
#define ASYSCTL_O_TSNSCTL         0x60U        // Temperature Sensor Control
                                               // Register
#define ASYSCTL_O_ANAREFCTL       0x68U        // Analog Reference Control
                                               // Register
#define ASYSCTL_O_VREGCTL         0x6AU        // Voltage Regulator Control
                                               // Register
#define ASYSCTL_O_VMONCTL         0x70U        // Voltage Monitor Control
                                               // Register
#define ASYSCTL_O_VMONSTS         0x71U        // Voltage Monitor Status
                                               // Register
#define ASYSCTL_O_VMONCLR         0x72U        // Voltage Monitor Clear
                                               // Register
#define ASYSCTL_O_CMPHPMXSEL      0x82U        // Bits to select one of the
                                               // many sources on CopmHP inputs.
                                               // Refer to Pimux diagram for
                                               // details.
#define ASYSCTL_O_CMPLPMXSEL      0x84U        // Bits to select one of the
                                               // many sources on CopmLP inputs.
                                               // Refer to Pimux diagram for
                                               // details.
#define ASYSCTL_O_CMPHNMXSEL      0x86U        // Bits to select one of the
                                               // many sources on CopmHN inputs.
                                               // Refer to Pimux diagram for
                                               // details.
#define ASYSCTL_O_CMPLNMXSEL      0x87U        // Bits to select one of the
                                               // many sources on CopmLN inputs.
                                               // Refer to Pimux diagram for
                                               // details.
#define ASYSCTL_O_ADCDACLOOPBACK  0x88U        // Enabble loopback from DAC to
                                               // ADCs
#define ASYSCTL_O_LOCK            0x8EU        // Lock Register
#define ASYSCTL_O_SYSAPLLCONFIG1  0xC0U        // APLL Module (SYSPLL Instance)
                                               // CONFIG1 Register
#define ASYSCTL_O_SYSAPLLCONFIG2  0xC1U        // APLL Module (SYSPLL Instance)
                                               // CONFIG2 Register
#define ASYSCTL_O_SYSAPLLCONFIG3  0xC2U        // APLL Module (SYSPLL Instance)
                                               // CONFIG3 Register
#define ASYSCTL_O_SYSAPLLCONFIG4  0xC3U        // APLL Module (SYSPLL Instance)
                                               // CONFIG4 Register
#define ASYSCTL_O_SYSAPLLCONFIG5  0xC4U        // APLL Module (SYSPLL Instance)
                                               // CONFIG5 Register
#define ASYSCTL_O_SYSAPLLCONFIG7  0xC6U        // APLL Module (SYSPLL Instance)
                                               // CONFIG7 Register
#define ASYSCTL_O_SYSAPLLCONFIG8  0xC7U        // APLL Module (SYSPLL Instance)
                                               // CONFIG8 Register
#define ASYSCTL_O_SYSAPLLCONFIG9  0xC8U        // APLL Module (SYSPLL Instance)
                                               // CONFIG9 Register
#define ASYSCTL_O_SYSAPLLCONFIG10  0xC9U        // APLL Module (SYSPLL Instance)
                                               // CONFIG10 Register
#define ASYSCTL_O_SYSAPLLDIGSTATUS1  0xCAU        // APLL Module (SYSPLL Instance)
                                               // DIGSTATUS1 Register
#define ASYSCTL_O_SYSAPLLDIGSTATUS2  0xCCU        // APLL Module (SYSPLL Instance)
                                               // DIGSTATUS2 Register
#define ASYSCTL_O_SYSAPLLDIGSTATUS3  0xCEU        // APLL Module (SYSPLL Instance)
                                               // DIGSTATUS3 Register
#define ASYSCTL_O_SYSAPLLSPAREIN  0xD6U        // APLL Module (SYSPLL Instance)
                                               // SPAREIN Register
#define ASYSCTL_O_SYSAPLLSPAREOUT  0xD7U        // APLL Module (SYSPLL Instance)
                                               // SPAREOUT Register
#define ASYSCTL_O_APLLREFTRIM     0xF0U        // APLL Module Reference Trim
                                               // Register
#define ASYSCTL_O_SYSAPLLLDOTRIM  0xF1U        // APLL Module (SYSPLL) LDO
                                               // trim.
#define ASYSCTL_O_SYSAPLLOSCTRIM  0xF2U        // APLL Module (SYSPLL) OSC
                                               // trim.
#define ASYSCTL_O_APLLREFCONFIG   0xF8U        // APLL Module Reference Config
                                               // Register
#define ASYSCTL_O_APLLLOCK        0xFEU        // APLL Config and Trim Lock
                                               // Register.

//*****************************************************************************
//
// The following are defines for the bit fields in the INTOSC1TRIM register
//
//*****************************************************************************
#define ASYSCTL_INTOSC1TRIM_VALFINETRIM_S  0U
#define ASYSCTL_INTOSC1TRIM_VALFINETRIM_M  0xFFFU       // Oscillator Value Fine Trim
                                               // Bits
#define ASYSCTL_INTOSC1TRIM_SLOPETRIM_S  16U
#define ASYSCTL_INTOSC1TRIM_SLOPETRIM_M  0xFF0000U    // Oscillator Slope Trim Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the INTOSC2TRIM register
//
//*****************************************************************************
#define ASYSCTL_INTOSC2TRIM_VALFINETRIM_S  0U
#define ASYSCTL_INTOSC2TRIM_VALFINETRIM_M  0xFFFU       // Oscillator Value Fine Trim
                                               // Bits
#define ASYSCTL_INTOSC2TRIM_SLOPETRIM_S  16U
#define ASYSCTL_INTOSC2TRIM_SLOPETRIM_M  0xFF0000U    // Oscillator Slope Trim Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the PUMPOSCTRIM register
//
//*****************************************************************************
#define ASYSCTL_PUMPOSCTRIM_VALFINETRIM_S  0U
#define ASYSCTL_PUMPOSCTRIM_VALFINETRIM_M  0xFFFU       // Oscillator Value Fine Trim
                                               // Bits
#define ASYSCTL_PUMPOSCTRIM_SLOPETRIM_S  16U
#define ASYSCTL_PUMPOSCTRIM_SLOPETRIM_M  0xFF0000U    // Oscillator Slope Trim Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the OSCREFTRIM register
//
//*****************************************************************************
#define ASYSCTL_OSCREFTRIM_BGVALTRIM_S  0U
#define ASYSCTL_OSCREFTRIM_BGVALTRIM_M  0x3FU        // Bandgap Value Trim
#define ASYSCTL_OSCREFTRIM_BGSLOPETRIM_S  6U
#define ASYSCTL_OSCREFTRIM_BGSLOPETRIM_M  0x7C0U       // Bandgap Slope Trim
#define ASYSCTL_OSCREFTRIM_IREFTRIM_S  11U
#define ASYSCTL_OSCREFTRIM_IREFTRIM_M  0xF800U      // Reference Current Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the ANAREFTRIMA register
//
//*****************************************************************************
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM_S  0U
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM_M  0x3FU        // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM_S  6U
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM_M  0x7C0U       // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMA_IREFTRIM_S  11U
#define ASYSCTL_ANAREFTRIMA_IREFTRIM_M  0xF800U      // Reference Current Trim
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM2P5_S  16U
#define ASYSCTL_ANAREFTRIMA_BGVALTRIM2P5_M  0x3F0000U    // Bandgap Value Trim
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM2P5_S  22U
#define ASYSCTL_ANAREFTRIMA_BGSLOPETRIM2P5_M  0x7C00000U   // Bandgap Slope Trim
#define ASYSCTL_ANAREFTRIMA_IREFTRIM2P5_S  27U
#define ASYSCTL_ANAREFTRIMA_IREFTRIM2P5_M  0xF8000000U  // Reference Current Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMVMONTRIM register
//
//*****************************************************************************
#define ASYSCTL_PMMVMONTRIM_VDDIOMONBORLTRIM_S  0U
#define ASYSCTL_PMMVMONTRIM_VDDIOMONBORLTRIM_M  0xFU         // VDDIO 3.3V Monitor Trim
#define ASYSCTL_PMMVMONTRIM_VDDIOMONBORHTRIM_S  4U
#define ASYSCTL_PMMVMONTRIM_VDDIOMONBORHTRIM_M  0x30U        // VDDIO 3.3V Over Voltage
                                               // Monitor Trim
#define ASYSCTL_PMMVMONTRIM_VDDIOMONPORTRIM_S  6U
#define ASYSCTL_PMMVMONTRIM_VDDIOMONPORTRIM_M  0xC0U        // Trim bits for trimming 3.3V
                                               // POR threshold
#define ASYSCTL_PMMVMONTRIM_VDDINTVREGMONTRIM_S  8U
#define ASYSCTL_PMMVMONTRIM_VDDINTVREGMONTRIM_M  0xF00U       // VDD 1.2V Monitor Trim for
                                               // internal VREG.
#define ASYSCTL_PMMVMONTRIM_VDDEXTVREGMONTRIM_S  12U
#define ASYSCTL_PMMVMONTRIM_VDDEXTVREGMONTRIM_M  0xF000U      // VDD 1.2V Monitor Trim forn
                                               // external VREG.
#define ASYSCTL_PMMVMONTRIM_PORBORFILTERTRIM_S  16U
#define ASYSCTL_PMMVMONTRIM_PORBORFILTERTRIM_M  0x70000U     // Trim Bits for the POR/BOR
                                               // filter
#define ASYSCTL_PMMVMONTRIM_PORBORFILTERTRIM_RANGE_SELECT  0x80000U     // Power Supply glitch
                                               // sensititivity trim.
#define ASYSCTL_PMMVMONTRIM_PMMSPAREPROG_S  20U
#define ASYSCTL_PMMVMONTRIM_PMMSPAREPROG_M  0xF00000U    // Spare For Future use

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMVMONTRIM2 register
//
//*****************************************************************************
#define ASYSCTL_PMMVMONTRIM2_BORHINTVREGTRIM_S  0U
#define ASYSCTL_PMMVMONTRIM2_BORHINTVREGTRIM_M  0xFU         // VDD 1.2V BORH coarse Trim
                                               // when for internal VREG
#define ASYSCTL_PMMVMONTRIM2_BORHEXTVREGTRIM_S  4U
#define ASYSCTL_PMMVMONTRIM2_BORHEXTVREGTRIM_M  0xF0U        // VDD 1.2V BORH coarse Trim for
                                               // external VREG
#define ASYSCTL_PMMVMONTRIM2_BORHFINEVREGTRIM_S  8U
#define ASYSCTL_PMMVMONTRIM2_BORHFINEVREGTRIM_M  0xF00U       // VDD 1.2V BORH fine Trim.

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMREFTRIM register
//
//*****************************************************************************
#define ASYSCTL_PMMREFTRIM_BGVALTRIM_S  0U
#define ASYSCTL_PMMREFTRIM_BGVALTRIM_M  0x3FU        // Bandgap Value Trim
#define ASYSCTL_PMMREFTRIM_BGSLOPETRIM_S  6U
#define ASYSCTL_PMMREFTRIM_BGSLOPETRIM_M  0x7C0U       // Bandgap Slope Trim
#define ASYSCTL_PMMREFTRIM_IREFTRIM_S  11U
#define ASYSCTL_PMMREFTRIM_IREFTRIM_M  0xF800U      // Reference Current Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMVREGTRIM register
//
//*****************************************************************************
#define ASYSCTL_PMMVREGTRIM_VREGTRIM_S  0U
#define ASYSCTL_PMMVREGTRIM_VREGTRIM_M  0xFFU        // Core VDD Voltage Regulator
                                               // Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the LPPUMPTRIM register
//
//*****************************************************************************
#define ASYSCTL_LPPUMPTRIM_TRIM0P8_S  0U
#define ASYSCTL_LPPUMPTRIM_TRIM0P8_M  0xFU         // TRIM0P8 setting for LPPUMP
#define ASYSCTL_LPPUMPTRIM_TRIM3P4_S  4U
#define ASYSCTL_LPPUMPTRIM_TRIM3P4_M  0xF0U        // TRIM3P4setting for LPPUMP
#define ASYSCTL_LPPUMPTRIM_TRIM1P7_S  8U
#define ASYSCTL_LPPUMPTRIM_TRIM1P7_M  0x300U       // TRIM1P7 setting for LPPUMP

//*****************************************************************************
//
// The following are defines for the bit fields in the REFSYSTRIM register
//
//*****************************************************************************
#define ASYSCTL_REFSYSTRIM_BGVALTRIM_S  0U
#define ASYSCTL_REFSYSTRIM_BGVALTRIM_M  0x3FU        // Bandgap Value Trim
#define ASYSCTL_REFSYSTRIM_BGSLOPETRIM_S  6U
#define ASYSCTL_REFSYSTRIM_BGSLOPETRIM_M  0x7C0U       // Bandgap Slope Trim
#define ASYSCTL_REFSYSTRIM_IREFTRIM_S  11U
#define ASYSCTL_REFSYSTRIM_IREFTRIM_M  0xF800U      // Reference Current Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the TRIMLOCK register
//
//*****************************************************************************
#define ASYSCTL_TRIMLOCK_INTOSC1TRIM  0x1U         // INTOSC1TRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_INTOSC2TRIM  0x2U         // INTOSC2TRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_PUMPOSCTRIM  0x4U         // PUMPOSCTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_OSCREFTRIM  0x8U         // OSCREFTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_ANAREFTRIMA  0x10U        // ANAREFTRIMA Register Lock bit
#define ASYSCTL_TRIMLOCK_ANAREFTRIMC  0x40U        // ANAREFTRIMC Register Lock bit
#define ASYSCTL_TRIMLOCK_PMMVMONTRIM  0x80U        // PMMVMONTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_PMMVMONTRIM2  0x100U       // PMMVMONTRIM2 Register Lock
                                               // bit
#define ASYSCTL_TRIMLOCK_PMMREFTRIM  0x200U       // PMMREFTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_PMMVREGTRIM  0x400U       // PMMVREGTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_LPPUMPTRIM  0x2000U      // LPPUMPTRIM Register Lock bit
#define ASYSCTL_TRIMLOCK_REFSYSTRIM  0x4000U      // REFSYSTRIM Register Lock bit

//*****************************************************************************
//
// The following are defines for the bit fields in the INTOSCCSR1 register
//
//*****************************************************************************
#define ASYSCTL_INTOSCCSR1_OSCC_FREQ_PROG_1  0x1U         // OSCC_48MHZ freq prog.
#define ASYSCTL_INTOSCCSR1_OSCC_FREQ_PROG_2  0x2U         // OSCC_48MHZ freq prog.
#define ASYSCTL_INTOSCCSR1_OSCC_FREQ_PROG_3  0x4U         // OSCC_48MHZ freq prog.
#define ASYSCTL_INTOSCCSR1_OSC_AB_FREQ_PROG_1  0x8U         // OSCA/B Frequency 16MHz mode
#define ASYSCTL_INTOSCCSR1_OSC_AB_FREQ_PROG_2  0x10U        // OSCA/B Frequency 10MHz mode
#define ASYSCTL_INTOSCCSR1_MEAS_PTAT_HI  0x20U        // MEASURE_PTAT_HI
#define ASYSCTL_INTOSCCSR1_MEAS_PTAT_LO  0x40U        // MEASURE_PTAT_LO
#define ASYSCTL_INTOSCCSR1_BG_ITRIM  0x80U        // BANDGAP IREFTRIM
#define ASYSCTL_INTOSCCSR1_BG_VTRIM  0x100U       // BANDGAP VREFTRIM
#define ASYSCTL_INTOSCCSR1_DISABLE_OSC_SLOPETRIM  0x200U       // DISABLE SLOPETRIM OSCA/B/C
#define ASYSCTL_INTOSCCSR1_IGEN_BIAS_PROG  0x400U       // V2IAMP_BIAS_CURRENT_PROG for
                                               // OSCA/B/C
#define ASYSCTL_INTOSCCSR1_COMP_BIAS_PROG  0x800U       // COMPARATOR_BIAS_CURRENT_PROG
                                               // for OSCA/B/C

//*****************************************************************************
//
// The following are defines for the bit fields in the INTOSCCSR2 register
//
//*****************************************************************************
#define ASYSCTL_INTOSCCSR2_DISABLE_PSR_FILT  0x1U         // DISABLE_PSR_FILTER for 
                                               // OSCA/B/C
#define ASYSCTL_INTOSCCSR2_SEL_OSCA  0x2U         // Select OSCA for measurements
#define ASYSCTL_INTOSCCSR2_SEL_OSCB  0x4U         // Select OSCB for measurements
#define ASYSCTL_INTOSCCSR2_SEL_OSCC  0x8U         // Select OSCC for measurements
#define ASYSCTL_INTOSCCSR2_MEAS_VFB_HI  0x10U        // Measure VFB_HI
#define ASYSCTL_INTOSCCSR2_MEAS_VFB_LO  0x20U        // Measure VFB_LO
#define ASYSCTL_INTOSCCSR2_MEAS_AMPOUT  0x40U        // Measure V2IAMP_VOUT
#define ASYSCTL_INTOSCCSR2_MEAS_ICHARGE  0x80U        // Bring charging current to Pad
#define ASYSCTL_INTOSCCSR2_EN_DUTY  0x100U       // ENABLE_DUTYCYCLE_MEAS_48MHZ
#define ASYSCTL_INTOSCCSR2_EN_CLKZ  0x200U       // Enable inversion of 48MHz
                                               // clock output

//*****************************************************************************
//
// The following are defines for the bit fields in the INTOSCCSR3 register
//
//*****************************************************************************
#define ASYSCTL_INTOSCCSR3_OSCC_TC_CORR  0x1U         // ADD_TC_CORR_CAP_OSCC
#define ASYSCTL_INTOSCCSR3_OSCA_PDN  0x2U         // Power down OSCA
#define ASYSCTL_INTOSCCSR3_OSCB_PDN  0x4U         // Power down OSCB
#define ASYSCTL_INTOSCCSR3_OSCC_PDN  0x8U         // Power down OSCC
#define ASYSCTL_INTOSCCSR3_MEASZTCRTOP  0x10U        // MEASURE_ZTCR_TOP
#define ASYSCTL_INTOSCCSR3_MEASZTCRGND  0x20U        // MEASURE_ZTCR_GND
#define ASYSCTL_INTOSCCSR3_DISABLE_CASCODE  0x40U        // DISABLE_CASCODE_IOUT for
                                               // OSCA/B/C
#define ASYSCTL_INTOSCCSR3_OSC_TC2_CORR_1  0x80U        // OSC_TC2_CORRECTION
#define ASYSCTL_INTOSCCSR3_OSC_TC2_CORR_2  0x100U       // OSC_TC2_CORRECTION
#define ASYSCTL_INTOSCCSR3_ILEAK_CALIB  0x200U       // ILEAK_CALIB

//*****************************************************************************
//
// The following are defines for the bit fields in the TSNSCFG register
//
//*****************************************************************************
#define ASYSCTL_TSNSCFG_BGINP     0x1U         // BGINP or buffer1 input sense
                                               // on testana0
#define ASYSCTL_TSNSCFG_BGINM     0x2U         // BGINM or buffer2 input sense
                                               // on testana1
#define ASYSCTL_TSNSCFG_VBUF1     0x4U         // buffer1 output sense on
                                               // testana1
#define ASYSCTL_TSNSCFG_VBUF2     0x8U         // buffer2 output sense on
                                               // testana0
#define ASYSCTL_TSNSCFG_TSOUT     0x10U        // Tempsensor output sense on
                                               // testana0
#define ASYSCTL_TSNSCFG_BGINMFLT  0x20U        // sense for BGINM or buffer2 
                                               // input after filter [[br]]on
                                               // testana1
#define ASYSCTL_TSNSCFG_BGINPFLT  0x40U        // sense for BGINP or buffer1
                                               // input after filter [[br]]on
                                               // testana0

//*****************************************************************************
//
// The following are defines for the bit fields in the REFCONFIGA register
//
//*****************************************************************************
#define ASYSCTL_REFCONFIGA_BURSTNOISE_FIX  0x1U         // Burst noise fix enable
#define ASYSCTL_REFCONFIGA_MEAS_IBIAS  0x2U         // measure bias current
#define ASYSCTL_REFCONFIGA_MEAS_VBG  0x4U         // Measure BGAP voltage
#define ASYSCTL_REFCONFIGA_IBIAS_EXT  0x8U         // External IREF mode
#define ASYSCTL_REFCONFIGA_CHOPCLK_DIV_S  4U
#define ASYSCTL_REFCONFIGA_CHOPCLK_DIV_M  0x30U        // Clock chop div prog
#define ASYSCTL_REFCONFIGA_BURST_BASERES0  0x40U        // Burst noise base res prog
                                               // bit0
#define ASYSCTL_REFCONFIGA_SEL_CLK_CHOP  0x80U        // Clock chop source select
#define ASYSCTL_REFCONFIGA_BURST_BASERES21_S  8U
#define ASYSCTL_REFCONFIGA_BURST_BASERES21_M  0x300U       // Burst noise base res prog
                                               // bit1
#define ASYSCTL_REFCONFIGA_REFBUF_GAIN_PROG_S  10U
#define ASYSCTL_REFCONFIGA_REFBUF_GAIN_PROG_M  0xC00U       // Burst noise base res prog
                                               // bit2
#define ASYSCTL_REFCONFIGA_REFBUF_BIAS_PROG  0x1000U      // REFBUF bias current program
#define ASYSCTL_REFCONFIGA_REFBUF_DRV_STAGE_PROG  0x2000U      // REFBUF output stage drive
                                               // program
#define ASYSCTL_REFCONFIGA_EN_BGAP  0x4000U      // BGAP enable testmode
#define ASYSCTL_REFCONFIGA_BGAP_FLT_PROG  0x8000U      // BGAP filter program
#define ASYSCTL_REFCONFIGA_CLK_CHOP_EXT  0x10000U     // Chop clock external option
#define ASYSCTL_REFCONFIGA_ADC_LSB_RES_DAMP_S  17U
#define ASYSCTL_REFCONFIGA_ADC_LSB_RES_DAMP_M  0x60000U     // ADC LSB damp res prog
#define ASYSCTL_REFCONFIGA_ADC_MSB_RES_DAMP_S  19U
#define ASYSCTL_REFCONFIGA_ADC_MSB_RES_DAMP_M  0x180000U    // ADC MSB damp res prog
#define ASYSCTL_REFCONFIGA_ADC_RES_DAMP_CONFIG  0x200000U    // ADC damp res config int to
                                               // ext
#define ASYSCTL_REFCONFIGA_TM_EN_VMEAS_PGAE  0x400000U    // Voltage measure on PGAE OUTF
#define ASYSCTL_REFCONFIGA_TM_EN_IFORCE_PGAE  0x800000U    // Enable iforce on PGAE OUTF
#define ASYSCTL_REFCONFIGA_SPARE1_S  24U
#define ASYSCTL_REFCONFIGA_SPARE1_M  0x3F000000U  // Spare for future use
#define ASYSCTL_REFCONFIGA_ADC_CHSEL_SOC_DEL_PROG  0x40000000U  // CHSEL rise to ADCSOC rise
                                               // delay program
#define ASYSCTL_REFCONFIGA_ADCINCAL_ISO  0x80000000U  // ADCINCAL_INT to ADCINCAL pin
                                               // isolation control

//*****************************************************************************
//
// The following are defines for the bit fields in the REFCONFIGC register
//
//*****************************************************************************
#define ASYSCTL_REFCONFIGC_BURSTNOISE_FIX  0x1U         // Burst noise fix enable
#define ASYSCTL_REFCONFIGC_MEAS_IBIAS  0x2U         // measure bias current
#define ASYSCTL_REFCONFIGC_MEAS_VBG  0x4U         // Measure BGAP voltage
#define ASYSCTL_REFCONFIGC_IBIAS_EXT  0x8U         // External IREF mode
#define ASYSCTL_REFCONFIGC_CHOPCLK_DIV_S  4U
#define ASYSCTL_REFCONFIGC_CHOPCLK_DIV_M  0x30U        // Clock chop div prog
#define ASYSCTL_REFCONFIGC_BURST_BASERES0  0x40U        // Burst noise base res prog
                                               // bit0
#define ASYSCTL_REFCONFIGC_SEL_CLK_CHOP  0x80U        // Clock chop source select
#define ASYSCTL_REFCONFIGC_BURST_BASERES21_S  8U
#define ASYSCTL_REFCONFIGC_BURST_BASERES21_M  0x300U       // Burst noise base res prog
                                               // bit1
#define ASYSCTL_REFCONFIGC_REFBUF_GAIN_PROG_S  10U
#define ASYSCTL_REFCONFIGC_REFBUF_GAIN_PROG_M  0xC00U       // Burst noise base res prog
                                               // bit2
#define ASYSCTL_REFCONFIGC_REFBUF_BIAS_PROG  0x1000U      // REFBUF bias current program
#define ASYSCTL_REFCONFIGC_REFBUF_DRV_STAGE_PROG  0x2000U      // REFBUF output stage drive
                                               // program
#define ASYSCTL_REFCONFIGC_EN_BGAP  0x4000U      // BGAP enable testmode
#define ASYSCTL_REFCONFIGC_BGAP_FLT_PROG  0x8000U      // BGAP filter program
#define ASYSCTL_REFCONFIGC_CLK_CHOP_EXT  0x10000U     // Chop clock external option
#define ASYSCTL_REFCONFIGC_ADC_LSB_RES_DAMP_S  17U
#define ASYSCTL_REFCONFIGC_ADC_LSB_RES_DAMP_M  0x60000U     // ADC LSB damp res prog
#define ASYSCTL_REFCONFIGC_ADC_MSB_RES_DAMP_S  19U
#define ASYSCTL_REFCONFIGC_ADC_MSB_RES_DAMP_M  0x180000U    // ADC MSB damp res prog
#define ASYSCTL_REFCONFIGC_ADC_RES_DAMP_CONFIG  0x200000U    // ADC damp res config int to
                                               // ext
#define ASYSCTL_REFCONFIGC_TM_EN_VMEAS_PGAD  0x400000U    // Voltage measure on PGAD OUTF
#define ASYSCTL_REFCONFIGC_TM_EN_IFORCE_PGAD  0x800000U    // Enable iforce on PGAD OUTF
#define ASYSCTL_REFCONFIGC_SPARE4_S  24U
#define ASYSCTL_REFCONFIGC_SPARE4_M  0x3F000000U  // Spare for future use
#define ASYSCTL_REFCONFIGC_ADC_CHSEL_SOC_DEL_PROG  0x40000000U  // CHSEL rise to ADCSOC rise
                                               // delay program
#define ASYSCTL_REFCONFIGC_SPARE5  0x80000000U  // Spare for future use

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMCSR register
//
//*****************************************************************************
#define ASYSCTL_PMMCSR_PMMSTATUS_S  28U
#define ASYSCTL_PMMCSR_PMMSTATUS_M  0xF0000000U  // Status bits from the PMM

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMCONFIGDEBUG0 register
//
//*****************************************************************************
#define ASYSCTL_PMMCONFIGDEBUG0_DISMASK  0x1U         // Test mode to disable Mask
#define ASYSCTL_PMMCONFIGDEBUG0_PDNREFSYS  0x2U         // Test mode to power down PMM
                                               // refsys
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE6  0x4U         // Spare6 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE7  0x8U         // Spare7 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_SHORTVREGILIMITRES  0x10U        // Zero addition in the
                                               // VREG-ILIMIT loop
#define ASYSCTL_PMMCONFIGDEBUG0_DISVREGINRUSH  0x20U        // Disable mode for internal
                                               // current limit of VREG.
#define ASYSCTL_PMMCONFIGDEBUG0_VREGIBIASPROG_S  6U
#define ASYSCTL_PMMCONFIGDEBUG0_VREGIBIASPROG_M  0xC0U        // Bits to control the bias
                                               // current of VREG
#define ASYSCTL_PMMCONFIGDEBUG0_VREGINRUSHPROG_S  8U
#define ASYSCTL_PMMCONFIGDEBUG0_VREGINRUSHPROG_M  0x300U       // Bits to control the Inrush
                                               // current of VREG
#define ASYSCTL_PMMCONFIGDEBUG0_VREGCUTAMPGAIN  0x400U       // Bits to control the gain of
                                               // the VREG amp
#define ASYSCTL_PMMCONFIGDEBUG0_OVFCOMPFILTPROG  0x800U       // To increase the filter on fb
                                               // of OVF Comp
#define ASYSCTL_PMMCONFIGDEBUG0_VREGADDCAP  0x1000U      // Bits to control the
                                               // compensation cap in VREG
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE8  0x2000U      // Spare8 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE9  0x4000U      // Spare9 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE10  0x8000U      // Spare10 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_VREFEXT  0x10000U     // External VREF option.
#define ASYSCTL_PMMCONFIGDEBUG0_VREF1P2  0x20000U     // Internal Band gap reference
                                               // of 1.2V
#define ASYSCTL_PMMCONFIGDEBUG0_BGUP  0x40000U     // Band gap up signal
#define ASYSCTL_PMMCONFIGDEBUG0_VDDIOPMMSENSE  0x80000U     // Sense used for POR(I/O) and
                                               // BOR(I/O)
#define ASYSCTL_PMMCONFIGDEBUG0_VDDIOREFSYSSENSE  0x100000U    // The Refsys supply sense
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE11  0x200000U    // Spare11 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE12  0x400000U    // Spare12 For Future use
#define ASYSCTL_PMMCONFIGDEBUG0_IREFEXT  0x800000U    // IREF external option
#define ASYSCTL_PMMCONFIGDEBUG0_VSSASENSEREFSYS  0x1000000U   // Ground sense taken from the
                                               // Feedback ladder of REFSYS
#define ASYSCTL_PMMCONFIGDEBUG0_VSSASENSE  0x2000000U   // Ground sense line  for
                                               // VSSA_PMM
#define ASYSCTL_PMMCONFIGDEBUG0_PORTEST  0x4000000U   // Internal Test node brought
                                               // out from POR circuit
#define ASYSCTL_PMMCONFIGDEBUG0_VDDIOPMM  0x8000000U   // Sense line taken from PMM
                                               // supply
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE13_S  28U
#define ASYSCTL_PMMCONFIGDEBUG0_SPARE13_M  0xF0000000U  // Spare13 For Future use

//*****************************************************************************
//
// The following are defines for the bit fields in the PMMCONFIGDFT register
//
//*****************************************************************************
#define ASYSCTL_PMMCONFIGDFT_TESTINTVREG  0x1U         // To test POR and OVF on core
#define ASYSCTL_PMMCONFIGDFT_CONTCLK  0x2U         // When set the internal
                                               // oscillator is always on
#define ASYSCTL_PMMCONFIGDFT_ENCHOPPER  0x4U         // When set High enables the
                                               // chopping for Band gap reference
#define ASYSCTL_PMMCONFIGDFT_SELCHOPPERPHASE  0x8U         // To put chopper switches in
                                               // different phases
#define ASYSCTL_PMMCONFIGDFT_SPARE18  0x10U        // Spare18 For Future use
#define ASYSCTL_PMMCONFIGDFT_SPARE19  0x20U        // Spare19 For Future use
#define ASYSCTL_PMMCONFIGDFT_SPARE20  0x40U        // Spare20 For Future use
#define ASYSCTL_PMMCONFIGDFT_LKGCALIBRATION  0x80U        // To enable leakage calibration
                                               // mode
#define ASYSCTL_PMMCONFIGDFT_VREF0P7  0x100U       // Internal Band gap reference
                                               // of 0.7V
#define ASYSCTL_PMMCONFIGDFT_VDDSENSE  0x200U       // Sense line taken from VDD
                                               // supply
#define ASYSCTL_PMMCONFIGDFT_VSSASENSEREFSYS0  0x400U       // Sense line from Refsys gnd
#define ASYSCTL_PMMCONFIGDFT_IREFTRIM  0x800U       // Internal 40uA IREF line
                                               // brought out for trimming
#define ASYSCTL_PMMCONFIGDFT_VSSASENSEREFSYS1  0x1000U      // Sense line from Refsys gnd
#define ASYSCTL_PMMCONFIGDFT_SPARE21  0x2000U      // Spare21 For Future use
#define ASYSCTL_PMMCONFIGDFT_STATUSMUXSELPMM_S  14U
#define ASYSCTL_PMMCONFIGDFT_STATUSMUXSELPMM_M  0xC000U      // Select lines for internal Mux
                                               // on PMMSTATUS lines
#define ASYSCTL_PMMCONFIGDFT_PFETRESMEA  0x100000U    // to measure the PMOS-
                                               // Power-FET resistance
#define ASYSCTL_PMMCONFIGDFT_NFETRESMEA  0x200000U    // to measure the NMOS-
                                               // Power-FET resistance
#define ASYSCTL_PMMCONFIGDFT_ILIMITSTANDALONE  0x4000000U   // This enables the standalone
                                               // testing of ILIMIT
#define ASYSCTL_PMMCONFIGDFT_SPARE22_S  27U
#define ASYSCTL_PMMCONFIGDFT_SPARE22_M  0xF8000000U  // Spare22 For Future use

//*****************************************************************************
//
// The following are defines for the bit fields in the REFSYSCSR register
//
//*****************************************************************************
#define ASYSCTL_REFSYSCSR_TMREFSYSPDN  0x1U         // Refsys Power down bit
#define ASYSCTL_REFSYSCSR_SELCHOPPHASE  0x2U         // Chopper phase selection
#define ASYSCTL_REFSYSCSR_ENTESTMUXDECODER  0x4U         // Enable for Test mux decoder
#define ASYSCTL_REFSYSCSR_TESTSEL0  0x8U         // Bit of the decoder
#define ASYSCTL_REFSYSCSR_TESTSEL1  0x10U        // Bit of the decoder
#define ASYSCTL_REFSYSCSR_TESTSEL2  0x20U        // Bit of the decoder
#define ASYSCTL_REFSYSCSR_SPARE24  0x40U        // Spare24 For Future use
#define ASYSCTL_REFSYSCSR_TRIMIREF16U  0x80U        // To bring out 16uA for
                                               // trimming
#define ASYSCTL_REFSYSCSR_TMSELEXTVREF  0x100U       // External VREF option.
#define ASYSCTL_REFSYSCSR_VSSASENSE  0x200U       // Sense on gnd
#define ASYSCTL_REFSYSCSR_TMSELEXTIREF  0x400U       // External IREF option
#define ASYSCTL_REFSYSCSR_ENTRIMIREF  0x800U       // enable for IREF trim
#define ASYSCTL_REFSYSCSR_TRIMIREF8U16U40U  0x1000U      // programmability to bring out
                                               // different currents
#define ASYSCTL_REFSYSCSR_LKGCALIBRATION  0x2000U      // To enable leakage calibration
                                               // mode
#define ASYSCTL_REFSYSCSR_SLECHOPAMP  0x4000U      // This is for offset
                                               // calibration of Main Amplifier
#define ASYSCTL_REFSYSCSR_SLECHOPBUF  0x8000U      // This is for offset
                                               // calibration of Buffer
#define ASYSCTL_REFSYSCSR_ITRIMTESTANASEL  0x10000U     // To select TESTANA port for
                                               // IREF Trimming
#define ASYSCTL_REFSYSCSR_REFSYSCSR_BITS_S  17U
#define ASYSCTL_REFSYSCSR_REFSYSCSR_BITS_M  0xFFFE0000U  // REFSYS CSR bits, to be
                                               // defined.

//*****************************************************************************
//
// The following are defines for the bit fields in the INTERNALTESTCTL register
//
//*****************************************************************************
#define ASYSCTL_INTERNALTESTCTL_TESTSEL_S  0U
#define ASYSCTL_INTERNALTESTCTL_TESTSEL_M  0x1FU        // Test Select
#define ASYSCTL_INTERNALTESTCTL_TESTANASWCTL_S  5U
#define ASYSCTL_INTERNALTESTCTL_TESTANASWCTL_M  0xE0U        // Swtich control of 
                                               // ADCCIO/rest testana signals.
#define ASYSCTL_INTERNALTESTCTL_KEY_S  16U
#define ASYSCTL_INTERNALTESTCTL_KEY_M  0xFFFF0000U  // Key to Enable writes

//*****************************************************************************
//
// The following are defines for the bit fields in the PUMPVREADCT register
//
//*****************************************************************************
#define ASYSCTL_PUMPVREADCT_LPVREADCT_S  0U
#define ASYSCTL_PUMPVREADCT_LPVREADCT_M  0xFU         // VREADCT of the LPPUMP

//*****************************************************************************
//
// The following are defines for the bit fields in the PUMPVCG2P5CT register
//
//*****************************************************************************
#define ASYSCTL_PUMPVCG2P5CT_LPVCG2P5CT_S  0U
#define ASYSCTL_PUMPVCG2P5CT_LPVCG2P5CT_M  0xFU         // VCG2P5CT of the LPPUMP

//*****************************************************************************
//
// The following are defines for the bit fields in the CONFIGLOCK register
//
//*****************************************************************************
#define ASYSCTL_CONFIGLOCK_LOCKBIT  0x1U         // Locks config

//*****************************************************************************
//
// The following are defines for the bit fields in the TSNSCTL register
//
//*****************************************************************************
#define ASYSCTL_TSNSCTL_ENABLE    0x1U         // Temperature Sensor Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the ANAREFCTL register
//
//*****************************************************************************
#define ASYSCTL_ANAREFCTL_ANAREFASEL  0x1U         // Analog Reference A Select
#define ASYSCTL_ANAREFCTL_ANAREFCSEL  0x4U         // Analog Reference C Select
#define ASYSCTL_ANAREFCTL_ANAREFA2P5SEL  0x100U       // Analog Reference A Select
#define ASYSCTL_ANAREFCTL_ANAREFC2P5SEL  0x400U       // Analog Reference C Select

//*****************************************************************************
//
// The following are defines for the bit fields in the VREGCTL register
//
//*****************************************************************************
#define ASYSCTL_VREGCTL_PWRDNVREG  0x1U         // Power down internal voltage
                                               // regulator
#define ASYSCTL_VREGCTL_ENMASK    0x8000U      // Enable VMON function mask on
                                               // a TRIM load

//*****************************************************************************
//
// The following are defines for the bit fields in the VMONCTL register
//
//*****************************************************************************
#define ASYSCTL_VMONCTL_OVMONENZ  0x1U         // Over Voltage Monitor Enable
                                               // Low
#define ASYSCTL_VMONCTL_OVFNMIEN  0x2U         // Over Voltage Fault NMI Enable
#define ASYSCTL_VMONCTL_BORLVMONDIS  0x100U       // Disable BORL(ow) feature on
                                               // VDDIO

//*****************************************************************************
//
// The following are defines for the bit fields in the VMONSTS register
//
//*****************************************************************************
#define ASYSCTL_VMONSTS_OVF       0x1U         // Over Voltage Fault

//*****************************************************************************
//
// The following are defines for the bit fields in the VMONCLR register
//
//*****************************************************************************
#define ASYSCTL_VMONCLR_OVF       0x1U         // Over Voltage Fault Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the CMPHPMXSEL register
//
//*****************************************************************************
#define ASYSCTL_CMPHPMXSEL_CMP1HPMXSEL_S  0U
#define ASYSCTL_CMPHPMXSEL_CMP1HPMXSEL_M  0x7U         // CMP1HPMXSEL bits
#define ASYSCTL_CMPHPMXSEL_CMP2HPMXSEL_S  3U
#define ASYSCTL_CMPHPMXSEL_CMP2HPMXSEL_M  0x38U        // CMP2HPMXSEL bits
#define ASYSCTL_CMPHPMXSEL_CMP3HPMXSEL_S  6U
#define ASYSCTL_CMPHPMXSEL_CMP3HPMXSEL_M  0x1C0U       // CMP3HPMXSEL bits
#define ASYSCTL_CMPHPMXSEL_CMP4HPMXSEL_S  9U
#define ASYSCTL_CMPHPMXSEL_CMP4HPMXSEL_M  0xE00U       // CMP4HPMXSEL bits

//*****************************************************************************
//
// The following are defines for the bit fields in the CMPLPMXSEL register
//
//*****************************************************************************
#define ASYSCTL_CMPLPMXSEL_CMP1LPMXSEL_S  0U
#define ASYSCTL_CMPLPMXSEL_CMP1LPMXSEL_M  0x7U         // CMP1LPMXSEL bits
#define ASYSCTL_CMPLPMXSEL_CMP2LPMXSEL_S  3U
#define ASYSCTL_CMPLPMXSEL_CMP2LPMXSEL_M  0x38U        // CMP2LPMXSEL bits
#define ASYSCTL_CMPLPMXSEL_CMP3LPMXSEL_S  6U
#define ASYSCTL_CMPLPMXSEL_CMP3LPMXSEL_M  0x1C0U       // CMP3LPMXSEL bits
#define ASYSCTL_CMPLPMXSEL_CMP4LPMXSEL_S  9U
#define ASYSCTL_CMPLPMXSEL_CMP4LPMXSEL_M  0xE00U       // CMP4LPMXSEL bits

//*****************************************************************************
//
// The following are defines for the bit fields in the CMPHNMXSEL register
//
//*****************************************************************************
#define ASYSCTL_CMPHNMXSEL_CMP1HNMXSEL  0x1U         // CMP1HNMXSEL bits
#define ASYSCTL_CMPHNMXSEL_CMP2HNMXSEL  0x2U         // CMP2HNMXSEL bits
#define ASYSCTL_CMPHNMXSEL_CMP3HNMXSEL  0x4U         // CMP3HNMXSEL bits
#define ASYSCTL_CMPHNMXSEL_CMP4HNMXSEL  0x8U         // CMP4HNMXSEL bits

//*****************************************************************************
//
// The following are defines for the bit fields in the CMPLNMXSEL register
//
//*****************************************************************************
#define ASYSCTL_CMPLNMXSEL_CMP1LNMXSEL  0x1U         // CMP1LNMXSEL bits
#define ASYSCTL_CMPLNMXSEL_CMP2LNMXSEL  0x2U         // CMP2LNMXSEL bits
#define ASYSCTL_CMPLNMXSEL_CMP3LNMXSEL  0x4U         // CMP3LNMXSEL bits
#define ASYSCTL_CMPLNMXSEL_CMP4LNMXSEL  0x8U         // CMP4LNMXSEL bits

//*****************************************************************************
//
// The following are defines for the bit fields in the ADCDACLOOPBACK register
//
//*****************************************************************************
#define ASYSCTL_ADCDACLOOPBACK_ENLB2ADCA  0x1U         // Enable DACA loopback to ADCA
#define ASYSCTL_ADCDACLOOPBACK_ENLB2ADCC  0x4U         // Enable DACA loopback to ADCC
#define ASYSCTL_ADCDACLOOPBACK_KEY_S  16U
#define ASYSCTL_ADCDACLOOPBACK_KEY_M  0xFFFF0000U  // Key to enable writes

//*****************************************************************************
//
// The following are defines for the bit fields in the LOCK register
//
//*****************************************************************************
#define ASYSCTL_LOCK_TSNSCTL      0x1U         // TSNSCTL Register lock bit
#define ASYSCTL_LOCK_ANAREFCTL    0x2U         // ANAREFCTL Register lock bit
#define ASYSCTL_LOCK_VMONCTL      0x4U         // VMONCTL Register lock bit
#define ASYSCTL_LOCK_CMPHPMXSEL   0x20U        // CMPHPMXSEL Register lock bit
#define ASYSCTL_LOCK_CMPLPMXSEL   0x40U        // CMPLPMXSEL Register lock bit
#define ASYSCTL_LOCK_CMPHNMXSEL   0x80U        // CMPHNMXSEL Register lock bit
#define ASYSCTL_LOCK_CMPLNMXSEL   0x100U       // CMPLNMXSEL Register lock bit
#define ASYSCTL_LOCK_VREGCTL      0x200U       // VREGCTL Register lock bit

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG1 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG1_CP_CTRL_S  0U
#define ASYSCTL_SYSAPLLCONFIG1_CP_CTRL_M  0xFU         // Charge Pump Control
#define ASYSCTL_SYSAPLLCONFIG1_PFD_CLR  0x10U        // To open the feedback loop
#define ASYSCTL_SYSAPLLCONFIG1_HI_BW  0x20U        // Increases the Chargepump
                                               // current by 60%
#define ASYSCTL_SYSAPLLCONFIG1_CP_DAC_S  6U
#define ASYSCTL_SYSAPLLCONFIG1_CP_DAC_M  0x3C0U       // Charge Pump DAC override
#define ASYSCTL_SYSAPLLCONFIG1_C1_DAC_S  10U
#define ASYSCTL_SYSAPLLCONFIG1_C1_DAC_M  0x7C00U      // Cap DAC (Zero Capacitor)
#define ASYSCTL_SYSAPLLCONFIG1_OSC_SEL  0x8000U      // Source of clock for loop
                                               // bandwidth calculation

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG2 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG2_PU_DEL_SEL_S  0U
#define ASYSCTL_SYSAPLLCONFIG2_PU_DEL_SEL_M  0x3U         // Cold power up delay counter
                                               // for Mdiv_ack
#define ASYSCTL_SYSAPLLCONFIG2_RES_TRIM_S  2U
#define ASYSCTL_SYSAPLLCONFIG2_RES_TRIM_M  0x1CU        // Resistor Trim code generated
                                               // from R-unit
#define ASYSCTL_SYSAPLLCONFIG2_INT_N_EXT  0x20U        // Internal LBW state machine or
                                               // ext control
#define ASYSCTL_SYSAPLLCONFIG2_DIS_OUTPUT  0x40U        // Disables the output clock
#define ASYSCTL_SYSAPLLCONFIG2_PWDN_MDIV  0x80U        // Turns-off feedback divider
#define ASYSCTL_SYSAPLLCONFIG2_DLY_CTRL_S  8U
#define ASYSCTL_SYSAPLLCONFIG2_DLY_CTRL_M  0x300U       // Controls ref_clock delay
#define ASYSCTL_SYSAPLLCONFIG2_GM_RES_TRIM_S  10U
#define ASYSCTL_SYSAPLLCONFIG2_GM_RES_TRIM_M  0x1C00U      // detailed config in other doc
#define ASYSCTL_SYSAPLLCONFIG2_SPARE25  0x2000U      // Spare25 For Future use
#define ASYSCTL_SYSAPLLCONFIG2_LPF_C_CTRL  0x4000U      // 0=default,1= 1pF less
#define ASYSCTL_SYSAPLLCONFIG2_VCO_RANGE_TEST  0x8000U      // Enables control of all bits
                                               // of MDIV, REFDIV and ODIV.

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG3 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG3_LIMP_CTRL_S  0U
#define ASYSCTL_SYSAPLLCONFIG3_LIMP_CTRL_M  0xFU         // config limp mode frequency
#define ASYSCTL_SYSAPLLCONFIG3_LOCK_IND_CLR  0x10U        // 0=on, 1=off
#define ASYSCTL_SYSAPLLCONFIG3_LOCK_IND_CTRL_S  5U
#define ASYSCTL_SYSAPLLCONFIG3_LOCK_IND_CTRL_M  0x1E0U       // 1:0 for sensitivity, 3:2 for
                                               // counter time
#define ASYSCTL_SYSAPLLCONFIG3_OSC_FORCE_ON  0x200U       // Force on internal osc for
                                               // trim
#define ASYSCTL_SYSAPLLCONFIG3_REF_LOST_EN  0x400U       // Enable Reference-Lost
                                               // condition to generate SLIP interrupt.
#define ASYSCTL_SYSAPLLCONFIG3_VCO_DAC_EXT_SEL  0x800U       // 0=auto, 1=manual code
#define ASYSCTL_SYSAPLLCONFIG3_VCO_DAC_EXT_S  12U
#define ASYSCTL_SYSAPLLCONFIG3_VCO_DAC_EXT_M  0x7000U      // manual VCO DAC code
#define ASYSCTL_SYSAPLLCONFIG3_SPARE26  0x8000U      // Spare26 For Future use

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG4 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG4_LOCK_FILTER_COUNT_S  0U
#define ASYSCTL_SYSAPLLCONFIG4_LOCK_FILTER_COUNT_M  0x3FU        // define how many cycles of no
                                               // updn_div2 flip before flag
                                               // lock
#define ASYSCTL_SYSAPLLCONFIG4_SLIP_FILTER_BASE_S  6U
#define ASYSCTL_SYSAPLLCONFIG4_SLIP_FILTER_BASE_M  0xFC0U       // Define out of how many cycles
                                               // we can define slip
#define ASYSCTL_SYSAPLLCONFIG4_LDO_LOAD_ALWAYSON_S  12U
#define ASYSCTL_SYSAPLLCONFIG4_LDO_LOAD_ALWAYSON_M  0x3000U      // 1=intentional load always on
                                               // to improve stability
#define ASYSCTL_SYSAPLLCONFIG4_LDO_OK_FORCE_H_S  14U
#define ASYSCTL_SYSAPLLCONFIG4_LDO_OK_FORCE_H_M  0xC000U      // To force LDO_OK signal=1

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG5 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG5_LOCK_IND_IP_DIS  0x1U         // Externally shut down dig part
                                               // of lock_ind
#define ASYSCTL_SYSAPLLCONFIG5_LOCK_IND_SLIP_QUALITY_S  1U
#define ASYSCTL_SYSAPLLCONFIG5_LOCK_IND_SLIP_QUALITY_M  0x7EU        // Define cycles of updn_div2
                                               // flip before flag slip
#define ASYSCTL_SYSAPLLCONFIG5_SPARE27_S  7U
#define ASYSCTL_SYSAPLLCONFIG5_SPARE27_M  0x780U       // Spare27 For Future use
#define ASYSCTL_SYSAPLLCONFIG5_OBS_MUX_SEL_S  11U
#define ASYSCTL_SYSAPLLCONFIG5_OBS_MUX_SEL_M  0x1800U      // Observation mux select
#define ASYSCTL_SYSAPLLCONFIG5_SPARE28_S  13U
#define ASYSCTL_SYSAPLLCONFIG5_SPARE28_M  0xE000U      // Spare28 For Future use

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG7 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG7_SEL_UPDN_BY2  0x1U         // Sel UPDN_DIV2 from CLK_AUX
#define ASYSCTL_SYSAPLLCONFIG7_ANA_TEST_MUX_SEL_S  1U
#define ASYSCTL_SYSAPLLCONFIG7_ANA_TEST_MUX_SEL_M  0x6U         // Analog test Mux select
#define ASYSCTL_SYSAPLLCONFIG7_CLK_AUX_EN  0x8U         // Enable CLKAUX
#define ASYSCTL_SYSAPLLCONFIG7_CLK_AUX_DIV_S  4U
#define ASYSCTL_SYSAPLLCONFIG7_CLK_AUX_DIV_M  0xF0U        // 4 bit Q-divider for CLKAUX
#define ASYSCTL_SYSAPLLCONFIG7_CLK_AUX_MUX_SEL_S  8U
#define ASYSCTL_SYSAPLLCONFIG7_CLK_AUX_MUX_SEL_M  0x300U       // CLKAUX MuxSelect
#define ASYSCTL_SYSAPLLCONFIG7_BIAS_SEL  0x400U       // Bias current Select
#define ASYSCTL_SYSAPLLCONFIG7_CP_VCO_TEST_S  11U
#define ASYSCTL_SYSAPLLCONFIG7_CP_VCO_TEST_M  0x1800U      // DFT LDO Enable
#define ASYSCTL_SYSAPLLCONFIG7_VCO_FREERUN  0x2000U      // Free running VCO mode
#define ASYSCTL_SYSAPLLCONFIG7_EN_ATB_BUF  0x4000U      // Enable ATB BUF
#define ASYSCTL_SYSAPLLCONFIG7_EN_ATB_UNBUF  0x8000U      // Enable ATB UNBUF

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG8 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG8_DFT_LDO_EN_S  0U
#define ASYSCTL_SYSAPLLCONFIG8_DFT_LDO_EN_M  0x3U         // DFT LDO Enable
#define ASYSCTL_SYSAPLLCONFIG8_LDO_CONFIG_S  2U
#define ASYSCTL_SYSAPLLCONFIG8_LDO_CONFIG_M  0xFCU        // LDO Config bits

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG9 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG9_APLL_LOCK_FILTER_S  0U
#define ASYSCTL_SYSAPLLCONFIG9_APLL_LOCK_FILTER_M  0x1FU        // APLL_LOCK_FILTER Value
#define ASYSCTL_SYSAPLLCONFIG9_APLL_LOCK_SLIP_FILTER_S  5U
#define ASYSCTL_SYSAPLLCONFIG9_APLL_LOCK_SLIP_FILTER_M  0x3E0U       // APLL_LOCK_SLIP_FILTER Value
#define ASYSCTL_SYSAPLLCONFIG9_SUMMARY_LOCK_CNTR_VALUE_S  10U
#define ASYSCTL_SYSAPLLCONFIG9_SUMMARY_LOCK_CNTR_VALUE_M  0x1C00U      // SUMMARY_LOCK_CNTR_VALUE Value

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLCONFIG10 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLCONFIG10_LDO_LO_COMP_EN  0x1U         // LDO_LO_COMP_EN Value
#define ASYSCTL_SYSAPLLCONFIG10_APLL_LOCK_MASK  0x2U         // APLL_LOCK_MASK Value
#define ASYSCTL_SYSAPLLCONFIG10_SUMMARY_LOCK_MASK  0x4U         // SUMMARY_LOCK_MASK Value
#define ASYSCTL_SYSAPLLCONFIG10_SLIP_UNMASK  0x8U         // SLIP_UNMASK Value
#define ASYSCTL_SYSAPLLCONFIG10_PLLEN_UNMASK  0x10U        // PLLEN_UNMASK Value

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLDIGSTATUS1 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLDIGSTATUS1_CP_STATUS_S  0U
#define ASYSCTL_SYSAPLLDIGSTATUS1_CP_STATUS_M  0xFU         // Charge-pump control loop
                                               // value
#define ASYSCTL_SYSAPLLDIGSTATUS1_CAP_STATUS_S  4U
#define ASYSCTL_SYSAPLLDIGSTATUS1_CAP_STATUS_M  0x1F0U       // Capacitor control loop value
#define ASYSCTL_SYSAPLLDIGSTATUS1_OSC_STATUS_S  9U
#define ASYSCTL_SYSAPLLDIGSTATUS1_OSC_STATUS_M  0x600U       // Internal oscillator current
                                               // value
#define ASYSCTL_SYSAPLLDIGSTATUS1_INT_CLRZ  0x800U       // Internal digital CLRz signal
#define ASYSCTL_SYSAPLLDIGSTATUS1_Q_DIV_UPDATE_REQ_R  0x2000U      // Q_div_update_req_r
#define ASYSCTL_SYSAPLLDIGSTATUS1_SPARE29_S  14U
#define ASYSCTL_SYSAPLLDIGSTATUS1_SPARE29_M  0xC000U      // Spare29 For Future use
#define ASYSCTL_SYSAPLLDIGSTATUS1_LOCK_IND_DEBUG_S  16U
#define ASYSCTL_SYSAPLLDIGSTATUS1_LOCK_IND_DEBUG_M  0xFF0000U    // Lock_ind_debug
#define ASYSCTL_SYSAPLLDIGSTATUS1_SPARE_BITS_S  24U
#define ASYSCTL_SYSAPLLDIGSTATUS1_SPARE_BITS_M  0xFF000000U  // Spare Bits

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLDIGSTATUS2 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLDIGSTATUS2_LOCK_TIME_ACC_S  0U
#define ASYSCTL_SYSAPLLDIGSTATUS2_LOCK_TIME_ACC_M  0xFFFFU      // Lock time accumulator
#define ASYSCTL_SYSAPLLDIGSTATUS2_SLIP_INCIDENCE_CNT_S  16U
#define ASYSCTL_SYSAPLLDIGSTATUS2_SLIP_INCIDENCE_CNT_M  0xFF0000U    // Slip count

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLDIGSTATUS3 register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLDIGSTATUS3_LDO_OK_S  0U
#define ASYSCTL_SYSAPLLDIGSTATUS3_LDO_OK_M  0x7U         // LDO_OK Status
#define ASYSCTL_SYSAPLLDIGSTATUS3_LBW_CAL_RDY  0x40U        // LBW_CAL_RDY Status
#define ASYSCTL_SYSAPLLDIGSTATUS3_APLL_LOCK_RAW  0x80U        // APLL_LOCK_RAW Status
#define ASYSCTL_SYSAPLLDIGSTATUS3_APLL_LOCK_QUAL  0x100U       // APLL_LOCK_QUAL Status
#define ASYSCTL_SYSAPLLDIGSTATUS3_APLL_LOCK_SUMMARY  0x200U       // APLL_LOCK_SUMMARY Status
#define ASYSCTL_SYSAPLLDIGSTATUS3_QSLIPS  0x400U       // SYSPLL Quasi Slip Status Bit

//*****************************************************************************
//
// The following are defines for the bit fields in the APLLREFTRIM register
//
//*****************************************************************************
#define ASYSCTL_APLLREFTRIM_BGVALTRIM_S  0U
#define ASYSCTL_APLLREFTRIM_BGVALTRIM_M  0x3FU        // Bandgap Value Trim
#define ASYSCTL_APLLREFTRIM_BGSLOPETRIM_S  6U
#define ASYSCTL_APLLREFTRIM_BGSLOPETRIM_M  0x7C0U       // Bandgap Slope Trim
#define ASYSCTL_APLLREFTRIM_IREFTRIM_S  11U
#define ASYSCTL_APLLREFTRIM_IREFTRIM_M  0xF800U      // Reference Current Trim

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLLDOTRIM register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLLDOTRIM_PROG_VDD_1P5V_S  0U
#define ASYSCTL_SYSAPLLLDOTRIM_PROG_VDD_1P5V_M  0xFU         // 1.5V LDO trim
#define ASYSCTL_SYSAPLLLDOTRIM_PROG_VDD_1P2V_S  4U
#define ASYSCTL_SYSAPLLLDOTRIM_PROG_VDD_1P2V_M  0xF0U        // 1.5V LDO trim

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSAPLLOSCTRIM register
//
//*****************************************************************************
#define ASYSCTL_SYSAPLLOSCTRIM_OSC_FREQ_S  0U
#define ASYSCTL_SYSAPLLOSCTRIM_OSC_FREQ_M  0x7U         // Internal oscillation
                                               // frequency options

//*****************************************************************************
//
// The following are defines for the bit fields in the APLLREFCONFIG register
//
//*****************************************************************************
#define ASYSCTL_APLLREFCONFIG_TMREFSYSPDN  0x1U         // Refsys Power down bit
#define ASYSCTL_APLLREFCONFIG_SELCHOPPHASE  0x2U         // Chopper phase selection
#define ASYSCTL_APLLREFCONFIG_ENTESTMUXDECODER  0x4U         // Enable for Test mux decoder
#define ASYSCTL_APLLREFCONFIG_TESTSEL0  0x8U         // Bit of the decoder
#define ASYSCTL_APLLREFCONFIG_TESTSEL1  0x10U        // Bit of the decoder
#define ASYSCTL_APLLREFCONFIG_TESTSEL2  0x20U        // Bit of the decoder
#define ASYSCTL_APLLREFCONFIG_SPARE24  0x40U        // Spare24 For Future use
#define ASYSCTL_APLLREFCONFIG_TRIMIREF16U  0x80U        // To bring out 16uA for
                                               // trimming
#define ASYSCTL_APLLREFCONFIG_TMSELEXTVREF  0x100U       // External VREF option.
#define ASYSCTL_APLLREFCONFIG_VSSASENSE  0x200U       // Sense on gnd
#define ASYSCTL_APLLREFCONFIG_TMSELEXTIREF  0x400U       // External IREF option
#define ASYSCTL_APLLREFCONFIG_ENTRIMIREF  0x800U       // enable for IREF trim
#define ASYSCTL_APLLREFCONFIG_TRIMIREF8U16U40U  0x1000U      // programmability to bring out
                                               // different currents
#define ASYSCTL_APLLREFCONFIG_LKGCALIBRATION  0x2000U      // To enable leakage calibration
                                               // mode
#define ASYSCTL_APLLREFCONFIG_SLECHOPAMP  0x4000U      // This is for offset
                                               // calibration of Main Amplifier
#define ASYSCTL_APLLREFCONFIG_SLECHOPBUF  0x8000U      // This is for offset
                                               // calibration of Buffer
#define ASYSCTL_APLLREFCONFIG_ITRIMTESTANASEL  0x10000U     // To select TESTANA port for
                                               // IREF Trimming
#define ASYSCTL_APLLREFCONFIG_REFSYSCSR_BITS_S  17U
#define ASYSCTL_APLLREFCONFIG_REFSYSCSR_BITS_M  0xFFFE0000U  // REFSYS CSR bits, to be
                                               // defined.

//*****************************************************************************
//
// The following are defines for the bit fields in the APLLLOCK register
//
//*****************************************************************************
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG1  0x1U         // SYSAPLLCONFIG1 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG2  0x2U         // SYSAPLLCONFIG2 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG3  0x4U         // SYSAPLLCONFIG3 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG4  0x8U         // SYSAPLLCONFIG4 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG5  0x10U        // SYSAPLLCONFIG5 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG6  0x20U        // SYSAPLLCONFIG6 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG7  0x40U        // SYSAPLLCONFIG7 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG8  0x80U        // SYSAPLLCONFIG8 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG9  0x100U       // SYSAPLLCONFIG9 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLCONFIG10  0x200U       // SYSAPLLCONFIG10 Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLLDOTRIM  0x400U       // SYSAPLLLDOTRIM Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_SYSAPLLOSCTRIM  0x800U       // SYSAPLLOSCTRIM Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_APLLREFTRIM  0x1000U      // APLLREFTRIM Register Lock bit
#define ASYSCTL_APLLLOCK_SYSAPLLSPAREIN  0x8000U      // SYSAPLLSPAREIN Register Lock
                                               // bit
#define ASYSCTL_APLLLOCK_APLLREFCONFIG  0x40000000U  // APLLREFCONFIG Register Lock
                                               // bit
#endif
