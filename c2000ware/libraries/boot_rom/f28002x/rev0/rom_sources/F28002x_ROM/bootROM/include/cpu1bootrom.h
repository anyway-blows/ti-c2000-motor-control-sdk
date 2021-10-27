//###########################################################################
//
// FILE:   cpu1bootrom.h
//
// TITLE:  BootROM Definitions.
//
//###########################################################################
// $TI Release: F28004x Boot ROM V1.0 $
// $Release Date: July , 2015 $
// $Copyright:  $
//###########################################################################



#ifndef C_BOOTROM_H_
#define C_BOOTROM_H_

#include <stdint.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_sysctl.h"
#include "hw_sci.h"
#include "hw_i2c.h"
#include "sysctl.h"
#include "cpu.h"
#include "interrupt.h"
#include "flash.h"
#include "dcsm.h"
#include "pin_map.h"
#include "gpio.h"
#include "sci.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"
#include "dcc.h"
#include "cpu1brom_pbist.h"
#include "hw_memcfg.h"


#define RAM_GSX_NOT_DONE                0x0U
#define BROM_PLL_CONFIG_ERROR           0xFFFFU
#define BROM_PLL_CONFIG_SUCCESS         0x0U

//
// Flash Configurations
//
#define CPU1_FLASH_15MHZ_PSLEEP        (0xAAU * 2U)
#define CPU1_FLASH_15MHZ_RWAIT         (0x1U)
#define CPU1_FLASH_DEFAULT_PSLEEP      (0x3E8U * 2U)
#define CPU1_FLASH_95MHZ_RWAIT         (0x4U)
#define CPU1_FLASH_DEFAULT_RWAIT       (0xFU)
#define CPU1_FLASH_15MHZ_TIMEOUT_VALUE (192U)     //Loop ~28cycles (Freq = 15MHz)
                                                  //(Loop timeout value = (360us * freq)/28cycles)


//When the ROM is run with an emulator connected, these four addresses are used
//to emulate OTP configuration.
#define EMU_BOOTPIN_CONFIG  0xD00    //Equivalent to GPREG1
#define EMU_BOOT_GPREG2     0xD02    //Equivalent to GPREG2
#define EMU_BOOTDEF_LOW     0xD04    //Equivalent to GPREG3
#define EMU_BOOTDEF_HIGH    0xD06    //Equivalent to BOOTCTRL

//Emulation boot pin configuration fields. Currently, only EMU_BOOTPIN_CONFIG_KEY
//is used.
#define EMU_BOOTPIN_CONFIG_KEY      ((HWREG(EMU_BOOTPIN_CONFIG) & (uint32_t)0xFF000000UL) >> 24)

#define EMU_BOOTDEF_L(x)            ((HWREG(EMU_BOOTDEF_LOW)  & ((uint32_t)0xFF << (8*x))) >> (8*x))
#define EMU_BOOTDEF_H(x)            ((HWREG(EMU_BOOTDEF_HIGH) & ((uint32_t)0xFF << (8*x))) >> (8*x))

#define Z1_OTP_BOOTPIN_CONFIG       (DCSMBANK0_Z1_BASE + DCSM_O_Z1_BOOTPIN_CONFIG)
#define Z1_OTP_BOOT_GPREG2          (DCSMBANK0_Z1_BASE + DCSM_O_Z1_GPREG2)
#define Z1_OTP_BOOTDEF_LOW          (DCSMBANK0_Z1_BASE + DCSM_O_Z1_BOOTDEF_LOW)
#define Z1_OTP_BOOTDEF_HIGH         (DCSMBANK0_Z1_BASE + DCSM_O_Z1_BOOTDEF_HIGH)

#define Z2_OTP_BOOTPIN_CONFIG       (DCSMBANK0_Z2_BASE + DCSM_O_Z1_BOOTPIN_CONFIG)
#define Z2_OTP_BOOT_GPREG2          (DCSMBANK0_Z2_BASE + DCSM_O_Z1_GPREG2)
#define Z2_OTP_BOOTDEF_LOW          (DCSMBANK0_Z2_BASE + DCSM_O_Z1_BOOTDEF_LOW)
#define Z2_OTP_BOOTDEF_HIGH         (DCSMBANK0_Z2_BASE + DCSM_O_Z1_BOOTDEF_HIGH)

/*
Z1-GPREG2[31:24] => VALIDITY_KEY (=0x5A);
Z1-GPREG2[23:8] => RESERVED; no usage defined yet.
Z1-GPREG2[7:6] => 00 - Run PBIST with PLL disabled (10MHz internal oscillator) 
                       (includes checksum test on 128KB unsecure ROM)
                  01 – Run PBIST at 95MHz
                  10 - Run PBIST at 47.5MHz
                  11 – Do not run PBIST
Z1-GPREG2[5:4] => ERROR_STS_PIN config; this tells which GPIO pin is supposed to be used as ERROR_PIN and boot ROM configures the mux as such for the said pin.
                0 – GPIO24, MUX Option 13
                1 – GPIO28, MUX Option 13
                2 – GPIO29, MUX Option 13
                3 – ERROR_STS function Disable  (default)

Z1-GPREG2[3:0]  =>  CJTAGNODEID[3:0];
                boot ROM takes this values and programs the lower 4 bits of the CJTAGNODEID register.
*/

#define HWREAD_Z1_OTP_BOOT_GPREG2_KEY            ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0xFF000000UL) >> 24U)
#define HWREAD_Z1_OTP_BOOT_GPREG2_PBIST_CONFIG   ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x000000C0UL) >> 6U)
#define HWREAD_Z1_OTP_BOOT_GPREG2_ERRSTS_CONFIG  ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x00000030UL) >> 4U)
#define HWREAD_Z1_OTP_BOOT_GPREG2_CJTAGNODEID    ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x0000000FUL))

#define HWREAD_Z2_OTP_BOOT_GPREG2_KEY            ((HWREG(Z2_OTP_BOOT_GPREG2) & (uint32_t)0xFF000000UL) >> 24U)
#define HWREAD_Z2_OTP_BOOT_GPREG2_PBIST_CONFIG   ((HWREG(Z2_OTP_BOOT_GPREG2) & (uint32_t)0x000000C0UL) >> 6U)
#define HWREAD_Z2_OTP_BOOT_GPREG2_ERRSTS_CONFIG  ((HWREG(Z2_OTP_BOOT_GPREG2) & (uint32_t)0x00000030UL) >> 4U)
#define HWREAD_Z2_OTP_BOOT_GPREG2_CJTAGNODEID    ((HWREG(Z2_OTP_BOOT_GPREG2) & (uint32_t)0x0000000FUL))

#define ERRORSTS_PIN_24                    0x0UL
#define ERRORSTS_PIN_28                    0x1UL
#define ERRORSTS_PIN_29                    0x2UL

#define GPREG2_KEY                         0x5AU

#define GPREG2_PBIST_RUN_PLL_BYPASS        0x0UL
#define GPREG2_PBIST_RUN_SYSCLK_95MHZ      0x1UL
#define GPREG2_PBIST_RUN_SYSCLK_47_5MHZ    0x2UL
#define GPREG2_PBIST_DISABLED              0x3UL

#define PBIST_CHECKSUM_SUCCESS             0x0U

//
// Get key to validate Z1 OTP BOOTPIN_CONFIG
//
#define HWREAD_Z1_OTP_BOOTPIN_CONFIG_KEY        ((HWREG(Z1_OTP_BOOTPIN_CONFIG) & (uint32_t)0xFF000000UL) >> 24U)
#define HWREAD_Z2_OTP_BOOTPIN_CONFIG_KEY        ((HWREG(Z2_OTP_BOOTPIN_CONFIG) & (uint32_t)0xFF000000UL) >> 24U)

//
// Standalone macros to extract boot definition from BOOTDEF table at
// specified index
//
#define HWREAD_Z1_OTP_BOOTDEF_L(x)             ((HWREG(Z1_OTP_BOOTDEF_LOW)  & ((uint32_t)0xFFUL << (8UL*(x)))) >> (8UL*(x)))
#define HWREAD_Z1_OTP_BOOTDEF_H(x)             ((HWREG(Z1_OTP_BOOTDEF_HIGH) & ((uint32_t)0xFFUL << (8UL*(x)))) >> (8UL*(x)))
#define HWREAD_Z2_OTP_BOOTDEF_L(x)             ((HWREG(Z2_OTP_BOOTDEF_LOW)  & ((uint32_t)0xFFUL << (8UL*(x)))) >> (8UL*(x)))
#define HWREAD_Z2_OTP_BOOTDEF_H(x)             ((HWREG(Z2_OTP_BOOTDEF_HIGH) & ((uint32_t)0xFFUL << (8UL*(x)))) >> (8UL*(x)))

//#define EMU_BOOTDEF0              ((HWREG(EMU_BOOTDEF_LOW) & 0xFF) & 0x1F)
//#define EMU_BOOTDEF0_ALT_OPTIONS  (((HWREG(EMU_BOOTDEF_LOW) & 0xFF) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF1              (((HWREG(EMU_BOOTDEF_LOW) & 0xFF00)>>8) & 0x1F)
//#define EMU_BOOTDEF1_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & 0xFF00)>>8) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF2              (((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF0000) >> 16) & 0x1F)
//#define EMU_BOOTDEF2_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF0000) >> 16) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF3              (((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF000000) >> 24) & 0x1F)
//#define EMU_BOOTDEF3_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF000000) >> 24) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF4              ((HWREG(EMU_BOOTDEF_HIGH) & 0xFF) & 0x1F)
//#define EMU_BOOTDEF4_ALT_OPTIONS  (((HWREG(EMU_BOOTDEF_HIGH) & 0xFF) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF5              (((HWREG(EMU_BOOTDEF_HIGH) & 0xFF00)>>8) & 0x1F)
//#define EMU_BOOTDEF5_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & 0xFF00)>>8) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF6              (((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF0000) >> 16) & 0x1F)
//#define EMU_BOOTDEF6_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF0000) >> 16) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF7              (((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF000000) >> 24) & 0x1F)
//#define EMU_BOOTDEF7_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF000000) >> 24) & 0xE0) >> 5)


#define FACTORY_DEFAULT_BMSP0       32      //GPIO32
#define FACTORY_DEFAULT_BMSP1       24      //GPIO24

#define PARALLEL_BOOT           0x00U
#define PARALLEL_BOOT_ALT1      0x20U

#define SCI_BOOT                0x01U   //GPIO29; GPIO28 (CCARD)
#define SCI_BOOT_ALT1           0x21U   //GPIO16; GPIO17
#define SCI_BOOT_ALT2           0x41U   //GPIO8;  GPIO9
#define SCI_BOOT_ALT3           0x61U   //GPIO2;  GPIO3
#define SCI_BOOT_ALT4           0x81U   //GPIO16; GPIO3

#define CAN_BOOT                0x02U   //GPIO4; GPIO5
#define CAN_BOOT_ALT1           0x22U   //GPIO32; GPIO33
#define CAN_BOOT_ALT2           0x42U   //GPIO2; GPIO3
#define CAN_BOOT_SENDTEST       0x82U   //GPIO4; GPIO5
#define CAN_BOOT_ALT1_SENDTEST  0xA2U   //GPIO32; GPIO33
#define CAN_BOOT_ALT2_SENDTEST  0xC2U   //GPIO2; GPIO3

#define FLASH_BOOT              0x03U   //begin of BANK 0 Sector 0
#define FLASH_BOOT_ALT1         0x23U   //begin of BANK 0 Sector 4
#define FLASH_BOOT_ALT2         0x43U   //begin of BANK 0 Sector 8
#define FLASH_BOOT_ALT3         0x63U   //begin of BANK 0 Sector 15

#define WAIT_BOOT               0x04U   //with WDOG enabled
#define WAIT_BOOT_ALT1          0x24U   //without WDOG enabled

#define RAM_BOOT                0x05U

#define SPI_MASTER_BOOT         0x06U   //GPIO 2,1,3,5
#define SPI_MASTER_BOOT_ALT1    0x26U   //GPIO 16,1,3,0
#define SPI_MASTER_BOOT_ALT2    0x46U   //GPIO 8,10,9,11
#define SPI_MASTER_BOOT_ALT3    0x66U   //GPIO 8,17,9,11

#define I2C_MASTER_BOOT         0x07U   //GPIO32, GPIO33
#define I2C_MASTER_BOOT_ALT1    0x27U   //GPIO18; GPIO19
#define I2C_MASTER_BOOT_ALT2    0x47U   //GPIO0;  GPIO1

#define BROM_ASYSCTL_O_PMMREFTRIM   0x12U       //Power Management Module Reference Trim Register
#define BROM_ASYSCTL_O_PMMVREGTRIM  0x14U       //Power Management Module VREG Trim Register
#define BROM_ASYSCTL_O_PMMVMONTRIM  0x0EU       //Power Management Module VMON Trim Register
#define BROM_ASYSCTL_O_PMMVMONTRIM2 0x10U       //Power Management Module VMON Trim Register

#define BROM_ASYSCTL_O_OSCREFTRIM   0x06U       // OSC IREF,BGSLOPE,BGVAL TRIM
#define BROM_ASYSCTL_O_OSC1_TRIM    0x00U       // INTOSC1 SLOPE, VALFINE TRIMs
#define BROM_ASYSCTL_O_OSC2_TRIM    0x02U       // INTOSC2 SLOPE, VALFINE TRIMs


//---------------------------------------------------------------------------
// Fixed boot entry points:
//
#define FLASH_ENTRY_POINT               0x080000    //BANK0 Sector 0
#define FLASH_ENTRY_POINT_ALT1          0x084000    //BANK0 Sector 4
#define FLASH_ENTRY_POINT_ALT2          0x088000    //BANK0 sector 8
#define FLASH_ENTRY_POINT_ALT3          0x08EFF0    //BANK0 sector 15 end - 16


#define RAM_ENTRY_POINT                 0x000000U    //M0 start address

#define ERROR                   1U
#define NO_ERROR                0U

#define BROM_EIGHT_BIT_HEADER           0x08AAU

//---------------------------------------------------------------------------
//
#define HWREAD_TI_OTP_PARTID_L          (HWREG(0x70200))
#define HWREAD_TI_OTP_PARTID_H          (HWREG(0x70202))

#define TI_OTP_REG_DC01                 (HWREGH(0x70205UL))
#define TI_OTP_REG_DC03                 (HWREGH(0x70206UL))
#define TI_OTP_REG_DC15                 (HWREGH(0x70207UL))
#define TI_OTP_REG_DC21                 (HWREGH(0x70208UL))

#define TI_OTP_REG_CJTAGNODEID          (HWREGH(0x70223UL))
#define TI_OTP_REG_VREGCTL_ENMASK       (HWREGH(0x70224UL))

#define TI_OTP_CPUROM_DC1               (HWREGH(0x7020CUL))
#define TI_OTP_CPUROM_DC2               (HWREGH(0x7020DUL))
#define TI_OTP_CPUROM_DC3               (HWREGH(0x7020EUL))
#define TI_OTP_CPUROM_DC4               (HWREGH(0x7020FUL))

#define BROM_DCX_ALWAYS_ENABLED         0xFFFFFFFFUL
#define BROM_DCX_ENABLE_HIGH            0xFFFF0000UL
#define BROM_DCX_ENABLE_LOW             0x0000FFFFUL

#define TI_OTP_PKG_TYPE                 (HWREGH(0x70204UL))
#define PKG_TYPE_KEY                    0x5AU

#define BROM_ANALOG_SYSCTL_O_VREGCTL    0x006AU
#define VREGCTL_ENMASK_KEY              0x5AU

//bits15:8 is the KEY ; if Value == 0x5A then the remaining bits are valid
//bits 7:2 => reserved
//bits 0:1 if set to b'00 BROM will program 0x01 in VREGCTL.ENMASK
//          - any other value the VREGCTL.ENMASK will be left at reset state.

#define TI_OTP_REG_VREGCTL_ENMASK_VAL   ((TI_OTP_REG_VREGCTL_ENMASK) & 0x03U)

#define TI_OTP_REG_VREGCTL_ENMASK_KEY   (((TI_OTP_REG_VREGCTL_ENMASK) & 0xFF00U) >> 0x8U)


#define TI_OTP_SECDC                    0x703F0

//---------------------------------------------------------------------------
//


typedef uint16_t (*uint16fptr)();
extern  uint16fptr GetWordData;

#define HWREAD_TI_OTP_DEVCAL_KEY        (HWREGH(0x701D9UL))
#define DEVICE_CAL_LOCATION             0x70228
#define CBROM_DEVCAL                    ((void (*)(void))DEVICE_CAL_LOCATION)

#define HWREAD_OTP_VERSION_FOR_BOOT     (HWREGH(0x70222UL))

//Bits [1:0]    If 01, enable the PLL, otherwise leave it disabled
//Bits [7:2]    PLL divider to use when the PLL is enabled
//Bits [31:24]  If 0x5A, use this configuration word, otherwise use the default settings
#define OTP_BOOT_CONFIGURE_WORD         (HWREG(0x703EEUL))
#define BOOT_CONFIGURE_ENABLE_PLL       0x1U

#define BOOTPIN_CONFIG_STANDALONE_KEY   0xA5U
#define BOOTPIN_CONFIG_KEY              0x5AUL

#define APLL_MULT_38                    38UL
#define APLL_DIV_2                      1UL

#define SYSCLK_DIV_2                    1UL
#define SYSCLK_DIV_4                    2UL

#define CAN_BOOT_DEFAULT_BIT_TIMING     0x0U
#define CAN_BOOT_USE_XTAL               0x1U


//
// APLL Trim - Analog Subsystem Register Offsets
//
// Refer to hw_asysctl.h

//
// APLL OTP Trim Values
//
#define HWREAD_TI_OTP_APLL_TRIM_KEY            (HWREGH(0x701D6UL))

#define HWREAD_TI_OTP_APLLREF_TRIM_ADDRESS     (HWREG(0x701E4UL))
#define HWREAD_TI_OTP_APLLREF_TRIM_VALUE       (HWREGH(HWREAD_TI_OTP_APLLREF_TRIM_ADDRESS + 0U))

#define TI_OTP_APLL_SYS_TRIM_ADDRESS           (HWREG(0x701E6UL))
#define HWREAD_TI_OTP_APLL_SYS_TRIM_VALUE      (HWREGH(TI_OTP_APLL_SYS_TRIM_ADDRESS + 0U))

//
// APLL OTP Trim Masks/Shifts
//
#define TI_OTP_APLL_OSC_FREQ_MASK       0x700U
#define TI_OTP_APLL_OSC_FREQ_SHIFT      0x8U
#define TI_OTP_APLL_OSC_VDD_MASK        0xFFU

//
// APLL Lock - Analog Subsystem Register Offset
//
// Refer to hw_asysctl.h

//
// APLL Lock (OTP)
//
#define HWREAD_TI_OTP_APLLLOCK_KEY             (HWREGH(0x701D7UL))
#define HWREAD_TI_OTP_APLLLOCK_VALUE           (HWREG(0x70214))


//
// APLL Configurations (Both SYS and AUX) (OTP)
//
#define HWREAD_TI_OTP_APLLCONFIG_KEY           (HWREGH(0x701D8UL))
#define TI_OTP_APLLCONFIG_BASE_ADDRESS         (0x70216UL)
#define HWREAD_TI_OTP_APLLCONFIG1_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 0U))
#define HWREAD_TI_OTP_APLLCONFIG2_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 1U))
#define HWREAD_TI_OTP_APLLCONFIG3_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 2U))
#define HWREAD_TI_OTP_APLLCONFIG4_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 3U))
#define HWREAD_TI_OTP_APLLCONFIG5_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 4U))
#define HWREAD_TI_OTP_APLLCONFIG7_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 6U))
#define HWREAD_TI_OTP_APLLCONFIG8_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 7U))
#define HWREAD_TI_OTP_APLLCONFIG9_VALUE        (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 8U))
#define HWREAD_TI_OTP_APLLCONFIG10_VALUE       (HWREGH(TI_OTP_APLLCONFIG_BASE_ADDRESS + 9U))


#define OTP_BOOT_ESCAPE_TABLE_END       0x703EC

//extern void otp_func_refs();
#define CPU1BROM_TI_OTP_ESCAPE_POINT_15         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-28))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_14         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-26))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_13         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-24))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_12         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-22))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_11         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-20))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_10         (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-18))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_9          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-16))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_8          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-14))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_7          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-12))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_6          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-10))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_5          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-8))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_4          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-6))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_3          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-4))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_2          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-2))
#define CPU1BROM_TI_OTP_ESCAPE_POINT_1          (HWREG((OTP_BOOT_ESCAPE_TABLE_END)-0))

//
// Boot Selection Defines
//
#define CPU1_ALL_BMSP_DISABLED_MASK     0x00FFFFFFUL
#define CPU1_PIN_CONFIG_MASK            0xFFU
#define CPU1_BMSP_DISABLED              0xFFU

//
// OTP Keys
//
#define TI_OTP_KEY                              0x5A5AU

//
// SIMRESET Key
//
#define SIMRESET_KEY                            0xA5A5UL

//
// PMM Trim - Analog Subsystem Register Offsets
//
// Refer to hw_asysctl.h

// //
// // PMM OTP Trim Values
// //
#define TI_OTP_PMM_LC_TRIM_KEY                  (HWREGH(0x701D4))
#define TI_OTP_PMM_REF_TRIM_ADDRESS             HWREG(0x701DAUL)
#define HWREAD_TI_OTP_PMM_REF_TRIM_VALUE        (HWREGH(TI_OTP_PMM_REF_TRIM_ADDRESS + 0U))
#define TI_OTP_PMM_VMON_TRIM_ADDRESS            HWREG(0x701DCUL)
#define HWREAD_TI_OTP_PMM_VMON_TRIM_VALUE       (HWREGH(TI_OTP_PMM_VMON_TRIM_ADDRESS + 0U))

//
// Internal Oscillator OTP Trim Values
//
#define HWREAD_TI_OTP_INTOSC_TRIM_KEY           (HWREGH(0x701D5))
#define HWREAD_TI_OTP_OSC_REF_ADDRESS           (HWREG(0x701DEUL))
#define HWREAD_TI_OTP_INTOSC1_ADDRESS           (HWREG(0x701E0UL))
#define HWREAD_TI_OTP_INTOSC2_ADDRESS           (HWREG(0x701E2UL))

#define HWREAD_TI_OTP_OSC_REF_TRIM_VALUE       (HWREGH(HWREAD_TI_OTP_OSC_REF_ADDRESS + 0U))
#define HWREAD_TI_OTP_INTOSC1_TRIM_VALUE       (HWREG(HWREAD_TI_OTP_INTOSC1_ADDRESS))
#define HWREAD_TI_OTP_INTOSC2_TRIM_VALUE       (HWREG(HWREAD_TI_OTP_INTOSC2_ADDRESS))



#define DCSM_O_Zx_EXEONLYRAM        0x0UL
#define DCSM_O_Zx_EXEONLYSECT       0x2UL
#define DCSM_O_Zx_GRABRAM           0x4UL
#define DCSM_O_Zx_GRABSECT          0x6UL

//
// CPU1 Boot ROM Status Bit Fields
//
#define CPU1_BOOTROM_BOOTSTS_BOOT_MASK              0x000000FFUL

#define CPU1_BOOTROM_BOOTSTS_SYSTEM_START_BOOT      0x00000001UL  //Set during the initialization phase of the boot ROM
#define CPU1_BOOTROM_BOOTSTS_IN_FLASH_BOOT          0x00000002UL
#define CPU1_BOOTROM_BOOTSTS_IN_PARALLEL_BOOT       0x00000004UL
#define CPU1_BOOTROM_BOOTSTS_IN_RAM_BOOT            0x00000008UL
#define CPU1_BOOTROM_BOOTSTS_IN_SCI_BOOT            0x00000010UL
#define CPU1_BOOTROM_BOOTSTS_IN_SPI_BOOT            0x00000020UL
#define CPU1_BOOTROM_BOOTSTS_IN_I2C_BOOT            0x00000040UL
#define CPU1_BOOTROM_BOOTSTS_IN_CAN_BOOT            0x00000080UL

#define CPU1_BOOTROM_RAM_INIT_COMPLETE              0x00000100UL
#define CPU1_BOOTROM_DCSM_INIT_COMPLETE             0x00000200UL
#define CPU1_BOOTROM_POR_MEM_TEST_COMPLETE          0x00000400UL
#define CPU1_BOOTROM_RESC_HANDLED                   0x00000800UL
#define CPU1_BOOTROM_HANDLED_XRSN                   0x00001000UL
#define CPU1_BOOTROM_HANDLED_POR                    0x00002000UL
#define CPU1_BOOTROM_BOOT_COMPLETE                  0x00008000UL
                                                    
#define CPU1_BOOTROM_GOT_ITRAP                      0x00010000UL
#define CPU1_BOOTROM_GOT_A_PIE_MISMATCH             0x00020000UL
#define CPU1_BOOTROM_GOT_AN_ERAD_NMI                0x00040000UL
#define CPU1_BOOTROM_GOT_A_RL_NMI                   0x00080000UL
#define CPU1_BOOTROM_GOT_A_FLASH_UNCERR_NMI         0x00100000UL
#define CPU1_BOOTROM_GOT_A_RAM_UNCERR_NMI           0x00200000UL
#define CPU1_BOOTROM_GOT_A_MCLK_NMI                 0x00400000UL
#define CPU1_BOOTROM_GOT_A_HWBIST_NMI               0x00800000UL

#define BOOTROM_PLL_ENABLE_SUCCESS                  0x01000000UL
#define CPU1_BOOTROM_BOOTSTS_IN_WAIT_BOOT           0x40000000UL
#define BOOTROM_HANDLED_HWBIST                      0x80000000UL

//
// BootROM System Clock (10MHz)
//
#define BOOTROM_SYSCLK                 10000000UL



//Function prototypes
extern void cbrom_configure_flash();
extern void CPU1BROM_initDCSM(void);
extern uint32_t Gather_Bx_Zx_ZSB(uint16_t bank, uint16_t zone, uint32_t *csmkey);

extern interrupt void CPU1BROM_itrapISR(void);
extern interrupt void CPU1BROM_nmiHandler(void);

extern uint32_t GetLongData(void);
extern void CopyData(void);
extern void ReadReservedFn(void);

extern uint32_t I2C_Boot(uint32_t  bootMode);
extern uint32_t SCI_Boot(uint32_t  bootMode);
extern uint32_t SPI_Boot(uint32_t  bootMode);
uint32_t DCAN_Boot(uint32_t bootMode, uint32_t bitTimingRegValue, uint16_t switchToXTAL);
extern uint32_t SPI_Alternate_IO_Boot();
extern uint32_t Parallel_Boot(uint32_t  BootMode);

extern uint32_t CPU1BROM_selectBootMode(void);

extern void CPU1BROM_triggerSysPLLLock(uint32_t multiplier, uint32_t divider);
extern uint16_t CPU1BROM_switchToPLL(void);
extern uint32_t PBIST_PORMemoryTest(void);

#endif //C_BOOTROM_H_
