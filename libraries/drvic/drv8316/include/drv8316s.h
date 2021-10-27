//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:26 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef DRV8316S_H
#define DRV8316S_H

//! \file   libraries/drvic/drv8316/include/drv8316s.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8316 object
//!

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup DRV8316 DRV8316
//! @{
//
//*****************************************************************************

// the includes
#include <math.h>

// drivers
#include "spi.h"
#include "gpio.h"

// **************************************************************************
// modules

// **************************************************************************
// solutions

// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8316_ADDR_MASK                   (0x7E00)

//! \brief Defines the data mask
//!
#define DRV8316_DATA_MASK                   (0x00FF)

//! \brief Defines the R/W mask
//!
#define DRV8316_RW_MASK                     (0x8000)

//
// STATUS00
//
//! \brief Defines the R/W mask
//!
#define DRV8316_FAULT_TYPE_MASK             (0x00FF)

//
// STATUS00
//
#define DRV8316_STAT00_FAULT_BITS           (1 << 0)
#define DRV8316_STAT00_OT_BITS              (1 << 1)
#define DRV8316_STAT00_OVP_BITS             (1 << 2)
#define DRV8316_STAT00_NPOR_BITS            (1 << 3)
#define DRV8316_STAT00_OCP_BITS             (1 << 4)
#define DRV8316_STAT00_SPI_FLT_BITS         (1 << 5)
#define DRV8316_STAT00_BK_FLT_BITS          (1 << 6)
#define DRV8316_STAT00_RESERVED_BITS        (1 << 7)

//
// STATUS01
//
#define DRV8316_STAT01_OCP_LA_BITS          (1 << 0)
#define DRV8316_STAT01_OCP_HA_BITS          (1 << 1)
#define DRV8316_STAT01_OCP_LB_BITS          (1 << 2)
#define DRV8316_STAT01_OCP_HB_BITS          (1 << 3)
#define DRV8316_STAT01_OCP_LC_BITS          (1 << 4)
#define DRV8316_STAT01_OCP_HC_BITS          (1 << 5)
#define DRV8316_STAT01_OTS_BITS             (1 << 6)
#define DRV8316_STAT01_OTW_BITS             (1 << 7)

//
// STATUS02
//
#define DRV8316_STAT02_SPI_ADDR_FLT_BITS    (1 << 0)
#define DRV8316_STAT02_SPI_SCLK_FLT_BITS    (1 << 1)
#define DRV8316_STAT02_SPI_PARITY_BITS      (1 << 2)
#define DRV8316_STAT02_VCP_UV_BITS          (1 << 3)
#define DRV8316_STAT02_BUCK_UV_BITS         (1 << 4)
#define DRV8316_STAT02_BUCK_OCP_BITS        (1 << 5)
#define DRV8316_STAT02_OTP_ERR_BITS         (1 << 6)
#define DRV8316_STAT02_RESERVED_BITS        (1 << 7)

//
// CTRL03
//
#define DRV8316_CTRL03_REG_LOCK_BITS        (7 << 0)
#define DRV8316_CTRL03_RESERVED1_BITS       (1 << 3)
#define DRV8316_CTRL03_RESERVED2_BITS       (1 << 4)
#define DRV8316_CTRL03_RESERVED3_BITS       (1 << 5)
#define DRV8316_CTRL03_RESERVED4_BITS       (1 << 6)
#define DRV8316_CTRL03_RESERVED5_BITS       (1 << 7)

//
// CTRL04
//
#define DRV8316_CTRL04_CLR_FLT_BITS         (1 << 0)
#define DRV8316_CTRL04_PWM_MODE_BITS        (3 << 1)
#define DRV8316_CTRL04_SLEW_BITS            (3 << 3)
#define DRV8316_CTRL04_RESERVED1_BITS       (1 << 5)
#define DRV8316_CTRL04_RESERVED2_BITS       (1 << 6)
#define DRV8316_CTRL04_RESERVED3_BITS       (1 << 7)

//
// CTRL05
//
#define DRV8316_CTRL05_OTW_REP_BITS         (1 << 0)
#define DRV8316_CTRL05_SPI_FLT_REP_BITS     (1 << 1)
#define DRV8316_CTRL05_OVP_EN_BITS          (1 << 2)
#define DRV8316_CTRL05_OVP_SEL_BITS         (1 << 3)
#define DRV8316_CTRL05_RESERVED1_BITS       (1 << 4)
#define DRV8316_CTRL05_RESERVED2_BITS       (1 << 5)
#define DRV8316_CTRL05_RESERVED3_BITS       (1 << 6)
#define DRV8316_CTRL05_RESERVED4_BITS       (1 << 7)

//
// CTRL06
//
#define DRV8316_CTRL06_OCP_MODE_BITS        (3 << 0)
#define DRV8316_CTRL06_OCP_LVL_BITS         (1 << 2)
#define DRV8316_CTRL06_OCP_RETRY_BITS       (1 << 3)
#define DRV8316_CTRL06_OCP_DEG_BITS         (3 << 4)
#define DRV8316_CTRL06_OCP_CBC_BITS         (1 << 6)
#define DRV8316_CTRL06_RESERVED_BITS        (1 << 7)

//
// CTRL07
//
#define DRV8316_CTRL07_CSA_GAIN_BITS        (3 << 0)
#define DRV8316_CTRL07_EN_ASR_BITS          (1 << 2)
#define DRV8316_CTRL07_EN_AAR_BITS          (1 << 3)
#define DRV8316_CTRL07_AD_COMP_TH_HS_BITS   (1 << 4)
#define DRV8316_CTRL07_AD_COMP_TH_LS_BITS   (1 << 5)
#define DRV8316_CTRL07_ILIM_RECIR_BITS      (1 << 6)
#define DRV8316_CTRL07_BEMF_TH_BITS         (1 << 7)

//
// CTRL08
//
#define DRV8316_CTRL08_BUCK_DIS_BITS        (1 << 0)
#define DRV8316_CTRL08_BUCK_SEL_BITS        (3 << 1)
#define DRV8316_CTRL08_BUCK_CL_BITS         (1 << 3)
#define DRV8316_CTRL08_BUCK_PS_DIS_BITS     (1 << 4)
#define DRV8316_CTRL08_BUCK_SR_BITS         (1 << 5)
#define DRV8316_CTRL08_RESERVED1_BITS       (1 << 6)
#define DRV8316_CTRL08_RESERVED2_BITS       (1 << 7)

// **************************************************************************
// the typedefs

//------------------------------------------------------------------------------
//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DRV8316_CTRLMODE_WRITE    = (0 << 15),  //!< Write Mode
    DRV8316_CTRLMODE_READ     = (1 << 15)   //!< Read Mode
} DRV8316_CtrlMode_e;

//! \brief Enumeration for the register addresses
//!
typedef enum
{
    DRV8316_ADDRESS_STATUS_0   = (0x0 << 9),  //!< Status Register 0
    DRV8316_ADDRESS_STATUS_1   = (0x1 << 9),  //!< Status Register 1
    DRV8316_ADDRESS_STATUS_2   = (0x2 << 9),  //!< Status Register 2
    DRV8316_ADDRESS_CONTROL_1  = (0x3 << 9),  //!< Control Register 1
    DRV8316_ADDRESS_CONTROL_2  = (0x4 << 9),  //!< Control Register 2
    DRV8316_ADDRESS_CONTROL_3  = (0x5 << 9),  //!< Control Register 3
    DRV8316_ADDRESS_CONTROL_4  = (0x6 << 9),  //!< Control Register 4
    DRV8316_ADDRESS_CONTROL_5  = (0x7 << 9),  //!< Control Register 5
    DRV8316_ADDRESS_CONTROL_6  = (0x8 << 9),  //!< Control Register 6
    DRV8316_ADDRESS_CONTROL_10 = (0xC << 9)   //!< Control Register 10
} DRV8316_Address_e;

//! \brief Enumeration for the Status 0 register, faults
//!
typedef enum
{
    DRV8316_FAULT       = (1 << 0),    //!< Device Fault
    DRV8316_OT          = (1 << 1),    //!< Overtemperature Fault
    DRV8316_OVP         = (1 << 2),    //!< Supply Overvoltage Protection
    DRV8316_NPOR        = (1 << 3),    //!< Supply Power On Reset
    DRV8316_OCP         = (1 << 4),    //!< Over Current Protection
    DRV8316_SPI_FLT     = (1 << 5),    //!< SPI Fault
    DRV8316_BK_FLT      = (1 << 6)     //!< Buck Fault
} DRV8316_STATUS00_e;

//! \brief Enumeration for the Status 1 register, faults
//!
typedef enum
{
    DRV8316_OCP_LA      = (1 << 0),    //!< Overcurrent Status on Low-side switch of OUTA
    DRV8316_OCP_HA      = (1 << 1),    //!< Overcurrent Status on High-side switch of OUTA
    DRV8316_OCP_LB      = (1 << 2),    //!< Overcurrent Status on Low-side switch of OUTB
    DRV8316_OCP_HB      = (1 << 3),    //!< Overcurrent Status on High-side switch of OUTB
    DRV8316_OCL_LC      = (1 << 4),    //!< Overcurrent Status on Low-side switch of OUTC
    DRV8316_OCP_HC      = (1 << 5),    //!< Overcurrent Status on High-side switch of OUTC
    DRV8316_OTS         = (1 << 6),    //!< Overtemperature Shutdown Status
    DRV8316_OTW         = (1 << 7)     //!< Overtemperature Warning Status
} DRV8316_STATUS01_e;

//! \brief Enumeration for the Status 1 register, faults
//!
typedef enum
{
    DRV8316_SPI_ADDR_FLT  = (1 << 0),    //!< SPI Address Error
    DRV8316_SPI_SCLK_FLT  = (1 << 1),    //!< SPI Clock Framing Error
    DRV8316_SPI_PARITY    = (1 << 2),    //!< SPI Parity Error
    DRV8316_VCP_UV        = (1 << 3),    //!< Charge Pump Undervoltage Status
    DRV8316_BUCK_UV       = (1 << 4),    //!< Buck Regulator Undervoltage Staus
    DRV8316_BUCK_OCP      = (1 << 5),    //!< Buck Regulator Overcurrent Staus
    DRV8316_OTP_ERR       = (1 << 6),    //!< One Time Programmabilty Error
    DRV8316_RESERVED      = (1 << 7)     //!< Reserved
} DRV8316_STATUS02_e;

//! \brief Enumeration for the high side gate drive peak source current;
//! gate currents not consistent with DS
//!
typedef enum
{
    DRV8316_REG_LOCK_0 = 0,  //!< No effect unless locked or unlocked
    DRV8316_REG_LOCK_1 = 1,  //!< No effect unless locked or unlocked
    DRV8316_REG_LOCK_2 = 2,  //!< No effect unless locked or unlocked
    DRV8316_REG_LOCK_3 = 3,  //!< Write 011b to this register to unlock all registers
    DRV8316_REG_LOCK_4 = 4,  //!< No effect unless locked or unlocked
    DRV8316_REG_LOCK_5 = 5,  //!< No effect unless locked or unlocked
    DRV8316_REG_LOCK_6 = 6,  //!< Write 110b to lock the settings by ignoring further register
    DRV8316_REG_LOCK_7 = 7   //!< No effect unless locked or unlocked
} DRV8316_CTRL01_RegLock_e;

//! \brief Enumeration for the driver PWM mode
//!
typedef enum
{
    DRV8316_PWMMODE_6_N  = 0,     //!< PWM_MODE = 6 inputs
    DRV8316_PWMMODE_6_LC = 1,     //!< PWM_MODE = 6 inputs with current limit
    DRV8316_PWMMODE_3_N  = 2,     //!< PWM_MODE = 3 inputs
    DRV8316_PWMMODE_3_LC = 3,     //!< PWM_MODE = 3 inputs with current limit
} DRV8316_CTRL02_PWMMode_e;

//! \brief Enumeration for the slew rate
//!
typedef enum
{
    DRV8316_SLEW_25V  = 0,      //!< Slew rate is 25  V/μs
    DRV8316_SLEW_50V  = 1,      //!< Slew rate is 50  V/μs
    DRV8316_SLEW_150V = 2,      //!< Slew rate is 150 V/μs
    DRV8316_SLEW_200V = 3       //!< Slew rate is 200 V/μs
} DRV8316_CTRL02_SlewRate_e;

//! \brief Enumeration for the SDO Mode Setting
//!
typedef enum
{
    DRV8316_SDOMODE_OPEN_DRAIN  = 0,     //!< SDO IO in Open Drain Mode
    DRV8316_SDOMODE_PUSH_PULL   = 1      //!< SDO IO in Push Pull Mode
} DRV8316_CTRL02_SDOMode_e;


//! \brief Enumeration for the Overvoltage Level Setting
//!
typedef enum
{
    DRV8316_OVPSEL_32V  = 0,        //!< VM overvoltage level is 32-V
    DRV8316_OVPSEL_20V   = 1        //!< VM overvoltage level is 20-V
} DRV8316_CTRL03_OVPSEL_e;


//! \brief Enumeration for the Freqency of PWM at 100% Duty Cycle
//!
typedef enum
{
    DRV8316_PWM_100_DUTY_SEL_20KHz  = 0,     //!< 20KHz
    DRV8316_PWM_100_DUTY_SEL_40KHz   = 1      //!< 40KHz
} DRV8316_CTRL03_DUTYSEL_e;


//! \brief Enumeration for the OCP Fault Options
//!
typedef enum
{
    DRV8316_OCP_MODE_LATCH   = 0,   //!< Overcurrent causes a latched fault
    DRV8316_OCP_MODE_RETRY   = 1,   //!< Overcurrent causes an automatic retrying fault
    DRV8316_OCP_MODE_REPORT  = 2,   //!< Overcurrent is report only but no action is taken
    DRV8316_OCP_MODE_NO      = 3    //!< Overcurrent is not reported and no action is taken
} DRV8316_CTRL04_OCPMODE_e;

//! \brief Enumeration for the Overcurrent Level Setting
//!
typedef enum
{
    DRV8316_OCP_LVL_16A  = 0,     //!< 16A
    DRV8316_OCP_LVL_24A  = 1      //!< 24A
} DRV8316_CTRL04_OCPLVL_e;

//! \brief Enumeration for the OCP Retry Time Settings
//!
typedef enum
{
    DRV8316_OCP_RETRY_5ms   = 0,     //!< OCP retry time is 5 ms
    DRV8316_OCP_RETRY_500ms = 1      //!< OCP retry time is 500 ms
} DRV8316_CTRL04_OCPRETRY_e;

//! \brief Enumeration for the OCP Deglitch Time Settings
//!
typedef enum
{
    DRV8316_OCP_DEG_0p2us  = 0,   //!< OCP deglitch time is 0.2 μs
    DRV8316_OCP_DEG_0p6us  = 1,   //!< OCP deglitch time is 0.6 μs
    DRV8316_OCP_DEG_1p1us  = 2,   //!< OCP deglitch time is 1.1 μs
    DRV8316_OCP_DEG_1p6us  = 3    //!< OCP deglitch time is 1.6 μs
} DRV8316_CTRL04_OCPDEGE_e;

//! \brief Enumeration for the Current Sense Amplifier's Gain Settings
//!
typedef enum
{
    DRV8316_CSA_GAIN_0p15VpA   = 0,   //!< CSA gain is 0.15 V/A
    DRV8316_CSA_GAIN_0p1875VpA = 1,   //!< CSA gain is 0.1875 V/A
    DRV8316_CSA_GAIN_0p25VpA   = 2,   //!< CSA gain is 0.25 V/A
    DRV8316_CSA_GAIN_0p375VpA  = 3,   //!< CSA gain is 0.375 V/A
} DRV8316_CTRL05_CSAGain_e;

//! \brief Enumeration for the Current Limit Recirculation Settings
//!
typedef enum
{
    DRV8316_ILIM_RECIR_Brake   = 0,     //!< Current recirculation through FETs (Brake Mode)
    DRV8316_ILIM_RECIR_Coast   = 1      //!< Current recirculation through diodes (Coast Mode)
} DRV8316_CTRL05_ILIMRECIR_e;


//! \brief Enumeration for the Buck Voltage Selection
//!
typedef enum
{
    DRV8316_BUCK_SEL_3p3V  = 0,   //!< Buck voltage is 3.3 V
    DRV8316_BUCK_SEL_5p0V  = 1,   //!< Buck voltage is 5.0 V
    DRV8316_BUCK_SEL_4p0V  = 2,   //!< Buck voltage is 4.0 V
    DRV8316_BUCK_SEL_5p7V  = 3,   //!< Buck voltage is 5.7 V
} DRV8316_CTRL06_BUCKSEL_e;

//! \brief Enumeration for the Current Limit Recirculation Settings
//!
typedef enum
{
    DRV8316_BUCK_CL_600mA   = 0,     //!< Buck regulator current limit is set to 600 mA
    DRV8316_BUCK_CL_150mA   = 1      //!< Buck regulator current limit is set to 150 mA
} DRV8316_CTRL06_BUCKCL_e;


//! \brief Enumeration for the Delay Target for Driver Delay Compensation
//!
typedef enum
{
    DRV8316_DLY_TARGET_0p0us  = 0x0,   //!< Delay is 0 us
    DRV8316_DLY_TARGET_0p4us  = 0x1,   //!< Delay is 0.4 us
    DRV8316_DLY_TARGET_0p6us  = 0x2,   //!< Delay is 0.6 us
    DRV8316_DLY_TARGET_0p8us  = 0x3,   //!< Delay is 0.8 us
    DRV8316_DLY_TARGET_1p0us  = 0x4,   //!< Delay is 1.0 us
    DRV8316_DLY_TARGET_1p2us  = 0x5,   //!< Delay is 1.2 us
    DRV8316_DLY_TARGET_1p4us  = 0x6,   //!< Delay is 1.4 us
    DRV8316_DLY_TARGET_1p6us  = 0x7,   //!< Delay is 1.6 us
    DRV8316_DLY_TARGET_1p8us  = 0x8,   //!< Delay is 1.8 us
    DRV8316_DLY_TARGET_2p0us  = 0x9,   //!< Delay is 2.0 us
    DRV8316_DLY_TARGET_2p2us  = 0xA,   //!< Delay is 2.2 us
    DRV8316_DLY_TARGET_2p4us  = 0xB,   //!< Delay is 2.4 us
    DRV8316_DLY_TARGET_2p6us  = 0xC,   //!< Delay is 2.6 us
    DRV8316_DLY_TARGET_2p8us  = 0xD,   //!< Delay is 2.8 us
    DRV8316_DLY_TARGET_3p0us  = 0xE,   //!< Delay is 3.0 us
    DRV8316_DLY_TARGET_3p2us  = 0xF    //!< Delay is 3.2 us
} DRV8316_CTRL10_DLYTARGET_e;

//------------------------------------------------------------------------------
//! \brief Object for the DRV8316 STATUS00 register
//!
struct DRV8316_STAT00_BITS {                    // bits description
    bool                FAULT:1;            // Bits 0
    bool                OT:1;               // Bits 1
    bool                OVP:1;              // Bits 2
    bool                NPOR:1;             // Bits 3
    bool                OCP:1;              // Bits 4
    bool                SPI_FLT:1;          // Bits 5
    bool                BK_FLT:1;           // Bits 6
    bool                rsvd1:1;            // Bits 7
};

union DRV8316_STAT00_REG
{
    uint16_t all;
    struct   DRV8316_STAT00_BITS bit;
};

//! \brief Object for the DRV8316 STATUS01 register
//!
struct DRV8316_STAT01_BITS {                    // bits description
    bool                OCP_LA:1;           // Bits 0
    bool                OCP_HA:1;           // Bits 1
    bool                OCP_LB:1;           // Bits 2
    bool                OCP_HB:1;           // Bits 3
    bool                OCL_LC:1;           // Bits 4
    bool                OCP_HC:1;           // Bits 5
    bool                OTS:1;              // Bits 6
    bool                OTW:1;              // Bits 7
};

union DRV8316_STAT01_REG
{
    uint16_t all;
    struct   DRV8316_STAT01_BITS bit;
};

//! \brief Object for the DRV8316 STATUS02 register
//!
struct DRV8316_STAT02_BITS {                    // bits description
    bool                SPI_ADDR_FLT:1;     // Bits 0
    bool                SPI_SCLK_FLT:1;     // Bits 1
    bool                SPI_PARITY:1;       // Bits 2
    bool                VCP_UV:1;           // Bits 3
    bool                BUCK_UV:1;          // Bits 4
    bool                BUCK_OCP:1;         // Bits 5
    bool                OTP_ERR:1;          // Bits 6
    uint16_t            rsvd2:5;            // Bits 7 Reserved
};

union DRV8316_STAT02_REG
{
    uint16_t all;
    struct   DRV8316_STAT02_BITS bit;
};

//! \brief Object for the DRV8316 CTRL01 register
//!
struct DRV8316_CTRL01_BITS {                    // bits description
    DRV8316_CTRL01_RegLock_e    REG_LOCK:3;       // Bits 2:0
    uint16_t                    rsvd1:5;          // 7:3 Reserved
};

union DRV8316_CTRL01_REG
{
    uint16_t all;
    struct   DRV8316_CTRL01_BITS bit;
};

//! \brief Object for the DRV8316 CTRL03 register
//!
struct DRV8316_CTRL02_BITS {                    // bits description
    bool                      CLR_FLT:1;        // Bits 0
    DRV8316_CTRL02_PWMMode_e  PWM_MODE:2;       // Bits 2-1
    DRV8316_CTRL02_SlewRate_e SLEW:2;           // Bits 4-3
    DRV8316_CTRL02_SDOMode_e  SDO_MODE:1;       // Bits 5
    uint16_t                  rsvd1:2;          // 7:6 Reserved
};

union DRV8316_CTRL02_REG
{
    uint16_t all;
    struct   DRV8316_CTRL02_BITS bit;
};

//! \brief Object for the DRV8316 CTRL03 register
//!
struct DRV8316_CTRL03_BITS {                    // bits description
    bool                      OTW_REP:1;            // Bits 0
    bool                      SPI_FLT_REP:1;        // Bits 1
    bool                      OVP_EN:1;             // Bits 2
    DRV8316_CTRL03_OVPSEL_e   OVP_SEL:1;            // Bits 3
    DRV8316_CTRL03_DUTYSEL_e  PWM_100_DUTY_SEL:1;   // Bits 4
    uint16_t                  rsvd1:3;              // 7:5 Reserved
};

union DRV8316_CTRL03_REG
{
    uint16_t all;
    struct   DRV8316_CTRL03_BITS bit;
};

//! \brief Object for the DRV8316 CTRL04 register
//!
struct DRV8316_CTRL04_BITS {                    // bits description
    DRV8316_CTRL04_OCPMODE_e   OCP_MODE:2;      // Bits 1-0
    DRV8316_CTRL04_OCPLVL_e    OCP_LVL:1;       // Bits 2
    DRV8316_CTRL04_OCPRETRY_e  OCP_RETRY:1;     // Bits 3
    DRV8316_CTRL04_OCPDEGE_e   OCP_DEG:2;       // Bits 5-4
    bool                       OCP_CBC:1;       // Bits 6
    bool                       DRV_OFF:1;       // Bits 7
};

union DRV8316_CTRL04_REG
{
    uint16_t all;
    struct   DRV8316_CTRL04_BITS bit;
};

//! \brief Object for the DRV8316 CTRL05 register
//!
struct DRV8316_CTRL05_BITS {                    // bits description
    DRV8316_CTRL05_CSAGain_e   CSA_GAIN:2;      // Bits 1-0
    bool                       EN_ASR:1;        // Bits 2
    bool                       EN_AAR:1;        // Bits 3
    uint16_t                   rsvd1:1;         // Bits 4 Reserved
    uint16_t                   rsvd2:1;         // Bits 5 Reserved
    DRV8316_CTRL05_ILIMRECIR_e ILIM_RECIR:1;    // Bits 6
    uint16_t                   rsvd3:1;         // Bits 7 Reserved
};

union DRV8316_CTRL05_REG
{
    uint16_t all;
    struct   DRV8316_CTRL05_BITS bit;
};

//! \brief Object for the DRV8316 CTRL06 register
//!
struct DRV8316_CTRL06_BITS {                    // bits description
    bool                        BUCK_DIS:1;     // Bits 0
    DRV8316_CTRL06_BUCKSEL_e    BUCK_SEL:2;     // Bits 2-1
    DRV8316_CTRL06_BUCKCL_e     BUCK_CL:1;      // Bits 3
    bool                        BUCK_PS_DIS:1;  // Bits 4
    uint16_t                    rsvd1:3;        // Bits 7:5 Reserved
};

union DRV8316_CTRL06_REG
{
    uint16_t all;
    struct   DRV8316_CTRL06_BITS bit;
};

//! \brief Object for the DRV8316 CTRL10 register
//!
struct DRV8316_CTRL10_BITS {                    // bits description
    DRV8316_CTRL10_DLYTARGET_e  DLY_TARGET:4;   // Bits 3-0
    bool                        DLYCMP_EN:1;    // Bits 4
    uint16_t                    rsvd1:3;        // Bits 7:5 Reserved
};

union DRV8316_CTRL10_REG
{
    uint16_t all;
    struct   DRV8316_CTRL10_BITS bit;
};

//! \brief Object for the DRV8316 registers and commands
//!
typedef struct _DRV8316_VARS_t_
{
    union DRV8316_STAT00_REG    statReg00;
    union DRV8316_STAT01_REG    statReg01;
    union DRV8316_STAT02_REG    statReg02;

    union DRV8316_CTRL01_REG    ctrlReg01;
    union DRV8316_CTRL02_REG    ctrlReg02;
    union DRV8316_CTRL03_REG    ctrlReg03;
    union DRV8316_CTRL04_REG    ctrlReg04;
    union DRV8316_CTRL05_REG    ctrlReg05;
    union DRV8316_CTRL06_REG    ctrlReg06;
    union DRV8316_CTRL10_REG    ctrlReg10;

    bool                writeCmd;
    bool                readCmd;
    uint16_t            manWriteAddr;
    uint16_t            manReadAddr;
    uint16_t            manWriteData;
    uint16_t            manReadData;
    bool                manWriteCmd;
    bool                manReadCmd;
}DRV8316_VARS_t;

//! \brief Defines the DRV8316_VARS_t handle
//!
typedef struct _DRV8316_VARS_t_ *DRV8316VARS_Handle;

//! \brief Defines the DRV8316 object
//!
typedef struct _DRV8316_Obj_
{
    uint32_t  spiHandle;     //!< handle for the serial peripheral interface
    uint32_t  gpioNumber_CS; //!< GPIO connected to the DRV8316 CS pin
    uint32_t  gpioNumber_EN; //!< GPIO connected to the DRV8316 enable pin
    bool      rxTimeOut;     //!< timeout flag for the RX FIFO
    bool      enableTimeOut; //!< timeout flag for DRV8316 enable
} DRV8316_Obj;

//! \brief Defines the DRV8316 handle
//!
typedef struct _DRV8316_Obj_ *DRV8316_Handle;

//! \brief Defines the DRV8316 Word type
//!
typedef  uint16_t    DRV_Word_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8316 object
//! \param[in] pMemory   A pointer to the memory for the DRV8316 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8316
//!                      object, bytes
//! \return    The DRV8316 object handle
extern DRV8316_Handle DRV8316_init(void *pMemory);

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV_Word_t DRV8316_buildCtrlWord(
                                            const DRV8316_CtrlMode_e ctrlMode,
                                            const DRV8316_Address_e regAddr,
                                            const uint16_t data)
{
    uint16_t p_addr = regAddr;
    uint16_t p_data = data;
    uint16_t p_mode = ctrlMode;

    uint16_t calc = (p_mode & 0x8000) | (p_addr & 0x7E00) | (p_data & 0x00FF);
    uint16_t parity = 0;
    while(calc)
    {
        parity ^= (calc & 1);
        calc >>= 1;
    }

    parity <<= 8;

    DRV_Word_t ctrlWord = ctrlMode | regAddr | parity | (data & DRV8316_DATA_MASK);

    return(ctrlWord);
} // end of DRV8316_buildCtrlWord() function

//! \brief     Enables the DRV8316
//! \param[in] handle     The DRV8316 handle
extern void DRV8316_enable(DRV8316_Handle handle);

//! \brief     Sets the SPI handle in the DRV8316
//! \param[in] handle     The DRV8316 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8316_setSPIHandle(DRV8316_Handle handle,uint32_t spiHandle);

//! \brief     Sets the GPIO number in the DRV8316
//! \param[in] handle       The DRV8316 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8316_setGPIOCSNumber(DRV8316_Handle handle,uint32_t gpioNumber);

//! \brief     Sets the GPIO number in the DRV8316
//! \param[in] handle       The DRV8316 handle
//! \param[in] gpioHandle   The GPIO number to use
void DRV8316_setGPIOENNumber(DRV8316_Handle handle,uint32_t gpioNumber);

//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8316 handle
static inline void DRV8316_resetEnableTimeout(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    obj->enableTimeOut = false;

    return;
} // end of DRV8316_resetEnableTimeout() function

//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8316 handle
static inline void DRV8316_resetRxTimeout(DRV8316_Handle handle)
{
    DRV8316_Obj *obj = (DRV8316_Obj *)handle;

    obj->rxTimeOut = false;

    return;
} // end of DRV8316_resetRxTimeout() function

//! \brief     Initialize the interface to all DRV8316 SPI variables
//! \param[in] handle  The DRV8316 handle
extern void DRV8316_setupSPI(DRV8316_Handle handle,
                             DRV8316_VARS_t *drv8316Vars);

//! \brief     Reads data from the DRV8316 register
//! \param[in] handle   The DRV8316 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8316_readSPI(DRV8316_Handle handle,
                                const DRV8316_Address_e regAddr);

//! \brief     Writes data to the DRV8316 register
//! \param[in] handle   The DRV8316 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8316_writeSPI(DRV8316_Handle handle,
                             const DRV8316_Address_e regAddr,
                             const uint16_t data);

//! \brief     Write to the DRV8316 SPI registers
//! \param[in] handle  The DRV8316 handle
//! \param[in] drv8316Vars  The (DRV8316_VARS_t) structure that contains
//!                           all DRV8316 Status/Control register options
extern void DRV8316_writeData(DRV8316_Handle handle,
                              DRV8316_VARS_t *drv8316Vars);

//! \brief     Read from the DRV8316 SPI registers
//! \param[in] handle  The DRV8316 handle
//! \param[in] drv8316Vars  The (DRV8316_VARS_t) structure that contains
//!                           all DRV8316 Status/Control register options
extern void DRV8316_readData(DRV8316_Handle handle,
                             DRV8316_VARS_t *drv8316Vars);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // end of DRV8316S_H definition
