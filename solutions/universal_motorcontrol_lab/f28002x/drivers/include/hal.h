//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
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

#ifndef HAL_H
#define HAL_H

//! \file   \solutions\fast_uni_lab\f28002x\drivers\include\hal.h
//! \brief  Contains public interface to various functions related
//!         to the HAL object
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
//! \addtogroup HAL HAL
//! @{
//
//*****************************************************************************

// the includes
#include "userParams.h"

// platforms
#include "hal_obj.h"

#include "svgen_current.h"

#if defined(MOTOR1_DCLINKSS)
#include "dclink_ss.h"
#endif // MOTOR1_DCLINKSS

#ifdef _FLASH
extern uint16_t ctrlfuncsLoadStart;
extern uint16_t ctrlfuncsLoadEnd;
extern uint16_t ctrlfuncsLoadSize;
extern uint16_t ctrlfuncsRunStart;
extern uint16_t ctrlfuncsRunEnd;
extern uint16_t ctrlfuncsRunSize;
#endif

//SPRUIN7, P91, Table 3-3. PIE Channel Mapping
//SPRUIN7, P1304, Figure 14-1. Analog Subsystem Block Diagram (80-Pin and 64-Pin QFP)
//SPRUIN7, P1305, Figure 14-3. Analog Group Connections
//SPRUIN7, P1502, Figure 16-1. CMPSS Module Block Diagram
//SPRUIN7, P1599, Figure 17-49. Digital-Compare Submodule High-Level Block Diagram
//SPRUIN7, P1600, Figure 17-50. GPIO MUX-to-Trip Input Connectivity

//SPRSP45, P37&38, Table 4-6. GPIO Muxed Pins
//SPRSP45, P76, Figure 5-24. Analog Subsystem Block Diagram (80-Pin PN and 64-Pin PM LQFPs)
//SPRSP45, P78, Table 5-34. Analog Pins and Internal Connections
//SPRSP45, P89, Figure 5-30. CMPSS Connectivity

//SPRSP45, P141, Table 6-1. Memory Map

// **************************************************************************
// the defines
//
// 50MHz LSPCLK frequency based on the DEVICE_SYSCLK_FREQ and
#define C28X_LSPCLK_PRESCALE        SYSCTL_LSPCLK_PRESCALE_4
#define C28X_LSPCLK_FREQ            (DEVICE_SYSCLK_FREQ / 4)


#define USER_RAM_START_ADDR         0x00000240  // M0, M1
#define USER_RAM_VARS_SIZE          0x000005B0

#define CTRL_RAM_START_ADDR         0x0000C000  // GS0
#define CTRL_RAM_VARS_SIZE          0x00000000

#define MTRS_RAM_START_ADDR         0x0000C000  // GS0
#define MTRS_RAM_VARS_SIZE          0x000007F0

//! Trip Zones all interrupt
//!
#define HAL_TZFLAG_INTERRUPT_ALL    EPWM_TZ_INTERRUPT_DCBEVT2 |                \
                                    EPWM_TZ_INTERRUPT_DCBEVT1 |                \
                                    EPWM_TZ_INTERRUPT_DCAEVT2 |                \
                                    EPWM_TZ_INTERRUPT_DCAEVT1 |                \
                                    EPWM_TZ_INTERRUPT_OST |                    \
                                    EPWM_TZ_INTERRUPT_CBC

#define HAL_TZSEL_SIGNALS_ALL       EPWM_TZ_SIGNAL_CBC1 |                      \
                                    EPWM_TZ_SIGNAL_CBC2 |                      \
                                    EPWM_TZ_SIGNAL_CBC3 |                      \
                                    EPWM_TZ_SIGNAL_CBC4 |                      \
                                    EPWM_TZ_SIGNAL_CBC5 |                      \
                                    EPWM_TZ_SIGNAL_CBC6 |                      \
                                    EPWM_TZ_SIGNAL_DCAEVT2 |                   \
                                    EPWM_TZ_SIGNAL_DCBEVT2 |                   \
                                    EPWM_TZ_SIGNAL_OSHT1 |                     \
                                    EPWM_TZ_SIGNAL_OSHT2 |                     \
                                    EPWM_TZ_SIGNAL_OSHT3 |                     \
                                    EPWM_TZ_SIGNAL_OSHT4 |                     \
                                    EPWM_TZ_SIGNAL_OSHT5 |                     \
                                    EPWM_TZ_SIGNAL_OSHT6 |                     \
                                    EPWM_TZ_SIGNAL_DCAEVT1 |                   \
                                    EPWM_TZ_SIGNAL_DCBEVT1

//! \brief Defines the PWM frequency for PWMDAC
//!
#define HA_PWMDAC_FREQ_KHZ         100.0f

//! \brief Defines the comparator number for current protection
//!
#define HAL_NUM_CMPSS_CURRENT       3

//------------------------------------------------------------------------------
#if defined(BSXL8323RS_REVA)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE      // N/A
#define EPWMDAC2_BASE           EPWM6_BASE      // N/A
#define EPWMDAC3_BASE           EPWM6_BASE      // N/A
#define EPWMDAC4_BASE           EPWM6_BASE      // N/A

//------------------------------------------------------------------------------
#if defined(LPD_SITE_J1_J4)     // site 1 (near emulator) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#if defined(MOTOR1_ISBLDC)
#define MTR1_DRV_INL_A          1
#define MTR1_DRV_INL_B          3
#define MTR1_DRV_INL_C          15
#endif  // MOTOR1_ISBLDC

#define MTR1_CMPSS_IDC_BASE     CMPSS3_BASE

#define MTR1_CMPSS_U_BASE       CMPSS1_BASE
#define MTR1_CMPSS_V_BASE       CMPSS3_BASE
#define MTR1_CMPSS_W_BASE       CMPSS1_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       29

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     34

//! \brief Defines the gpio for the SPI_CS of DRV device
#define MTR1_DRV_SPI_CS_GPIO    8

//! \brief Defines the gpio for calibration
#define MTR1_GATE_CAL_GPIO      27

#define MTR1_SPI_BASE           SPIA_BASE

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6
#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
// Single-shunt
#define MTR1_IDC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCB  // EPWM1_SOCB

#define MTR1_IDC1_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCA  // EPWM2_SOCA
#define MTR1_IDC2_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCB  // EPWM2_SOCB
#define MTR1_IDC3_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCA  // EPWM3_SOCA
#define MTR1_IDC4_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCB  // EPWM3_SOCB

#define MTR1_IDC1_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC2_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC3_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC4_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*

#define MTR1_IDC1_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC2_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC3_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC4_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*

#define MTR1_IDC1_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC2_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC3_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC4_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*

#define MTR1_IDC1_ADC_SOC_NUM   ADC_SOC_NUMBER0         // ADCC-A14/C4* -SOC0-PPB1
#define MTR1_IDC2_ADC_SOC_NUM   ADC_SOC_NUMBER1         // ADCC-A14/C4* -SOC1-PPB2
#define MTR1_IDC3_ADC_SOC_NUM   ADC_SOC_NUMBER2         // ADCC-A14/C4* -SOC2-PPB3
#define MTR1_IDC4_ADC_SOC_NUM   ADC_SOC_NUMBER3         // ADCC-A14/C4* -SOC3-PPB4

#define MTR1_IDC1_ADC_PPB_NUM   ADC_PPB_NUMBER1         // ADCC-A14/C4* -SOC0-PPB1
#define MTR1_IDC2_ADC_PPB_NUM   ADC_PPB_NUMBER2         // ADCC-A14/C4* -SOC1-PPB2
#define MTR1_IDC3_ADC_PPB_NUM   ADC_PPB_NUMBER3         // ADCC-A14/C4* -SOC2-PPB3
#define MTR1_IDC4_ADC_PPB_NUM   ADC_PPB_NUMBER4         // ADCC-A14/C4* -SOC3-PPB4

#define MTR1_IDC1_ADCRESLT      MTR1_IDC1_ADCRES_BASE + ADC_O_RESULT0     //
#define MTR1_IDC2_ADCRESLT      MTR1_IDC2_ADCRES_BASE + ADC_O_RESULT1     //
#define MTR1_IDC3_ADCRESLT      MTR1_IDC3_ADCRES_BASE + ADC_O_RESULT2     //
#define MTR1_IDC4_ADCRESLT      MTR1_IDC4_ADCRES_BASE + ADC_O_RESULT3     //

// Three-shunt
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCA_BASE               // ADCA-A11*/C0
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC-A15/C7*
#define MTR1_VU_ADC_BASE        ADCA_BASE               // ADCA-A6*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCC-A3*/C5
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCA-A2/C9*
#define MTR1_VDC_ADC_BASE       ADCC_BASE               // ADCC-C6*
#define MTR1_POT_ADC_BASE       ADCA_BASE               // ADCA-A12*/C1

#define MTR1_IU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A11*/C0
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A15/C7*
#define MTR1_VU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A6*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCC-A3*/C5
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCA-A2/C9*
#define MTR1_VDC_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-C6*
#define MTR1_POT_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A12*/C1

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A11*/C0
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCC-A15/C7*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN6           // ADCA-A6*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCC-A3*/C5
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A2/C9*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN6           // ADCC-C6*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN12          // ADCA-A12*/C1

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A15/C7* -SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER4         // ADCA-A6*     -SOC4
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCC-C6*     -SOC6
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCA-A12*/C1 -SOC6

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A15/C7*- SOC2-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT1     // ADCA-A11*/C10-SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT1     // ADCC-A14/C4* -SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT2     // ADCC-A15/C7*- SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT4     // ADCA-A6*     -SOC4
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT5     // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT5     // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT6    // ADCC-C6*     -SOC6
#define MTR1_POT_ADCRESLT       MTR1_POT_ADCRES_BASE + ADC_O_RESULT6    // ADCA-A12*    -SOC6

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCC_BASE               // ADCC-C6  -SOC6
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCC_INT1-SOC6
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER6         // ADCC_INT1-SOC6

#define MTR1_PIE_INT_NUM        INT_ADCC1               // ADCC_INT1-SOC6
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCC_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCC_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IDC_CMPHP_SEL      ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-A14/C4*
#define MTR1_IDC_CMPLP_SEL      ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-A14/C4*

#define MTR1_IDC_CMPHP_MUX      4                            // CMPSS3-A14/C4*
#define MTR1_IDC_CMPLP_MUX      4                            // CMPSS3-A14/C4*

#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-A11
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-A11, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C4
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C4

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-C7, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-C7

#define MTR1_IU_CMPHP_MUX       1                            // CMPSS1-A11
#define MTR1_IU_CMPLP_MUX       1                            // CMPSS1-A11

#define MTR1_IV_CMPHP_MUX       4                            // CMPSS3-C4
#define MTR1_IV_CMPLP_MUX       4                            // CMPSS3-C4

#define MTR1_IW_CMPHP_MUX       3                            // CMPSS1-C7
#define MTR1_IW_CMPLP_MUX       3                            // CMPSS1-C7

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IDC_XBAR_EPWM_MUX  XBAR_EPWM_MUX05_CMPSS3_CTRIPL   // CMPSS3-LP
#define MTR1_IDC_XBAR_MUX       XBAR_MUX05                      // CMPSS3-LP

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX00_CMPSS1_CTRIPH       // CMPSS1-HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX01_CMPSS1_CTRIPL       // CMPSS1-LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX00          // CMPSS1-HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX01          // CMPSS1-LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH       // CMPSS1-HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX01_CMPSS1_CTRIPOUTL       // CMPSS1-LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5           // 50ns

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5           // 50ns

// end of definition for site 1 (near emulator) on launchPad
//------------------------------------------------------------------------------
#elif defined(LPD_SITE_J5_J8)   // site 2 (away emulator) on launchPad
//! \Motor 2
#define MTR1_PWM_U_BASE         EPWM7_BASE
#define MTR1_PWM_V_BASE         EPWM4_BASE
#define MTR1_PWM_W_BASE         EPWM5_BASE

#if defined(MOTOR1_ISBLDC)
#define MTR1_DRV_INL_A          13
#define MTR1_DRV_INL_B          7
#define MTR1_DRV_INL_C          17
#endif  // MOTOR1_ISBLDC

#define MTR1_CMPSS_U_BASE       CMPSS2_BASE
#define MTR1_CMPSS_V_BASE       CMPSS4_BASE
#define MTR1_CMPSS_W_BASE       CMPSS2_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT8        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       24                   // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     41

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define MTR1_DRV_SPI_CS_GPIO    32

#define MTR1_SPI_BASE           SPIB_BASE           // N/A

#if defined(ENC_QEP1)
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#else   // ENC_QEP2
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#endif  // ENC_QEP2

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6
#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // EPWM7_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-A10/C10*
#define MTR1_IV_ADC_BASE        ADCA_BASE               // ADCA-A9*/C8
#define MTR1_IW_ADC_BASE        ADCA_BASE               // ADCA-A4*/C14
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-A5/C2*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA-A7*/C3
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-A8/C11*
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA-A1*
#define MTR1_POT_ADC_BASE       ADCC_BASE               // ADCC-A0/C15*

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A10/C10*
#define MTR1_IV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A9*/C8
#define MTR1_IW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A4*/C14
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A5/C2*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A7*/C3
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A8/C11*
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A1*
#define MTR1_POT_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-A0/C15*

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCC-A10/C10*
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A9*/C8
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A4*/C14
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN2           // ADCC-A5/C2*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCA-A7*/C3
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A8/C11*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1           // ADCA-A1*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN15          // ADCC-A0/C15*

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER9         // ADCA-A4*/C14 -SOC9-PPB4
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCC-A5/C2*  -SOC12
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCA-A7*/C3  -SOC12
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER13        // ADCC-A8/C11* -SOC13
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER13        // ADCA-A1*     -SOC13
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER14        // ADCC-A0/C15* -SOC14

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER4         // ADCA-A4*/C14 -SOC9-PPB4

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT8     // ADCC-A10/C10*-SOC8
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT8     // ADCA-A9*/C8  -SOC8
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT9     // ADCA-A4*/C14 -SOC9

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT12    // ADCC-A5/C2* -SOC12
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT12    // ADCA-A7*/C3 -SOC12
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT13    // ADCC-A8/C11*-SOC13
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT13   // ADCA-A1*    -SOC13
#define MTR1_POT_ADCRESLT       MTR2_POT_ADCRES_BASE + ADC_O_RESULT14   // ADCC-A0*/C15-SOC14

//------------------------------------------------------------------------------
// Interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A1  -SOC13
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC13
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER13        // ADCA_INT1-SOC13

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC13
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A10/C10*, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A4*/C14, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A4*/C14

#define MTR1_IU_CMPHP_MUX       3       // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_MUX       3       // CMPSS2-A10/C10*

#define MTR1_IV_CMPHP_MUX       0       // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_MUX       0       // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_MUX       0       // CMPSS2-A4*/C14
#define MTR1_IW_CMPLP_MUX       0       // CMPSS2-A4*/C14

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP8MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP8MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH       // CMPSS2_HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX03_CMPSS2_CTRIPL       // CMPSS2_LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX02                  // CMPSS2_HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX06                  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX03                  // CMPSS2_LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT2
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT2

#define MTR1_XBAR_TRIP          XBAR_TRIP8
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN8

#define MTR1_XBAR_OUTPUT        XBAR_OUTPUT8

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH       // CMPSS2_HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL       // CMPSS2_LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5         // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5         // 2.45us

// end of definition for site 2 (away emulator) on launchPad
#else
#error Not select a right site on launchPad
#endif

// end of BSXL8323RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8323RH_REVB)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE      // N/A
#define EPWMDAC2_BASE           EPWM6_BASE      // N/A
#define EPWMDAC3_BASE           EPWM6_BASE      // N/A
#define EPWMDAC4_BASE           EPWM6_BASE      // N/A

//------------------------------------------------------------------------------
#if defined(LPD_SITE_J1_J4)     // site 1 (near emulator) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#if defined(MOTOR1_ISBLDC)
#define MTR1_DRV_INL_A          1
#define MTR1_DRV_INL_B          3
#define MTR1_DRV_INL_C          15
#endif  // MOTOR1_ISBLDC

#define MTR1_CMPSS_IDC_BASE     CMPSS3_BASE

#define MTR1_CMPSS_U_BASE       CMPSS1_BASE
#define MTR1_CMPSS_V_BASE       CMPSS3_BASE
#define MTR1_CMPSS_W_BASE       CMPSS1_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       29

//! \brief Defines the gpio for setting mode
#define MTR1_GATE_MODE_GPIO     42

//! \brief Defines the gpio for setting gain
#define MTR1_GATE_GAIN_GPIO     8

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     34

//! \brief Defines the gpio for calibration
#define MTR1_GATE_CAL_GPIO      27

#define MTR1_SPI_BASE           SPIA_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
// Single-shunt
#define MTR1_IDC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCB  // EPWM1_SOCB

#define MTR1_IDC1_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCA  // EPWM2_SOCA
#define MTR1_IDC2_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCB  // EPWM2_SOCB
#define MTR1_IDC3_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCA  // EPWM3_SOCA
#define MTR1_IDC4_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCB  // EPWM3_SOCB

#define MTR1_IDC1_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC2_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC3_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IDC4_ADC_BASE      ADCC_BASE               // ADCC-A14/C4*

#define MTR1_IDC1_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC2_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC3_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IDC4_ADCRES_BASE   ADCCRESULT_BASE         // ADCC-A14/C4*

#define MTR1_IDC1_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC2_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC3_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IDC4_ADC_CH_NUM    ADC_CH_ADCIN4           // ADCC-A14/C4*

#define MTR1_IDC1_ADC_SOC_NUM   ADC_SOC_NUMBER0         // ADCC-A14/C4* -SOC0-PPB1
#define MTR1_IDC2_ADC_SOC_NUM   ADC_SOC_NUMBER1         // ADCC-A14/C4* -SOC1-PPB2
#define MTR1_IDC3_ADC_SOC_NUM   ADC_SOC_NUMBER2         // ADCC-A14/C4* -SOC2-PPB3
#define MTR1_IDC4_ADC_SOC_NUM   ADC_SOC_NUMBER3         // ADCC-A14/C4* -SOC3-PPB4

#define MTR1_IDC1_ADC_PPB_NUM   ADC_PPB_NUMBER1         // ADCC-A14/C4* -SOC0-PPB1
#define MTR1_IDC2_ADC_PPB_NUM   ADC_PPB_NUMBER2         // ADCC-A14/C4* -SOC1-PPB2
#define MTR1_IDC3_ADC_PPB_NUM   ADC_PPB_NUMBER3         // ADCC-A14/C4* -SOC2-PPB3
#define MTR1_IDC4_ADC_PPB_NUM   ADC_PPB_NUMBER4         // ADCC-A14/C4* -SOC3-PPB4

#define MTR1_IDC1_ADCRESLT      MTR1_IDC1_ADCRES_BASE + ADC_O_RESULT0     //
#define MTR1_IDC2_ADCRESLT      MTR1_IDC2_ADCRES_BASE + ADC_O_RESULT1     //
#define MTR1_IDC3_ADCRESLT      MTR1_IDC3_ADCRES_BASE + ADC_O_RESULT2     //
#define MTR1_IDC4_ADCRESLT      MTR1_IDC4_ADCRES_BASE + ADC_O_RESULT3     //

// Three-shunt
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCA_BASE               // ADCA-A11*/C0
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC-A15/C7*
#define MTR1_VU_ADC_BASE        ADCA_BASE               // ADCA-A6*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCC-A3*/C5
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCA-A2/C9*
#define MTR1_VDC_ADC_BASE       ADCC_BASE               // ADCC-C6*
#define MTR1_POT_ADC_BASE       ADCA_BASE               // ADCA-A12*/C1

#define MTR1_IU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A11*/C0
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A15/C7*
#define MTR1_VU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A6*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCC-A3*/C5
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCA-A2/C9*
#define MTR1_VDC_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-C6*
#define MTR1_POT_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A12*/C1

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A11*/C0
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCC-A15/C7*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN6           // ADCA-A6*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCC-A3*/C5
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A2/C9*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN6           // ADCC-C6*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN12          // ADCA-A12*/C1

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A15/C7* -SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER4         // ADCA-A6*     -SOC4
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCC-C6*     -SOC6
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCA-A12*/C1 -SOC6

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A15/C7*- SOC2-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT1     // ADCA-A11*/C10-SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT1     // ADCC-A14/C4* -SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT2     // ADCC-A15/C7*- SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT4     // ADCA-A6*     -SOC4
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT5     // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT5     // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT6    // ADCC-C6*     -SOC6
#define MTR1_POT_ADCRESLT       MTR1_POT_ADCRES_BASE + ADC_O_RESULT6    // ADCA-A12*/C1 -SOC6

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCC_BASE               // ADCC-C6  -SOC6
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCC_INT1-SOC6
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER6         // ADCC_INT1-SOC6

#define MTR1_PIE_INT_NUM        INT_ADCC1               // ADCC_INT1-SOC6
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCC_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCC_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IDC_CMPHP_SEL      ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-A14/C4*
#define MTR1_IDC_CMPLP_SEL      ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-A14/C4*

#define MTR1_IDC_CMPHP_MUX      4                            // CMPSS3-A14/C4*
#define MTR1_IDC_CMPLP_MUX      4                            // CMPSS3-A14/C4*

#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-A11
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-A11, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C4
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C4

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-C7, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-C7

#define MTR1_IU_CMPHP_MUX       1                            // CMPSS1-A11
#define MTR1_IU_CMPLP_MUX       1                            // CMPSS1-A11

#define MTR1_IV_CMPHP_MUX       4                            // CMPSS3-C4
#define MTR1_IV_CMPLP_MUX       4                            // CMPSS3-C4

#define MTR1_IW_CMPHP_MUX       3                            // CMPSS1-C7
#define MTR1_IW_CMPLP_MUX       3                            // CMPSS1-C7

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IDC_XBAR_EPWM_MUX  XBAR_EPWM_MUX05_CMPSS3_CTRIPL   // CMPSS3-LP
#define MTR1_IDC_XBAR_MUX       XBAR_MUX05                      // CMPSS3-LP

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX00_CMPSS1_CTRIPH       // CMPSS1-HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX01_CMPSS1_CTRIPL       // CMPSS1-LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX00          // CMPSS1-HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX01          // CMPSS1-LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH       // CMPSS1-HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX01_CMPSS1_CTRIPOUTL       // CMPSS1-LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5           // 50ns

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5           // 50ns

// end of definition for site 1 (near emulator) on launchPad
//------------------------------------------------------------------------------
#elif defined(LPD_SITE_J5_J8)   // site 2 (away emulator) on launchPad
//! \Motor 2
#define MTR1_PWM_U_BASE         EPWM7_BASE
#define MTR1_PWM_V_BASE         EPWM4_BASE
#define MTR1_PWM_W_BASE         EPWM5_BASE

#if defined(MOTOR1_ISBLDC)
#define MTR1_DRV_INL_A          13
#define MTR1_DRV_INL_B          7
#define MTR1_DRV_INL_C          17
#endif  // MOTOR1_ISBLDC

#define MTR1_CMPSS_U_BASE       CMPSS2_BASE
#define MTR1_CMPSS_V_BASE       CMPSS4_BASE
#define MTR1_CMPSS_W_BASE       CMPSS2_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT8        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       24                   // N/A

//! \brief Defines the gpio for setting mode
#define MTR1_GATE_MODE_GPIO     45

//! \brief Defines the gpio for setting gain
#define MTR1_GATE_GAIN_GPIO     32

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     41

//! \brief Defines the gpio for calibration
#define MTR1_GATE_CAL_GPIO      46

#define MTR1_SPI_BASE           SPIB_BASE           // N/A

#if defined(ENC_QEP1)
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#else   // ENC_QEP2
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#endif  // ENC_QEP2

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6
#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // EPWM7_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-A10/C10*
#define MTR1_IV_ADC_BASE        ADCA_BASE               // ADCA-A9*/C8
#define MTR1_IW_ADC_BASE        ADCA_BASE               // ADCA-A4*/C14
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-A5/C2*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA-A7*/C3
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-A8/C11*
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA-A1*
#define MTR1_POT_ADC_BASE       ADCC_BASE               // ADCC-A0/C15*

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A10/C10*
#define MTR1_IV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A9*/C8
#define MTR1_IW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A4*/C14
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A5/C2*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A7*/C3
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A8/C11*
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A1*
#define MTR1_POT_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-A0/C15*

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCC-A10/C10*
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A9*/C8
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A4*/C14
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN2           // ADCC-A5/C2*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCA-A7*/C3
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A8/C11*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1           // ADCA-A1*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN15          // ADCC-A0/C15*

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER9         // ADCA-A4*/C14 -SOC9-PPB4
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCC-A5/C2*  -SOC12
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCA-A7*/C3  -SOC12
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER13        // ADCC-A8/C11* -SOC13
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER13        // ADCA-A1*     -SOC13
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER14        // ADCC-A0/C15* -SOC14

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER4         // ADCA-A4*/C14 -SOC9-PPB4

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT8     // ADCC-A10/C10*-SOC8
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT8     // ADCA-A9*/C8  -SOC8
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT9     // ADCA-A4*/C14 -SOC9

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT12    // ADCC-A5/C2* -SOC12
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT12    // ADCA-A7*/C3 -SOC12
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT13    // ADCC-A8/C11*-SOC13
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT13   // ADCA-A1*    -SOC13
#define MTR1_POT_ADCRESLT       MTR2_POT_ADCRES_BASE + ADC_O_RESULT14   // ADCC-A0*/C15-SOC14

//------------------------------------------------------------------------------
// Interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A1  -SOC13
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC13
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER13        // ADCA_INT1-SOC13

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC13
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A10/C10*, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A4*/C14, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A4*/C14

#define MTR1_IU_CMPHP_MUX       3       // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_MUX       3       // CMPSS2-A10/C10*

#define MTR1_IV_CMPHP_MUX       0       // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_MUX       0       // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_MUX       0       // CMPSS2-A4*/C14
#define MTR1_IW_CMPLP_MUX       0       // CMPSS2-A4*/C14

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP8MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP8MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH       // CMPSS2_HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX03_CMPSS2_CTRIPL       // CMPSS2_LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX02                  // CMPSS2_HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX06                  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX03                  // CMPSS2_LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT2
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT2

#define MTR1_XBAR_TRIP          XBAR_TRIP8
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN8

#define MTR1_XBAR_OUTPUT        XBAR_OUTPUT8

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH       // CMPSS2_HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL       // CMPSS2_LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5         // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5         // 2.45us

// end of definition for site 2 (away emulator) on launchPad
#else
#error Not select a right site on launchPad
#endif

// end of BSXL8323RH_REVB

//------------------------------------------------------------------------------
#elif defined(BSXL8353RS_REVA)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE      // N/A
#define EPWMDAC2_BASE           EPWM6_BASE      // N/A
#define EPWMDAC3_BASE           EPWM6_BASE      // N/A
#define EPWMDAC4_BASE           EPWM6_BASE      // N/A

//------------------------------------------------------------------------------
#if defined(LPD_SITE_J1_J4)     // site 1 (near emulator) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#define MTR1_CMPSS_U_BASE       CMPSS1_BASE
#define MTR1_CMPSS_V_BASE       CMPSS3_BASE
#define MTR1_CMPSS_W_BASE       CMPSS1_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       29                  // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     34

//! \brief Defines the gpio for the SPI_CS of DRV device
#define MTR1_DRV_SPI_CS_GPIO    8

//! \brief Defines the gpio for calibration
#define MTR1_GATE_CAL_GPIO      27

#define MTR1_SPI_BASE           SPIA_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)

#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
// Three-shunt
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCA_BASE               // ADCA-A11*/C0
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC-A14/C4*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC-A15/C7*
#define MTR1_VU_ADC_BASE        ADCA_BASE               // ADCA-A6*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCC-A3*/C5
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCA-A2/C9*
#define MTR1_VDC_ADC_BASE       ADCC_BASE               // ADCC-C6*
#define MTR1_POT_ADC_BASE       ADCA_BASE               // ADCA-A12*/C1

#define MTR1_IU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A11*/C0
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A14/C4*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A15/C7*
#define MTR1_VU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A6*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCC-A3*/C5
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCA-A2/C9*
#define MTR1_VDC_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-C6*
#define MTR1_POT_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A12*/C1

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A11*/C0
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A14/C4*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCC-A15/C7*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN6           // ADCA-A6*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCC-A3*/C5
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A2/C9*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN6           // ADCC-C6*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN12          // ADCA-A12*/C1

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A15/C7* -SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER4         // ADCA-A6*     -SOC4
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCC-C6*     -SOC6
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER6         // ADCA-A12*/C1 -SOC6

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A11*/C10-SOC1-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-A14/C4* -SOC1-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A15/C7*- SOC2-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT1     // ADCA-A11*/C10-SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT1     // ADCC-A14/C4* -SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT2     // ADCC-A15/C7*- SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT4     // ADCA-A6*     -SOC4
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT5     // ADCC-A3*/C5  -SOC5
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT5     // ADCA-A2/C9*  -SOC5
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT6    // ADCC-C6*     -SOC6
#define MTR1_POT_ADCRESLT       MTR1_POT_ADCRES_BASE + ADC_O_RESULT6    // ADCA-A12*/C1 -SOC7

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCC_BASE               // ADCC-C6  -SOC6
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCC_INT1-SOC6
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER6         // ADCC_INT1-SOC6

#define MTR1_PIE_INT_NUM        INT_ADCC1               // ADCC_INT1-SOC6
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCC_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCC_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-A11
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-A11, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C4
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C4

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-C7, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-C7

#define MTR1_IU_CMPHP_MUX       1                            // CMPSS1-A11
#define MTR1_IU_CMPLP_MUX       1                            // CMPSS1-A11

#define MTR1_IV_CMPHP_MUX       4                            // CMPSS3-C4
#define MTR1_IV_CMPLP_MUX       4                            // CMPSS3-C4

#define MTR1_IW_CMPHP_MUX       3                            // CMPSS1-C7
#define MTR1_IW_CMPLP_MUX       3                            // CMPSS1-C7

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IDC_XBAR_EPWM_MUX  XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IDC_XBAR_MUX       XBAR_MUX04          // CMPSS3-HP&LP

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX00_CMPSS1_CTRIPH       // CMPSS1-HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX01_CMPSS1_CTRIPL       // CMPSS1-LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX00          // CMPSS1-HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX01          // CMPSS1-LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH       // CMPSS1-HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L  // CMPSS3-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX01_CMPSS1_CTRIPOUTL       // CMPSS1-LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5           // 50ns

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5           // 50ns

// end of definition for site 1 (near emulator) on launchPad
//------------------------------------------------------------------------------
#elif defined(LPD_SITE_J5_J8)   // site 2 (away emulator) on launchPad
//! \Motor 2
#define MTR1_PWM_U_BASE         EPWM7_BASE
#define MTR1_PWM_V_BASE         EPWM4_BASE
#define MTR1_PWM_W_BASE         EPWM5_BASE

#define MTR1_CMPSS_U_BASE       CMPSS2_BASE
#define MTR1_CMPSS_V_BASE       CMPSS4_BASE
#define MTR1_CMPSS_W_BASE       CMPSS2_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT8        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       24                   // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     41

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define MTR1_DRV_SPI_CS_GPIO    32

#define MTR1_SPI_BASE           SPIB_BASE           // N/A

#if defined(ENC_QEP1)
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#else   // ENC_QEP2
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#endif  // ENC_QEP2

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6
#elif defined(CMD_CAP_EN)
#define MTR1_CAP_FREQ_GPIO      40
#define MTR1_CAP_FREQ_BASE      ECAP1_BASE
#define MTR1_CAP_FREQ_XBAR      XBAR_INPUT4
#define MTR1_CAP_FREQ_INSEL     ECAP_INPUT_INPUTXBAR4
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
#define MTR1_CMD_SWITCH_GPIO    23
#define MTR1_CMD_STATE_GPIO     35
#endif  // CMD_SWITCH_EN

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // EPWM7_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-A10/C10*
#define MTR1_IV_ADC_BASE        ADCA_BASE               // ADCA-A9*/C8
#define MTR1_IW_ADC_BASE        ADCA_BASE               // ADCA-A4*/C14
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-A5/C2*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA-A7*/C3
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-A8/C11*
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA-A1*
#define MTR1_POT_ADC_BASE       ADCC_BASE               // ADCC-A0/C15*

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A10/C10*
#define MTR1_IV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A9*/C8
#define MTR1_IW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A4*/C14
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A5/C2*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A7*/C3
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A8/C11*
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A1*
#define MTR1_POT_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-A0/C15*

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCC-A10/C10*
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A9*/C8
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A4*/C14
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN2           // ADCC-A5/C2*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCA-A7*/C3
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-A8/C11*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1           // ADCA-A1*
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN15          // ADCC-A0/C15*

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER8         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER9         // ADCA-A4*/C14 -SOC9-PPB4
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCC-A5/C2*  -SOC12
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER12        // ADCA-A7*/C3  -SOC12
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER13        // ADCC-A8/C11* -SOC13
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER13        // ADCA-A1*     -SOC13
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER14        // ADCC-A0/C15* -SOC14

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCC-A10/C10*-SOC8-PPB3
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCA-A9*/C8  -SOC8-PPB3
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER4         // ADCA-A4*/C14 -SOC9-PPB4

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT8     // ADCC-A10/C10*-SOC8
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT8     // ADCA-A9*/C8  -SOC8
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT9     // ADCA-A4*/C14 -SOC9

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT12    // ADCC-A5/C2* -SOC12
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT12    // ADCA-A7*/C3 -SOC12
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT13    // ADCC-A8/C11*-SOC13
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT13   // ADCA-A1*    -SOC13
#define MTR1_POT_ADCRESLT       MTR2_POT_ADCRES_BASE + ADC_O_RESULT14   // ADCC-A0*/C15-SOC14

//------------------------------------------------------------------------------
// Interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A1  -SOC13
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC13
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER13        // ADCA_INT1-SOC13

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC13
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A10/C10*, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A4*/C14, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A4*/C14

#define MTR1_IU_CMPHP_MUX       3       // CMPSS2-A10/C10*
#define MTR1_IU_CMPLP_MUX       3       // CMPSS2-A10/C10*

#define MTR1_IV_CMPHP_MUX       0       // CMPSS4-A9*/C8
#define MTR1_IV_CMPLP_MUX       0       // CMPSS4-A9*/C8

#define MTR1_IW_CMPHP_MUX       0       // CMPSS2-A4*/C14
#define MTR1_IW_CMPLP_MUX       0       // CMPSS2-A4*/C14

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP8MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP8MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH       // CMPSS2_HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX03_CMPSS2_CTRIPL       // CMPSS2_LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX02                  // CMPSS2_HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX06                  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX03                  // CMPSS2_LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT2
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT2

#define MTR1_XBAR_TRIP          XBAR_TRIP8
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN8

#define MTR1_XBAR_OUTPUT        XBAR_OUTPUT8

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH       // CMPSS2_HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL       // CMPSS2_LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5         // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5         // 2.45us

// end of definition for site 2 (away emulator) on launchPad
#else
#error Not select a right site on launchPad
#endif

// end of BSXL8353RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8316RT_REVA)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE      // N/A
#define EPWMDAC2_BASE           EPWM6_BASE      // N/A
#define EPWMDAC3_BASE           EPWM6_BASE      // N/A
#define EPWMDAC4_BASE           EPWM6_BASE      // N/A

//------------------------------------------------------------------------------
#if defined(LPD_SITE_J1_J4)     // site 1 (near emulator) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#define MTR1_CMPSS_IDC_BASE     CMPSS3_BASE

#define MTR1_CMPSS_U_BASE       CMPSS1_BASE
#define MTR1_CMPSS_V_BASE       CMPSS3_BASE
#define MTR1_CMPSS_W_BASE       CMPSS1_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7

//! \brief Defines the gpio for enabling Power Module
#define MTR1_DRV_SLEEP_GPIO     29
#define MTR1_GATE_EN_GPIO       29

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     34

//! \brief Defines the gpio for the SPI_CS of DRV device
#define MTR1_DRV_SPI_CS_GPIO    5

//! \brief Defines the gpio for DRV_OFF
#define MTR1_DRV_OFF_GPIO      23

#define MTR1_SPI_BASE           SPIA_BASE

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

//------------------------------------------------------------------------------
// ADC
// Three-shunt
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCA_BASE               // ADCA-A14*/C4
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC -A11/C0*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC -A12/C1*
#define MTR1_VU_ADC_BASE        ADCA_BASE               // ADCA -A6*
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA -A3*/C5
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC -A2/C9*
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA -A15*/C7


#define MTR1_IU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A14*/C4
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A11/C0*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A12/C1*
#define MTR1_VU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A6*
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A3*/C5
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A2/C9*
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A15*/C7


#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN14          // ADCA-A14*/C4
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN0           // ADCC-A11/C0*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN1           // ADCC-A12/C1*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN6           // ADCA-A6*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCA-A3*/C5
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCC-A2/C9*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN15          // ADCA-A15*/C7

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A14*/C4-SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-A11/C0*-SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A12/C1*-SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A6*    -SOC2
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCA-A3*/C5 -SOC3
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCC-A2/C9* -SOC3
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER4         // ADCA-A15*/C7-SOC4

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A14*/C4-SOC1-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-A11/C0*-SOC1-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A12/C1*-SOC2-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE  + ADC_O_RESULT1    // ADCA-A14*/C4-SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE  + ADC_O_RESULT1    // ADCC-A11/C0*-SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE  + ADC_O_RESULT2    // ADCC-A12/C1*-SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE  + ADC_O_RESULT2    // ADCC-A6*    -SOC2
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE  + ADC_O_RESULT3    // ADCA-A3*/C5 -SOC3
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE  + ADC_O_RESULT3    // ADCC-A2/C9* -SOC3
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT4    // ADCA-A15*/C7-SOC4

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A15 -SOC4
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC4
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER4         // ADCA_INT1-SOC4

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC4
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-A14*/C4
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-A14*/C4

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-A11/C0*
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-A11/C0*

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A12/C1*
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A12/C1*

#define MTR1_IU_CMPHP_MUX       4                            // CMPSS3-A14*/C4
#define MTR1_IU_CMPLP_MUX       4                            // CMPSS3-A14*/C4

#define MTR1_IV_CMPHP_MUX       1                            // CMPSS1-A11/C0*
#define MTR1_IV_CMPLP_MUX       1                            // CMPSS1-A11/C0*

#define MTR1_IW_CMPHP_MUX       1                            // CMPSS2-A12/C1*
#define MTR1_IW_CMPLP_MUX       1                            // CMPSS2-A12/C1*

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L  // CMPSS1-HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L  // CMPSS2-HP&LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP&LP
#define MTR1_IV_XBAR_MUX        XBAR_MUX00          // CMPSS1-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX02          // CMPSS2-HP&LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L    // CMPSS3-HP&LP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH_OR_L    // CMPSS1-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH_OR_L    // CMPSS2-HP&LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5           // 50ns

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5           // 50ns

// end of definition for site 1 (near emulator) on launchPad
//------------------------------------------------------------------------------
#elif defined(LPD_SITE_J5_J8)   // site 2 (away emulator) on launchPad
//! \Motor 2
#define MTR1_PWM_U_BASE         EPWM7_BASE
#define MTR1_PWM_V_BASE         EPWM4_BASE
#define MTR1_PWM_W_BASE         EPWM5_BASE

#define MTR1_CMPSS_U_BASE       CMPSS2_BASE
#define MTR1_CMPSS_V_BASE       CMPSS4_BASE
#define MTR1_CMPSS_W_BASE       CMPSS2_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT8        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       24                   // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     41

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define MTR1_DRV_SPI_CS_GPIO    32

#define MTR1_SPI_BASE           SPIB_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // EPWM7_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-C10
#define MTR1_IV_ADC_BASE        ADCA_BASE               // ADCA-A9
#define MTR1_IW_ADC_BASE        ADCA_BASE               // ADCA-A4
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-C2
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA-A7
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-C11
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA-A1
#define MTR1_POT_ADC_BASE       ADCC_BASE               // ADCC-C15

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C10
#define MTR1_IV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A9
#define MTR1_IW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A4
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C2
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A7
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C11
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A1
#define MTR1_POT_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-C15

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCC-C10
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCA-A9
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN4           // ADCC-A4
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN2           // ADCC-C2
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCA-A7
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCA-C11
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN1           // ADCA-A1
#define MTR1_POT_ADC_CH_NUM     ADC_CH_ADCIN15          // ADCC-C15

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCC-C10-SOC5-PPB3
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCA-A9 -SOC5-PPB2
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER6         // ADCA-A4 -SOC6-PPB3
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER6         // ADCC-C2 -SOC6
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER7         // ADCA-A7 -SOC7
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER7         // ADCC-C11-SOC7
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER8         // ADCA-A1 -SOC8
#define MTR1_POT_ADC_SOC_NUM    ADC_SOC_NUMBER8         // ADCC-C15-SOC8

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCC-C10-SOC5-PPB3
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCA-A9 -SOC5-PPB2
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCA-A4 -SOC6-PPB3

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT5     // ADCC-C10-SOC5
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT5     // ADCA-A9 -SOC5
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT6     // ADCA-A4 -SOC6

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT6     // ADCC-C2 -SOC6
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT7     // ADCA-A7 -SOC7
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT7     // ADCC-C11-SOC7
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT8    // ADCA-A1 -SOC8
#define MTR1_POT_ADCRESLT       MTR2_POT_ADCRES_BASE + ADC_O_RESULT8    // ADCC-C15-SOC8

//------------------------------------------------------------------------------
// Interrupt
#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A1  -SOC8
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC8
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER8         // ADCA_INT1-SOC8
#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC8
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-C10
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-C10, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A9
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A9

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A4, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A4

#define MTR1_IU_CMPHP_MUX       3       // CMPSS2-C10
#define MTR1_IU_CMPLP_MUX       3       // CMPSS2-C10

#define MTR1_IV_CMPHP_MUX       0       // CMPSS4-A9
#define MTR1_IV_CMPLP_MUX       0       // CMPSS4-A9

#define MTR1_IW_CMPHP_MUX       0       // CMPSS2-A4
#define MTR1_IW_CMPLP_MUX       0       // CMPSS2-A4

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP8MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP8MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH       // CMPSS2_HP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX03_CMPSS2_CTRIPL       // CMPSS2_LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX02                  // CMPSS2_HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX06                  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX03                  // CMPSS2_LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT2
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT2

#define MTR1_XBAR_TRIP          XBAR_TRIP8
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN8

#define MTR1_XBAR_OUTPUT        XBAR_OUTPUT8

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH       // CMPSS2_HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L  // CMPSS4_HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX03_CMPSS2_CTRIPOUTL       // CMPSS2_LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5         // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5         // 2.45us

// end of definition for site 2 (away emulator) on launchPad
#else
#error Not select a right site on launchPad
#endif
// end of BSXL8316RT_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL3PHGAN_REVA)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE      // N/A
#define EPWMDAC2_BASE           EPWM6_BASE      // N/A
#define EPWMDAC3_BASE           EPWM6_BASE      // N/A
#define EPWMDAC4_BASE           EPWM6_BASE      // N/A

//------------------------------------------------------------------------------
#if defined(LPD_SITE_J1_J4)     // site 1 (near emulator) on launchPad
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#if defined(MOTOR1_ISBLDC)
#define MTR1_DRV_INL_A          1
#define MTR1_DRV_INL_B          3
#define MTR1_DRV_INL_C          15
#endif  // MOTOR1_ISBLDC

#define MTR1_CMPSS_U_BASE       CMPSS3_BASE
#define MTR1_CMPSS_V_BASE       CMPSS1_BASE
#define MTR1_CMPSS_W_BASE       CMPSS2_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       23                  //

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     42

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define MTR1_DRV_SPI_CS_GPIO    5                   // N/A

#define MTR1_SPI_BASE           SPIA_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCA_BASE               // ADCA-A14*/C4
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC -A11/C0*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC -A12/C1*
#define MTR1_VU_ADC_BASE        ADCA_BASE               // ADCA -A3*/C5
#define MTR1_VV_ADC_BASE        ADCC_BASE               // ADCC -A2/C9*
#define MTR1_VW_ADC_BASE        ADCA_BASE               // ADCA -A15*/C7
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA -A6*

#define MTR1_IU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A14*/C4
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A11/C0*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A12/C1*
#define MTR1_VU_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A3*/C5
#define MTR1_VV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A2/C9*
#define MTR1_VW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A15*/C7
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A6*

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN14          // ADCA-A14*/C4
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN0           // ADCC-A11/C0*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN1           // ADCC-A12/C1*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCA-A3*/C5
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN9           // ADCC-A2/C9*
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN15          // ADCA-A15*/C7
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN6           // ADCA-A6*

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A14*/C4-SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-A11/C0*-SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-A12/C1*-SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCA-A3*/C5 -SOC2
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCC-A2/C9* -SOC3
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCA-A15*/C7-SOC3
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER4         // ADCC-A6*    -SOC4

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A14*/C4-SOC1-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-A11/C0*-SOC1-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A12/C1*-SOC2-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE  + ADC_O_RESULT1    // ADCA-A14*/C4-SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE  + ADC_O_RESULT1    // ADCC-A11/C0*-SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE  + ADC_O_RESULT2    // ADCC-A12/C1*-SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE  + ADC_O_RESULT2    // ADCA-A3*/C5 -SOC2
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE  + ADC_O_RESULT3    // ADCC-A2/C9* -SOC3
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE  + ADC_O_RESULT3    // ADCA-A15*/C7-SOC3
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT4    // ADCC-A6*    -SOC4

//------------------------------------------------------------------------------
// interrupt
#define MTR1_PWM_INT_BASE       MTR1_PWM_U_BASE         // EPWM1

#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A6  -SOC4
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC4
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER4         // ADCA_INT1-SOC4

#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC4
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-A14*/C4
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-A14*/C4

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-A11/C0*
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-A11/C0*

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A12/C1*
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A12/C1*

#define MTR1_IU_CMPHP_MUX       4                            // CMPSS3-A14*/C4
#define MTR1_IU_CMPLP_MUX       4                            // CMPSS3-A14*/C4

#define MTR1_IV_CMPHP_MUX       1                            // CMPSS1-A11/C0*
#define MTR1_IV_CMPLP_MUX       1                            // CMPSS1-A11/C0*

#define MTR1_IW_CMPHP_MUX       1                            // CMPSS2-A12/C1*
#define MTR1_IW_CMPLP_MUX       1                            // CMPSS2-A12/C1*

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3-HP&LP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L  // CMPSS1-HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L  // CMPSS2-HP&LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP&LP
#define MTR1_IV_XBAR_MUX        XBAR_MUX00          // CMPSS1-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX02          // CMPSS2-HP&LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L    // CMPSS3-HP&LP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX00_CMPSS1_CTRIPOUTH_OR_L    // CMPSS1-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH_OR_L    // CMPSS2-HP&LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5           // 50ns

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5           // 50ns

// end of definition for site 1 (near emulator) on launchPad
//------------------------------------------------------------------------------
#elif defined(LPD_SITE_J5_J8)   // site 2 (away emulator) on launchPad
//! \Motor 2
#define MTR1_PWM_U_BASE         EPWM7_BASE
#define MTR1_PWM_V_BASE         EPWM4_BASE
#define MTR1_PWM_W_BASE         EPWM5_BASE

#define MTR1_CMPSS_U_BASE       CMPSS4_BASE
#define MTR1_CMPSS_V_BASE       CMPSS2_BASE
#define MTR1_CMPSS_W_BASE       CMPSS3_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT8        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       39                   // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     45

//! \brief Defines the gpio for the SPI_CS of DRV device
//!
#define MTR1_DRV_SPI_CS_GPIO    33

#define MTR1_SPI_BASE           SPIB_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        44
#define MTR1_HALL_V_GPIO        37
#define MTR1_HALL_W_GPIO        43
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        14
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

//------------------------------------------------------------------------------
// ADC
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // EPWM7_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-C8
#define MTR1_IV_ADC_BASE        ADCA_BASE               // ADCA-A10
#define MTR1_IW_ADC_BASE        ADCA_BASE               // ADCA-A0
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-C3
#define MTR1_VV_ADC_BASE        ADCA_BASE               // ADCA-A8
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-C14
#define MTR1_VDC_ADC_BASE       ADCA_BASE               // ADCA-A5

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C8
#define MTR1_IV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A10
#define MTR1_IW_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A0
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C3
#define MTR1_VV_ADCRES_BASE     ADCARESULT_BASE         // ADCA-A8
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C14
#define MTR1_VDC_ADCRES_BASE    ADCARESULT_BASE         // ADCA-A5

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN8           // ADCC-C8
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCA-A10
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN0           // ADCC-A0
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN3           // ADCC-C3
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN8           // ADCA-A8
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN14          // ADCA-C14
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN5           // ADCA-A5

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCC-C8 -SOC1-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER1         // ADCA-A10-SOC1-PPB1
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCA-A0 -SOC2-PPB2
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-C3 -SOC2
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCA-A8 -SOC3
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCC-C14-SOC3
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER4         // ADCA-A5 -SOC4

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-C8 -SOC5-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCA-A10-SOC5-PPB1
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCA-A0 -SOC6-PPB2

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT1     // ADCC-C8 -SOC1
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT1     // ADCA-A10-SOC1
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT2     // ADCA-A0 -SOC2

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT2     // ADCC-C3 -SOC2
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT3     // ADCA-A8 -SOC3
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT3     // ADCC-C14-SOC3
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT4    // ADCA-A5 -SOC4

//------------------------------------------------------------------------------
// Interrupt
#define MTR1_ADC_INT_BASE       ADCA_BASE               // ADCA-A5  -SOC4
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCA_INT1-SOC4
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER4         // ADCA_INT1-SOC4
#define MTR1_PIE_INT_NUM        INT_ADCA1               // ADCA_INT1-SOC4
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCA_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCA_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-C8
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-C8

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A10
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A10

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-A0
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-A0

#define MTR1_IU_CMPHP_MUX       0       // CMPSS4-C8
#define MTR1_IU_CMPLP_MUX       0       // CMPSS4-C8

#define MTR1_IV_CMPHP_MUX       3       // CMPSS2-A10
#define MTR1_IV_CMPLP_MUX       3       // CMPSS2-A10

#define MTR1_IW_CMPHP_MUX       2       // CMPSS3-A0
#define MTR1_IW_CMPLP_MUX       2       // CMPSS3-A0

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP8MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP8MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L  // CMPSS4_HP&LP
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L  // CMPSS2_HP&LP
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L  // CMPSS3_HP&LP

#define MTR1_IU_XBAR_MUX        XBAR_MUX06                  // CMPSS4_HP&LP
#define MTR1_IV_XBAR_MUX        XBAR_MUX02                  // CMPSS2_HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX04                  // CMPSS3_HP&LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT2
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT2

#define MTR1_XBAR_TRIP          XBAR_TRIP8
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN8

#define MTR1_XBAR_OUTPUT        XBAR_OUTPUT8

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX06_CMPSS4_CTRIPOUTH_OR_L    // CMPSS4_HP&LP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH_OR_L    // CMPSS2_HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH_OR_L    // CMPSS3_HP&LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      5         // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      5         // 2.45us


// end of definition for site 2 (away emulator) on launchPad
#else
#error Not select a right site on launchPad
#endif
// end of BSXL3PHGAN_REVA

//------------------------------------------------------------------------------
#elif defined(HVMTRPFC_REV1P1)
//------------------------------------------------------------------------------
#define EPWMDAC1_BASE           EPWM6_BASE
#define EPWMDAC2_BASE           EPWM6_BASE
#define EPWMDAC3_BASE           EPWM7_BASE
#define EPWMDAC4_BASE           EPWM7_BASE

//------------------------------------------------------------------------------
//! \ Motor 1
#define MTR1_PWM_U_BASE         EPWM1_BASE
#define MTR1_PWM_V_BASE         EPWM2_BASE
#define MTR1_PWM_W_BASE         EPWM3_BASE

#define MTR1_CMPSS_U_BASE       CMPSS3_BASE
#define MTR1_CMPSS_V_BASE       CMPSS2_BASE
#define MTR1_CMPSS_W_BASE       CMPSS3_BASE

#define MTR1_XBAROUTPUT         XBAR_OUTPUT7        // N/A

//! \brief Defines the gpio for enabling Power Module
#define MTR1_GATE_EN_GPIO       9                   // N/A

//! \brief Defines the gpio for the nFAULT of Power Module
#define MTR1_PM_nFAULT_GPIO     39

#define MTR1_SPI_BASE           SPIA_BASE           // N/A

#if defined(ENC_QEP2)
#define MTR1_QEP_BASE           EQEP2_BASE

#define MTR1_HALL_U_GPIO        40
#define MTR1_HALL_V_GPIO        41
#define MTR1_HALL_W_GPIO        23
#else   // ENC_QEP1
#define MTR1_QEP_BASE           EQEP1_BASE

#define MTR1_HALL_U_GPIO        24
#define MTR1_HALL_V_GPIO        25
#define MTR1_HALL_W_GPIO        26
#endif  // ENC_QEP1

#define MTR1_CAP_U_BASE         ECAP1_BASE
#define MTR1_CAP_V_BASE         ECAP2_BASE
#define MTR1_CAP_W_BASE         ECAP3_BASE

#define MTR1_CAP_U_XBAR         XBAR_INPUT4
#define MTR1_CAP_V_XBAR         XBAR_INPUT5
#define MTR1_CAP_W_XBAR         XBAR_INPUT6

#define MTR1_CAP_U_INSEL        ECAP_INPUT_INPUTXBAR4
#define MTR1_CAP_V_INSEL        ECAP_INPUT_INPUTXBAR5
#define MTR1_CAP_W_INSEL        ECAP_INPUT_INPUTXBAR6

//------------------------------------------------------------------------------
// ADC
// Single-shunt , Not available in current version software
#define MTR1_IDC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCB  // EPWM1_SOCB

#define MTR1_IDC1_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCA  // EPWM2_SOCA
#define MTR1_IDC2_TIGGER_SOC    ADC_TRIGGER_EPWM2_SOCB  // EPWM2_SOCB
#define MTR1_IDC3_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCA  // EPWM3_SOCA
#define MTR1_IDC4_TIGGER_SOC    ADC_TRIGGER_EPWM3_SOCB  // EPWM3_SOCB

#define MTR1_IDC1_ADC_BASE      ADCA_BASE               // ADCA-A2*/C9
#define MTR1_IDC2_ADC_BASE      ADCA_BASE               // ADCA-A2*/C9
#define MTR1_IDC3_ADC_BASE      ADCA_BASE               // ADCA-A2*/C9
#define MTR1_IDC4_ADC_BASE      ADCA_BASE               // ADCA-A2*/C9

#define MTR1_IDC1_ADCRES_BASE   ADCARESULT_BASE         // ADCA-A2*/C9
#define MTR1_IDC2_ADCRES_BASE   ADCARESULT_BASE         // ADCA-A2*/C9
#define MTR1_IDC3_ADCRES_BASE   ADCARESULT_BASE         // ADCA-A2*/C9
#define MTR1_IDC4_ADCRES_BASE   ADCARESULT_BASE         // ADCA-A2*/C9

#define MTR1_IDC1_ADC_CH_NUM    ADC_CH_ADCIN2           // ADCA-A2*/C9
#define MTR1_IDC2_ADC_CH_NUM    ADC_CH_ADCIN2           // ADCA-A2*/C9
#define MTR1_IDC3_ADC_CH_NUM    ADC_CH_ADCIN2           // ADCA-A2*/C9
#define MTR1_IDC4_ADC_CH_NUM    ADC_CH_ADCIN2           // ADCA-A2*/C9

#define MTR1_IDC1_ADC_SOC_NUM   ADC_SOC_NUMBER0         // ADCA-A2*/C9 -SOC0-PPB1
#define MTR1_IDC2_ADC_SOC_NUM   ADC_SOC_NUMBER1         // ADCA-A2*/C9 -SOC1-PPB2
#define MTR1_IDC3_ADC_SOC_NUM   ADC_SOC_NUMBER2         // ADCA-A2*/C9 -SOC2-PPB3
#define MTR1_IDC4_ADC_SOC_NUM   ADC_SOC_NUMBER3         // ADCA-A2*/C9 -SOC3-PPB4

#define MTR1_IDC1_ADC_PPB_NUM   ADC_PPB_NUMBER1         // ADCA-A2*/C9 -SOC0-PPB1
#define MTR1_IDC2_ADC_PPB_NUM   ADC_PPB_NUMBER2         // ADCA-A2*/C9 -SOC1-PPB2
#define MTR1_IDC3_ADC_PPB_NUM   ADC_PPB_NUMBER3         // ADCA-A2*/C9 -SOC2-PPB3
#define MTR1_IDC4_ADC_PPB_NUM   ADC_PPB_NUMBER4         // ADCA-A2*/C9 -SOC3-PPB4

#define MTR1_IDC1_ADCRESLT      MTR1_IDC1_ADCRES_BASE + ADC_O_RESULT0     //
#define MTR1_IDC2_ADCRESLT      MTR1_IDC2_ADCRES_BASE + ADC_O_RESULT1     //
#define MTR1_IDC3_ADCRESLT      MTR1_IDC3_ADCRES_BASE + ADC_O_RESULT2     //
#define MTR1_IDC4_ADCRESLT      MTR1_IDC4_ADCRES_BASE + ADC_O_RESULT3     //

// Three-shunt
#define MTR1_ADC_TIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // EPWM1_SOCA
#define MTR1_ADC_I_SAMPLEWINDOW     14
#define MTR1_ADC_V_SAMPLEWINDOW     20

#define MTR1_IU_ADC_BASE        ADCC_BASE               // ADCC-C6*
#define MTR1_IV_ADC_BASE        ADCC_BASE               // ADCC-A9/C8*
#define MTR1_IW_ADC_BASE        ADCC_BASE               // ADCC-A5/C2*
#define MTR1_VU_ADC_BASE        ADCC_BASE               // ADCC-A8/C11*
#define MTR1_VV_ADC_BASE        ADCC_BASE               // ADCC-A10/C10*
#define MTR1_VW_ADC_BASE        ADCC_BASE               // ADCC-A15/C7*
#define MTR1_VDC_ADC_BASE       ADCC_BASE               // ADCC-A7/C3*

#define MTR1_IU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-C6*
#define MTR1_IV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A9/C8*
#define MTR1_IW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A5/C2*
#define MTR1_VU_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A8/C11*
#define MTR1_VV_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A10/C10*
#define MTR1_VW_ADCRES_BASE     ADCCRESULT_BASE         // ADCC-A15/C7*
#define MTR1_VDC_ADCRES_BASE    ADCCRESULT_BASE         // ADCC-A7/C3*

#define MTR1_IU_ADC_CH_NUM      ADC_CH_ADCIN6           // ADCC-C6*
#define MTR1_IV_ADC_CH_NUM      ADC_CH_ADCIN8           // ADCC-A9/C8*
#define MTR1_IW_ADC_CH_NUM      ADC_CH_ADCIN2           // ADCC-A5/C2*
#define MTR1_VU_ADC_CH_NUM      ADC_CH_ADCIN11          // ADCC-A8/C11*
#define MTR1_VV_ADC_CH_NUM      ADC_CH_ADCIN10          // ADCC-A10/C10*
#define MTR1_VW_ADC_CH_NUM      ADC_CH_ADCIN7           // ADCC-A15/C7*
#define MTR1_VDC_ADC_CH_NUM     ADC_CH_ADCIN3           // ADCC-A7/C3*

#define MTR1_IU_ADC_SOC_NUM     ADC_SOC_NUMBER2         // ADCC-C6*     -SOC2-PPB1
#define MTR1_IV_ADC_SOC_NUM     ADC_SOC_NUMBER3         // ADCC-A9/C8*  -SOC3-PPB2
#define MTR1_IW_ADC_SOC_NUM     ADC_SOC_NUMBER4         // ADCC-A5/C2*  -SOC4-PPB3
#define MTR1_VU_ADC_SOC_NUM     ADC_SOC_NUMBER5         // ADCC-A8/C11* -SOC5
#define MTR1_VV_ADC_SOC_NUM     ADC_SOC_NUMBER6         // ADCC-A10/C10*-SOC6
#define MTR1_VW_ADC_SOC_NUM     ADC_SOC_NUMBER7         // ADCC-A15/C7* -SOC7
#define MTR1_VDC_ADC_SOC_NUM    ADC_SOC_NUMBER8         // ADCC-A7/C3*  -SOC8

#define MTR1_IU_ADC_PPB_NUM     ADC_PPB_NUMBER1         // ADCC-C6*    -SOC2-PPB1
#define MTR1_IV_ADC_PPB_NUM     ADC_PPB_NUMBER2         // ADCC-A9/C8* -SOC3-PPB2
#define MTR1_IW_ADC_PPB_NUM     ADC_PPB_NUMBER3         // ADCC-A5/C2*- SOC4-PPB3

// only for datalog and PWMDAC
#define MTR1_IU_ADCRESLT        MTR1_IU_ADCRES_BASE + ADC_O_RESULT2     // ADCC-C6*     -SOC2
#define MTR1_IV_ADCRESLT        MTR1_IV_ADCRES_BASE + ADC_O_RESULT3     // ADCC-A9/C8*  -SOC3
#define MTR1_IW_ADCRESLT        MTR1_IW_ADCRES_BASE + ADC_O_RESULT4     // ADCC-A5/C2*  -SOC4

#define MTR1_VU_ADCRESLT        MTR1_VU_ADCRES_BASE + ADC_O_RESULT5     // ADCC-A8/C11* -SOC5
#define MTR1_VV_ADCRESLT        MTR1_VV_ADCRES_BASE + ADC_O_RESULT6     // ADCC-A10/C10*-SOC6
#define MTR1_VW_ADCRESLT        MTR1_VW_ADCRES_BASE + ADC_O_RESULT7     // ADCC-A15/C7* -SOC7
#define MTR1_VDC_ADCRESLT       MTR1_VDC_ADCRES_BASE + ADC_O_RESULT8    // ADCC-A7/C3*  -SOC8

//------------------------------------------------------------------------------
// interrupt
#define MTR1_ADC_INT_BASE       ADCC_BASE               // ADCC-A7/C3*-SOC8
#define MTR1_ADC_INT_NUM        ADC_INT_NUMBER1         // ADCC_INT1-SOC8
#define MTR1_ADC_INT_SOC        ADC_SOC_NUMBER6         // ADCC_INT1-SOC8

#define MTR1_PIE_INT_NUM        INT_ADCC1               // ADCC_INT1-SOC8
#define MTR1_CPU_INT_NUM        INTERRUPT_CPU_INT1      // ADCC_INT1-CPU_INT1
#define MTR1_INT_ACK_GROUP      INTERRUPT_ACK_GROUP1    // ADCC_INT1-CPU_INT1

//------------------------------------------------------------------------------
// CMPSS
#define MTR1_IU_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3H-C6*
#define MTR1_IU_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3L-C6*, N/A

#define MTR1_IV_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2H-A9/C8*
#define MTR1_IV_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2L-A9/C8*

#define MTR1_IW_CMPHP_SEL       ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3H-A5/C2*, N/A
#define MTR1_IW_CMPLP_SEL       ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3L-A5/C2*

#define MTR1_IU_CMPHP_MUX       0                            // CMPSS3H-C6*
#define MTR1_IU_CMPLP_MUX       0                            // CMPSS3L-C6*, N/A

#define MTR1_IV_CMPHP_MUX       2                            // CMPSS2H-A9/C8*
#define MTR1_IV_CMPLP_MUX       2                            // CMPSS2L-A9/C8*

#define MTR1_IW_CMPHP_MUX       1                            // CMPSS3H-A5/C2*, N/A
#define MTR1_IW_CMPLP_MUX       1                            // CMPSS3L-A5/C2*

#define MTR1_CMPSS_DACH_VALUE   2048+1024+512
#define MTR1_CMPSS_DACL_VALUE   2048-1024-512

//------------------------------------------------------------------------------
// XBAR-EPWM
#define MTR1_XBAR_TRIP_ADDRL    XBAR_O_TRIP7MUX0TO15CFG
#define MTR1_XBAR_TRIP_ADDRH    XBAR_O_TRIP7MUX16TO31CFG

#define MTR1_IU_XBAR_EPWM_MUX   XBAR_EPWM_MUX04_CMPSS3_CTRIPH       // CMPSS3-HP, C6*
#define MTR1_IV_XBAR_EPWM_MUX   XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L  // CMPSS2-HP&LP, A9/C8*
#define MTR1_IW_XBAR_EPWM_MUX   XBAR_EPWM_MUX05_CMPSS3_CTRIPL       // CMPSS3-LP, A5/C2*

#define MTR1_IU_XBAR_MUX        XBAR_MUX04          // CMPSS3-HP
#define MTR1_IV_XBAR_MUX        XBAR_MUX02          // CMPSS2-HP&LP
#define MTR1_IW_XBAR_MUX        XBAR_MUX05          // CMPSS3-LP

#define MTR1_XBAR_INPUT1        XBAR_INPUT1
#define MTR1_TZ_OSHT1           EPWM_TZ_SIGNAL_OSHT1

#define MTR1_XBAR_TRIP          XBAR_TRIP7
#define MTR1_DCTRIPIN           EPWM_DC_COMBINATIONAL_TRIPIN7

#define MTR1_IU_XBAR_OUT_MUX    XBAR_OUT_MUX04_CMPSS3_CTRIPOUTH       // CMPSS3-HP
#define MTR1_IV_XBAR_OUT_MUX    XBAR_OUT_MUX02_CMPSS2_CTRIPOUTH_OR_L  // CMPSS2-HP&LP
#define MTR1_IW_XBAR_OUT_MUX    XBAR_OUT_MUX05_CMPSS3_CTRIPOUTL       // CMPSS3-LP

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
#define MTR1_PWM_DBFED_CNT      225           // 2.25us

//! \brief Defines the PWM deadband rising edge delay count (system clocks)
#define MTR1_PWM_DBRED_CNT      245           // 2.45us

// end of HVMTRPFC_REV1P1
//------------------------------------------------------------------------------
#else
#error The HAL configuration is not defined in hal.h
#endif

// **************************************************************************
// the typedefs
//------------------------------------------------------------------------------
//! \brief Defines the function to turn LEDs off
//!
#define HAL_turnLEDOff              HAL_setGPIOHigh

//! \brief Defines the function to turn LEDs on
//!
#define HAL_turnLEDOn               HAL_setGPIOLow

//! \brief Defines the function to toggle LEDs
//!
#define HAL_toggleLED               HAL_toggleGPIO

//! \brief Enumeration for the LED numbers
//!

#if defined(BSXL8323RS_REVA)
typedef enum
{
    HAL_GPIO_LED1C   = 31,   //!< GPIO pin number for LaunchPad LED 1
    HAL_GPIO_LED2C   = 34,   //!< GPIO pin number for LaunchPad LED 2
    HAL_GPIO_LED1B   = 19,   //!< GPIO pin number for BoostxlPak LED 1
    HAL_GPIO_LED2B   = 19,   //!< GPIO pin number for BoostxlPak LED 2
    HAL_GPIO_ISR_M1  = 39    //!< GPIO pin number for ISR Executing Time
} HAL_GPIO_Number_e;
// BSXL8323RS_REVA
#elif defined(BSXL8323RH_REVB)
typedef enum
{
    HAL_GPIO_LED1C   = 31,   //!< GPIO pin number for LaunchPad LED 1
    HAL_GPIO_LED2C   = 31,   //!< GPIO pin number for LaunchPad LED 2
    HAL_GPIO_LED1B   = 19,   //!< GPIO pin number for BoostxlPak LED 1
    HAL_GPIO_LED2B   = 19,   //!< GPIO pin number for BoostxlPak LED 2
    HAL_GPIO_ISR_M1  = 28    //!< GPIO pin number for ISR Executing Time
} HAL_GPIO_Number_e;
// BSXL8323RH_REVB
#elif defined(BSXL8316RT_REVA)
typedef enum
{
    HAL_GPIO_LED1C   = 31,   //!< GPIO pin number for LaunchPad LED 1
    HAL_GPIO_LED2C   = 34,   //!< GPIO pin number for LaunchPad LED 2
    HAL_GPIO_LED1B   = 19,   //!< GPIO pin number for BoostxlPak LED 1
    HAL_GPIO_LED2B   = 19,   //!< GPIO pin number for BoostxlPak LED 2
    HAL_GPIO_ISR_M1  = 39    //!< GPIO pin number for ISR Executing Time
} HAL_GPIO_Number_e;
// BSXL8316RT_REVA
#elif defined(BSXL8353RS_REVA)
typedef enum
{
    HAL_GPIO_LED1C   = 31,   //!< GPIO pin number for LaunchPad LED 1
    HAL_GPIO_LED2C   = 34,   //!< GPIO pin number for LaunchPad LED 2
    HAL_GPIO_LED1B   = 39,   //!< GPIO pin number for BoostxlPak LED 1
    HAL_GPIO_LED2B   = 39,   //!< GPIO pin number for BoostxlPak LED 2
    HAL_GPIO_ISR_M1  = 39    //!< GPIO pin number for ISR Executing Time
} HAL_GPIO_Number_e;
// BSXL8353RS_REVA
#elif defined(BSXL3PHGAN_REVA)
typedef enum
{
    HAL_GPIO_LED1C   = 31,   //!< GPIO pin number for LaunchPad LED 1
    HAL_GPIO_LED2C   = 34,   //!< GPIO pin number for LaunchPad LED 2
    HAL_GPIO_LED1B   = 39,   //!< GPIO pin number for BoostxlPak LED 1
    HAL_GPIO_LED2B   = 39,   //!< GPIO pin number for BoostxlPak LED 2
    HAL_GPIO_ISR_M1  = 39    //!< GPIO pin number for ISR Executing Time
} HAL_GPIO_Number_e;
// BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
typedef enum
{
    HAL_GPIO_LED1C   = 31,    //!< GPIO pin number for ControlCard LED 1
    HAL_GPIO_LED2C   = 34,    //!< GPIO pin number for ControlCard LED 2
    HAL_GPIO_LED1B   = 42,    //!< GPIO pin number for InverterBoard LED 1
    HAL_GPIO_LED2B   = 42,    //!< GPIO pin number for InverterBoard LED 2
    HAL_GPIO_ISR_M1  = 42     //!< GPIO pin number for M1 ISR Executing Time
} HAL_GPIO_Number_e;
// HVMTRPFC_REV1P1
#else
#error Not defined GPIOs for LED & Debug in hal.h
#endif

//! \brief Enumeration for the sensor types
//!
typedef enum
{
    HAL_SENSORTYPE_CURRENT = 0,  //!< Enumeration for current sensor
    HAL_SENSORTYPE_VOLTAGE = 1   //!< Enumeration for voltage sensor
} HAL_SensorType_e;

//! \brief Enumeration for the QEP setup
//!
typedef enum
{
    HAL_QEP_QEP1=0,  //!< Select QEP1
    HAL_QEP_QEP2=1   //!< Select QEP2
} HAL_QEPSelect_e;

//! \brief Enumeration for the CPU Timer
//!
typedef enum
{
    HAL_CPU_TIMER0 = 0,  //!< Select CPU Timer0
    HAL_CPU_TIMER1 = 1,  //!< Select CPU Timer1
    HAL_CPU_TIMER2 = 2   //!< Select CPU Timer2
} HAL_CPUTimerNum_e;

// **************************************************************************
// the globals
extern __interrupt void motor1CtrlISR(void);

// CLA Tasks
extern __interrupt void task_initModules(void);
extern __interrupt void task_mainISR(void);
extern __interrupt void task_mainLoop(void);
extern __interrupt void cla1Task4(void);
extern __interrupt void cla1Task5(void);
extern __interrupt void cla1Task6(void);
extern __interrupt void cla1Task7(void);
extern __interrupt void cla1Task8(void);
extern __interrupt void cla_EST_run_BackgroundTask(void);

extern volatile uint16_t mtrPIEIER;
extern volatile uint16_t mtrIER;

extern HAL_Handle halHandle;

// **************************************************************************
// the function prototypes

//! \brief     Acknowledges an interrupt from the ADC so that another ADC
//!            interrupt can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_ackMtr1ADCInt(void)
{
#if defined(MOTOR1_DCLINKSS)
    // clear the PWM interrupt flag
    EPWM_clearEventTriggerInterruptFlag(MTR1_PWM_INT_BASE);

    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    // Acknowledge interrupt from PIE group
    Interrupt_clearACKGroup(MTR1_INT_ACK_GROUP);
#else   // !(MOTOR1_DCLINKSS)
    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    // Acknowledge interrupt from PIE group
    Interrupt_clearACKGroup(MTR1_INT_ACK_GROUP);
#endif  // !(MOTOR1_DCLINKSS)

    return;
} // end of HAL_ackADCInt() function


//! \brief      Disables global interrupts
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_disableGlobalInts(HAL_Handle handle);


//! \brief      Enables the ADC interrupts
//! \details    Enables the ADC interrupt in the PIE, and CPU.  Enables the 
//!             interrupt to be sent from the ADC peripheral.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableCtrlInts(HAL_Handle handle);


//! \brief      Enables the ADC interrupts without CPU interrupts
//! \details    Enables the ADC interrupts to only trigger CLA, and without
//!             interrupting the CPU
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableADCIntsToTriggerCLA(HAL_Handle handle);


//! \brief      Enables the debug interrupt
//! \details    The debug interrupt is used for the real-time debugger.  It is
//!             not needed if the real-time debugger is not used.  Clears
//!             bit 1 of ST1.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDebugInt(HAL_Handle handle);


//! \brief     Enables global interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableGlobalInts(HAL_Handle handle);


//! \brief     Gets the PWM duty cycle times
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] pDutyCycles  A pointer to memory for the duty cycle durations
static inline void
HAL_getDutyCycles(HAL_MTR_Handle handle,uint16_t *pDutyCycles)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  pDutyCycles[0] = EPWM_getCounterCompareValue(obj->pwmHandle[0],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[1] = EPWM_getCounterCompareValue(obj->pwmHandle[1],
                                               EPWM_COUNTER_COMPARE_A);
  pDutyCycles[2] = EPWM_getCounterCompareValue(obj->pwmHandle[2],
                                               EPWM_COUNTER_COMPARE_A);
  return;
} // end of HAL_getDutyCycles() function


//! \brief     Gets the number of current sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of current sensors
static inline uint16_t HAL_getNumCurrentSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
  
  return(obj->numCurrentSensors);
} // end of HAL_getNumCurrentSensors() function


//! \brief     Gets the number of voltage sensors
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The number of voltage sensors
static inline uint16_t HAL_getNumVoltageSensors(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->numVoltageSensors);
} // end of HAL_getNumVoltageSensors() function

//! \brief     Gets the pwm enable status
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
//! \return    The pwm enable
static inline bool HAL_getPwmEnableStatus(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  return(obj->flagEnablePWM);
} // end of HAL_getPwmStatus() function


//! \brief     Get the period of EPWM time-base module
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \return    The periode of EPWM time-base module
static inline uint16_t
HAL_getTimeBasePeriod(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    return(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));
} // end of HAL_getTimeBasePeriod() function


//! \brief      Configures the fault protection logic
//! \details    Sets up the trip zone inputs so that when a comparator
//!             signal from outside the micro-controller trips a fault,
//!             the EPWM peripheral blocks will force the
//!             power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupMtrFaults(HAL_MTR_Handle handle);

//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL_MTR object.
//! \param[in]  pMemory   A pointer to the memory for the hardware abstraction layer object
//! \param[in]  numBytes  The number of bytes allocated for the hardware abstraction layer object, bytes
//! \return     The hardware abstraction layer (HAL_MTR) object handle
extern HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes);


//! \brief      Initializes the interrupt vector table
//! \details    Points the ISR to the function mainISR.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_initIntVectorTable(HAL_Handle handle)
{
    // Motor_1->ADCA_INT1
    Interrupt_register(MTR1_PIE_INT_NUM, &motor1CtrlISR);

    return;
} // end of HAL_initIntVectorTable() function

//! \brief      Reads the ADC data with offset
//! \details    Reads in the ADC result registers and scales the values
//!             according to the settings in user_m1.h or user_m2.h.
//!             The structure gAdcData holds three phase voltages,
//!             three line currents, and one DC bus voltage.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
//! \param[in]  pADCData  A pointer to the ADC data buffer
static inline void
HAL_readMtr1ADCData(HAL_ADCData_t *pADCData)
{
    float32_t value;

#if defined(MOTOR1_ISBLDC)
    // convert dc-link current 1
    value = (float32_t)ADC_readPPBResult(MTR1_IDC1_ADCRES_BASE, MTR1_IDC1_ADC_PPB_NUM);
    pADCData->Idc1_A.value[0] = value * pADCData->current_sf;

    // convert dc-link current 2
    value = (float32_t)ADC_readPPBResult(MTR1_IDC2_ADCRES_BASE, MTR1_IDC2_ADC_PPB_NUM);
    pADCData->Idc1_A.value[1] = value * pADCData->current_sf;

    // convert phase A voltage
    value = (float32_t)ADC_readResult(MTR1_VU_ADCRES_BASE, MTR1_VU_ADC_SOC_NUM);
    pADCData->V_V.value[0] = value * pADCData->voltage_sf;

    // convert phase B voltage
    value = (float32_t)ADC_readResult(MTR1_VV_ADCRES_BASE, MTR1_VV_ADC_SOC_NUM);
    pADCData->V_V.value[1] = value * pADCData->voltage_sf;

    // convert phase C voltage
    value = (float32_t)ADC_readResult(MTR1_VW_ADCRES_BASE, MTR1_VW_ADC_SOC_NUM);
    pADCData->V_V.value[2] = value * pADCData->voltage_sf;

    // convert dc bus voltage
    value = (float32_t)ADC_readResult(MTR1_VDC_ADCRES_BASE, MTR1_VDC_ADC_SOC_NUM);
    pADCData->VdcBus_V = value * pADCData->dcBusvoltage_sf;

#if defined(CMD_POT_EN)
    // read POT adc value
    pADCData->potAdc = ADC_readResult(MTR1_POT_ADCRES_BASE, MTR1_POT_ADC_SOC_NUM);
#endif  // CMD_POT_EN
//------------------------------------------------------------------------------
#else   // !MOTOR1_ISBLDC
#if defined(MOTOR1_DCLINKSS)
    // convert dc-link current 1
    value = (float32_t)ADC_readPPBResult(MTR1_IDC1_ADCRES_BASE, MTR1_IDC1_ADC_PPB_NUM);
    pADCData->Idc1_A.value[0] = value * pADCData->current_sf;

    // convert dc-link current 2
    value = (float32_t)ADC_readPPBResult(MTR1_IDC2_ADCRES_BASE, MTR1_IDC2_ADC_PPB_NUM);
    pADCData->Idc1_A.value[1] = value * pADCData->current_sf;

    // convert dc-link current 3
    value = (float32_t)ADC_readPPBResult(MTR1_IDC3_ADCRES_BASE, MTR1_IDC3_ADC_PPB_NUM);
    pADCData->Idc2_A.value[0] = value * pADCData->current_sf;

    // convert dc-link current 4th
    value = (float32_t)ADC_readPPBResult(MTR1_IDC4_ADCRES_BASE, MTR1_IDC4_ADC_PPB_NUM);
    pADCData->Idc2_A.value[1] = value * pADCData->current_sf;
#else   // !(MOTOR1_DCLINKSS)

    // convert phase A current
    value = (float32_t)ADC_readPPBResult(MTR1_IU_ADCRES_BASE, MTR1_IU_ADC_PPB_NUM);
    pADCData->I_A.value[0] = value * pADCData->current_sf;

    // convert phase B current
    value = (float32_t)ADC_readPPBResult(MTR1_IV_ADCRES_BASE, MTR1_IV_ADC_PPB_NUM);
    pADCData->I_A.value[1] = value * pADCData->current_sf;

    // convert phase C current
    value = (float32_t)ADC_readPPBResult(MTR1_IW_ADCRES_BASE, MTR1_IW_ADC_PPB_NUM);
    pADCData->I_A.value[2] = value * pADCData->current_sf;
#endif  // !(MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST)
    // convert phase A voltage
    value = (float32_t)ADC_readResult(MTR1_VU_ADCRES_BASE, MTR1_VU_ADC_SOC_NUM);
    pADCData->V_V.value[0] = value * pADCData->voltage_sf;

    // convert phase B voltage
    value = (float32_t)ADC_readResult(MTR1_VV_ADCRES_BASE, MTR1_VV_ADC_SOC_NUM);
    pADCData->V_V.value[1] = value * pADCData->voltage_sf;

    // convert phase C voltage
    value = (float32_t)ADC_readResult(MTR1_VW_ADCRES_BASE, MTR1_VW_ADC_SOC_NUM);
    pADCData->V_V.value[2] = value * pADCData->voltage_sf;
#endif  // MOTOR1_FAST

    // convert dc bus voltage
    value = (float32_t)ADC_readResult(MTR1_VDC_ADCRES_BASE, MTR1_VDC_ADC_SOC_NUM);
    pADCData->VdcBus_V = value * pADCData->dcBusvoltage_sf;

#if defined(CMD_POT_EN)
    // read POT adc value
    pADCData->potAdc = ADC_readResult(MTR1_POT_ADCRES_BASE, MTR1_POT_ADC_SOC_NUM);
#endif  // CMD_POT_EN
#endif  // !MOTOR1_ISBLDC
    return;
} // end of HAL_readMtr1ADCData() functions

//! \brief     Reads the timer count
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] timerNumber  The timer number, 0,1 or 2
//! \return    The timer count
static inline uint32_t
HAL_readTimerCnt(HAL_Handle handle,const uint16_t timerNumber)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint32_t timerCnt = CPUTimer_getTimerCount(obj->timerHandle[timerNumber]);

    return(timerCnt);
} // end of HAL_readTimerCnt() function

//! \brief     Sets the GPIO pin high
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOHigh(HAL_Handle handle,const uint32_t gpioNumber)
{

  // set GPIO high
  GPIO_writePin(gpioNumber, 1);

  return;
} // end of HAL_setGPIOHigh() function


//! \brief     Read the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
//! \return    The GPIO pin
static inline uint32_t
HAL_readGPIOData(HAL_Handle handle,const uint32_t gpioNumber)
{
    uint32_t gpioPinData;

    // set GPIO high
    gpioPinData = GPIO_readPin(gpioNumber);

    return(gpioPinData);
} // end of HAL_readGPIOData() function


//! \brief     Sets the GPIO pin low
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_setGPIOLow(HAL_Handle handle,const uint32_t gpioNumber)
{
    // set GPIO low
    GPIO_writePin(gpioNumber, 0);

    return;
} // end of HAL_setGPIOLow() function


//! \brief     Sets the value of the internal DAC of the high comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the high comparator
static inline void
HAL_setCMPSSDACValueHigh(HAL_MTR_Handle handle,
                         const uint16_t cmpssNumber, uint16_t dacValue)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // set GPIO low
    CMPSS_setDACValueHigh(obj->cmpssHandle[cmpssNumber], dacValue);

    return;
} // end of HAL_setCMPSSDACValueHigh() function


//! \brief     Sets the value of the internal DAC of the low comparator
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] cmpssNumber The CMPSS number
//! \param[in] dacValue    The DAC value of the low comparator
static inline void
HAL_setCMPSSDACValueLow(HAL_MTR_Handle handle,
                        const uint16_t cmpssNumber, uint16_t dacValue)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // set GPIO low
  CMPSS_setDACValueLow(obj->cmpssHandle[cmpssNumber], dacValue);

  return;
} // end of HAL_setCMPSSDACValueLow() function

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
static inline void
HAL_setNumVoltageSensors(HAL_MTR_Handle handle,const uint16_t numVoltageSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numVoltageSensors = numVoltageSensors;

  return;
} // end of HAL_setNumVoltageSensors() function

//! \brief     Sets the number of current sensors
//! \param[in] handle             The hardware abstraction layer (HAL) handle
//! \param[in] numCurrentSensors  The number of current sensors
static inline void
HAL_setNumCurrentSensors(HAL_MTR_Handle handle,const uint16_t numCurrentSensors)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  obj->numCurrentSensors = numCurrentSensors;

  return;
} // end of HAL_setNumCurrentSensors() function

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.  Creates all of the scale
//!             factors for the ADC voltage and current conversions.  Sets the initial
//!             offset values for voltage and current measurements.
//! \param[in]  handle       The hardware abstraction layer (HAL) handle
extern void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams);

//! \brief      Sets up the ADCs (Analog to Digital Converters)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupADCs(HAL_Handle handle);

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCAPs(HAL_MTR_Handle handle);

//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle);

//! \brief     Read the CAP counters
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \return    The CAP counters
static inline uint32_t HAL_calcCAPCount(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;
    uint32_t capSumCount = 0;

    for(cnt = 0; cnt < 3; cnt++)
    {
        capSumCount += ECAP_getEventTimeStamp(obj->capHandle[cnt], ECAP_EVENT_2);
        capSumCount += ECAP_getEventTimeStamp(obj->capHandle[cnt], ECAP_EVENT_3);
    }

    return(capSumCount);
}
#elif defined(CMD_CAP_EN)
//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCAPs(HAL_MTR_Handle handle);

//! \brief      Sets up the CAP (Capture Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle);

//! \brief     Read the CAP counters
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \return    The CAP counters
static inline uint32_t HAL_calcCAPCount(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    uint32_t capSumCount = ECAP_getEventTimeStamp(obj->capHandle, ECAP_EVENT_1);
    capSumCount += ECAP_getEventTimeStamp(obj->capHandle, ECAP_EVENT_2);
    capSumCount += ECAP_getEventTimeStamp(obj->capHandle, ECAP_EVENT_3);
    capSumCount += ECAP_getEventTimeStamp(obj->capHandle, ECAP_EVENT_4);

    return(capSumCount);
}
#endif  // HALL_ENABLE || CMD_CAP_EN

//! \brief      Sets up the CMPSSs (Comparator Subsystems)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCMPSSs(HAL_MTR_Handle handle);

//! \brief      Sets up the clocks
//! \details    Sets up the micro-controller's main oscillator
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupClks(HAL_Handle handle);

//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGPIOs(HAL_Handle handle);

//! \brief     Sets up the FLASH.
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFlash(HAL_Handle handle);

//! \brief     Sets up the CLA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCLA(HAL_Handle handle);

//! \brief     Sets up the peripheral clocks
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPeripheralClks(HAL_Handle handle);

//! \brief     Sets up the PIE (Peripheral Interrupt Expansion)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPIE(HAL_Handle handle);

#if defined(EPWMDAC_MODE)
//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz);
#endif  // EPWMDAC_MODE

//! \brief     Sets up the QEP peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupQEP(HAL_MTR_Handle handle);

//! \brief     Sets up the SCIA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSCIA(HAL_Handle handle);

//! \brief     Sets up the I2CA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupI2CA(HAL_Handle halHandle);

// Declare HAL_setupGate and HAL_enableDRV
#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
//! \brief      Enables the drv8323/drv8353/drv8316 device
//! \details    Provides the correct timing to enable the drv8323/8353/8316
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);

//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_MTR_Handle handle);

//! \brief     Writes data to the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8323Vars  SPI variables
void HAL_writeDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars);

//! \brief     Reads data from the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8323Vars  SPI variables
void HAL_readDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars);

//! \brief     Sets up the SPI interface for the driver
//! \param[in] handle         The hardware abstraction layer (HAL) handle
//! \param[in] drv8323Vars  SPI variables
extern void HAL_setupDRVSPI(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars);

//! \brief     Sets up the SPI
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupSPI(HAL_MTR_Handle handle);
       // BSXL8323RS_REVA || BSXL8353RS_REVA || BSXL8316RT_REVA
#elif defined(BSXL8323RH_REVB) || defined(BSXL3PHGAN_REVA)
//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_MTR_Handle handle);

//! \brief      Enables the drv8323/8353/8316 device
//! \details    Provides the correct timing to enable the drv8323/8353/8316
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);
#elif defined(HVMTRPFC_REV1P1)
//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_MTR_Handle handle);

//! \brief      Enables the high voltage kit device
//! \details    Provides the correct timing to enable the high voltage kit
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableDRV(HAL_MTR_Handle handle);
#endif  // Declare HAL_setupGate and HAL_enableDRV

//! \brief     Sets up the CPU timer for time base
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupTimeBaseTimer(HAL_Handle handle,
                                   const float32_t timeBaseFreq_Hz);

//! \brief     Sets up the CPU timer for ADC trigger
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
extern void HAL_setupADCTriggerTimer(HAL_Handle handle,
                                     const float32_t adcTriggerFreq_Hz);

//! \brief     Sets up the timers for CPU usage diagnostics
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setupCPUUsageTimer(HAL_Handle handle);

//! \brief     Sets up the timers
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
static inline void
HAL_clearCPUTimerFlag(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    CPUTimer_clearOverflowFlag(obj->timerHandle[cpuTimerNumber]);

    return;
}   // end of HAL_clearTimerFlag() function


//! Sets the CPU timer period count.
//! \param base is the base address of the timer module.
//! \param timerCount is the CPU timer timer count.
static inline void HAL_setCPUTimerCount(HAL_Handle halHandle, uint32_t timerCount)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    //
    // Load the MSB timer Count
    //
    HWREG(obj->timerHandle[1] + CPUTIMER_O_TIM) = timerCount;

    return;
}

//! \brief     Gets CPU Timer status
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] cpuTimerNumber  The CPU timer number
static inline bool
HAL_getCPUTimerStatus(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    return (CPUTimer_getTimerOverflowStatus(obj->timerHandle[cpuTimerNumber]));
}

#if defined(DATALOGI4_EN) || defined(DATALOGF2_EN)
//! \brief     Sets up the DMA for datalog
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] dmaChNumber     The DMC Channel Number
//! \param[in] destAddr    The Datalog buffer dest address
//! \param[in] srcAddr     The Datalog buffer src address
void HAL_setupDMAforDLOG(HAL_Handle handle, const uint16_t dmaChNum,
                     const void *destAddr, const void *srcAddr);
#endif  // DATALOGI4_EN || DATALOGF2_EN

//! \brief     Sets up the DMA
//! \param[in] N/A
void HAL_setupDMA(void);

#if defined(DATALOGF2_EN) || defined(DATALOGI4_EN)
//! \brief     Force trig the DMA channel for datalog
//! \param[in] handle          The hardware abstraction layer (HAL) handle
//! \param[in] dmaChNumber     The DMC Channel Number
static inline void
HAL_trigDMAforDLOG(HAL_Handle handle, const uint16_t DMAChNum)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    DMA_startChannel(obj->dmaChHandle[DMAChNum]);

    DMA_forceTrigger(obj->dmaChHandle[DMAChNum]);

    return;
} // end of HAL_trigDlogWithDMA() function
#endif  // DATALOGF2_EN || DATALOGI4_EN

//! \brief     Toggles the GPIO pin
//! \param[in] handle      The hardware abstraction layer (HAL) handle
//! \param[in] gpioNumber  The GPIO number
static inline void HAL_toggleGPIO(HAL_Handle handle,const uint32_t gpioNumber)
{

    // set GPIO high
    GPIO_togglePin(gpioNumber);

    return;
} // end of HAL_toggleGPIO() function

#if defined(EPWMDAC_MODE)
//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
static inline void
HAL_writePWMDACData(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // convert values from float to unit16
    uint16_t cnt;
    float32_t period;
    float32_t dacData_sat_dc;
    float32_t dacData;
    float32_t value;

    int16_t cmpValue[4];

    period = (float32_t)pPWMDACData->periodMax;

    for(cnt = 0; cnt < 4; cnt++)
    {
        dacData = (*pPWMDACData->ptrData[cnt]);
        dacData_sat_dc = dacData * pPWMDACData->gain[cnt] +
                pPWMDACData->offset[cnt];

        value = dacData_sat_dc * period;

        cmpValue[cnt] = (int16_t)__fsat(value, period, 0);
    }

    // write the DAC data
    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_1],
                                EPWM_COUNTER_COMPARE_A, cmpValue[0]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_2],
                                EPWM_COUNTER_COMPARE_B, cmpValue[1]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_3],
                                EPWM_COUNTER_COMPARE_A, cmpValue[2]);

    // write the PWM data value
    EPWM_setCounterCompareValue(obj->pwmDACHandle[PWMDAC_NUMBER_4],
                                EPWM_COUNTER_COMPARE_B, cmpValue[3]);

    return;
} // end of HAL_writeDacData() function
#endif  // EPWMDAC_MODE

//! \brief     Writes DAC data to the PWM comparators for DAC output
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMDACData  The pointer to the DAC data
void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData);

//! \brief
//! \param[in]
//! \param[in]
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);

//! \brief     Reads PWM period register
//! \param[in] handle     The hardware abstraction layer (HAL) handle
//! \param[in] pwmNumber  The PWM number
//! \return    The PWM period value
static inline uint16_t
HAL_readPWMPeriod(HAL_MTR_Handle handle,const uint16_t pwmNumber)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  // the period value to be returned
  uint16_t pwmPeriodValue;

  pwmPeriodValue = EPWM_getTimeBasePeriod(obj->pwmHandle[pwmNumber]);

  return(pwmPeriodValue);
} // end of HAL_readPWMPeriod() function

//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMData(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    float32_t period = (float32_t)(EPWM_getTimeBasePeriod(obj->pwmHandle[0]));

    uint16_t pwmCnt;

    for(pwmCnt=0; pwmCnt<3; pwmCnt++)
    {
      // compute the value
        float32_t V_pu = -pPWMData->Vabc_pu.value[pwmCnt];      // Negative
        float32_t V_sat_pu = __fsat(V_pu, 0.5, -0.5);           // -0.5~0.5
        float32_t V_sat_dc_pu = V_sat_pu + 0.5;                 // 0~1.0
        pPWMData->cmpValue[pwmCnt]  = (int16_t)(V_sat_dc_pu * period);  //

        if(pPWMData->cmpValue[pwmCnt] < pPWMData->minCMPValue)
        {
            pPWMData->cmpValue[pwmCnt] = pPWMData->minCMPValue;
        }

        // write the PWM data value
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pPWMData->cmpValue[pwmCnt]);

        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_B,
                                    pPWMData->cmpValue[pwmCnt]);
    }

    return;
} // end of HAL_writePWMData() function


//! \brief      Enables the PWM devices for motor control
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enablePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
#if defined(HVMTRPFC_REV1P1)
    GPIO_writePin(obj->gateEnableGPIO, 0);

    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    GPIO_writePin(obj->gateEnableGPIO, 1);
#elif defined(BSXL3PHGAN_REVA)
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    GPIO_writePin(obj->gateEnableGPIO, 0);
#else   //!HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(MOTOR1_ISBLDC) && (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
    GPIO_writePin(MTR1_DRV_INL_A, 1);
    GPIO_writePin(MTR1_DRV_INL_B, 1);
    GPIO_writePin(MTR1_DRV_INL_C, 1);
#endif  // MOTOR1_ISBLDC && (BSXL8323RS_REVA || SXL8323RH_REVB)

    obj->flagEnablePWM = true;

    return;
} // end of HAL_enablePWM() function


//! \brief      Enables the PWM for braking
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_enableBrakePWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

#if defined(HVMTRPFC_REV1P1)
    GPIO_writePin(obj->gateEnableGPIO, 0);

    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, false);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, false);
    }

    GPIO_writePin(obj->gateEnableGPIO, 1);
#elif defined(BSXL3PHGAN_REVA)
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, false);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, false);
    }

    GPIO_writePin(obj->gateEnableGPIO, 0);
#else   //!HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_OUTPUT_LOW);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_OUTPUT_HIGH);

         // setup the Dead-Band Generator Control Register (DBCTL)
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, false);
         EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, false);
    }
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA

    obj->flagEnablePWM = false;

    return;
} // end of HAL_enableBrakePWM() function


//! \brief      Enables the PWM for braking
//! \details    Turns on the outputs of the EPWM peripheral which will allow
//!             the power switches to be controlled.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_exitBrakeResetPWM(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

#if defined(HVMTRPFC_REV1P1)
    GPIO_writePin(obj->gateEnableGPIO, 0);

    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }

    GPIO_writePin(obj->gateEnableGPIO, 1);
#elif defined(BSXL3PHGAN_REVA)
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }

    GPIO_writePin(obj->gateEnableGPIO, 0);
#else   //!HVMTRPFC_REV1P1 & BSXL3PHGAN_REVA
    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_A,
                                                  EPWM_AQ_SW_DISABLED);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
         EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                  EPWM_AQ_OUTPUT_B,
                                                  EPWM_AQ_SW_DISABLED);
    }
#endif  //!HVMTRPFC_REV1P1  & BSXL3PHGAN_REVA

    obj->flagEnablePWM = false;

    return;
} // end of HAL_enableBrakePWM() function

//! \brief      clear fault status of motor control
//! \details
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_clearMtrFaultStatus(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB
#else    // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
#if defined(HVMTRPFC_REV1P1)
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
#else   //!HVMTRPFC_REV1P1
    // Clear any comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
#endif  //!HVMTRPFC_REV1P1
#endif   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_clearMtrFaultStatus() function

//! \brief      Disables the PWM device for motor control
//! \details    Turns off the outputs of the EPWM peripherals which will put
//!             the power switches into a high impedance state.
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
static inline void HAL_disablePWM(HAL_MTR_Handle handle)
{
  HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

  EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
  EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(BSXL3PHGAN_REVA)
    GPIO_writePin(obj->gateEnableGPIO, 1);
#elif defined(BSXL8316RT_REVA)
//    GPIO_writePin(obj->drvOffGPIO, 1);
#endif  // BSXL8316RT_REVA

#if defined(MOTOR1_ISBLDC) && (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
    GPIO_writePin(MTR1_DRV_INL_A, 0);
    GPIO_writePin(MTR1_DRV_INL_B, 0);
    GPIO_writePin(MTR1_DRV_INL_C, 0);
#endif  // MOTOR1_ISBLDC && (BSXL8323RS_REVA || SXL8323RH_REVB)

  obj->flagEnablePWM = false;

  return;
} // end of HAL_disablePWM() function

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setupPWMs(HAL_MTR_Handle handle);

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
static inline uint16_t HAL_getMtrTripFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t tripFault = 0;

    tripFault = (EPWM_getTripZoneFlagStatus(obj->pwmHandle[0]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[1]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2)) |
                    (EPWM_getTripZoneFlagStatus(obj->pwmHandle[2]) &
            (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCAEVT2));

    return(tripFault);
}

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle          The hardware abstraction layer (HAL) handle
extern void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                               const uint16_t dacValH, const uint16_t dacValL);

//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] ignoreShunt  The low side shunt that should be ignored
//! \param[in] midVolShunt  The middle length of output voltage
static inline void HAL_setTrigger(HAL_MTR_Handle handle, HAL_PWMData_t *pPWMData,
                                  const SVGENCURRENT_IgnoreShunt_e ignoreShunt,
                                  const SVGENCURRENT_VmidShunt_e midVolShunt)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    int16_t pwmNum = midVolShunt;
    int16_t pwmCMPA = EPWM_getCounterCompareValue(obj->pwmHandle[pwmNum],
                                                   EPWM_COUNTER_COMPARE_A);

    int16_t pwmSOCCMP = 5;

    if(ignoreShunt == SVGENCURRENT_USE_ALL)
    {
        // Set up event source for ADC trigger
        EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC);
    }
    else
    {
        pwmSOCCMP = pwmCMPA - pPWMData->deadband - pPWMData->noiseWindow;

        if(pwmSOCCMP <= 0)
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_U_CMPC);
        }
        else
        {
            pwmSOCCMP = 5;

            // Set up event source for ADC trigger
            EPWM_setADCTriggerSource(obj->pwmHandle[0],
                                     EPWM_SOC_A,
                                     EPWM_SOC_TBCTR_D_CMPC);
        }

    }

    //
    pPWMData->socCMP = pwmSOCCMP;

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                pwmSOCCMP);
    return;
} // end of HAL_setTrigger() function

#if defined(MOTOR1_DCLINKSS)
//! \brief     PWM phase shift and ADC SOC timing for dc_link current sensing
//! \details   PWM phase shift compensation based on each phase duty and min duration.
//!            dc_link current sampling point is calcuated with shifted PWM compare value.
//! \param[in] handle       The hardware abstraction layer (HAL) handle
//! \param[in] dclinkHandle The dclink handle
//! \param[in] pVab_out     The pointer to the Vab voltage
//! \param[in] pPWMData     The pointer to the PWM data
static inline void
HAL_runSingleShuntCompensation(HAL_MTR_Handle handle,
                               DCLINK_SS_Handle dclinkHandle,
                               const MATH_vec2 *pVab_out,
                               HAL_PWMData_t *pPWMData, const float32_t Vdc_V)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    MATH_ui_vec2 upSoc, downSoc;
    MATH_ui_vec3 pwmCMPA, pwmCMPB;

    // get EPWMx CMPA/CMPB values
    pwmCMPA.value[0] = pwmCMPB.value[0] = pPWMData->cmpValue[0];
    pwmCMPA.value[1] = pwmCMPB.value[1] = pPWMData->cmpValue[1];
    pwmCMPA.value[2] = pwmCMPB.value[2] = pPWMData->cmpValue[2];

    // run PWM compensation for single-shunt
    DCLINK_SS_runPWMCompensation(dclinkHandle, pVab_out, Vdc_V,
                              &pwmCMPA, &pwmCMPB, &upSoc, &downSoc);

#if(DMC_BUILDLEVEL >= DMC_LEVEL_2)
    uint16_t pwmCnt;

    for(pwmCnt=0; pwmCnt<3; pwmCnt++)
    {
        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmCMPA.value[pwmCnt]);

        EPWM_setCounterCompareValue(obj->pwmHandle[pwmCnt],
                                    EPWM_COUNTER_COMPARE_B,
                                    pwmCMPB.value[pwmCnt]);
    }
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

    // set the SOC trigger point for UP count
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C,
                                upSoc.value[0]);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D,
                                upSoc.value[1]);

    // set the SOC trigger point for DOWN count
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C,
                                downSoc.value[0]);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D,
                                downSoc.value[1]);

#ifdef SST_ENABLE
    // set the CMPA/CMPB for SST
    EPWM_setCounterCompareValue(obj->sstpwmHandle,
                                EPWM_COUNTER_COMPARE_A,
                                downSoc.value[0]);

    EPWM_setCounterCompareValue(obj->sstpwmHandle,
                                EPWM_COUNTER_COMPARE_B,
                                downSoc.value[1]);
#endif  // SST_ENABLE

    return;
} // end of HAL_singleShuntCompensation() function

//! \brief     Set trigger point near period for dc-link current offset
//! \param[in] handle       The hardware abstraction layer (HAL) handle
static inline void HAL_setOffsetTrigger(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t pwmPRD = HAL_getTimeBasePeriod(handle);
    uint16_t offsetUpSoc = pwmPRD*(3.0f/4.0f);
    uint16_t offsetDownSoc = pwmPRD*(1.0f/4.0f);

    //
    // set the SOC trigger point for UP count
    //
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C,
                                offsetUpSoc);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D,
                                offsetUpSoc);

    //
    // set the SOC trigger point for DOWN count
    //
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C,
                                offsetDownSoc);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D,
                                offsetDownSoc);

    return;
} // end of HAL_setOffsetTrigger() function
#endif // MOTOR1_DCLINKSS

#if defined(MOTOR1_ISBLDC)

#define DUAL_PWM_NO

//! \brief     Writes PWM data to the PWM comparators for motor control
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] pPWMData  The pointer to the PWM data
static inline void
HAL_writePWMDataBLDC(HAL_MTR_Handle handle, float32_t pwmDuty, uint16_t pwmState)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    uint16_t period = EPWM_getTimeBasePeriod(obj->pwmHandle[0]);

#ifdef DUAL_PWM
    float32_t posDuty = (pwmDuty * 0.5f) + 0.5f;

    uint16_t negCmpValue = (uint16_t)(((float32_t)period) * posDuty);

    uint16_t posCmpValue = period - negCmpValue;

    uint16_t adcCmpValue = (posCmpValue + negCmpValue)>>1;

    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_B, adcCmpValue);
#else
    float32_t posDuty = pwmDuty;

    uint16_t negCmpValue = (uint16_t)(((float32_t)period) * posDuty);

    uint16_t posCmpValue = period - negCmpValue;

    negCmpValue = period + 1;

    uint16_t adcCmpValue = period - 3;

    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_B, adcCmpValue);
#endif

    switch(pwmState)
    {
        case 0:     // A->B, A-ON, B-ON, C-OFF
            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, period);

#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_A, 1);
            GPIO_writePin(MTR1_DRV_INL_B, 1);
            GPIO_writePin(MTR1_DRV_INL_C, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on A Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on B Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off C Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;

        case 1:     // A->C, A-ON, C-ON, B-OFF
            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, period);

#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_A, 1);
            GPIO_writePin(MTR1_DRV_INL_C, 1);
            GPIO_writePin(MTR1_DRV_INL_B, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on A Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on C Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off B Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;

        case 2:     // B->C, B-ON, C-ON, A-OFF
            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, period);

#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_B, 1);
            GPIO_writePin(MTR1_DRV_INL_C, 1);
            GPIO_writePin(MTR1_DRV_INL_A, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on B Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on C Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off A Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;

        case 3:     // B->A, B-ON, A-ON, C-OFF
            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, period);


#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_B, 1);
            GPIO_writePin(MTR1_DRV_INL_A, 1);
            GPIO_writePin(MTR1_DRV_INL_C, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on B Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on A Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off C Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;

        case 4:     // C->A, C-ON, A-ON, B-OFF
            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, period);

#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_C, 1);
            GPIO_writePin(MTR1_DRV_INL_A, 1);
            GPIO_writePin(MTR1_DRV_INL_B, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on C Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on A Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off B Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;

        case 5:     // C->B, C-ON, B-ON, A-OFF
            // write the PWM data value for C Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                        EPWM_COUNTER_COMPARE_A, posCmpValue);

            // write the PWM data value for B Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                        EPWM_COUNTER_COMPARE_A, negCmpValue);

            // write the PWM data value for A Phase
            EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                        EPWM_COUNTER_COMPARE_A, period);


#if (defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB))
            GPIO_writePin(MTR1_DRV_INL_C, 1);
            GPIO_writePin(MTR1_DRV_INL_B, 1);
            GPIO_writePin(MTR1_DRV_INL_A, 0);
#else   //! BSXL8323RH_REVB && ! BSXL8323RS_REVA
            // Enable PWM output for turning on C Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

            // Enable PWM output for turning on B Phase
            EPWM_clearTripZoneFlag(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);

            // Disable PWM output for turning off A Phase
            EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
#endif  // !BSXL8323RS_REVA && !BSXL8323RH_REVB

            break;
    }

    return;
} // end of HAL_bldcWritePWM() function
#endif  // MOTOR1_ISBLDC


//! \brief     Set trigger point in the middle of the low side pulse
//! \param[in] handle    The hardware abstraction layer (HAL) handle
//! \param[in] deadband     The setting deadband for mosfet gate driver
//! \param[in] noisewindow  The noise window
extern void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us);


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

#endif // end of HAL_H definition

