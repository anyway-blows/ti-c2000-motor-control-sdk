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

#ifndef USER_MTR1_H
#define USER_MTR1_H

//
//! \file   /solutions/fast_uni_lab/common/include/user_mtr1.h
//! \brief  Contains the user related definitions
//!
//

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
//! \addtogroup USER USER
//! @{
//
//*****************************************************************************

//
// the includes

// modules
#include "userParams.h"

#include "user_common.h"


// *****************************************************************************
// the defines

//------------------------------------------------------------------------------
#if defined(BSXL8320RS_REVA) // LaunchPad-F280049
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (57.528f)

//! \brief Defines the analog voltage filter pole location, Hz
//! 100nF->56nF/47nF
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (338.357f)     // 500~800Hz

#if defined(PGA_GAIN_6S)
//! \brief Defines the maximum current at the AD converter
// Gain = 6, Rin=3.3k, Rdac=27.4k
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (88.034f)
#elif defined(PGA_GAIN_12S)
//! \brief Defines the maximum current at the AD converter
// Gain = 12, Rin=2.49k, Rdac=27.4k
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (42.856f)
#endif

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (56.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (54.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.02f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (60.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.05f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (1800.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (10.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of BSXL8320RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8323RS_REVA)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (57.52845691f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (680.4839141f)     // 47nF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (47.14285714f)     // gain=10

//! \brief ADC current offsets for dc-link
// the dc-link offset current for BSXL8323RS_REVA
#define USER_M1_IDC_OFFSET_A            (USER_M1_ADC_FULL_SCALE_CURRENT_A / 2.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IDC_OFFSET_AD           (2048.0f)

#define USER_M1_IDC_OFFSET_AD_MAX       (USER_M1_IDC_OFFSET_AD + 100.0f)
#define USER_M1_IDC_OFFSET_AD_MIN       (USER_M1_IDC_OFFSET_AD - 100.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2030.99646f)
#define USER_M1_IB_OFFSET_AD    (2016.76001f)
#define USER_M1_IC_OFFSET_AD    (2007.99329f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.498977453f)
#define USER_M1_VB_OFFSET_SF    (0.497419506f)
#define USER_M1_VC_OFFSET_SF    (0.500700474f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (54.5f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (52.5f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (8.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (10.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (1500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

// end of BSXL8323RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8323RH_REVB)       // LaunchPad-F280025
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (57.52845691f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (680.4839141f)     // 47nF

//! \brief Defines the maximum current at the AD converter
//! Place a 47k(ohm) resistor (R22) pull-down on MODE pin
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (47.14285714f)     // gain=10

//! \brief ADC current offsets for dc-link
// the dc-link offset current for BSXL8323RH_REVB
#define USER_M1_IDC_OFFSET_A            (USER_M1_ADC_FULL_SCALE_CURRENT_A / 2.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IDC_OFFSET_AD           (2048.0f)

#define USER_M1_IDC_OFFSET_AD_MAX       (USER_M1_IDC_OFFSET_AD + 100.0f)
#define USER_M1_IDC_OFFSET_AD_MIN       (USER_M1_IDC_OFFSET_AD - 100.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2045.40015f)
#define USER_M1_IB_OFFSET_AD    (2012.86694f)
#define USER_M1_IC_OFFSET_AD    (2031.59741f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.507042527f)
#define USER_M1_VB_OFFSET_SF    (0.505379438f)
#define USER_M1_VC_OFFSET_SF    (0.50771445f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (54.5f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (52.5f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (8.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (10.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.01f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (90.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (1800.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of BSXL8323RH_REVB

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)

//------------------------------------------------------------------------------
#elif defined(BSXL8353RS_REVA)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V    (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V    (132.7979508f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz      (338.1100618f)      // 9.76k/47nF

//! \brief Defines the maximum current at the AD converter
//#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (94.28571429f)     // gain=5
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (47.14285714f)     // gain=10

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (40.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (36.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of BSXL8353RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL3PHGAN_REVA)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (81.49905213f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (1103.026917f)     // 33nF

//! \brief Defines the maximum current at the AD converter
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (33.0f)     // gain=20

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (40.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (36.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of BSXL3PHGAN_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8316RT_REVA)
//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (48.0f)

//! \brief Defines the maximum voltage at the AD converter
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (44.28f)

//! \brief Defines the analog voltage filter pole location, Hz
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (284.722f)     // 100nF

//! \brief Defines the maximum current at the AD converter
//!        DRV8316 = VREF = 3V, GAIN = 0.15V/A || 3.3V / 0.15V/A = 22.0A
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (22.0f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2048.0f)
#define USER_M1_IB_OFFSET_AD    (2048.0f)
#define USER_M1_IC_OFFSET_AD    (2048.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.500514159f)
#define USER_M1_VB_OFFSET_SF    (0.506255884f)
#define USER_M1_VC_OFFSET_SF    (0.503381569f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (40.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (36.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (10.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (12.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (50.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of BSXL8316RT_REVA

//------------------------------------------------------------------------------
#elif defined(HVMTRPFC_REV1P1)

//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_M1_NOMINAL_DC_BUS_VOLTAGE_V         (220.0f)

#ifndef LV_JUMPER_EN
//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (409.90f)

//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (375.55f)

#else   // Populate jumpers on J1/J2/J3/J4 for short R20/R23/R26/R37 for low voltage motor
//! \brief Defines the maximum voltage at the AD converter
//  Full scale voltage of AD converter, not the current voltage
#define USER_M1_ADC_FULL_SCALE_VOLTAGE_V         (112.21f)

//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_M1_VOLTAGE_FILTER_POLE_Hz           (381.15f)
#endif

//! \brief Defines the maximum current at the AD converter
//!
#define USER_M1_ADC_FULL_SCALE_CURRENT_A         (19.89f)

//! \brief ADC current offsets for A, B, and C phases
#define USER_M1_IA_OFFSET_AD    (2049.367f)
#define USER_M1_IB_OFFSET_AD    (2042.771f)
#define USER_M1_IC_OFFSET_AD    (2054.451f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF    (0.503290117f)
#define USER_M1_VB_OFFSET_SF    (0.500881076f)
#define USER_M1_VC_OFFSET_SF    (0.497107089f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_FAULT_V        (380.0f)

//! \brief DC bus over voltage threshold
#define USER_M1_OVER_VOLTAGE_NORM_V         (350.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_FAULT_V       (15.0f)

//! \brief DC bus under voltage threshold
#define USER_M1_UNDER_VOLTAGE_NORM_V        (20.0f)

//! \brief motor lost phase current threshold
#define USER_M1_LOST_PHASE_CURRENT_A        (0.2f)

//! \brief motor unbalance ratio percent threshold
#define USER_M1_UNBALANCE_RATIO             (0.2f)

//! \brief motor over load power threshold
#define USER_M1_OVER_LOAD_POWER_W           (250.0f)

//! \brief motor stall current threshold
#define USER_M1_STALL_CURRENT_A             (10.0f)

//! \brief motor fault check current threshold
#define USER_M1_FAULT_CHECK_CURRENT_A       (0.2f)

//! \brief motor failed maximum speed threshold
#define USER_M1_FAIL_SPEED_MAX_HZ           (500.0f)

//! \brief motor failed minimum speed threshold
#define USER_M1_FAIL_SPEED_MIN_HZ           (5.0f)

//! \brief Defines the number of failed torque
//!
#define USER_M1_TORQUE_FAILED_SET           (0.000001f)
// end of HVMTRPFC_REV1P1

//------------------------------------------------------------------------------
#else   // No Board Selection
#error The board parameters are not defined in user_mtr1.h
#endif  // No Board Selection

//------------------------------------------------------------------------------
//! \brief ADC current offsets checking value for A, B, and C phases
#define USER_M1_IA_OFFSET_AD_MAX    (USER_M1_IA_OFFSET_AD + 150.0f)
#define USER_M1_IB_OFFSET_AD_MAX    (USER_M1_IB_OFFSET_AD + 150.0f)
#define USER_M1_IC_OFFSET_AD_MAX    (USER_M1_IC_OFFSET_AD + 150.0f)

#define USER_M1_IA_OFFSET_AD_MIN    (USER_M1_IA_OFFSET_AD - 150.0f)
#define USER_M1_IB_OFFSET_AD_MIN    (USER_M1_IB_OFFSET_AD - 150.0f)
#define USER_M1_IC_OFFSET_AD_MIN    (USER_M1_IC_OFFSET_AD - 150.0f)

//! \brief ADC voltage offsets for A, B, and C phases
#define USER_M1_VA_OFFSET_SF_MAX    (USER_M1_VA_OFFSET_SF + 0.05f)
#define USER_M1_VB_OFFSET_SF_MAX    (USER_M1_VB_OFFSET_SF + 0.05f)
#define USER_M1_VC_OFFSET_SF_MAX    (USER_M1_VC_OFFSET_SF + 0.05f)

#define USER_M1_VA_OFFSET_SF_MIN    (USER_M1_VA_OFFSET_SF - 0.05f)
#define USER_M1_VB_OFFSET_SF_MIN    (USER_M1_VB_OFFSET_SF - 0.05f)
#define USER_M1_VC_OFFSET_SF_MIN    (USER_M1_VC_OFFSET_SF - 0.05f)

//******************************************************************************
//------------------------------------------------------------------------------
//! \brief Vbus used to calculate the voltage offsets A, B, and C
// =0.5*USER_M1_NOMINAL_DC_BUS_VOLTAGE_V
#define USER_M1_VBUS_OFFSET_V  (0.5*USER_M1_ADC_FULL_SCALE_VOLTAGE_V)


//! \brief Defines the maximum negative current to be applied in Id reference
//!
#define USER_M1_MAX_NEGATIVE_ID_REF_CURRENT_A       ((float32_t)(-2.0))


//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_M1_NUM_PWM_TICKS_PER_ISR_TICK          (1)


//! \brief Defines the number of ISR clock ticks per current controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_CURRENT_TICK      (1)


//! \brief Defines the number of ISR clock ticks per speed controller clock tick
//!
#define USER_M1_NUM_ISR_TICKS_PER_SPEED_TICK        (10)


//! \brief Defines the number of current sensors
//!
#define USER_M1_NUM_CURRENT_SENSORS                 (3)


//! \brief Defines the number of voltage sensors
//!
#define USER_M1_NUM_VOLTAGE_SENSORS                 (3)


//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
#define USER_M1_PWM_FREQ_kHz        ((float32_t)(15.0f))
#define USER_M1_PWM_TBPRD_NUM       (uint16_t)(USER_SYSTEM_FREQ_MHz * 1000.0f / USER_M1_PWM_FREQ_kHz / 2.0f)

//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//!
#define USER_M1_PWM_PERIOD_usec     ((float32_t)1000.0/USER_M1_PWM_FREQ_kHz)


//! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
//!
#define USER_M1_ISR_FREQ_Hz         (USER_M1_PWM_FREQ_kHz * (float32_t)1000.0 / (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)

//! \brief Defines the SFRA sampling, Hz
//!
#define MOTOR1_SAMPLING_FREQ_HZ     USER_M1_ISR_FREQ_Hz


//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_M1_ISR_PERIOD_usec     (USER_M1_PWM_PERIOD_usec * (float32_t)USER_M1_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_M1_VD_SF               ((float32_t)(0.95f))


//! \brief Defines the voltage scale factor for the system
//!
#define USER_M1_VOLTAGE_SF          (USER_M1_ADC_FULL_SCALE_VOLTAGE_V / 4096.0f)

//! \brief Defines the current scale factor for the system
//!
#define USER_M1_CURRENT_SF          (USER_M1_ADC_FULL_SCALE_CURRENT_A / 4096.0f)


//! \brief Defines the current scale invert factor for the system
//!
#define USER_M1_CURRENT_INV_SF      (4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A)


//! \brief Defines the analog voltage filter pole location, rad/s
//!
#define USER_M1_VOLTAGE_FILTER_POLE_rps  (MATH_TWO_PI * USER_M1_VOLTAGE_FILTER_POLE_Hz)

//! \brief Defines the maximum Vs magnitude in per units allowed
//! \brief This value sets the maximum magnitude for the output of the Id and
//! \brief Iq PI current controllers. The Id and Iq current controller outputs
//! \brief are Vd and Vq. The relationship between Vs, Vd, and Vq is:
//! \brief Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
//! \brief equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
//! \brief so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
//!
//! \brief Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
//! \brief SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
//! \brief is needed for this scenario.
//!
//! \brief Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
//! \brief with a peak at 100% duty cycle.  Current reconstruction
//! \brief will be needed for this scenario (Lab08).
//!
//! \brief Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
//! \brief voltage waveform.  Current reconstruction will be needed
//! \brief for this scenario (Lab08).
//!
//! \brief For space vector over-modulation, see lab08 for details on
//! \brief system requirements that will allow the SVM generator to
//! \brief go all the way to trapezoidal.
//!
//#define USER_M1_MAX_VS_MAG_PU            (0.66)
//#define USER_M1_MAX_VS_MAG_PU              (0.65)
#define USER_M1_MAX_VS_MAG_PU            (0.576)
//#define USER_M1_MAX_VS_MAG_PU            (0.565)
//#define USER_M1_MAX_VS_MAG_PU            (0.5)


//! \brief Defines the reference Vs magnitude in per units allowed
//! \      Set the value equal from 0.5 to 0.95 of the maximum Vs magnitude
#define USER_M1_VS_REF_MAG_PU             (float32_t)(0.8) * USER_MAX_VS_MAG_PU)

//! \brief Defines the R/L excitation frequency, Hz
//!
#define USER_M1_R_OVER_L_EXC_FREQ_Hz  ((float32_t)(300.0))


//! \brief Defines the R/L Kp scale factor, pu
//! \brief Kp used during R/L is USER_M1_R_OVER_L_KP_SF * USER_M1_NOMINAL_DC_BUS_VOLTAGE_V / USER_MOTOR1_MAX_CURRENT_A;
//!
#define USER_M1_R_OVER_L_KP_SF        ((float32_t)(0.02))


//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/sec
//!
#define USER_M1_MAX_ACCEL_Hzps        ((float32_t)(2.0))


//! \brief Defines the controller execution period, usec
//!
#define USER_M1_CTRL_PERIOD_usec      (USER_M1_ISR_PERIOD_usec)


//! \brief Defines the controller execution period, sec
//!
#define USER_M1_CTRL_PERIOD_sec       ((float32_t)USER_M1_CTRL_PERIOD_usec/(float32_t)1000000.0)


//! \brief Defines the IdRated delta to use during estimation
//!
#define USER_M1_IDRATED_DELTA_A                 ((float32_t)(0.0001))

#if defined(_SOFT_LIB)
//! \brief Defines the forced angle frequency, Hz
#define USER_M1_FORCE_ANGLE_FREQ_Hz             ((float32_t)(1.0))

//! \brief Defines the forced angle acceleration, Hz
#define USER_M1_FORCE_ANGLE_ACCEL_Hzps          ((float32_t)(10.0))
#else  // !_SOFT_LIB
//! \brief Defines the forced angle frequency, Hz
#define USER_M1_FORCE_ANGLE_FREQ_Hz             ((float32_t)(1.0))
#endif  // !_SOFT_LIB

//! \brief Defines the near zero speed limit for electrical frequency estimation, Hz
//!        The flux integrator uses this limit to regulate flux integration
#define USER_M1_FREQ_NEARZEROSPEEDLIMIT_Hz      ((float_t)(0.0f))

//! \brief Defines the fraction of IdRated to use during inductance estimation
//!
#define USER_M1_IDRATED_FRACTION_FOR_L_IDENT    ((float32_t)(0.5))


//! \brief Defines the fraction of SpeedMax to use during inductance estimation
//!
#define USER_M1_SPEEDMAX_FRACTION_FOR_L_IDENT  ((float32_t)(1.0))


//! \brief Defines the Power Warp gain for computing Id reference
//! \brief If motor parameters are known, set this gain to:
//! \brief USER_M1_PW_GAIN = SQRT(1.0 + USER_MOTOR1_Rr_Ohm / USER_MOTOR1_Rs_Ohm)
//!
#define USER_M1_PW_GAIN                        ((float32_t)(1.0))


//! \brief Defines the pole location for the DC bus filter, rad/sec
//!
#define USER_M1_DCBUS_POLE_rps                  ((float32_t)(100.0))


//! \brief Defines the pole location for the voltage and current offset estimation, rad/s
//!
#define USER_M1_OFFSET_POLE_rps                 ((float32_t)(20.0))


//! \brief Defines the pole location for the speed control filter, rad/sec
//!
#define USER_M1_SPEED_POLE_rps                  ((float32_t)(100.0))


//! \brief Defines the pole location for the direction filter, rad/sec
//!
#define USER_M1_DIRECTION_POLE_rps             (MATH_TWO_PI * (float32_t)(10.0))


//! \brief Defines the pole location for the second direction filter, rad/sec
//!
#define USER_M1_DIRECTION_POLE_2_rps           (MATH_TWO_PI * (float32_t)(100.0))


//! \brief Defines the pole location for the flux estimation, rad/sec
//!
#define USER_M1_FLUX_POLE_rps                  ( (float32_t)(10.0))

//! \brief Defines the pole location for the R/L estimation, rad/sec
//!
#define USER_M1_R_OVER_L_POLE_rps              (MATH_TWO_PI * (float32_t)(3.2))

//! \brief Defines the convergence factor for the estimator
//!
#define USER_M1_EST_KAPPAQ                          ((float32_t)(1.5f))


//! \brief Defines the scale factor for the flux estimation
//! the default value is 1.0f, change the value between 0.1f and 1.25f
//!
//#define USER_M1_EST_FLUX_HF_SF                     ((float32_t)(0.120f))
#define USER_M1_EST_FLUX_HF_SF                     ((float32_t)(0.250f))
//#define USER_M1_EST_FLUX_HF_SF                     ((float32_t)(1.00f))

//! \brief Defines the scale factor for the frequency estimation
//! the default value is 1.0f, change the value between 0.5f and 1.5f
//!
#define USER_M1_EST_FREQ_HF_SF                     ((float32_t)(1.00f))

//! \brief Defines the scale factor for the bemf estimation
//! the default value is 1.0f, change the value between 0.50f and 1.25f
//!
#define USER_M1_EST_BEMF_HF_SF                     ((float32_t)(1.00f))

//------------------------------------------------------------------------------
//! brief Define the Kp gain for Field Weakening Control
#define USER_M1_FWC_KP                 0.0225

//! brief Define the Ki gain for Field Weakening Control
#define USER_M1_FWC_KI                 0.00225

//! brief Define the maximum current vector angle for Field Weakening Control
#define USER_M1_FWC_MAX_ANGLE          -10.0f                        // degree
#define USER_M1_FWC_MAX_ANGLE_RAD      USER_M1_FWC_MAX_ANGLE /180.0f * MATH_PI

//! brief Define the minimum current vector angle for Field Weakening Control
#define USER_M1_FWC_MIN_ANGLE          0.0f                          // degree
#define USER_M1_FWC_MIN_ANGLE_RAD      USER_M1_FWC_MIN_ANGLE /180.0f * MATH_PI

//! \brief Defines the number of DC bus over/under voltage setting time
//!  timer base = 5ms
#define USER_M1_VOLTAGE_FAULT_TIME_SET          (500)

//! \brief Defines the number of motor over load setting time
//!  timer base = 5ms, 1s
#define USER_M1_OVER_LOAD_TIME_SET              (200)

//! \brief Defines the number of motor stall setting time
//!  timer base = 5ms, 1s
#define USER_M1_STALL_TIME_SET                  (200)

//! \brief Defines the number of phase unbalance setting time
//!  timer base = 5ms, 5s
#define USER_M1_UNBALANCE_TIME_SET              (1000)

//! \brief Defines the number of lost phase setting time
//!  timer base = 5ms, 10s
#define USER_M1_LOST_PHASE_TIME_SET             (2000)

//! \brief Defines the number of over speed setting time
//!  timer base = 5ms
#define USER_M1_OVER_SPEED_TIME_SET             (600)

//! \brief Defines the number of startup failed setting time
//!  timer base = 5ms, 10s
#define USER_M1_STARTUP_FAIL_TIME_SET           (2000)

//! \brief Defines the number of over load setting times
//!
#define USER_M1_OVER_CURRENT_TIMES_SET          (5)

//! \brief Defines the number of stop wait time
//!  timer base = 5ms, 10s
#define USER_M1_STOP_WAIT_TIME_SET              (2000)

//! \brief Defines the number of restart wait time
//!  timer base = 5ms, 10s
#define USER_M1_RESTART_WAIT_TIME_SET           (2000)

//! \brief Defines the number of restart time
//!
#define USER_M1_START_TIMES_SET                 (3)

//! \brief Defines the alignment time
//!
#define USER_M1_ALIGN_TIMES_SET                 (2000)     // ctrl period

//!
//!
#define USER_M1_QEP_UNIT_TIMER_TICKS            (uint32_t)(USER_SYSTEM_FREQ_MHz/(2.0f * USER_M1_ISR_FREQ_Hz) * 1000000.0f)

//==============================================================================
// Motor defines

//#define USER_MOTOR1 Estun_EMJ_04APB22_A           //*Tested, FAST/eSMO/ENC
//#define USER_MOTOR1 Estun_EMJ_04APB22_B
//#define USER_MOTOR1 Regal_Beloit_5SME39DL0756
//#define USER_MOTOR1 Anaheim_BLWS235D              //*Tested, FAST/Hall
//#define USER_MOTOR1 Anaheim_BLY341S_48V
//#define USER_MOTOR1 Anaheim_BLY341S_Y24V
//#define USER_MOTOR1 Anaheim_BLY341S_D24V
//#define USER_MOTOR1 tekin_redline_4600KV
//#define USER_MOTOR1 low_voltage_ceiling_fan

// ACI Motor
//#define USER_MOTOR1 Marathon_5K33GN2A
//#define USER_MOTOR1 Marathon_56H17T2011A
//#define USER_MOTOR1 Dayton_3N352C
//#define USER_MOTOR1 EMSYNERGY_LVACI

//#define USER_MOTOR1 Anaheim_BLY172S_24V           //*Tested, FAST/HALL
#define USER_MOTOR1 Teknic_M2310PLN04K              //*Tested, FAST/eSMO/ENC/HALL
//#define USER_MOTOR1 Nedic_EPSMS037_D12V
//#define USER_MOTOR1 Drone_BLK2BLADE               //*Tested
//#define USER_MOTOR1 Drone_SF_Black                //*Tested

//#define USER_MOTOR1 my_pm_motor_1
//#define USER_MOTOR1 my_aci_motor_2

//-----------------------------------------------------------------------------
#if (USER_MOTOR1 == Estun_EMJ_04APB22_A)
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.62655902f)
#define USER_MOTOR1_Ls_d_H                  (0.00860825367f)
#define USER_MOTOR1_Ls_q_H                  (0.00860825367f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.377903223f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (2.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_RATED_SPEED_KRPM        (3.0f)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_HZ             (400.0f)        // Hz

#define USER_MOTOR1_FREQ_LOW_HZ             (10.0f)         // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ            (200.0f)        // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)         // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)        // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)         // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)          // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)          // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (1.0f)          // A
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)          // A
#define USER_MOTOR1_OVER_CURRENT_A          (2.0f)          // A

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0f)         // Hz/s
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0f)        // Hz/s

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)          // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (2.0f)           // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)           // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)         // 100.0f

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

#elif (USER_MOTOR1 == Estun_EMJ_04APB22_B)
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0f)
#define USER_MOTOR1_Rs_Ohm                  (2.98774099f)
#define USER_MOTOR1_Ls_d_H                  (0.008926632f)
#define USER_MOTOR1_Ls_q_H                  (0.008926632f)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.445965141f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A           (3.82f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (20.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS           (2500.0)
#define USER_MOTOR1_INERTIA_Kgm2            (3.100017e-5)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (200.0f)
#define USER_MOTOR1_RATED_SPEED_KRPM        (3.0f)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0f)            // Hz
#define USER_MOTOR1_FREQ_MAX_HZ             (400.0f)          // Hz

#define USER_MOTOR1_FREQ_LOW_HZ             (10.0f)           // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ            (200.0f)          // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V              (200.0f)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05f)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (2.0f)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (7.5f)            //

#define USER_MOTOR1_SPEED_START_Hz          (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0f)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0f)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Teknic_M2310PLN04K)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.38157931f)
#define USER_MOTOR1_Ls_d_H                 (0.000188295482f)
#define USER_MOTOR1_Ls_q_H                 (0.000188295482f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0396642499f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0f)
#define USER_MOTOR1_MAX_CURRENT_A          (6.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0f)

#define USER_MOTOR1_FREQ_MIN_HZ            (9.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (600.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (5.0f)            // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0f)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)            // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)        // 60s/5ms

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (25.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.75f)      //
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (6.75f)      //
#define USER_MOTOR1_PLL_KP_MIN             (0.75f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (3.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (5)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Anaheim_BLY172S_24V)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.384005845f)
#define USER_MOTOR1_Ls_d_H                 (0.000639994687f)
#define USER_MOTOR1_Ls_q_H                 (0.000639994687f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0327013217f)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (4.5f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (4.80185e-06)

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0f)

#define USER_MOTOR1_FREQ_MIN_HZ            (9.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (600.0f)        // Hz
#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (5.0f)           // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0f)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0f)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (0.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (5.4f)           // A

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (15.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

//------------------------------------------------------------------------------
#elif (USER_MOTOR1 == Drone_BLK2BLADE)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (6)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0614009798f)
#define USER_MOTOR1_Ls_d_H                 (1.29998243e-05f)
#define USER_MOTOR1_Ls_q_H                 (1.29998243e-05f)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.00359785813f)

#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
//! \brief Defines the maximum current at the AD converter
// Gain = 12, Rin=2.49k, Rdac=27.4k
#define USER_MOTOR1_MAX_CURRENT_A          (20.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (120.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (3.06154e-04)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (2000.0)        // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (2.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
//! \brief Defines the maximum current at the AD converter
// Gain = 12, Rin=2.49k, Rdac=27.4k
#define USER_MOTOR1_OVER_CURRENT_A         (22.25)          //

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.55f)
#define USER_MOTOR1_KSLIDE_MIN             (0.10f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.75f)
#define USER_MOTOR1_PLL_KP_SF              (20.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

#elif (USER_MOTOR1 == Drone_SF_Black)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (7)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0276225284)
#define USER_MOTOR1_Ls_d_H                 (1.25800107e-05)
#define USER_MOTOR1_Ls_q_H                 (1.25800107e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.02469966)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (6.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A          (25.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (80.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-05)

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (2000.0)        // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (25.0)          //

#define USER_MOTOR1_SPEED_START_Hz         (20.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

#elif (USER_MOTOR1 == Nedic_EPSMS037_D12V)
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (4)
#define USER_MOTOR1_Rr_Ohm                  (0.0)
#define USER_MOTOR1_Rs_Ohm                  (0.0412785485)
#define USER_MOTOR1_Ls_d_H                  (0.000121561985)
#define USER_MOTOR1_Ls_q_H                  (0.000121561985)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.0572981499)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (5.0)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-4.0)
#define USER_MOTOR1_MAX_CURRENT_A           (12.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS           (1000.0)
#define USER_MOTOR1_INERTIA_Kgm2            (0.015)

#define USER_MOTOR1_RATED_VOLTAGE_V         (12.0)
#define USER_MOTOR1_RATED_SPEED_KRPM        (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ              (5.0)            // Hz
#define USER_MOTOR1_FREQ_MAX_HZ             (1500.0)          // Hz

#define USER_MOTOR1_FREQ_LOW_HZ             (10.0)           // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ            (1000.0)          // Hz
#define USER_MOTOR1_VOLT_MIN_V              (2.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V              (12.0)          // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A       (2.0)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (1.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (12.5)            //

#define USER_MOTOR1_SPEED_START_Hz          (10.0)           //
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

#elif (USER_MOTOR1 == Anaheim_BLWS235D)
#define USER_MOTOR1_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS          (2)
#define USER_MOTOR1_Rr_Ohm                  (NULL)

#define USER_MOTOR1_Rs_Ohm                  (5.44400978)
#define USER_MOTOR1_Ls_d_H                  (0.0224932302)
#define USER_MOTOR1_Ls_q_H                  (0.0224932302)
#define USER_MOTOR1_RATED_FLUX_VpHz         (0.560590863)      // 4.11V/krpm

#define USER_MOTOR1_MAGNETIZING_CURRENT_A   (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A       (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A       (-1.0)
#define USER_MOTOR1_MAX_CURRENT_A           (2.5)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz        (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS           (NULL)
#define USER_MOTOR1_INERTIA_Kgm2            (0.0000230206204) // 0.003261 oz-in-sec2

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz   (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V         (160.0)
#define USER_MOTOR1_RATED_SPEED_KRPM        (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_HZ             (200.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_HZ             (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ            (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V              (20.0)            // Volt
#define USER_MOTOR1_VOLT_MAX_V              (160.0)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A           (0.05)         // A
#define USER_MOTOR1_ALIGN_DELTA_A           (0.01)          // A
#define USER_MOTOR1_FLUX_CURRENT_A          (0.5)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A         (1.0)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A       (1.0)            //
#define USER_MOTOR1_TORQUE_CURRENT_A        (0.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A          (2.0)            //

#define USER_MOTOR1_SPEED_START_Hz          (30.0)
#define USER_MOTOR1_SPEED_FORCE_Hz          (20.0)
#define USER_MOTOR1_ACCEL_START_Hzps        (10.0)           //
#define USER_MOTOR1_ACCEL_MAX_Hzps          (20.0)          //

#define USER_MOTOR1_SPEED_FS_Hz             (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)       // 2.0f
#define USER_MOTOR1_KSLIDE_MIN             (0.75f)

#define USER_MOTOR1_PLL_KP_MAX             (10.0f)
#define USER_MOTOR1_PLL_KP_MIN             (2.0f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (2.0f)       // 1.0f
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)       // 2.5f
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)     // 100.0f

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

#elif (USER_MOTOR1 == Regal_Beloit_5SME39DL0756)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (3)
#define USER_MOTOR1_Rr_Ohm                 (0.0)
#define USER_MOTOR1_Rs_Ohm                 (4.581007)
#define USER_MOTOR1_Ls_d_H                 (0.03727356)
#define USER_MOTOR1_Ls_q_H                 (0.03727356)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.6589699)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR1_MAX_CURRENT_A          (2.6)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR1 == Anaheim_BLY341S_48V)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.463800967)
#define USER_MOTOR1_Ls_d_H                 (0.00114538975)
#define USER_MOTOR1_Ls_q_H                 (0.00114538975)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0978558362)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-2.0)
#define USER_MOTOR1_MAX_CURRENT_A          (20.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_MOTOR1 == Anaheim_BLY341S_Y24V)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.133211181)
#define USER_MOTOR1_Ls_d_H                 (0.000275031634)
#define USER_MOTOR1_Ls_q_H                 (0.000275031634)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0514687076)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-4.0)
#define USER_MOTOR1_MAX_CURRENT_A          (12.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (3.99683e-05)     // 0.00566 oz-in-sec2

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (300.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)             // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)            // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)           //
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.0)           //

#define USER_MOTOR1_SPEED_START_Hz         (10.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

#elif (USER_MOTOR1 == Anaheim_BLY341S_D24V)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0544644073)
#define USER_MOTOR1_Ls_d_H                 (9.58044038e-05)
#define USER_MOTOR1_Ls_q_H                 (9.58044038e-05)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0544644073)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (4.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.5)
#define USER_MOTOR1_MAX_CURRENT_A          (8.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (3.99683e-05)     // 0.00566 oz-in-sec2

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)             // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (300.0)           // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)            // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)           // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)             // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)            // Volt

#define USER_MOTOR1_STARTUP_CURRENT_A      (3.0)            //
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.5)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (8.5)           //

#define USER_MOTOR1_SPEED_START_Hz         (10.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

#elif (USER_MOTOR1 == tekin_redline_4600KV)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.0181193)
#define USER_MOTOR1_Ls_d_H                 (8.180002e-06)
#define USER_MOTOR1_Ls_q_H                 (8.180002e-06)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.0041173688)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (3.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR1_MAX_CURRENT_A          (5.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (60.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR1 == low_voltage_ceiling_fan)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (8)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (0.3974416)
#define USER_MOTOR1_Ls_d_H                 (0.0001718943)
#define USER_MOTOR1_Ls_q_H                 (0.0001718943)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.282237479574)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR1_MAX_CURRENT_A          (3.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR1 == EMSYNERGY_LVACI)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (1.05)
#define USER_MOTOR1_Rs_Ohm                 (1.79)
#define USER_MOTOR1_Ls_d_H                 (0.00681)
#define USER_MOTOR1_Ls_q_H                 (0.00681)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*14.7/50.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (1.15)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (2.50)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz


#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (1.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (1.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (5.0)           // A

#define USER_MOTOR1_SPEED_START_Hz         (30.0)
#define USER_MOTOR1_SPEED_FORCE_Hz         (25.0)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0)

// only for encoder, N/A
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO, N/A
#define USER_MOTOR1_KSLIDE_MAX             (1.50f)
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (7.25f)
#define USER_MOTOR1_PLL_KP_MIN             (1.25f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (2.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for IS-BLDC, N/A
#define USER_MOTOR1_RAMP_START_Hz           (5.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (1)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base


#elif (USER_MOTOR1 == Marathon_5K33GN2A)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (5.508003)
#define USER_MOTOR1_Rs_Ohm                 (10.71121)
#define USER_MOTOR1_Ls_d_H                 (0.05296588)
#define USER_MOTOR1_Ls_q_H                 (0.05296588)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*220.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (1.378)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (3.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

#elif (USER_MOTOR1 == Marathon_56H17T2011A)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (5.159403)
#define USER_MOTOR1_Rs_Ohm                 (7.924815)
#define USER_MOTOR1_Ls_d_H                 (0.03904648)
#define USER_MOTOR1_Ls_q_H                 (0.03904648)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8*0.8165*230.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (0.9941965)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (2.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR1_VOLT_MAX_V             (200.0)         // Volt

#elif (USER_MOTOR1 == Dayton_3N352C)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (2.428799)
#define USER_MOTOR1_Rs_Ohm                 (2.863202)
#define USER_MOTOR1_Ls_d_H                 (0.02391323)
#define USER_MOTOR1_Ls_q_H                 (0.02391323)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (3.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR1_NUM_ENC_SLOTS          (NULL)

//------------------------------------------------------------------
#elif (USER_MOTOR1 == my_pm_motor_1)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR1_NUM_POLE_PAIRS         (4)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (NULL)
#define USER_MOTOR1_Ls_d_H                 (NULL)
#define USER_MOTOR1_Ls_q_H                 (NULL)
#define USER_MOTOR1_RATED_FLUX_VpHz        (NULL)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (2.0f)
#define USER_MOTOR1_IND_EST_CURRENT_A      (-1.5f)
#define USER_MOTOR1_MAX_CURRENT_A          (6.0f)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (40.0f)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR1_INERTIA_Kgm2           (7.06154e-06)

#define USER_MOTOR1_FREQ_NEARZEROLIMIT_Hz  (5.0f)          // Hz

#define USER_MOTOR1_RATED_VOLTAGE_V        (24.0f)
#define USER_MOTOR1_RATED_SPEED_KRPM       (3.0f)

#define USER_MOTOR1_FREQ_MIN_HZ            (9.0f)          // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (600.0f)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (5.0f)            // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0f)          // Hz
#define USER_MOTOR1_VOLT_MIN_V             (1.0f)            // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0f)           // Volt

#define USER_MOTOR1_FORCE_DELTA_A          (0.05f)          // A
#define USER_MOTOR1_ALIGN_DELTA_A          (0.01f)          // A
#define USER_MOTOR1_FLUX_CURRENT_A         (0.5f)           // A
#define USER_MOTOR1_ALIGN_CURRENT_A        (1.5f)           // A
#define USER_MOTOR1_STARTUP_CURRENT_A      (3.5f)           // A
#define USER_MOTOR1_TORQUE_CURRENT_A       (2.0f)           // A
#define USER_MOTOR1_OVER_CURRENT_A         (6.5f)           // A

#define USER_MOTOR1_BRAKE_CURRENT_A        (1.0f)           // A
#define USER_MOTOR1_BRAKE_TIME_DELAY       (12000U)        // 60s/5ms

#define USER_MOTOR1_SPEED_START_Hz         (30.0f)
#define USER_MOTOR1_SPEED_FORCE_Hz         (25.0f)
#define USER_MOTOR1_ACCEL_START_Hzps       (10.0f)
#define USER_MOTOR1_ACCEL_MAX_Hzps         (20.0f)

#define USER_MOTOR1_SPEED_FS_Hz            (3.0f)

// only for encoder
#define USER_MOTOR1_ENC_POS_MAX            (USER_MOTOR1_NUM_ENC_SLOTS * 4 - 1)
#define USER_MOTOR1_ENC_POS_OFFSET         (668)

// Only for eSMO
#define USER_MOTOR1_KSLIDE_MAX             (0.75f)      //
#define USER_MOTOR1_KSLIDE_MIN             (0.15f)

#define USER_MOTOR1_PLL_KP_MAX             (6.75f)      //
#define USER_MOTOR1_PLL_KP_MIN             (0.75f)
#define USER_MOTOR1_PLL_KP_SF              (5.0f)

#define USER_MOTOR1_BEMF_THRESHOLD         (0.5f)
#define USER_MOTOR1_BEMF_KSLF_FC_Hz        (1.0f)
#define USER_MOTOR1_THETA_OFFSET_SF        (1.0f)
#define USER_MOTOR1_SPEED_LPF_FC_Hz        (200.0f)

// for IS-BLDC
#define USER_MOTOR1_RAMP_START_Hz           (3.0f)
#define USER_MOTOR1_RAMP_END_Hz             (30.0f)
#define USER_MOTOR1_RAMP_DELAY              (5)

#define USER_MOTOR1_ISBLDC_INT_MAX          (0.015f)
#define USER_MOTOR1_ISBLDC_INT_MIN          (0.010f)

// for Rs online calibration
#define USER_MOTOR1_RSONLINE_WAIT_TIME      (60000U)    // 5min/300s at 5ms base
#define USER_MOTOR1_RSONLINE_WORK_TIME      (24000U)     //2min/120s at 5ms base

#elif (USER_MOTOR1 == my_aci_motor_2)
#define USER_MOTOR1_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR1_NUM_POLE_PAIRS         (2)
#define USER_MOTOR1_Rr_Ohm                 (NULL)
#define USER_MOTOR1_Rs_Ohm                 (NULL)
#define USER_MOTOR1_Ls_d_H                 (NULL)
#define USER_MOTOR1_Ls_q_H                 (NULL)
#define USER_MOTOR1_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_MOTOR1_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR1_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR1_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR1_MAX_CURRENT_A          (5.0)
#define USER_MOTOR1_FLUX_EXC_FREQ_Hz       (5.0)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR1_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR1_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR1_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR1_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR1_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR1_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR1_VOLT_MAX_V             (24.0)          // Volt

#else
#error No motor type specified
#endif


//! \brief Defines the maximum current slope for Id trajectory
//!
#define USER_M1_MAX_CURRENT_DELTA_A        (USER_MOTOR1_RES_EST_CURRENT_A / USER_M1_ISR_FREQ_Hz)


//! \brief Defines the maximum current slope for Id trajectory during power warp mode
//!
#define USER_M1_MAX_CURRENT_DELTA_PW_A    (0.3 * USER_MOTOR1_RES_EST_CURRENT_A / USER_M1_ISR_FREQ_Hz)


#ifndef USER_MOTOR1
#error Motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_TYPE
#error The motor type is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rr_Ohm
#error The rotor resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Rs_Ohm
#error The stator resistance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_d_H
#error The direct stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_Ls_q_H
#error The quadrature stator inductance is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RATED_FLUX_VpHz
#error The rated flux of motor is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAGNETIZING_CURRENT_A
#error The magnetizing current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_RES_EST_CURRENT_A
#error The resistance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_IND_EST_CURRENT_A
#error The inductance estimation current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_MAX_CURRENT_A
#error The maximum current is not defined in user_mtr1.h
#endif

#ifndef USER_MOTOR1_FLUX_EXC_FREQ_Hz
#error The flux excitation frequency is not defined in user_mtr1.h
#endif

#if ((USER_M1_NUM_CURRENT_SENSORS < 2) || (USER_M1_NUM_CURRENT_SENSORS > 3))
#error The number of current sensors must be 2 or 3
#endif

#if (USER_M1_NUM_VOLTAGE_SENSORS != 3)
#error The number of voltage sensors must be 3
#endif

// **************************************************************************
// the Defines
#define USER_M1_POT_ADC_MIN                 (200U)

#define USER_M1_POT_SPEED_SF                USER_MOTOR1_FREQ_MAX_HZ / ((float32_t)(4096.0f - USER_M1_POT_ADC_MIN))

//! \brief Defines the minimum frequency for pot
#define USER_M1_POT_SPEED_MIN_Hz            (USER_MOTOR1_FREQ_MAX_HZ * 0.1f)

//! \brief Defines the maximum frequency input
#define USER_M1_POT_SPEED_MAX_Hz            (USER_MOTOR1_FREQ_MAX_HZ * 0.5f)

//! \brief Defines the pot signal wait delay time
#define USER_M1_WAIT_TIME_SET               (500U)         // 0.5s/1ms

//! \brief Defines the minimum frequency for pulse input
#define USER_M1_SPEED_CAP_MIN_Hz            (20.0f)

//! \brief Defines the maximum frequency for pulse input
#define USER_M1_SPEED_CAP_MAX_Hz            (600.0f)

//! \brief Defines the pulse capture wait delay time
#define USER_M1_CAP_WAIT_TIME_SET           (200U)     // 0.2s/1ms

//! \brief Defines the switch signal wait delay time
#define USER_M1_SWITCH_WAIT_TIME_SET        (50U)      // 0.05s/1ms

// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the functions
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setMotor1Params(USER_Params *pUserParams);

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

#endif // end of USER_MTR1_H definition

