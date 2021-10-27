//#############################################################################
//
// FILE:    motor_drive_user.h
//
// TITLE:   motor parameters definition
//
// Group:   C2000
//
// Target Family: F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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
//
//! \file  solutions/multi_axis_drive/f28004x/include/motor_drive_user.h
//! \brief header file to be included in all labs
//!
//

#ifndef MOTOR_DRIVE_USER_H
#define MOTOR_DRIVE_USER_H

//
// Include project specific include files.
//
#include "motor_drive_settings.h"

//
// Analog scaling with ADC
//
#define ADC_RESOLUTION              4096.0F             // 2^12, 12bits
#define ADC_PU_SCALE_FACTOR         0.000244140625F     // 1/2^12, 12bits ADC
#define ADC_PU_PPB_SCALE_FACTOR     0.000488281250F     // 1/2^11, 12bits ADC

//
// ADC and PWM Related defines for M1
//
#define M_IV_ADC_BASE         ADCC_BASE           //C0*/A11, Set up based board
#define M_IW_ADC_BASE         ADCA_BASE           //A12*/C1, Set up based board
#define M_IU_ADC_BASE         ADCC_BASE           //C4*/A14, Set up based board
#define M_VDC_ADC_BASE        ADCA_BASE           //A6,      Set up based board

#define M_IV_ADCRESULT_BASE   ADCCRESULT_BASE     //C0,  Set up based board
#define M_IW_ADCRESULT_BASE   ADCARESULT_BASE     //A12, Set up based board
#define M_IU_ADCRESULT_BASE   ADCCRESULT_BASE     //C4,  Set up based board
#define M_VDC_ADCRESULT_BASE  ADCARESULT_BASE     //A6,  Set up based board

#define M_IV_ADC_CH_NUM       ADC_CH_ADCIN0       //C0,  Set up based board
#define M_IW_ADC_CH_NUM       ADC_CH_ADCIN12      //A12, Set up based board
#define M_IU_ADC_CH_NUM       ADC_CH_ADCIN4       //C4,  Set up based board
#define M_VDC_ADC_CH_NUM      ADC_CH_ADCIN6       //A6,  Set up based board

#define M_IV_ADC_SOC_NUM      ADC_SOC_NUMBER0     //C0,  Set up based board
#define M_IW_ADC_SOC_NUM      ADC_SOC_NUMBER0     //A12, Set up based board
#define M_IU_ADC_SOC_NUM      ADC_SOC_NUMBER1     //C4,  Set up based board
#define M_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER1     //A6,  Set up based board

#define M_IV_ADC_PPB_NUM      ADC_PPB_NUMBER1     //C0,  Set up based board
#define M_IW_ADC_PPB_NUM      ADC_PPB_NUMBER1     //A12, Set up based board
#define M_IU_ADC_PPB_NUM      ADC_PPB_NUMBER2     //C4,  Set up based board
#define M_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER2     //A6,  Set up based board

#define M_U_CMPSS_BASE        CMPSS3_BASE         //C4,  Set up based board
#define M_V_CMPSS_BASE        CMPSS1_BASE         //C0,  Set up based board
#define M_W_CMPSS_BASE        CMPSS4_BASE         //A12, Set up based board

#define M_IFB_V      ADC_readResult(M_IV_ADCRESULT_BASE, M_IV_ADC_SOC_NUM)
#define M_IFB_W      ADC_readResult(M_IW_ADCRESULT_BASE, M_IW_ADC_SOC_NUM)
#define M_IFB_U      ADC_readResult(M_IU_ADCRESULT_BASE, M_IU_ADC_SOC_NUM)
#define M_VDC        ADC_readResult(M_VDC_ADCRESULT_BASE, M_VDC_ADC_SOC_NUM)

#define M_IFB_V_PPB  ADC_readPPBResult(M_IV_ADCRESULT_BASE, M_IV_ADC_PPB_NUM)
#define M_IFB_W_PPB  ADC_readPPBResult(M_IW_ADCRESULT_BASE, M_IW_ADC_PPB_NUM)
#define M_IFB_U_PPB  ADC_readPPBResult(M_IU_ADCRESULT_BASE, M_IU_ADC_PPB_NUM)
#define M_VDC_PPB    ADC_readPPBResult(M_VDC_ADCRESULT_BASE, M_VDC_ADC_PPB_NUM)

#define M_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // Set up based board
#define M_ADC_SAMPLE_WINDOW   20                      // Set up based board

#define M_U_PWM_BASE          EPWM1_BASE          // Set up based board
#define M_V_PWM_BASE          EPWM2_BASE          // Set up based board
#define M_W_PWM_BASE          EPWM3_BASE          // Set up based board

#define M_PWM_SYNC_BASE       EPWM5_BASE          // Set up based board

#define M_SYNC_NUM            0
#define M_SYNT_NUM            3

#define M_PWM_SNYC_SOC        FSI_EXT_TRIGSRC_EPWM1_SOCB

#define M_INT_PWM             INT_EPWM1           // Set up based board
#define M_INT_ADC_NUM         ADC_INT_NUMBER1     // Set up based board

#define M_QEP_BASE            EQEP1_BASE          // Set up based board

#define M_SPI_BASE            SPIA_BASE           // Set up based board

//! \brief Defines the gpio for enabling Power Module
#define M_EN_GATE_GPIO        23                   // Set up based board

//! \brief Defines the gpio for the nFAULT of Power Module
#define M_nFAULT_GPIO         42                   // Set up based board

//! \brief Defines the gpio for the nFAULT of XBAR INPUT
#define M_XBAR_INPUT_GPIO     42                   // Set up based board

//! \brief Defines the gpio for the DRV SPI_SCS
#define M_SPI_SCS_GPIO        5                    // Set up based board

//! \brief Defines the gpio for the clear fault
#define M_CLR_FAULT_GPIO      5                    //

//------------------------------------------------------------------------------
// interrupt
#define M_PIE_INT_NUM      INT_EPWM1                   // EPWM1_INT
#define M_CPU_INT_NUM      INTERRUPT_CPU_INT3          // EPWM1_INT-CPU_INT3
#define M_INT_ACK_GROUP    INTERRUPT_ACK_GROUP3        // EPWM1_INT-CPU_INT3

//------------------------------------------------------------------------------
// CMPSS
#define M_IU_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C4
#define M_IU_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C4

#define M_IV_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-C0
#define M_IV_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-C0

#define M_IW_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A12, N/A
#define M_IW_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A12

#define M_IU_CMPHP_MUX     4                            // CMPSS3-C4
#define M_IU_CMPLP_MUX     4                            // CMPSS3-C4, N/A

#define M_IV_CMPHP_MUX     1                            // CMPSS1-C0
#define M_IV_CMPLP_MUX     1                            // CMPSS1-C0

#define M_IW_CMPHP_MUX     2                            // CMPSS4-A12, N/A
#define M_IW_CMPLP_MUX     2                            // CMPSS4-A12

//------------------------------------------------------------------------------
// XBAR-EPWM
#define M_XBAR_TRIP_ADDRL      XBAR_O_TRIP4MUX0TO15CFG
#define M_XBAR_TRIP_ADDRH      XBAR_O_TRIP4MUX16TO31CFG

#define M_IU_XBAR_EPWM_MUX     XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L   // CMPSS3_HP&LP
#define M_IV_XBAR_EPWM_MUX     XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L   // CMPSS1_HP&LP
#define M_IW_XBAR_EPWM_MUX     XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L   // CMPSS4_HP&LP

//! \brief Defines the XBAR EPWM INPUT link to nFAULT
#define M_INPUT_XBAR_EPWM_MUX  XBAR_EPWM_MUX01_INPUTXBAR1

#define M_IU_XBAR_MUX          XBAR_MUX04                  // CMPSS3_HP&LP
#define M_IV_XBAR_MUX          XBAR_MUX00                  // CMPSS1_HP&LP
#define M_IW_XBAR_MUX          XBAR_MUX06                  // CMPSS4_HP&LP
#define M_EPWM_XBAR_MUX        XBAR_MUX01                  // INPUTXBAR1

//! \brief Defines the XBAR TRIP link to nFAULT
#define M_XBAR_TRIP_NUM        XBAR_TRIP4

//! \brief Defines the EPWM DC TRIP link to CMPSSs
#define M_DC_TRIP_NUM          EPWM_DC_COMBINATIONAL_TRIPIN4

//! \brief Defines the XBAR INPUT link to nFAULT
#define M_XBAR_INPUT_NUM       XBAR_INPUT3

//! \brief Defines the TZTRIP link to nFAULT
#define M_TZ_OSHT_NUM          EPWM_TZ_SIGNAL_OSHT3

//
// Define the base quantites
//
#define M_ADC_SCALE_VOLTAGE     81.50
                                        // Maximum sensing voltage range of ADC
#define M_BASE_VOLTAGE          M_ADC_SCALE_VOLTAGE/1.732050808
                                       // Base peak phase voltage (volt),
                                       // maximum measurable Vdc/sqrt(3)
#define M_BASE_SHUNT_CURRENT    16.5
                                       // Base peak phase current (amp),
                                       // maximum measurable peak curr.
#define M_BASE_LEM_CURRENT      12.0
                                       // Base peak phase current (amp),
                                       // maximum measurable peak current
#define M_BASE_CURRENT          M_BASE_SHUNT_CURRENT

#define M_MAXIMUM_CURRENT       6.0    // Motor maximum torque current (amp)

#define M_MAXIMUM_VOLTAGE       36.0   // DC bus maximum voltage (V)

#define M_MINIMUM_VOLTAGE       5.0    // DC bus minimum voltage (V)

//
// Motor Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M_PWM_FREQUENCY           (float32_t)(10.0)   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M_ISR_FREQUENCY           (float32_t)(M_PWM_FREQUENCY)
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M_ISR_FREQUENCY           (float32_t)(2*M_PWM_FREQUENCY)
#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M_INV_PWM_TICKS         ((SYSTEM_FREQUENCY / M_PWM_FREQUENCY) * 1000)
#define M_INV_PWM_DB            (200.0)
#define M_QEP_UNIT_TIMER_TICKS  (SYSTEM_FREQUENCY/(2*M_PWM_FREQUENCY) * 1000)

#define M_INV_PWM_TBPRD         (M_INV_PWM_TICKS / 2)
#define M_INV_PWM_HALF_TBPRD    (M_INV_PWM_TBPRD / 2)
#define M_SAMPLING_FREQ         (M_ISR_FREQUENCY * 1000)
#define M_CUR_LOOP_BANDWIDTH    (2.0F * PI * M_SAMPLING_FREQ / 18)

#define M_TPWM_CARRIER          (1000.0 / M_PWM_FREQUENCY)    //in uSec

//
// FCL Computation time predetermined from library
//
#define M_FCL_COMPUTATION_TIME  (1.00)  //in uS

//
// Define the electrical motor parameters (Estun Servomotor)
//
#if(USER_MOTOR == ESTUN_EMJ04APB222)
#define M_RS            2.35            // Stator resistance (ohm)
#define M_RR            NULL            // Rotor resistance (ohm)
#define M_LS            0.0065          // Stator inductance (H)
#define M_LD            M_LS            // Stator d-axis inductance (H)
#define M_LQ            M_LS            // Stator q-axis inductance (H)
#define M_LR            NULL            // Rotor inductance (H)
#define M_LM            NULL            // Magnetizing inductance (H)
#define M_POLES         8               // Number of poles
#define M_ENC_SLOTS     2500            // Numer of slots in the encoder

#define M_SPEED_REF     0.05            // reference speed (pu)
#define M_ID_START      0.1             // alignment reference d-axis current
#define M_ID_RUN        0.0             // alignment reference d-axis current
#define M_IQ_LEVEL5     0.10            // reference q-axis current for level5
#define M_IQ_NO_LEVEL5  0.10            // ref q-axis current for no level5

#define M_VD_TEST       0.0             // reference d-axis voltage for level2
#define M_VQ_TEST       0.10            // reference q-axis voltage for level2

#define M_VDCBUS_MAX    360.0           // maximum dc bus voltage for motor
#define M_VDCBUS_MIN    20.0            // minimum dc bus voltage for motor

//
// Define the electrical motor parameters (Teknic Servomotor)
//
#elif(USER_MOTOR == TEKNIC_2310PLN04K)
//
// Define the electrical motor parameters
//
#define M_RS            0.381334811     // Stator resistance (ohm)
#define M_RR            NULL            // Rotor resistance (ohm)
#define M_LS            0.000169791776  // Stator inductance (H)
#define M_LD            M_LS            // Stator d-axis inductance (H)
#define M_LQ            M_LS            // Stator q-axis inductance (H)
#define M_LR            NULL            // Rotor inductance (H)
#define M_LM            NULL            // Magnetizing inductance (H)
#define M_KB            0.039           // BEMF Constant (V/Hz)
#define M_POLES         8               // Number of poles
#define M_ENC_SLOTS     1000            // Numer of slots in the encoder

#define M_SPEED_REF     0.20            // reference speed (pu)
#define M_ID_START      0.3             // alignment reference d-axis current
#define M_ID_RUN        0.0             // alignment reference d-axis current
#define M_IQ_LEVEL5     0.10            // reference q-axis current for level5
#define M_IQ_NO_LEVEL5  0.10            // ref q-axis current for no level5

#define M_VD_TEST       0.0             // reference d-axis voltage for level2
#define M_VQ_TEST       0.20            // reference q-axis voltage for level2

#define M_VDCBUS_MAX    50.0            // maximum dc bus voltage for motor
#define M_VDCBUS_MIN    10.0            // minimum dc bus voltage for motor

#else
#error No motor type specified
#endif

#ifndef USER_MOTOR
#error Motor type is not defined in user.h
#endif

#define M_BASE_TORQUE           NULL    // Base torque (N.m)
#define M_BASE_FLUX             NULL    // Base flux linkage (volt.sec/rad)
#define M_BASE_FREQ             250     // Base electrical frequency (Hz)

#define M_MAXIMUM_FREQ          125.0   // Motor maximum frequency (Hz)
#define M_STARTUP_FREQ          10.0    // Motor startup frequency (Hz)

//
// Analog scaling with ADC
//
#define M_ADC_PU_SCALE_FACTOR          (float32_t)(0.000244140625)    // 1/2^12
#define M_ADC_PPB_PU_SCALE_FACTOR      (float32_t)(0.000488281250)    // 1/2^11

//
// Current Scale
//
#define M_ADC_SCALE_CURRENT            (float32_t)(M_BASE_CURRENT * 2.0)
#define M_CURRENT_SENSE_SCALE          (float32_t)(M_ADC_SCALE_CURRENT / 4096.0)

//
// Voltage Scale
//
#define M_ADC_SCALE_VOLATGE            (float32_t)(M_BASE_VOLTAGE * 1.732050808)
#define M_VOLTAGE_SENSE_SCALE          (float32_t)(M_ADC_SCALE_VOLATGE / 4096.0)


#endif  // end of MOTOR_DRIVE_USER_H definition


