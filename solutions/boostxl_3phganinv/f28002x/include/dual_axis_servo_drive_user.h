//#############################################################################
//
// FILE:    dual_axis_servo_drive_user.h
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

#ifndef DUAL_AXIS_SERVO_DRIVE_USER_H
#define DUAL_AXIS_SERVO_DRIVE_USER_H

//
// Include project specific include files.
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions for motor 1
// motor 2, can be set separately
//
#define DM_PWM_FREQUENCY        12.5   // in KHz

//
// Analog scaling with ADC
//
#define ADC_RESOLUTION              4096.0F             // 2^12, 12bits
#define ADC_PU_SCALE_FACTOR         0.000244140625F     // 1/2^12, 12bits ADC
#define ADC_PU_PPB_SCALE_FACTOR     0.000488281250F     // 1/2^11, 12bits ADC

//
// ADC and PWM Related defines for M1
//
#define M1_IV_ADC_BASE         ADCC_BASE           //C0*/A11, Set up based board
#define M1_IW_ADC_BASE         ADCA_BASE           //A12*/C1, Set up based board
#define M1_IU_ADC_BASE         ADCC_BASE           //C4*/A14, Set up based board
#define M1_VDC_ADC_BASE        ADCA_BASE           //A6,      Set up based board

#define M1_IV_ADCRESULT_BASE   ADCCRESULT_BASE     //C0,  Set up based board
#define M1_IW_ADCRESULT_BASE   ADCARESULT_BASE     //A12, Set up based board
#define M1_IU_ADCRESULT_BASE   ADCCRESULT_BASE     //C4,  Set up based board
#define M1_VDC_ADCRESULT_BASE  ADCARESULT_BASE     //A6,  Set up based board

#define M1_IV_ADC_CH_NUM       ADC_CH_ADCIN0       //C0,  Set up based board
#define M1_IW_ADC_CH_NUM       ADC_CH_ADCIN12      //A12, Set up based board
#define M1_IU_ADC_CH_NUM       ADC_CH_ADCIN4       //C4,  Set up based board
#define M1_VDC_ADC_CH_NUM      ADC_CH_ADCIN6       //A6,  Set up based board

#define M1_IV_ADC_SOC_NUM      ADC_SOC_NUMBER0     //C0,  Set up based board
#define M1_IW_ADC_SOC_NUM      ADC_SOC_NUMBER0     //A12, Set up based board
#define M1_IU_ADC_SOC_NUM      ADC_SOC_NUMBER1     //C4,  Set up based board
#define M1_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER1     //A6,  Set up based board

#define M1_IV_ADC_PPB_NUM      ADC_PPB_NUMBER1     //C0,  Set up based board
#define M1_IW_ADC_PPB_NUM      ADC_PPB_NUMBER1     //A12, Set up based board
#define M1_IU_ADC_PPB_NUM      ADC_PPB_NUMBER2     //C4,  Set up based board
#define M1_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER2     //A6,  Set up based board

#define M1_U_CMPSS_BASE        CMPSS3_BASE         //C4,  Set up based board
#define M1_V_CMPSS_BASE        CMPSS1_BASE         //C0,  Set up based board
#define M1_W_CMPSS_BASE        CMPSS4_BASE         //A12, Set up based board

#define M1_IFB_V      ADC_readResult(M1_IV_ADCRESULT_BASE, M1_IV_ADC_SOC_NUM)
#define M1_IFB_W      ADC_readResult(M1_IW_ADCRESULT_BASE, M1_IW_ADC_SOC_NUM)
#define M1_IFB_U      ADC_readResult(M1_IU_ADCRESULT_BASE, M1_IU_ADC_SOC_NUM)
#define M1_VDC        ADC_readResult(M1_VDC_ADCRESULT_BASE, M1_VDC_ADC_SOC_NUM)

#define M1_IFB_V_PPB  ADC_readPPBResult(M1_IV_ADCRESULT_BASE, M1_IV_ADC_PPB_NUM)
#define M1_IFB_W_PPB  ADC_readPPBResult(M1_IW_ADCRESULT_BASE, M1_IW_ADC_PPB_NUM)
#define M1_IFB_U_PPB  ADC_readPPBResult(M1_IU_ADCRESULT_BASE, M1_IU_ADC_PPB_NUM)
#define M1_VDC_PPB    ADC_readPPBResult(M1_VDC_ADCRESULT_BASE, M1_VDC_ADC_PPB_NUM)

#define M1_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // Set up based board

#define M1_U_PWM_BASE          EPWM1_BASE          // Set up based board
#define M1_V_PWM_BASE          EPWM2_BASE          // Set up based board
#define M1_W_PWM_BASE          EPWM3_BASE          // Set up based board

#define M1_INT_PWM             INT_EPWM1           // Set up based board
#define M1_INT_ADC_NUM         ADC_INT_NUMBER1     // Set up based board

#define M1_QEP_BASE            EQEP1_BASE          // Set up based board

#define M1_SPI_BASE            SPIA_BASE           // Set up based board

//! \brief Defines the XBAROUTPUT
#define M1_XBAROUTPUT          XBAR_OUTPUT7

//! \brief Defines the gpio for enabling Power Module
#define M1_EN_GATE_GPIO        23                   // Set up based board

//! \brief Defines the gpio for the nFAULT of Power Module
#define M1_nFAULT_GPIO         42                   // Set up based board

//! \brief Defines the gpio for the nFAULT of XBAR INPUT
#define M1_XBAR_INPUT_GPIO     42                   // Set up based board

//! \brief Defines the gpio for the DRV SPI_SCS
#define M1_SPI_SCS_GPIO        5                    // Set up based board

//! \brief Defines the gpio for the clear fault
#define M1_CLR_FAULT_GPIO      5                    //

//------------------------------------------------------------------------------
// interrupt
#define M1_PIE_INT_NUM      INT_EPWM1                   // EPWM1_INT
#define M1_CPU_INT_NUM      INTERRUPT_CPU_INT3          // EPWM1_INT-CPU_INT3
#define M1_INT_ACK_GROUP    INTERRUPT_ACK_GROUP3        // EPWM1_INT-CPU_INT3

//------------------------------------------------------------------------------
// CMPSS
#define M1_IU_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C4
#define M1_IU_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C4, N/A

#define M1_IV_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_1    // CMPSS1-C0
#define M1_IV_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_1    // CMPSS1-C0

#define M1_IW_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A12, N/A
#define M1_IW_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A12

#define M1_IU_CMPHP_MUX     4                            // CMPSS3-C4
#define M1_IU_CMPLP_MUX     4                            // CMPSS3-C4, N/A

#define M1_IV_CMPHP_MUX     1                            // CMPSS1-C0
#define M1_IV_CMPLP_MUX     1                            // CMPSS1-C0

#define M1_IW_CMPHP_MUX     2                            // CMPSS4-A12, N/A
#define M1_IW_CMPLP_MUX     2                            // CMPSS4-A12

//------------------------------------------------------------------------------
// XBAR-EPWM
#define M1_XBAR_TRIP_ADDRL      XBAR_O_TRIP4MUX0TO15CFG
#define M1_XBAR_TRIP_ADDRH      XBAR_O_TRIP4MUX16TO31CFG

#define M1_IU_XBAR_EPWM_MUX     XBAR_EPWM_MUX04_CMPSS3_CTRIPH       // CMPSS3_HP
#define M1_IV_XBAR_EPWM_MUX     XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L  // CMPSS1_HP&LP
#define M1_IW_XBAR_EPWM_MUX     XBAR_EPWM_MUX07_CMPSS4_CTRIPL       // CMPSS4_LP

//! \brief Defines the XBAR EPWM INPUT link to nFAULT
#define M1_INPUT_XBAR_EPWM_MUX  XBAR_EPWM_MUX01_INPUTXBAR1

#define M1_IU_XBAR_MUX          XBAR_MUX04                  // CMPSS3_HP
#define M1_IV_XBAR_MUX          XBAR_MUX00                  // CMPSS1_HP&LP
#define M1_IW_XBAR_MUX          XBAR_MUX07                  // CMPSS4_LP
#define M1_EPWM_XBAR_MUX        XBAR_MUX01                  // INPUTXBAR1

//! \brief Defines the XBAR TRIP link to nFAULT
#define M1_XBAR_TRIP_NUM        XBAR_TRIP4

//! \brief Defines the EPWM DC TRIP link to CMPSSs
#define M1_DC_TRIP_NUM          EPWM_DC_COMBINATIONAL_TRIPIN4

//! \brief Defines the XBAR INPUT link to nFAULT
#define M1_XBAR_INPUT_NUM       XBAR_INPUT1

//! \brief Defines the TZTRIP link to nFAULT
#define M1_TZ_OSHT_NUM          EPWM_TZ_SIGNAL_OSHT1

//
// ADC and PWM Related defines for M2
//
#define M2_IV_ADC_BASE         ADCA_BASE            //A10*/C10, Set up based board
#define M2_IW_ADC_BASE         ADCC_BASE            //C15*/A0,  Set up based board
#define M2_IU_ADC_BASE         ADCA_BASE            //A9*/C8,   Set up based board
#define M2_VDC_ADC_BASE        ADCC_BASE            //C2*/A5,   Set up based board

#define M2_IV_ADCRESULT_BASE   ADCARESULT_BASE      //A10, Set up based board
#define M2_IW_ADCRESULT_BASE   ADCCRESULT_BASE      //C15, Set up based board
#define M2_IU_ADCRESULT_BASE   ADCARESULT_BASE      //A9,  Set up based board
#define M2_VDC_ADCRESULT_BASE  ADCCRESULT_BASE      //C2,  Set up based board

#define M2_IV_ADC_CH_NUM       ADC_CH_ADCIN10       //A10, Set up based board
#define M2_IW_ADC_CH_NUM       ADC_CH_ADCIN15       //C15, Set up based board
#define M2_IU_ADC_CH_NUM       ADC_CH_ADCIN9        //A9, Set up based board
#define M2_VDC_ADC_CH_NUM      ADC_CH_ADCIN2        //C2,  Set up based board

#define M2_IV_ADC_SOC_NUM      ADC_SOC_NUMBER2      //A10, Set up based board
#define M2_IW_ADC_SOC_NUM      ADC_SOC_NUMBER2      //C15, Set up based board
#define M2_IU_ADC_SOC_NUM      ADC_SOC_NUMBER3      //A9,  Set up based board
#define M2_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER3      //C2,  Set up based board

#define M2_IV_ADC_PPB_NUM      ADC_PPB_NUMBER3      //A10, Set up based board
#define M2_IW_ADC_PPB_NUM      ADC_PPB_NUMBER3      //C15, Set up based board
#define M2_IU_ADC_PPB_NUM      ADC_PPB_NUMBER4      //A9, Set up based board
#define M2_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER4      //C2,  Set up based board

#define M2_U_CMPSS_BASE        CMPSS4_BASE          //A10, Set up based board
#define M2_V_CMPSS_BASE        CMPSS2_BASE          //C15, Set up based board
#define M2_W_CMPSS_BASE        CMPSS3_BASE          //A9,  Set up based board

#define M2_IFB_V      ADC_readResult(M2_IV_ADCRESULT_BASE, M2_IV_ADC_SOC_NUM)
#define M2_IFB_W      ADC_readResult(M2_IW_ADCRESULT_BASE, M2_IW_ADC_SOC_NUM)
#define M2_IFB_U      ADC_readResult(M2_IU_ADCRESULT_BASE, M2_IU_ADC_SOC_NUM)
#define M2_VDC        ADC_readResult(M2_VDC_ADCRESULT_BASE, M2_VDC_ADC_SOC_NUM)

#define M2_IFB_V_PPB  ADC_readPPBResult(M2_IV_ADCRESULT_BASE, M2_IV_ADC_PPB_NUM)
#define M2_IFB_W_PPB  ADC_readPPBResult(M2_IW_ADCRESULT_BASE, M2_IW_ADC_PPB_NUM)
#define M2_IFB_U_PPB  ADC_readPPBResult(M2_IU_ADCRESULT_BASE, M2_IU_ADC_PPB_NUM)
#define M2_VDC_PPB    ADC_readPPBResult(M2_VDC_ADCRESULT_BASE, M2_VDC_ADC_PPB_NUM)

#define M2_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM7_SOCA  // NC: Set up based board

#define M2_U_PWM_BASE          EPWM7_BASE          // NC: Set up based board
#define M2_V_PWM_BASE          EPWM4_BASE          // NC: Set up based board
#define M2_W_PWM_BASE          EPWM5_BASE          // NC: Set up based board

#define M2_INT_PWM             INT_EPWM7           // NC: Set up based board
#define M2_INT_ADC_NUM         ADC_INT_NUMBER1     // Set up based board

#define M2_QEP_BASE            EQEP2_BASE          // NC: Set up based board

#define M2_SPI_BASE            SPIB_BASE           // NC: Set up based board

#define M2_XBAROUTPUT          XBAR_OUTPUT8

//! \brief Defines the gpio for enabling Power Module
#define M2_EN_GATE_GPIO        39

//! \brief Defines the gpio for the nFAULT of Power Module
#define M2_nFAULT_GPIO         45

//! \brief Defines the gpio for the nFAULT of XBAR INPUT
#define M2_XBAR_INPUT_GPIO     45                   // Set up based board

//! \brief Defines the gpio for the DRV SPI_SCS
#define M2_SPI_SCS_GPIO        33                   // Set up based board

//! \brief Defines the gpio for the clear fault
#define M2_CLR_FAULT_GPIO      33                   //

//------------------------------------------------------------------------------
// interrupt
#define M2_PIE_INT_NUM      INT_EPWM7                   // EPWM7_INT
#define M2_CPU_INT_NUM      INTERRUPT_CPU_INT3          // EPWM7_INT-CPU_INT3
#define M2_INT_ACK_GROUP    INTERRUPT_ACK_GROUP3        // EPWM7_INT-CPU_INT3

//------------------------------------------------------------------------------
// CMPSS
#define M2_IU_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_4    // CMPSS4-A9
#define M2_IU_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_4    // CMPSS4-A9, N/A

#define M2_IV_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_2    // CMPSS2-A10
#define M2_IV_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_2    // CMPSS2-A10

#define M2_IW_CMPHP_SEL     ASYSCTL_CMPHPMUX_SELECT_3    // CMPSS3-C15, N/A
#define M2_IW_CMPLP_SEL     ASYSCTL_CMPLPMUX_SELECT_3    // CMPSS3-C15

#define M2_IU_CMPHP_MUX     0                            // CMPSS4-A9
#define M2_IU_CMPLP_MUX     0                            // CMPSS4-A9, N/A

#define M2_IV_CMPHP_MUX     3                            // CMPSS2-A10
#define M2_IV_CMPLP_MUX     3                            // CMPSS2-A10

#define M2_IW_CMPHP_MUX     2                            // CMPSS3-C15, N/A
#define M2_IW_CMPLP_MUX     2                            // CMPSS3-C15

//------------------------------------------------------------------------------
// XBAR-EPWM
#define M2_XBAR_TRIP_ADDRL      XBAR_O_TRIP5MUX0TO15CFG
#define M2_XBAR_TRIP_ADDRH      XBAR_O_TRIP5MUX16TO31CFG

#define M2_IU_XBAR_EPWM_MUX     XBAR_EPWM_MUX06_CMPSS4_CTRIPH       // CMPSS4_HP
#define M2_IV_XBAR_EPWM_MUX     XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L  // CMPSS2_HP&LP
#define M2_IW_XBAR_EPWM_MUX     XBAR_EPWM_MUX05_CMPSS3_CTRIPL       // CMPSS3_LP

//! \brief Defines the XBAR EPWM INPUT link to nFAULT
#define M2_INPUT_XBAR_EPWM_MUX  XBAR_EPWM_MUX03_INPUTXBAR2

#define M2_IU_XBAR_MUX          XBAR_MUX06                  // CMPSS4_HP
#define M2_IV_XBAR_MUX          XBAR_MUX02                  // CMPSS2_HP&LP
#define M2_IW_XBAR_MUX          XBAR_MUX05                  // CMPSS3_LP
#define M2_EPWM_XBAR_MUX        XBAR_MUX03                  // INPUTXBAR2

//! \brief Defines the XBAR TRIP link to nFAULT
#define M2_XBAR_TRIP_NUM        XBAR_TRIP5

//! \brief Defines the EPWM DC TRIP link to CMPSSs
#define M2_DC_TRIP_NUM          EPWM_DC_COMBINATIONAL_TRIPIN5

//! \brief Defines the XBAR INPUT link to nFAULT
#define M2_XBAR_INPUT_NUM       XBAR_INPUT2

//! \brief Defines the TZTRIP link to nFAULT
#define M2_TZ_OSHT_NUM          EPWM_TZ_SIGNAL_OSHT2

//
// Motor_1 Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M1_PWM_FREQUENCY           DM_PWM_FREQUENCY   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M1_ISR_FREQUENCY           (M1_PWM_FREQUENCY)

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M1_ISR_FREQUENCY           (2*M1_PWM_FREQUENCY)

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M1_INV_PWM_TICKS         ((SYSTEM_FREQUENCY / M1_PWM_FREQUENCY) * 1000)
#define M1_INV_PWM_DB            (50.0)
#define M1_QEP_UNIT_TIMER_TICKS  (SYSTEM_FREQUENCY/(2*M1_PWM_FREQUENCY) * 1000)

#define M1_INV_PWM_TBPRD         (M1_INV_PWM_TICKS / 2)
#define M1_INV_PWM_HALF_TBPRD    (M1_INV_PWM_TBPRD / 2)
#define M1_SAMPLING_FREQ         (M1_ISR_FREQUENCY * 1000)
#define M1_CUR_LOOP_BANDWIDTH    (2.0f * PI * M1_SAMPLING_FREQ / 100.0f)

#define M1_TPWM_CARRIER          (1000.0 / M1_PWM_FREQUENCY)    //in uSec

//
// FCL Computation time predetermined from library
//
#define M1_FCL_COMPUTATION_TIME  (1.00)  //in uS

//
// set the motor parameters to the one available
//
#define M1_ENCODER_LINES         1000 // Encoder lines for Tekic

//
// Define the electrical motor parameters
//
#define M1_RS      0.381334811     // Stator resistance (ohm)
#define M1_RR      NULL            // Rotor resistance (ohm)
#define M1_LS      0.000169791776  // Stator inductance (H)
#define M1_LD      M1_LS           // Stator d-axis inductance (H)
#define M1_LQ      M1_LS           // Stator q-axis inductance (H)
#define M1_LR      NULL            // Rotor inductance (H)
#define M1_LM      NULL            // Magnetizing inductance (H)
#define M1_KB      0.040           // BEMF Constant (V/Hz)
#define M1_POLES   8               // Number of poles

//
// NOTE:-
// Base voltage and base current information from TIDA-00909 doc is
// based off of an ADC that works at 3.3V reference.
// The base current = 16.5A (for a spread of 3.3V - 1.65V = 1.65V)
// The base voltage  = 81.5 / sqrt(3)=
// Define the base quantites
//
#define M1_BASE_VOLTAGE     47.05   // Base peak phase voltage (volt), Vdc/sqrt(3)
#define M1_BASE_CURRENT     16.5    // Base peak phase current (amp),
                                    // the maximum measurable peak current
#define M1_BASE_TORQUE      NULL    // Base torque (N.m)
#define M1_BASE_FLUX        NULL    // Base flux linkage (volt.sec/rad)
#define M1_BASE_FREQ        400     // Base electrical frequency (Hz)
#define M1_MAXIMUM_CURRENT  7.5     // Motor maximum torque current (amp)

#define M1_MAXIMUM_VOLTAGE  36.0    // DC bus maximum voltage (V)
#define M1_MINIMUM_VOLTAGE  5.0     // DC bus minimum voltage (V)

#define M1_MAXIMUM_FREQ     125.0   // Motor maximum frequency (Hz)
#define M1_STARTUP_FREQ     10.0    // Motor startup frequency (Hz)

#define M1_SPEED_LSW        0.05    // reference speed (pu)
#define M1_SPEED_REF        0.10    // reference speed (pu)
#define M1_ID_START         0.2     // alignment reference d-axis current
#define M1_ID_RUN           0.0     // alignment reference d-axis current
#define M1_IQ_LEVEL5        0.10    // reference q-axis current for level5
#define M1_IQ_NO_LEVEL5     0.10    // ref q-axis current for no level5

#define M1_VD_TEST          0.0     // reference d-axis voltage for level2
#define M1_VQ_TEST          0.20    // reference q-axis voltage for level2

#define M1_VDCBUS_MAX       50.0    // maximum dc bus voltage for motor
#define M1_VDCBUS_MIN       10.0    // minimum dc bus voltage for motor

//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M1_CURRENT_SCALE(A)   (2048 * A / M1_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M1_ADC_PU_SCALE_FACTOR      0.000244140625f     // 1/2^12
#define M1_ADC_PPB_PU_SCALE_FACTOR  0.000488281250f     // 1/2^11

//
// Current Scale
//
#define M1_MAXIMUM_SCALE_CURRENT    33.0
#define M1_CURRENT_SF               (M1_MAXIMUM_SCALE_CURRENT / 4096.0)
#define M1_CURRENT_INV_SF           (4096.0 / M1_MAXIMUM_SCALE_CURRENT)

//
// Voltage Scale
//
#define M1_MAXIMUM_SCALE_VOLATGE    81.5
#define M1_VOLTAGE_SF               (M1_MAXIMUM_SCALE_VOLATGE / 4096.0)
#define M1_VOLTAGE_INV_SF           (4096.0 / M1_MAXIMUM_SCALE_VOLATGE)

//
// Motor_2 Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M2_PWM_FREQUENCY            DM_PWM_FREQUENCY   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M2_ISR_FREQUENCY           (M2_PWM_FREQUENCY)

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M2_ISR_FREQUENCY           (2 * M2_PWM_FREQUENCY)

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M2_INV_PWM_TICKS         ((SYSTEM_FREQUENCY / M2_PWM_FREQUENCY) * 1000)
#define M2_INV_PWM_DB            (50.0)
#define M2_QEP_UNIT_TIMER_TICKS  ((SYSTEM_FREQUENCY)/(2*M2_PWM_FREQUENCY)*1000)

#define M2_INV_PWM_TBPRD         (M2_INV_PWM_TICKS / 2)
#define M2_INV_PWM_HALF_TBPRD    (M2_INV_PWM_TBPRD / 2)
#define M2_SAMPLING_FREQ         (M2_ISR_FREQUENCY * 1000)
#define M2_CUR_LOOP_BANDWIDTH    (2.0f * PI * M2_SAMPLING_FREQ / 100.0f)

#define M2_TPWM_CARRIER          (1000.0 / M2_PWM_FREQUENCY)    //in uSec

//
// FCL Computation time predetermined from library
// tests on F28004x
//
#define M2_FCL_COMPUTATION_TIME    (1.00)  //in uS

//
// set the motor parameters to the one available
//
#define M2_ENCODER_LINES            1000        // Encoder lines

//
// Define the electrical motor parameters
//
#define M2_RS       0.381334811     // Stator resistance (ohm)
#define M2_RR       NULL            // Rotor resistance (ohm)
#define M2_LS       0.000169791776  // Stator inductance (H)
#define M2_LD       M1_LS           // Stator d-axis inductance (H)
#define M2_LQ       M1_LS           // Stator q-axis inductance (H)
#define M2_LR       NULL            // Rotor inductance (H)
#define M2_LM       NULL            // Magnetizing inductance (H)
#define M2_KB       0.040           // BEMF Constant (V/Hz)
#define M2_POLES    8               // Number of poles

//
// Define the base quantites
//
#define M2_BASE_VOLTAGE     42.78 // Base peak phase voltage (volt), Vdc/sqrt(3)
#define M2_BASE_CURRENT     16.5  // Base peak phase current (amp),
                                  // the maximum measurable peak current
#define M2_BASE_TORQUE      NULL  // Base torque (N.m)
#define M2_BASE_FLUX        NULL  // Base flux linkage (volt.sec/rad)
#define M2_BASE_FREQ        400   // Base electrical frequency (Hz)
#define M2_MAXIMUM_CURRENT  7.5   // Motor maximum torque current (amp)

#define M2_MAXIMUM_VOLTAGE  36.0    // DC bus maximum voltage (V)
#define M2_MINIMUM_VOLTAGE  5.0     // DC bus minimum voltage (V)

#define M2_MAXIMUM_FREQ     125.0   // Motor maximum frequency (Hz)
#define M2_STARTUP_FREQ     10.0    // Motor startup frequency (Hz)

#define M2_SPEED_LSW        0.05    // reference speed (pu)
#define M2_SPEED_REF        0.10    // reference speed (pu)
#define M2_ID_START         0.2     // alignment reference d-axis current
#define M2_ID_RUN           0.0     // alignment reference d-axis current
#define M2_IQ_LEVEL5        0.10    // reference q-axis current for level5
#define M2_IQ_NO_LEVEL5     0.10    // ref q-axis current for no level5

#define M2_VD_TEST          0.0     // reference d-axis voltage for level2
#define M2_VQ_TEST          0.20    // reference q-axis voltage for level2

#define M2_VDCBUS_MAX       50.0    // maximum dc bus voltage for motor
#define M2_VDCBUS_MIN       10.0    // minimum dc bus voltage for motor
//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M2_CURRENT_SCALE(A)             (2048 * A / M2_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M2_ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define M2_ADC_PPB_PU_SCALE_FACTOR      0.000488281250     // 1/2^11

//
// Current Scale
//
#define M2_MAXIMUM_SCALE_CURRENT    33.0
#define M2_CURRENT_SF               (M2_MAXIMUM_SCALE_CURRENT / 4096.0)
#define M2_CURRENT_INV_SF           (4096.0 / M2_MAXIMUM_SCALE_CURRENT)

//
// Voltage Scale
//
#define M2_MAXIMUM_SCALE_VOLATGE    81.50
#define M2_VOLTAGE_SF               (M2_MAXIMUM_SCALE_VOLATGE / 4096.0)
#define M2_VOLTAGE_INV_SF           (4096.0 / M2_MAXIMUM_SCALE_VOLATGE)


#endif  // end of DUAL_AXIS_SERVO_DRIVE_USER_H definition

