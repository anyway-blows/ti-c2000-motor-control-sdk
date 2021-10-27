//#############################################################################
//
// FILE:    motor_drive_user.h
//
// TITLE:   motor parameters definition
//
// Group:   C2000
//
// Target Family: F2837x
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
//! \file  solutions/multi_axis_drive/f2838x/include/motor_drive_user.h
//! \brief header file to be included in all labs
//!
//

#ifndef MOTOR_DRIVE_USER_H
#define MOTOR_DRIVE_USER_H

//
//! \defgroup MASTER_DRIVE
//! @{
//

//
// includes
//
#include "motor_drive_settings.h"

// Analog scaling with ADC
//
#define ADC_RESOLUTION              4096.0F             // 2^12, 12bits
#define ADC_PU_SCALE_FACTOR         0.000244140625F     // 1/2^12, 12bits ADC
#define ADC_PU_PPB_SCALE_FACTOR     0.000488281250F     // 1/2^11, 12bits ADC

//
// ADC and PWM Related defines for motor
//
#define M_IU_ADC_BASE         ADCC_BASE           //C2,  S  Set up based board
#define M_IV_ADC_BASE         ADCA_BASE           //A2,  L: Set up based board
#define M_IW_ADC_BASE         ADCB_BASE           //B2,  L: Set up based board
#define M_VDC_ADC_BASE        ADCB_BASE           //B0,  S: Set up based board

#define M_IU_ADCRESULT_BASE   ADCCRESULT_BASE     //C2,  S: Set up based board
#define M_IV_ADCRESULT_BASE   ADCARESULT_BASE     //A2,  L: Set up based board
#define M_IW_ADCRESULT_BASE   ADCBRESULT_BASE     //B2,  L: Set up based board
#define M_VDC_ADCRESULT_BASE  ADCBRESULT_BASE     //B0, S: Set up based board

#define M_IU_ADC_CH_NUM       ADC_CH_ADCIN2       //C2,  S: Set up based board
#define M_IV_ADC_CH_NUM       ADC_CH_ADCIN2       //A2,  L: Set up based board
#define M_IW_ADC_CH_NUM       ADC_CH_ADCIN2       //B2,  L: Set up based board
#define M_VDC_ADC_CH_NUM      ADC_CH_ADCIN0       //B0, S: Set up based board

#define M_IU_ADC_SOC_NUM      ADC_SOC_NUMBER0     //C2,  S: Set up based board
#define M_IV_ADC_SOC_NUM      ADC_SOC_NUMBER0     //A2,  L: Set up based board
#define M_IW_ADC_SOC_NUM      ADC_SOC_NUMBER0     //B2,  L: Set up based board
#define M_VDC_ADC_SOC_NUM     ADC_SOC_NUMBER1     //B0, S: Set up based board

#define M_IU_ADC_PPB_NUM      ADC_PPB_NUMBER1     // S: Set up based board
#define M_IV_ADC_PPB_NUM      ADC_PPB_NUMBER1     // L: Set up based board
#define M_IW_ADC_PPB_NUM      ADC_PPB_NUMBER1     // L: Set up based board
#define M_VDC_ADC_PPB_NUM     ADC_PPB_NUMBER2     // S: Set up based board

#define M_U_CMPSS_BASE        CMPSS1_BASE         // S: Set up based board
#define M_V_CMPSS_BASE        CMPSS1_BASE         // L: Set up based board
#define M_W_CMPSS_BASE        CMPSS3_BASE         // L: Set up based board
#define M_VDC_CMPSS_BASE      CMPSS1_BASE         // S: Set up based board

#define M_ADC_SAMPLE_WINDOW   12                  // ADC Sample window

#define M_ADC_TRIGGER_SOC     ADC_TRIGGER_EPWM1_SOCA  // M: Set up based board

#define M_U_PWM_BASE          EPWM1_BASE          // M: Set up based board
#define M_V_PWM_BASE          EPWM2_BASE          // M: Set up based board
#define M_W_PWM_BASE          EPWM3_BASE          // M: Set up based board

#define M_INT_PWM             INT_EPWM1           // M: Set up based board

#define M_QEP_BASE            EQEP1_BASE          // M: Set up based board

#define M_SPI_BASE            SPIB_BASE           // M: Set up based board

#define M_IFB_U      ADC_readResult(M_IU_ADCRESULT_BASE, M_IU_ADC_SOC_NUM)
#define M_IFB_V      ADC_readResult(M_IV_ADCRESULT_BASE, M_IV_ADC_SOC_NUM)
#define M_IFB_W      ADC_readResult(M_IW_ADCRESULT_BASE, M_IW_ADC_SOC_NUM)

#define M_VDC        ADC_readResult(M_VDC_ADCRESULT_BASE, M_VDC_ADC_SOC_NUM)

#define M_IFB_U_PPB  ADC_readPPBResult(M_IU_ADCRESULT_BASE, M_IU_ADC_PPB_NUM)
#define M_IFB_V_PPB  ADC_readPPBResult(M_IV_ADCRESULT_BASE, M_IV_ADC_PPB_NUM)
#define M_IFB_W_PPB  ADC_readPPBResult(M_IW_ADCRESULT_BASE, M_IW_ADC_PPB_NUM)

#define M_VDC_PPB    ADC_readPPBResult(M_VDC_ADCRESULT_BASE, M_VDC_ADC_PPB_NUM)

//
// define EN_GATE and SPI_CS pin for DRV device for Motor 1
//
#define M_DRV_SPI_CS            27          // NC: Set up based IDDK board
#define M_EN_GATE_GPIO          10          // NC: Set up based IDDK board

#define M_CLR_FAULT_GPIO        41          // NC: Set up based IDDK board
#define M_nFAULT_GPIO           40          // NC: Set up based IDDK board
#define M_XBAR_INPUT_GPIO       40          // NC: Set up based IDDK board
#define M_TRIP_CC_GPIO          58          // NC: Set up based IDDK board

//
// Motor Parameters
//

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define M_PWM_FREQUENCY           10.0   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define M_ISR_FREQUENCY           (M_PWM_FREQUENCY)
#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define M_ISR_FREQUENCY           (2*M_PWM_FREQUENCY)
#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define M_INV_PWM_TICKS         (((SYSTEM_FREQUENCY / 2.0) / M_PWM_FREQUENCY) * 1000)
#define M_INV_PWM_DB            (200.0)
#define M_QEP_UNIT_TIMER_TICKS  (SYSTEM_FREQUENCY / (2.0 * M_PWM_FREQUENCY) * 1000)

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
#define M_LS            0.0084          // Stator inductance (H)
#define M_LD            M_LS            // Stator d-axis inductance (H)
#define M_LQ            M_LS            // Stator q-axis inductance (H)
#define M_LR            NULL            // Rotor inductance (H)
#define M_LM            NULL            // Magnetizing inductance (H)
#define M_KB            0.38            // BEMF Constant (V/Hz)
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
#define M_KB            0.8             // BEMF Constant (V/Hz)
#define M_POLES         8               // Number of poles
#define M_ENC_SLOTS     1000            // Numer of slots in the encoder

#define M_SPEED_REF     0.05            // reference speed (pu)
#define M_ID_START      0.2             // alignment reference d-axis current
#define M_ID_RUN        0.0             // running d-axis current
#define M_IQ_LEVEL5     0.10            // reference q-axis current for level5
#define M_IQ_NO_LEVEL5  0.10            // ref q-axis current for no level5

#define M_VD_TEST       0.0             // reference d-axis voltage for level2
#define M_VQ_TEST       0.20            // reference q-axis voltage for level2

#define M_VDCBUS_MAX    200.0           // maximum dc bus voltage for motor
#define M_VDCBUS_MIN    0.0             // minimum dc bus voltage for motor

#else
#error No motor type specified
#endif

#ifndef USER_MOTOR
#error Motor type is not defined in user.h
#endif

//
// NOTE:-
// Base voltage and base current information from TIDA-00909 doc is
// based off of an ADC that works at 3.3V reference.
// The base current = 16.5A (for a spread of 3.3V - 1.65V = 1.65V)
// The base voltage  = 81.5 / sqrt(3)=
// Define the base quantites
//
#define M_BASE_VOLTAGE          236.14 // Base peak phase voltage (volt),
                                       // maximum measurable Vdc/sqrt(3)
#define M_BASE_SHUNT_CURRENT    9.95   // Base peak phase current (amp),
                                       // maximum measurable peak curr.
#define M_BASE_LEM_CURRENT      12.0   // Base peak phase current (amp),
                                       // maximum measurable peak current
#define M_BASE_CURRENT          M_BASE_LEM_CURRENT
#define M_BASE_TORQUE           NULL    // Base torque (N.m)
#define M_BASE_FLUX             NULL    // Base flux linkage (volt.sec/rad)
#define M_BASE_FREQ             250     // Base electrical frequency (Hz)

#define M_MAXIMUM_FREQ          200.0   // Motor maximum frequency (Hz)
#define M_STARTUP_FREQ          10.0    // Motor startup frequency (Hz)

#define M_MAXIMUM_CURRENT       8.0     // Motor maximum torque current (amp)
#define M_MAXIMUM_VOLTAGE       350.0   // DC bus maximum voltage (V)
#define M_MINIMUM_VOLTAGE       20.0    // DC bus minimum voltage (V)

//
// Current sensors scaling
// 1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define M_CURRENT_SCALE(A)             (uint16_t)(2048 * A / M_BASE_CURRENT)

//
// Analog scaling with ADC
//
#define M_ADC_PU_SCALE_FACTOR          0.000244140625     // 1/2^12
#define M_ADC_PPB_PU_SCALE_FACTOR      0.000488281250     // 1/2^11


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

//
// Close the Doxygen group.
//! @} //defgroup MOTOR_DRIVE_USER
//

#endif  // end of MOTOR_DRIVE_USER_H definition

