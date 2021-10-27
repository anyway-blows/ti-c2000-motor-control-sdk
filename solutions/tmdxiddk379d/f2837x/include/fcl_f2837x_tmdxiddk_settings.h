//#############################################################################
//
// FILE:    fcl_f2837x_tmdxiddk_settings.h
//
// TITLE:   User settings
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

#ifndef FCL_F2837X_TMDXIDDK_SETTINGS_H
#define FCL_F2837X_TMDXIDDK_SETTINGS_H

//
// Include project specific include files.
//

//
// define math type as float(1)
//
#define   MATH_TYPE      1

#include "IQmathLib.h"
#include "device.h"

#include "ipark.h"              // Include header for the IPARK object
#include "pi.h"                 // Include header for the PI  object
#include "fcl_pi.h"             // Include header for the FCL_PI object
#include "svgen.h"              // Include header for the SVGENDQ object
#include "rampgen.h"            // Include header for the RAMPGEN object
#include "rmp_cntl.h"           // Include header for the RMPCNTL object
#include "volt_calc.h"          // Include header for the PHASEVOLTAGE object
#include "speed_fr.h"           // Include header for the SPEED_MEAS_QEP object
#include "resolver.h"
#include "pid_grando.h"
#include "pid_reg3.h"
#include "speed_observer.h"

#include <math.h>

#include "dlog_4ch_f.h"


//
// List of control GND configurations - COLD or HOT
//
#define  COLD  1           // control GND is COLD
#define  HOT   2           // control GND is HOT

//
// Following is the list of the Build Level choices.
//
#define  FCL_LEVEL1  1           // Verify SVGEN module and PWM generation
#define  FCL_LEVEL2  2           // Verify ADC, park/clarke, calibrate the
                                 // offset and speed measurement
#define  FCL_LEVEL3  3           // Verify closed current(torque) loop + its PI
#define  FCL_LEVEL4  4           // Verify speed loop and speed PID
#define  FCL_LEVEL5  5           // Verify position loop
#define  FCL_LEVEL6  6           // SFRA integration to verify bandwidth

//
// This line sets the SAMPLING FREQUENCY to one of the available choices
//
#define  SINGLE_SAMPLING    1
#define  DOUBLE_SAMPLING    2

//
// Following is the list of Current sense options
//
#define  SHUNT_CURRENT_SENSE   1
#define  LEM_CURRENT_SENSE     2
#define  SD_CURRENT_SENSE      3

//
// Following is the list of Position Encoder options
// Select Position Feedback Option
//
#define  QEP_POS_ENCODER       1
#define  RESOLVER_POS_ENCODER  2
#define  BISS_POS_ENCODER      3
#define  ENDAT_POS_ENCODER     4
#define  SINCOS_POS_ENCODER    5
#define  T_FORMAT_ENCODER      6

#define  ENCODER_MAX_TYPES     7    // for Position Sensor Suite array size

#ifndef  ENCODER_MAX_TYPES
#error  Critical: ENCODER_MAX_TYPES must be defined !!
#endif

#if(ENCODER_MAX_TYPES <= T_FORMAT_ENCODER)
#error  Critical: the Position Sensor Suite array size is too small
#endif

//
// Here below, pick current loop controller option
//
#define  CMPLX_CNTLR     1
#define  PI_CNTLR        2

//
// User can select choices from available control configurations
//
#define  CGND                COLD
#define  BUILDLEVEL          FCL_LEVEL1
#define  SAMPLING_METHOD     SINGLE_SAMPLING   // DOUBLE_SAMPLING  //
#define  FCL_CNTLR           PI_CNTLR          // CMPLX_CNTLR       //
#define  CURRENT_SENSE       LEM_CURRENT_SENSE // SD_CURRENT_SENSE  //
#define  POSITION_ENCODER    T_FORMAT_ENCODER  // QEP_POS_ENCODER   //

//
// Generate error if
//
#if((POSITION_ENCODER != QEP_POS_ENCODER) &&                                   \
     (POSITION_ENCODER != T_FORMAT_ENCODER))
#error  Critical: Only QEP_POS_ENCODER or T_FORMAT_ENCODER is supported        \
    in this example
#endif

#ifndef BUILDLEVEL
#error  Critical: BUILDLEVEL must be defined !!
#endif  // BUILDLEVEL

#define PI 3.14159265358979

//
// Define the system frequency (MHz)
//
#define SYSTEM_FREQUENCY    (DEVICE_SYSCLK_FREQ / 1000000U)

//
// Timer definitions based on System Clock
//
#define     MICROSEC         SYSTEM_FREQUENCY
#define     MICROSEC_50       50 * MICROSEC     // 50 uS
#define     MICROSEC_100     100 * MICROSEC     // 0.1 mS
#define     MICROSEC_150     150 * MICROSEC     // 0.15 mS
#define     MILLISEC        1000 * MICROSEC     // 1 mS

#define     MILSEC_0_5       0.5 * MILLISEC     // 0.5 mS
#define     MILSEC_1         1.0 * MILLISEC     // 1.0 mS
#define     MILSEC_2         2.0 * MILLISEC     // 2.0 mS
#define     MILSEC_5         5.0 * MILLISEC     // 5.0 mS
#define     MILSEC_7_5       7.5 * MILLISEC     // 7.5 mS
#define     MILSEC_10         10 * MILLISEC     // 10 mS
#define     MILSEC_20         20 * MILLISEC     // 20 mS
#define     MILSEC_50         50 * MILLISEC     // 50 mS
#define     MILSEC_100       100 * MILLISEC     // 100 mS
#define     MILSEC_500       500 * MILLISEC     // 500 mS
#define     MILSEC_1000     1000 * MILLISEC     // 1000 mS

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions
//
#define PWM_FREQUENCY           10   // in KHz

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define ISR_FREQUENCY           (PWM_FREQUENCY)

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define ISR_FREQUENCY           (2 * PWM_FREQUENCY)

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
#define INV_PWM_TICKS          (((SYSTEM_FREQUENCY / 2.0) / PWM_FREQUENCY)*1000)
#define INV_PWM_DB             (200.0)
#define QEP_UNIT_TIMER_TICKS   (SYSTEM_FREQUENCY / (2 * PWM_FREQUENCY) * 1000)

#define INV_PWM_TBPRD          (INV_PWM_TICKS / 2)
#define INV_PWM_HALF_TBPRD     (INV_PWM_TBPRD / 2)
#define SAMPLING_FREQ          (ISR_FREQUENCY * 1000)
#define CUR_LOOP_BANDWIDTH     (2.0F * PI * SAMPLING_FREQ / 18)

#define TPWM_CARRIER           (1000.0 / PWM_FREQUENCY)    //in uSec

//
// FCL Computation time predetermined from library
// tests on F2837xD
//
#define FCL_COMPUTATION_TIME    (1.00)  //in uS


//
// set the motor parameters to the one available
//
#define ESTUN_EMJ04APB222        1
#define TEKNIC_2310PLN04K        2

#define USER_MOTOR               TEKNIC_2310PLN04K     // ESTUN_EMJ04APB222   //

//
// Define the electrical motor parameters (Estun Servomotor)
//
#if(USER_MOTOR == ESTUN_EMJ04APB222)
#define RS          2.35                // Stator resistance (ohm)
#define RR          NULL                // Rotor resistance (ohm)
#define LS          0.0065              // Stator inductance (H)
#define LR          NULL                // Rotor inductance (H)
#define LM          NULL                // Magnetizing inductance (H)
#define POLES       8                   // Number of poles
#define ENC_SLOTS   2500                // Numer of slots in the encoder

#define M_ID_START  0.1                 // alignment reference d-axis current
#define M_IQ_LI5    0.05                // reference q-axis current for level5
#define M_IQ_LN5    0.03                // ref q-axis current for no level5

//
// Define the electrical motor parameters (Teknic Servomotor)
//
#elif(USER_MOTOR == TEKNIC_2310PLN04K)
#define RS          0.381334811         // Stator resistance (ohm)
#define RR          NULL                // Rotor resistance (ohm)
#define LS          0.000169791776      // Stator inductance (H)
#define LR          NULL                // Rotor inductance (H)
#define LM          NULL                // Magnetizing inductance (H)
#define FLUX        0.0398557819        // BEMF constant (V/Hz)
#define POLES       8                   // Number of poles
#define ENC_SLOTS   1000                // Numer of slots in the encoder

#define M_ID_START  0.2                 // alignment reference d-axis current
#define M_IQ_LI5    0.10                // reference q-axis current for level5
#define M_IQ_LN5    0.10                // ref q-axis current for no level5

#else
#error No motor type specified
#endif

#ifndef USER_MOTOR
#error Motor type is not defined in user.h
#endif

//
// Define the base quantites
//
#define BASE_VOLTAGE        236.14 // Base peak phase voltage (volt), Vdc/sqrt(3)
#define BASE_SHUNT_CURRENT  9.95   // Base peak phase current (amp),
                                   // Max. measurable peak curr.
#define BASE_LEM_CURRENT    12.0   //  ----- do -----
#define BASE_TORQUE                // Base torque (N.m)
#define BASE_FLUX                  // Base flux linkage (volt.sec/rad)
#define BASE_FREQ           250    // Base electrical frequency (Hz)

//
// Sigma Delta Filter Module - frame setting
//
#define  SDFM_TICKS    5
#define  OSR_RATE      (128 - 1)

//
// Current sensors scaling
// LEM    1.0pu current ==> 12.0A -> 2048 counts ==> 8A -> 1365
// SHUNT  1.0pu current ==> 9.95A -> 2048 counts ==> 8A -> 1647
//
#define LEM(A)     (2048 * A / BASE_LEM_CURRENT)
#define SHUNT(A)   (2048 * A / BASE_SHUNT_CURRENT)

//
// Analog scaling with ADC
//
#define ADC_PU_SCALE_FACTOR        0.000244140625     // 1/2^12
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250     // 1/2^11

//
// Analog scaling with SDFM
//
#define SD_PU_SCALE_FACTOR         0.000030517578125  // 1/2^15

//
// 100v corresponds to 0.212 filter output reading
//
#define SD_VOLTAGE_SENSE_SCALE     (SD_PU_SCALE_FACTOR * (100 / 0.212))

//
// ADC Related defines
//
#define IFB_SV          ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)
#define IFB_SW          ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1)
#define IFB_SV_PPB      ADC_readPPBResult(ADCARESULT_BASE, ADC_PPB_NUMBER2)
#define IFB_SW_PPB      ADC_readPPBResult(ADCBRESULT_BASE, ADC_PPB_NUMBER2)


#define IFB_LEMV        ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)
#define IFB_LEMW        ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0)
#define IFB_LEMV_PPB    ADC_readPPBResult(ADCARESULT_BASE, ADC_PPB_NUMBER1)
#define IFB_LEMW_PPB    ADC_readPPBResult(ADCBRESULT_BASE, ADC_PPB_NUMBER1)


#define IV_SDFM_FILTER          SDFM_FILTER_1
#define IW_SDFM_FILTER          SDFM_FILTER_2
#define VDC_SDFM_FILTER         SDFM_FILTER_3

#define SDFM_FILTER_TYPE        3
#define SDFM_OSR                128

//
// Scaling factors to bring all current feedbacks to normal scale
//   matching with shunt based measurement
//  With shunt, 1.0pu current  == 9.945A
//       LEM,   1.0pu current  == 12A
//       SDFM,  0.8906pu current  == 12.5A
//
#define  LEM_TO_SHUNT    1.206637   // (12.0/9.945)
#define  SDFM_TO_SHUNT   1.41131    // (12.5/0.8906)/9.945


#endif // end of FCL_F2837X_TMDXIDDK_SETTINGS_H definition
