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

#ifndef MOTOR_COMMON_H
#define MOTOR_COMMON_H

//
//! \file  solutions/fast_uni_lab/common/include/motor_comm.h
//! \brief  header file to be included in all labs
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
//! \addtogroup MOTOR COMM
//! @{
//
//*****************************************************************************

// includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#include "userParams.h"

#include "clarke.h"
#include "est.h"
#include "filter_fo.h"
#include "ipark.h"
#include "park.h"
#include "pi.h"
#include "svgen.h"
#include "svgen_current.h"
#include "traj.h"
#include "fwc.h"
#include "mtpa.h"
#include "vsf.h"

#include "vs_freq.h"
#include "angle_gen.h"

#include "volt_recons.h"

#if defined(MOTOR1_ISBLDC)
#include "isbldc.h"
#include "rimpulse.h"
#include "mod6cnt.h"
#endif  // MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
#include "encoder.h"
#include "speedcalc.h"
#include "hall.h"
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
#include "hall.h"
#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
#include "esmo.h"
#include "speedfr.h"
#endif  // MOTOR1_ESMO

#include "est_trajState.h"
#include "fluxHF.h"

#if defined(MOTOR1_SSIPD)
#include "ssipd.h"
#endif // MOTOR1_SSIPD || MOTOR2_SSIPD

#if defined(MOTOR1_DCLINKSS)
#include "dclink_ss.h"
#endif // MOTOR1_DCLINKSS

// solutions
#if !defined(__TMS320C28XX_CLA__)
#include "user.h"
#include "hal.h"
#endif

#if defined(SFRA_ENABLE)
#include "sfra_settings.h"
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
#include "step_response.h"
#endif  // STEP_RP_EN

#if defined(CMD_CAN_EN)
#include "communication.h"
#endif // CMD_CAN_EN

//*****************************************************************************
#define M_OVER_VOLTAGE_BIT          0x0001    // DC Bus Over Voltage Fault
#define M_UNDER_VOLTAGE_BIT         0x0002    // DC Bus Under Voltage Fault
#define M_MOTOR_OVER_TEMPER_BIT     0x0004    // Motor over temperature Fault
#define M_MODULE_OVER_TEMPER_BIT    0x0008    // Module over temperature Fault

#define M_MODULE_OVER_CURRENT_BIT   0x0010    // Hardware Over Current Fault
#define M_OVER_PEAK_CURRENT_BIT     0x0020    // internal CMPSS Over Current Fault
#define M_MOTOR_OVER_LOAD_BIT       0x0040    // Over Load Error
#define M_MOTOR_LOST_PHASE_BIT      0x0080    // Motor Lost Phase

#define M_CURRENT_UNBALANCE_BIT     0x0100    // Motor Phase Current Unbalance
#define M_MOTOR_STALL_BIT           0x0200    // Motor Stall
#define M_STARTUP_FAILE_BIT         0x0400    // Startup failed
#define M_MOTOR_OVER_SPEED_BIT      0x0800    // Motor Over Speed

#define M_RESERVE_12_BIT            0x1000    // Reserved for further use
#define M_RESERVE_13_BIT            0x2000    // Reserved for further use
#define M_CURRENT_OFFSET_BIT        0x4000    // Current offsets
#define M_VOLTAGE_OFFSET_BIT        0x8000    // voltage offsets

#define M_MASK_ALL_FAULT_BITS       0x0000
#define M_ENABLE_ALL_FAULT_BITS     0xFFFF


//
// Block all fault protection except current, voltage and temperature faults
//
#define MTR_FAULT_ENABLE_OC             M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT

#define MTR_FAULT_ENABLE_OC_OUV         M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT
//
// Enable all fault protection
//
#define MTR_FAULT_ENABLE_ALL            M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT               \
                                      + M_MODULE_OVER_CURRENT_BIT              \
                                      + M_OVER_PEAK_CURRENT_BIT                \
                                      + M_MOTOR_OVER_LOAD_BIT                  \
                                      + M_MOTOR_LOST_PHASE_BIT                 \
                                      + M_CURRENT_UNBALANCE_BIT                \
                                      + M_MOTOR_STALL_BIT                      \
                                      + M_STARTUP_FAILE_BIT                    \
                                      + M_MOTOR_OVER_SPEED_BIT                 \
                                      + M_CURRENT_OFFSET_BIT                   \
                                      + M_VOLTAGE_OFFSET_BIT

// Clear all fault protection except over/under voltage and offset error
//
#define MTR_FAULT_CLEAR                 M_OVER_VOLTAGE_BIT                     \
                                      + M_UNDER_VOLTAGE_BIT                    \
                                      + M_MOTOR_OVER_TEMPER_BIT                \
                                      + M_MODULE_OVER_TEMPER_BIT

#define MTR_FAULT_DISABLE_ALL           0x0000

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC
#else
#define MTR1_FAULT_MASK_SET             MTR_FAULT_ENABLE_OC_OUV
//#define MTR1_FAULT_MASK_SET           MTR_FAULT_ENABLE_ALL
#endif

//------------------------------------------------------------------------------
//
//! \brief Enumeration for the kit boards
//
typedef enum
{
    BOARD_BSXL8300RT_REVA  = 0,         //!< the board is BOOSTXL_8300RT, TBD
    BOARD_BSXL8320RS_REVA  = 1,         //!< the board is BOOSTXL_8320RS, TBD
    BOARD_BSXL8323RS_REVA  = 2,         //!< the board is BOOSTXL_8323RS, OK
    BOARD_BSXL8323RH_REVB  = 3,         //!< the board is BOOSTXL_8323RH, OK
    BOARD_BSXL8353RS_REVA  = 4,         //!< the board is BOOSTXL_8353RS, OK
    BOARD_BSXL3PHGAN_REVA  = 5,         //!< the board is BOOSTXL_3PHGAN, OK
    BOARD_HVMTRPFC_REV1P1  = 6,         //!< the board is HVMTRPFC_REV1P1, OK
    BOARD_BSXL8316RT_REVA  = 7          //!< the board is BOOSTXL_8316RT, OK

} Board_Kit_e;

//
//! \brief Enumeration for the estimator mode
//
#if defined(MOTOR1_ISBLDC) && (defined(MOTOR1_FAST) || \
    defined(MOTOR1_ESMO) || defined(MOTOR1_ENC) || defined(MOTOR1_HALL))
#error ISBLDC can't work with other estimaor simultaneously
#elif defined(MOTOR1_ENC) && defined(MOTOR1_HALL)
#error Can't support ENC and HALL simultaneously
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_HALL)
#error Can't support ESMO and HALL simultaneously
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0,             //!< FAST estimator
    ESTIMATOR_MODE_ENC   = 3              //!< Encoder
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0,             //!< FAST estimator
    ESTIMATOR_MODE_ESMO  = 1              //!< ESMO estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0,             //!< FAST estimator
    ESTIMATOR_MODE_HALL  = 5              //!< Hall sensor
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
typedef enum
{
    ESTIMATOR_MODE_ESMO  = 1,             //!< ESMO estimator
    ESTIMATOR_MODE_ENC   = 3              //!< Encoder
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_FAST)
typedef enum
{
    ESTIMATOR_MODE_FAST  = 0              //!< FAST estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ESMO)
typedef enum
{
    ESTIMATOR_MODE_ESMO  = 1              //!< ESMO estimator
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ENC)
typedef enum
{
    ESTIMATOR_MODE_ENC  = 3               //!< Encoder
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_HALL)
typedef enum
{
    ESTIMATOR_MODE_HALL  = 5              //!< Hall sensor
} ESTIMATOR_Mode_e;
#elif defined(MOTOR1_ISBLDC)
typedef enum
{
    ESTIMATOR_MODE_BINT  = 4              //!< BEMF Int
} ESTIMATOR_Mode_e;
#else
#error Not select a right estimator for this project
#endif  // MOTOR1_FAST->MOTOR1_ENC

//
//! \brief Enumeration for the using estimator algorithm
//
typedef enum
{
    EST_TYPE_FAST      = 0,         //!< the estimator is only FAST
    EST_TYPE_ESMO      = 1,         //!< the estimator is only ESMO
    EST_TYPE_ENC       = 2,         //!< the estimator is only ENC
    EST_TYPE_HALL      = 3,         //!< the estimator is only HALL
    EST_TYPE_FAST_ENC  = 4,         //!< the estimator is FAST and ENC
    EST_TYPE_FAST_ESMO = 5,         //!< the estimator is FAST and ESMO
    EST_TYPE_FAST_HALL = 6,         //!< the estimator is FAST and HALL
    EST_TYPE_ESMO_ENC  = 7,         //!< the estimator is ESMO and ENC
    EST_TYPE_ISBLDC    = 8          //!< the estimator is only ISBLDC
} EST_Type_e;

typedef enum
{
    FAST_TYPE_SOFTLIB_FLASH  = 0,   //!< the FAST software library and run in Flash
    FAST_TYPE_ROMLIB_FLASH   = 1,   //!< the FAST ROM library and run in Flash
    FAST_TYPE_ROMLIB_RAM     = 2,   //!< the FAST ROM library and run in RAM
    FAST_TYPE_NONFAST_FLASH  = 3,   //!< the non FAST and run in Flash
    FAST_TYPE_NONFAST_RAM    = 4    //!< the non FAST and run in RAM
} FASTLIB_Type_e;

typedef enum
{
    CURSEN_TYPE_SINGLE_SHUNT  = 0,   //!< the single shunt
    CURSEN_TYPE_THREE_SHUNT   = 1,   //!< the three shunt
    CURSEN_TYPE_INLINE_SHUNT  = 2,   //!< the inline shunt with ISO AMP
    CURSEN_TYPE_INLINE_HALL   = 3,   //!< the hall effect sensor
    CURSEN_TYPE_INLINE_SDFM   = 4    //!< the inline shunt with SDFM
} CURRENTSEN_Type_e;

#if defined(CMD_POT_EN)
typedef struct _CMDPOT_Vars_t_
{
    float32_t speedSet_Hz;
    float32_t speedMin_Hz;
    float32_t speedMax_Hz;
    float32_t speedConv_sf;
    uint16_t adcMin;
    uint16_t waitTimeCnt;
    uint16_t waitTimeSet;
    bool    flagCmdRun;
    bool    flagEnableCmd;
} CMDPOT_Vars_t;
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
typedef struct _CMDCAP_Vars_t_
{
    float32_t speedRef_Hz;
    float32_t freqScaler;       // Scaler converting 1/N CPU cycles
    float32_t speedSet_Hz;
    float32_t speedMin_Hz;
    float32_t speedMax_Hz;
    uint32_t  timeStamp;
    uint16_t  waitTimeCnt;
    uint16_t  waitTimeSet;
    bool      flagCmdRun;
    bool      flagEnableCmd;
} CMDCAP_Vars_t;
#endif // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
typedef struct _CMDSWITCH_Vars_t_
{
    uint16_t delayTimeSet;
    uint16_t highTimeCnt;
    uint16_t lowTimeCnt;
    bool     flagCmdRun;
    bool     flagEnablCmd;
} CMDSWITCH_Vars_t;
#endif  // CMD_SWITCH_EN

#if defined(CMD_CAN_EN)
typedef struct _CMDCAN_Vars_t_
{
    float32_t speedSet_Hz;
    bool      flagCmdRun;
    bool      flagEnablCmd;
    bool      flagEnablSyncLead;
} CMDCAN_Vars_t;
#endif // CMD_CAN_EN

//
//! \brief typedefs for the fault
//
typedef struct _FAULT_MTR_BITS_
{             // bits  description
    uint16_t overVoltage:1;         // 0  DC Bus Over Voltage Fault
    uint16_t underVoltage:1;        // 1  DC Bus Under Voltage Fault
    uint16_t motorOverTemp:1;       // 2  Motor over temperature Fault
    uint16_t moduleOverTemp:1;      // 3  Power module over temperature Fault

    uint16_t moduleOverCurrent:1;   // 4  Hardware Over Current Fault Flag
    uint16_t overPeakCurrent:1;     // 5  internal CMPSS Over Current Fault Flag
    uint16_t overLoad:1;            // 6  Over Load Error
    uint16_t motorLostPhase:1;      // 7  Motor Lost Phase

    uint16_t currentUnbalance:1;    // 8  Motor Phase Current imbalance
    uint16_t motorStall:1;          // 9  Motor Stall
    uint16_t startupFailed:1;       // 10 Startup failed
    uint16_t overSpeed:1;           // 11 Motor Over Speed

    uint16_t reserve12:1;           // 12 Reserved
    uint16_t reserve13:1;           // 13 Reserved
    uint16_t currentOffset:1;       // 14 Current offset check
    uint16_t voltageOffset:1;       // 15 voltage offset check
} FAULT_MTR_BITS;

typedef union _FAULT_MTR_REG_t
{
    uint16_t        all;
    FAULT_MTR_BITS  bit;
}FAULT_MTR_REG_t;

typedef struct _MOTOR_SetVars_t_
{
    float32_t Rr_Ohm;
    float32_t Rs_Ohm;
    float32_t Ls_d_H;
    float32_t Ls_q_H;
    float32_t flux_VpHz;
    float32_t flux_Wb;
    float32_t RoverL_rps;

    float32_t RsOnLine_Ohm;

    float32_t Kp_spd;
    float32_t Ki_spd;

    float32_t Kp_Id;
    float32_t Ki_Id;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Kp_fwc;
    float32_t Ki_fwc;
    float32_t angleFWCMax_rad;

    float32_t overModulation;
    float32_t RsOnLineCurrent_A;
    float32_t magneticCurrent_A;

    float32_t lostPhaseSet_A;
    float32_t unbalanceRatioSet;
    float32_t overLoadSet_W;
    float32_t stallCurrentSet_A;

    float32_t IsFailedChekSet_A;

    float32_t speedFailMaxSet_Hz;
    float32_t speedFailMinSet_Hz;

    float32_t toqueFailMinSet_Nm;

    float32_t maxPeakCurrent_A;
    float32_t overCurrent_A;
    float32_t currentInv_sf;

    float32_t overVoltageFault_V;
    float32_t overVoltageNorm_V;
    float32_t underVoltageFault_V;
    float32_t underVoltageNorm_V;

    uint16_t voltageFaultTimeSet;
    uint16_t overLoadTimeSet;
    uint16_t motorStallTimeSet;
    uint16_t unbalanceTimeSet;
    uint16_t lostPhaseTimeSet;
    uint16_t overSpeedTimeSet;
    uint16_t startupFailTimeSet;

    uint16_t overCurrentTimesSet;

#if defined(MOTOR1_FAST)
    uint16_t RsOnlineWaitTimeSet;
    uint16_t RsOnlineWorkTimeSet;
#endif  // MOTOR1_FAST

    uint16_t stopWaitTimeSet;
    uint16_t restartWaitTimeSet;
    uint16_t restartTimesSet;

    uint16_t  dacCMPValH;                   // 9
    uint16_t  dacCMPValL;                   // 10

} MOTOR_SetVars_t;

//! \brief Defines the MOTOR_SetVars_t handle
//!
typedef struct _MOTOR_SetVars_t_ *MOTORSETS_Handle;

//******************************************************************************
// typedefs
typedef struct _MOTOR_Vars_t_
{
    float32_t direction;                    // 1.0f->forward, -1.0f->reserve

    float32_t speedRef_Hz;
    float32_t speed_int_Hz;                 // Speed reference value, Hz
    float32_t speed_Hz;
    float32_t speed_rps;

    float32_t speedStart_Hz;
    float32_t speedForce_Hz;
    float32_t speedAbs_Hz;
    float32_t speedFilter_Hz;
    float32_t speedFlyingStart_Hz;

    float32_t accelerationMax_Hzps;
    float32_t accelerationStart_Hzps;

    float32_t angleFWC_rad;
    float32_t angleCurrent_rad;

    float32_t angleOffsetIPD_rad;
    float32_t angleDetectIPD_rad;

    float32_t Is_A;
    float32_t Vs_V;
    float32_t VsRef_pu;
    float32_t VsRef_V;
    float32_t oneOverDcBus_invV;    //!< the DC Bus inverse, 1/V

#if(DMC_BUILDLEVEL <= DMC_LEVEL_3)
    //!< the reference current on d&q rotation axis
    MATH_Vec2 Idq_set_A;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_3)

    float32_t IdRated_A;
    float32_t IsRef_A;
    float32_t IsSet_A;

    float32_t fluxCurrent_A;
    float32_t alignCurrent_A;
    float32_t startCurrent_A;
    float32_t maxCurrent_A;
    float32_t brakingCurrent_A;

    float32_t torque_Nm;

    MATH_Vec2 Vab_out_V;             // the output control voltage on alpha&beta axis

    MATH_Vec2 Vdq_out_V;            // the output control voltage on d&q axis

    MATH_Vec2 Vdq_offset_V;         // the output offset voltage on d&q axis

    MATH_Vec2 Iab_A;                // the alpha/beta current values, A

    MATH_Vec2 Vab_V;                // the alpha/beta current values, V

    // the d&q axis current are converter from 3-phase sampling input current of motor
    MATH_Vec2 Idq_in_A;

    // the reference current on d&q rotation axis
    MATH_Vec2 IdqRef_A;

    // the reference output current on d&q rotation axis
    MATH_Vec2 Idq_out_A;

    float32_t frswPos_sf;

    float32_t angleDelayed_sf;

    // the rotor angle compensation value
    float32_t angleDelta_rad;

    // the rotor angle delay compensation value
    float32_t angleComp_rad;

    // the rotor angle from Generator modules
    float32_t angleGen_rad;

    // the rotor angle from FOC modules
    float32_t angleFOC_rad;

    // the rotor angle from EST modules
    float32_t angleEST_rad;

    // the rotor angle from PLL modules
    float32_t anglePLL_rad;

    // the rotor angle from Encoder modules
    float32_t angleENC_rad;

    // the rotor angle from Hall Sensor
    float32_t angleHall_rad;

    // the rotor angle for braking
    float32_t angleBrake_rad;

    // the speed from EST module
    float32_t speedEST_Hz;

    // the speed from PLL module
    float32_t speedPLL_Hz;

    // the speed from Encoder module
    float32_t speedENC_Hz;

    // the speed from Hall Sensor
    float32_t speedHall_Hz;

    // the speed from ISBLDC module
    float32_t speedINT_Hz;

    float32_t VIrmsIsrScale;
    float32_t IrmsCalSF;
    float32_t VrmsCalSF;
    float32_t power_sf;

    float32_t IrmsCalSum[3];
    float32_t IrmsPrdSum[3];
    float32_t Irms_A[3];

    float32_t VrmsCalSum[3];
    float32_t VrmsPrdSum[3];
    float32_t Vrms_V[3];

    float32_t unbalanceRatio;
    float32_t powerActive_W;
    float32_t powerReal_W;

    uint32_t ISRCount;

    MOTORSETS_Handle motorSetsHandle;

    userParams_Handle   userParamsHandle;

    // the handle for the hardware abstraction layer to motor control
    HAL_MTR_Handle halMtrHandle;

#if defined(MOTOR1_FAST)
    // the handle for the estimator
    EST_Handle    estHandle;

    // the handle for the voltage Clarke transform
    CLARKE_Handle clarkeHandle_V;
#endif  // MOTOR1_FAST || MOTOR2_FAST

    // the handle for the current Clarke transform
    CLARKE_Handle clarkeHandle_I;

    // the handle for the inverse Park transform
    IPARK_Handle  iparkHandle_V;

    // the handle for the Park object
    PARK_Handle   parkHandle_I;

    // the handle for the Park object
    PARK_Handle   parkHandle_V;

    // the handle for the speed PI controller
    PI_Handle     piHandle_spd;

    // the handle for the Id PI controller
    PI_Handle     piHandle_Id;

    // the handle for the Iq PI controller
    PI_Handle     piHandle_Iq;

    // the handle for the speed reference trajectory
    TRAJ_Handle  trajHandle_spd;

    // the handle for the space vector generator
    SVGEN_Handle  svgenHandle;

#if defined(MOTOR1_OVM)
    // the space vector generator current object
    SVGENCURRENT_Handle svgencurrentHandle;
    MATH_Vec3 adcDataPrev;
    MATH_Vec3 pwmDataPrev;
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle;
    SVGENCURRENT_VmidShunt_e midVolShunt;
#endif  // MOTOR1_OVM

#if defined(MOTOR1_FWC)
    // the handle for the fwc PI controller
    PI_Handle    piHandle_fwc;
#endif  // MOTOR1_FWC || MOTOR2_FWC

#if defined(MOTOR1_ISBLDC)
    //!< the handle for the isbldc object
    ISBLDC_Handle isbldcHandle;

    //!< the handle for the rimpulse object
    RIMPULSE_Handle rimpulseHandle;

    //!< the handle for the mod6cnt object
    MOD6CNT_Handle mod6cntHandle;

    //!< the handle for the bldc object
    BLDC_Handle bldcHandle;
#endif  // MOTOR1_ISBLDC

#if defined(MOTOR1_ENC)
    //!< the handle for the enc object
    ENC_Handle encHandle;

    //!< the handle for the speedcalc object
    SPDCALC_Handle spdcalcHandle;
#endif  // MOTOR1_ENC

#if defined(MOTOR1_HALL)
    //!< the handle for the Hall object
    HALL_Handle hallHandle;
#endif  // MOTOR1_HALL

#if defined(MOTOR1_ESMO)
    //!< the handle for the speedfr object
    SPDFR_Handle spdfrHandle;

    //!< the handle for the esmo object
    ESMO_Handle esmoHandle;
#endif  // MOTOR1_ESMO

#if (DMC_BUILDLEVEL <= DMC_LEVEL_3) || defined(MOTOR1_VOLRECT) || \
               defined(MOTOR1_ESMO) || defined(MOTOR1_ENC)
    //!< the handles for Angle Generate for open loop control
    ANGLE_GEN_Handle angleGenHandle;
#endif  // DMC_BUILDLEVEL <= DMC_LEVEL_3 || MOTOR1_ESMO || MOTOR1_VOLRECT

#if defined(MOTOR1_MTPA)
    //!< the handle for the Maximum torque per ampere (MTPA)
    MTPA_Handle  mtpaHandle;

    float32_t angleMTPA_rad;                // 7
    float32_t mtpaKconst;                   // 8
    float32_t LsOnline_d_H;                 // 9
    float32_t LsOnline_q_H;                 // 10
    float32_t fluxOnline_Wb;                // 11
#endif  // MOTOR1_MTPA

#if defined(MOTOR1_SSIPD)
    SSIPD_Handle    ssipdHandle;
#endif  // MOTOR1_SSIPD

#if defined(MOTOR1_DCLINKSS)
    //!< the handle for single-shunt current reconstruction
    DCLINK_SS_Handle dclinkHandle;
#endif  // MOTOR1_DCLINKSS

#if defined(MOTOR1_VOLRECT)
    //!< the handle for the voltage reconstruct
    VOLREC_Handle volrecHandle;
#endif  // MOTOR1_VOLRECT

#if(DMC_BUILDLEVEL == DMC_LEVEL_2)
    //!< the handles for Vs per Freq for open loop control
    VS_FREQ_Handle VsFreqHandle;
#endif // (DMC_BUILDLEVEL == DMC_LEVEL_2)

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
    DRVICVARS_Handle drvVarsHandle;
#endif  // BSXL8320RS_REVA || BSXL8323RS_REVA ||
        // BSXL8353RS_REVA || BSXL8316RT_REVA
#if defined(MOTOR1_FAST)
    EST_InputData_t estInputData;
    EST_OutputData_t estOutputData;
#endif  // MOTOR1_FAST

#if defined(CMD_POT_EN)
    CMDPOT_Vars_t cmdPot;
#endif  // CMD_POT_EN

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
    CMDCAP_Vars_t cmdCAP;
#endif // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
    CMDSWITCH_Vars_t cmdSwtich;
#endif  // CMD_SWITCH_EN

#if defined(CMD_CAN_EN)
    CMDCAN_Vars_t cmdCAN;
#endif // CMD_CAN_EN

    uint16_t VIrmsIsrSet;
    uint16_t VIrmsIsrCnt;

    uint16_t overVoltageTimeCnt;
    uint16_t underVoltageTimeCnt;

    uint16_t overLoadTimeCnt;
    uint16_t motorStallTimeCnt;
    uint16_t unbalanceTimeCnt;
    uint16_t lostPhaseTimeCnt;
    uint16_t overSpeedTimeCnt;
    uint16_t startupFailTimeCnt;

    uint16_t overCurrentTimesCnt;

    uint16_t powerOnTimeCnt;
    uint16_t stopWaitTimeCnt;
    uint16_t restartTimesCnt;
    uint16_t startSumTimesCnt;

#ifdef BRAKE_ENABLE
    uint16_t brakingTimeDelay;
    uint16_t brakingTimeCnt;
#endif  // BRAKE_ENABLE

    FAULT_MTR_REG_t faultMtrNow;
    FAULT_MTR_REG_t faultMtrUse;
    FAULT_MTR_REG_t faultMtrMask;
    FAULT_MTR_REG_t faultMtrPrev;

    HAL_ADCData_t adcData;
    HAL_PWMData_t pwmData;

    uint16_t stateRunTimeCnt;

    uint16_t forceRunTimeDelay;
    uint16_t alignTimeDelay;
    uint16_t fwcTimeDelay;
    uint16_t flyingStartTimeDelay;

    uint16_t counterSpeed;
    uint16_t counterTrajSpeed;

    ESTIMATOR_Mode_e estimatorMode;
    SAMPLE_Mode_e sampleMode;
    MOTOR_Status_e motorState;
    MCTRL_State_e  mctrlState;

    FlyingStart_Mode_e flyingStartMode;
    BRAKE_Mode_e brakingMode;
    OPERATE_Mode_e operateMode;

    SVM_Mode_e svmMode;
    MotorNum_e motorNum;

#if defined(MOTOR1_FAST)
    EST_State_e estState;
    EST_Traj_State_e trajState;

    uint16_t RsOnlineTimeCnt;

    bool flagEnableMotorIdentify;

    bool flagEnableRsRecalc;
    bool flagEnablePowerWarp;
    bool flagBypassLockRotor;
    bool flagEnableRsOnLine;
    bool flagStartRsOnLine;
    bool flagRsOnLineContinue;
#endif  // MOTOR1_FAST

    bool flagEnableRestart;
    bool flagEnableRunAndIdentify;
    bool flagRunIdentAndOnLine;
    bool flagMotorIdentified;
    bool flagEnableForceAngle;
    bool flagEnableOffsetCalc;
    bool flagEnableAlignment;

    bool flagSetupController;
    bool flagEnableSpeedCtrl;
    bool flagEnableCurrentCtrl;

    bool flagEnableFlyingStart;
    bool flagStateFlyingStart;
    bool flagEnableBraking;

    bool flagEnableIPD;
    bool flagEnableFWC;
    bool flagEnableMTPA;
    bool flagUpdateMTPAParams;

    bool flagClearFaults;
    bool flagVIrmsCal;

    bool enableSpeedCtrl;
    bool enableCurrentCtrl;
}MOTOR_Vars_t;

//! \brief Defines the MOTOR_Vars_t handle
//!
typedef struct _MOTOR_Vars_t_ *MOTOR_Handle;

//*****************************************************************************
extern HAL_Handle    halHandle;
extern HAL_Obj       hal;

#if defined(SFRA_ENABLE)
extern float32_t   sfraNoiseId;
extern float32_t   sfraNoiseIq;
extern float32_t   sfraNoiseSpd;
extern float32_t   sfraNoiseOut;
extern float32_t   sfraNoiseFdb;
extern SFRATest_e  sfraTestLoop;
extern bool        sfraCollectStart;
#endif  // SFRA_ENABLE

//*****************************************************************************
// the function prototypes

//! \brief calculate motor over current threshold
static inline void calcMotorOverCurrentThreshold(MOTOR_Handle handle)
{
    MOTOR_SetVars_t *obj = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

    float32_t overCurrent_A;

    overCurrent_A = obj->overCurrent_A;

    if(overCurrent_A > obj->maxPeakCurrent_A)
    {
        overCurrent_A = obj->maxPeakCurrent_A;
    }

    int16_t cmpValue = (int16_t)(overCurrent_A * obj->currentInv_sf);

    obj->dacCMPValH = 2048 + cmpValue;
    obj->dacCMPValL = 2048 - cmpValue;

    return;
}

//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
extern void setupClarke_I(CLARKE_Handle handle, const uint16_t numCurrentSensors);

//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
extern void setupClarke_V(CLARKE_Handle handle, const uint16_t numVoltageSensors);

//! \brief  Update the controllers
static inline void updateControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

#if defined(MOTOR1_ISBLDC)
    if(obj->mctrlState >= MCTRL_CONT_RUN)
    {
        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);
    }
#elif defined(MOTOR1_FAST)   // !MOTOR1_ISBLDC
    if((obj->mctrlState >= MCTRL_CONT_RUN) && (obj->flagMotorIdentified == true))
    {
        // update the Id controller
        PI_setGains(obj->piHandle_Id, objSets->Kp_Id, objSets->Ki_Id);

        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);
    }
#else   // !MOTOR1_ISBLDC
    if(obj->mctrlState >= MCTRL_CONT_RUN)
    {
        // update the Id controller
        PI_setGains(obj->piHandle_Id, objSets->Kp_Id, objSets->Ki_Id);

        // update the Iq controller
        PI_setGains(obj->piHandle_Iq, objSets->Kp_Iq, objSets->Ki_Iq);

        // update the speed controller
        PI_setGains(obj->piHandle_spd, objSets->Kp_spd, objSets->Ki_spd);
    }
#endif  // !MOTOR1_ISBLDC

}


//! \brief  Get the controllers Parameters
static inline void getControllers(MOTOR_Handle handle)
{
    MOTOR_Vars_t *obj = (MOTOR_Vars_t *)handle;
    MOTOR_SetVars_t *objSets = (MOTOR_SetVars_t *)(handle->motorSetsHandle);

#if defined(MOTOR1_ISBLDC)
    // Get the Iq controller parameters
    objSets->Kp_Iq = PI_getKp(obj->piHandle_Iq);
    objSets->Ki_Iq = PI_getKi(obj->piHandle_Iq);

    // Get the speed controller parameters
    objSets->Kp_spd = PI_getKp(obj->piHandle_spd);
    objSets->Ki_spd = PI_getKi(obj->piHandle_spd);
#else   // !MOTOR1_ISBLDC
    // Get the Id controller parameters
    objSets->Kp_Id = PI_getKp(obj->piHandle_Id);
    objSets->Ki_Id = PI_getKi(obj->piHandle_Id);

    // Get the Iq controller parameters
    objSets->Kp_Iq = PI_getKp(obj->piHandle_Iq);
    objSets->Ki_Iq = PI_getKi(obj->piHandle_Iq);

    // Get the speed controller parameters
    objSets->Kp_spd = PI_getKp(obj->piHandle_spd);
    objSets->Ki_spd = PI_getKi(obj->piHandle_spd);
#endif  // !MOTOR1_ISBLDC
}

//! \brief  Sets up control parameters for stopping motor
extern void stopMotorControl(MOTOR_Handle handle);

//! \brief  Sets up control parameters for restarting motor
extern void restartMotorControl(MOTOR_Handle handle);

//! \brief  Resets motor control parameters for restarting motor
extern void resetMotorControl(MOTOR_Handle handle);

//! \brief  Sets up the current controllers
extern void setupCurrentControllers(MOTOR_Handle handle);

//! \brief  Sets up the controllers
extern void setupControllers(MOTOR_Handle handle);

//! \brief  Collect the current and voltage data to calculate the RMS
extern void collectRMSData(MOTOR_Handle handle);

//! \brief  Calculate the RMS data
extern void calculateRMSData(MOTOR_Handle handle);

//! \brief run motor monitor in main loop timer
extern void runMotorMonitor(MOTOR_Handle handle);

#if defined(MOTOR1_FAST) || defined(MOTOR2_FAST)
//! \brief Rs online calibration
extern void runRsOnLine(MOTOR_Handle handle);

//! \brief      Updates the global motor variables
//! \param[in]  estHandle   The estimator (EST) handle
extern void updateGlobalVariables(MOTOR_Handle handle);
#endif // MOTOR1_FAST || MOTOR2_FAST

//! \brief      Updates the FWC parameters
extern void updateFWCParams(MOTOR_Handle handle);

//! \brief      Updates the MTPA parameters
extern void updateMTPAParams(MOTOR_Handle handle);

#if defined(CMD_CAN_EN)
//! \brief     Update CAN communication data
//! \param[in] handle  The motor control object handle
extern void updateCANCmdFreq(MOTOR_Handle handle);
#endif // CMD_CAN_EN

#if defined(CMD_POT_EN)
//! \brief
extern void setExtCmdPotParams(MOTOR_Handle handle);

//! \brief
extern void updateExtCmdPotFreq(MOTOR_Handle handle);
#endif  // CMD_POT_EN

#if defined(CMD_CAP_EN)
//! \brief
extern void setExtCmdCapParams(MOTOR_Handle handle);

//! \brief
extern void updateExtCmdCapFreq(MOTOR_Handle handle, const uint32_t timeStamp);
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
//! \brief
extern void setExtCmdSwitchParams(MOTOR_Handle handle);

//! \brief
extern void updateCmdSwitch(MOTOR_Handle handle);

//! \brief
extern void outputCmdState(MOTOR_Handle handle);
#endif  // CMD_SWITCH_EN

#if defined(SFRA_ENABLE)
//------------------------------------------------------------------------------
// Using SFRA tool :
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
inline void injectSFRA(void)
{
    float32_t sfraNoiseInj_pu = 0.0f;

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;

    sfraNoiseInj_pu = SFRA_F32_inject(0.0);

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseId = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseIq = sfraNoiseInj_pu * USER_M1_ADC_FULL_SCALE_CURRENT_A;
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseSpd = sfraNoiseInj_pu * USER_MOTOR1_FREQ_MAX_HZ;
    }

    return;
}

//------------------------------------------------------------------------------
inline void collectSFRA(MOTOR_Handle handle)
{
    MOTOR_Vars_t *objMtr = (MOTOR_Vars_t *)handle;

    if(sfraTestLoop == SFRA_TEST_D_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[0] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_Q_AXIS)
    {
        sfraNoiseOut = objMtr->Vdq_out_V.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V);
        sfraNoiseFdb = objMtr->Idq_in_A.value[1] * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
    }
    else if(sfraTestLoop == SFRA_TEST_SPEEDLOOP)
    {
        sfraNoiseOut = objMtr->IsRef_A * (1.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A);
        sfraNoiseFdb = objMtr->speed_Hz * (1.0f / USER_MOTOR1_FREQ_MAX_HZ);
    }

    SFRA_F32_collect(&sfraNoiseOut, &sfraNoiseFdb);

    return;
}
//------------------------------------------------------------------------------

#endif  // SFRA_ENABLE

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

#endif // end of MOTOR_COMMON_H definition
