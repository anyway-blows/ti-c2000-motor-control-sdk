//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef SERVO_MAIN_H
#define SERVO_MAIN_H

//**************************************************************************
//! \file   solutions/servo_drive_with_can/common/include/servo_main.h
//! \brief  header file to be included in all labs
//!
//**************************************************************************


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
//! \addtogroup SERVO MAIN
//! @{
//
//*****************************************************************************

//
// modules
//
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/include/CLAmath.h"
#else
#include "libraries/math/include/math.h"
#include <math.h>
#endif

#if defined(CMD_CAN_ENABLE)
#include "communication.h"
#endif  // CMD_CAN_ENABLE

#include "userParams_updated.h"

#include "ctrl.h"
#include "clarke.h"
#include "filter_fo.h"
#include "ipark.h"
#include "park.h"
#include "pi.h"
#include "svgen.h"
#include "svgen_current.h"
#include "traj.h"
#include "fwc.h"
#include "mtpa.h"
#include "vs_freq.h"

//
// Only for lab03&lab04--position open loop control
//
#include "angle_gen.h"

#ifdef STEP_RESPONSE_EN
#include "graph.h"
#endif  // STEP_RESPONSE_EN

#include "qep_sensor.h"

#include "cpu_time.h"

//
// solutions
//
#if !defined(__TMS320C28XX_CLA__)
#include "user.h"
#include "hal.h"
#endif

#ifdef DRV8301_SPI
#include "drv8301.h"
#endif

#ifdef DRV8320_SPI
#include "drv8320.h"
#endif


//=============================================================================
// Incremental Build options for System check-out
//=============================================================================
#define DMC_LEVEL_1     1   // Open loop control to verify integrity of user hardware and calibration of feedbacks
#define DMC_LEVEL_2     2   // Closing speed loop with step response Generation & Graphing for Controller Tuning
#define DMC_LEVEL_3     3   // Closing speed loop using QEP encoder with CAN communication

#define DMC_BUILDLEVEL  DMC_LEVEL_3


#define LED_BLINK_FREQ_Hz           (1.0f)    // 1Hz

#define FLYINGSTART_DELAY_TIME      300       // pwm cycles

#define OVER_VOLTAGE_BITS           0x0001    // DC Bus Over Voltage Fault
#define UNDER_VOLTAGE_BITS          0x0002    // DC Bus Under Voltage Fault
#define MOTOR_OVER_TEMPER_BITS      0x0004    // Motor over temperature Fault
#define MODULE_OVER_TEMPER_BITS     0x0008    // Module over temperature Fault
#define MODULE_OVER_CURRENT_BITS    0x0010    // Hardware Over Current Fault
#define OVER_RMS_CURRENT_BITS       0x0020    // Motor Phase Over Current Fault
#define OVER_PEAK_CURRENT_BITS      0x0040    // Software Over Current Fault
#define MULTI_OVER_CURRENT_BITS     0x0080    // Multiple times over current
#define MOTOR_LOST_PHASE_BITS       0x0100    // Motor Lost Phase
#define CURRENT_UNBALANCE_BITS      0x0200    // Motor Phase Current Unbalance
#define MOTOR_STALL_BITS            0x0400    // Motor Stall
#define MOTOR_OVER_SPEED_BITS       0x0800    // Motor Over Speed
#define STARTUP_FAILE_BITS          0x1000    // Startup failed
#define MOTOR_OVER_LOAD_BITS        0x2000    // Over Load Error
#define CONTROLLER_ERROR_BITS       0x4000    // FAST Controller error
#define RESERVE_15_BITS             0x8000    // Reserved for further use

#define MASK_ALL_FAULT_BITS         0x0000
#define ENABLE_ALL_FAULT_BITS       0xFFFF

//
// Block all fault protection except over current and over voltage faults
//
#define FAULT_MASK_OC_OV            OVER_VOLTAGE_BITS                          \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + MASK_ALL_FAULT_BITS

//
// Block all fault protection except current, voltage and temperature faults
//
#define FAULT_MASK_OC_FU_OT         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

//
// Block some fault protection
//
#define FAULT_MASK_OC_FV_OL         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + OVER_RMS_CURRENT_BITS                      \
                                  + OVER_PEAK_CURRENT_BITS                     \
                                  + MULTI_OVER_CURRENT_BITS                    \
                                  + CURRENT_UNBALANCE_BITS                     \
                                  + MOTOR_OVER_LOAD_BITS                       \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

//
// Enable all fault protection
//
#define FAULT_MASK_ALL_FLTS         OVER_VOLTAGE_BITS                          \
                                  + UNDER_VOLTAGE_BITS                         \
                                  + MOTOR_OVER_TEMPER_BITS                     \
                                  + MODULE_OVER_TEMPER_BITS                    \
                                  + MODULE_OVER_CURRENT_BITS                   \
                                  + OVER_RMS_CURRENT_BITS                      \
                                  + OVER_PEAK_CURRENT_BITS                     \
                                  + MULTI_OVER_CURRENT_BITS                    \
                                  + MOTOR_LOST_PHASE_BITS                      \
                                  + CURRENT_UNBALANCE_BITS                     \
                                  + MOTOR_STALL_BITS                           \
                                  + MOTOR_OVER_SPEED_BITS                      \
                                  + STARTUP_FAILE_BITS                         \
                                  + MOTOR_OVER_LOAD_BITS                       \
                                  + CONTROLLER_ERROR_BITS                      \
                                  + MASK_ALL_FAULT_BITS

// Clear all fault protection except over/under voltage and offset error
//
#define  FAULT_CLEAR_NVS               OVER_VOLTAGE_BITS                       \
                                      + UNDER_VOLTAGE_BITS                     \
                                      + MOTOR_OVER_TEMPER_BITS                 \
                                      + MODULE_OVER_TEMPER_BITS

//
//! \brief typedefs for the fault
//
typedef struct _FAULT_BITS_
{             // bits  description
    uint16_t overVoltage:1;         // 0  DC Bus Over Voltage Fault
    uint16_t underVoltage:1;        // 1  DC Bus Under Voltage Fault
    uint16_t motorOverTemp:1;       // 2  Motor over temperature Fault
    uint16_t moduleOverTemp:1;      // 3  Power module over temperature Fault

    uint16_t moduleOverCurrent:1;   // 4  Hardware Over Current Fault Flag
    uint16_t overRmsCurrent:1;      // 5  Motor Phase Over Current Fault
    uint16_t overPeakCurrent:1;     // 6  Software Over Current Fault Flag
    uint16_t multiOverCurrent:1;    // 7  Multiple times over current

    uint16_t motorLostPhase:1;      // 8  Motor Lost Phase
    uint16_t currentUnbalance:1;    // 9  Motor Phase Current Unbalance
    uint16_t motorStall:1;          // 10 Motor Stall
    uint16_t overSpeed:1;           // 11 Motor Over Speed

    uint16_t startupFailed:1;       // 12 Startup failed
    uint16_t overLoad:1;            // 13 Over Load Error
    uint16_t controllerError:1;     // 14 FAST Controller error
    uint16_t reserve15:1;           // 15 Reserved
} FAULT_BITS;

typedef union _FAULT_REG_t {
    uint16_t            all;
    FAULT_BITS          bit;
}FAULT_REG_t;


//
//! \brief Enumeration for the kit boards
//
typedef enum
{
    BOARD_BSXL8320RS_REVA      = 0,          //!< the board is BOOSTXL_8320RS
    BOARD_BSXL8323RS_REVA      = 1,          //!< the board is BOOSTXL_8323RS
    BOARD_BSXL3PHGAN_REVA      = 2           //!< the board is BOOSTXL_3PhGaN
} Board_Kit_e;

//
//! \brief Initialization values of global variables
//
#define MOTOR_VARS_INIT {                                                      \
    false, /* flagEnableSys */                                                 \
    false, /* flagRunIdentAndOnLine */                                         \
    false, /* flagSetupController */                                           \
    true,  /* flagEnableOffsetcalc */                                          \
                                                                               \
    true,  /* flagEnableSpeedCtrl */                                           \
    true,  /* flagEnableCurrentCtrl */                                         \
    false, /* flagStateMotorRunning */                                         \
    MOTORCTRL_MODE_SPEED,  /* motorCtrlMode */                                 \
                                                                               \
    CTRL_STATE_IDLE, /* ctrlState */                                           \
    BOARD_BSXL8320RS_REVA,  /* boardKit */                                     \
                                                                               \
    false, /* flagRunCmdCAN */                                                 \
    10.0f,  /* speedSet_Hz */                                                  \
                                                                               \
    10.0f,  /* speedRef_Hz */                                                  \
    0.300f, /* speedRef_krpm */                                                \
    0.0f,   /* speedTraj_Hz */                                                 \
    0.0f,   /* speed_Hz */                                                     \
    0.0f,   /* speed_krpm */                                                   \
    0.0f,   /* speedFdbAbs_Hz */                                               \
    10.0f,  /* accelerationMax_Hzps */                                         \
                                                                               \
    0.1f,  /* Kp_spd */                                                        \
    0.01f, /* Ki_spd */                                                        \
                                                                               \
    0.5f,  /* Kp_Id */                                                         \
    0.01f, /* Ki_Id */                                                         \
                                                                               \
    0.5f,  /* Kp_Iq */                                                         \
    0.01f, /* Ki_Iq */                                                         \
                                                                               \
    0.0f,  /* Vs_V */                                                          \
    0.0f,  /* VdcBus_V */                                                      \
                                                                               \
    0.0f,  /* Is_A */                                                          \
    0.0f,  /* IsRef_A */                                                       \
    (USER_MOTOR_MAX_CURRENT_A * 0.5f),  /* overCurrent_A */                    \
                                                                               \
    {IA_OFFSET_A, IB_OFFSET_A, IC_OFFSET_A}, /* offsets_I_A */                 \
    {VA_OFFSET_V, VB_OFFSET_V, VC_OFFSET_V}, /* offsets_V_V */                 \
                                                                               \
    1.0f,  /* Vbus_sf */                                                       \
    0.0f,  /* VbusFilter_V */                                                  \
                                                                               \
    0,  /* pwmISRCount */                                                      \
    0,  /* mainLoopCount */                                                    \
    0,  /* timerCnt_1ms */                                                     \
                                                                               \
    2048 + 1024 + 512 + 256, /* dacValH */                                     \
    2048 - 1024 - 512 - 256, /* dacValL */                                     \
    2048,  /* dacaVal */                                                       \
    2048,  /* dacbVal */                                                       \
                                                                               \
    0,  /* faultNow */                                                         \
    0,  /* faultUse */                                                         \
    0,  /* faultOld */                                                         \
    0,  /* faultMask */                                                        \
    0   /* flagClearFaults */                                                  \
}

//
//!  \brief typedefs for motorVars
//
typedef struct _MOTOR_Vars_t_
{
    bool flagEnableSys;
    bool flagRunIdentAndOnLine;
    bool flagSetupController;
    bool flagEnableOffsetCalc;

    bool flagEnableSpeedCtrl;
    bool flagEnableCurrentCtrl;
    bool flagStateMotorRunning;

    MotorCtrl_Mode_e motorCtrlMode;
    CTRL_State_e ctrlState;
    Board_Kit_e boardKit;

    bool flagRunCmdCAN;
    float32_t speedSetCAN_Hz;

    float32_t speedRef_Hz;
    float32_t speedRef_krpm;
    float32_t speedTraj_Hz;
    float32_t speed_Hz;
    float32_t speed_krpm;
    float32_t speedFdbAbs_Hz;
    float32_t accelerationMax_Hzps;

    float32_t Kp_spd;
    float32_t Ki_spd;

    float32_t Kp_Id;
    float32_t Ki_Id;

    float32_t Kp_Iq;
    float32_t Ki_Iq;

    float32_t Vs_V;
    float32_t VdcBus_V;

    float32_t Is_A;
    float32_t IsRef_A;

    float32_t overCurrent_A;

    MATH_Vec3 offsets_I_A;
    MATH_Vec3 offsets_V_V;

    float32_t Vbus_sf;
    float32_t VbusFilter_V;

    uint32_t pwmISRCount;
    uint32_t mainLoopCount;
    uint32_t timerCnt_1ms;

    uint16_t  dacValH;
    uint16_t  dacValL;
    uint16_t  dacaVal;
    uint16_t  dacbVal;

    FAULT_REG_t faultNow;
    FAULT_REG_t faultUse;
    FAULT_REG_t faultOld;
    FAULT_REG_t faultMask;
    bool        flagClearFaults;
}MOTOR_Vars_t;


extern volatile MOTOR_Vars_t motorVars;
extern USER_Params   userParams;

extern PI_Handle     piHandle_Id;
extern PI_Handle     piHandle_Iq;
extern PI_Handle     piHandle_spd;

extern TRAJ_Handle   trajHandle_spd;

extern HAL_ADCData_t adcData;
extern HAL_PWMData_t pwmData;

extern MATH_Vec2 Idq_in_A;
extern MATH_Vec2 Vab_out_V;
extern MATH_Vec2 Vdq_out_V;

extern QEP_SENSOR_Obj qep_sensor;

//
// the function prototypes
//

//
//! \brief The main interrupt service (ISR) routine
//
extern __interrupt void mainISR(void);
extern __interrupt void canISR(void);

//
//! \brief runs offset calculation using filters
//
extern void runOffsetsCalculation(void);


//! \brief calculate motor over current threshold
static inline void calcMotorOverCurrentThreshold(void)
{
    float32_t overCurrent_A;

    overCurrent_A = motorVars.overCurrent_A;

    if(motorVars.overCurrent_A > (USER_ADC_FULL_SCALE_CURRENT_A * 0.45f))
    {
        overCurrent_A = USER_ADC_FULL_SCALE_CURRENT_A * 0.45f;
    }

    int16_t cmpValue = (int16_t)(overCurrent_A * (4096.0f / USER_ADC_FULL_SCALE_CURRENT_A));

    motorVars.dacValH = 2048 + cmpValue;
    motorVars.dacValL = 2048 - cmpValue;

    return;
}

//
//! \brief     Sets the number of current sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
//
static inline void
setupClarke_I(CLARKE_Handle handle, const uint_least8_t numCurrentSensors)
{
    float32_t alpha_sf, beta_sf;

    //
    // initialize the Clarke transform module for current
    //
    if(3 == numCurrentSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else if(2 == numCurrentSensors)
    {
        alpha_sf = 1.0f;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    //
    // set the parameters
    //
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numCurrentSensors);

    return;
} // end of setupClarke_I() function

//
//! \brief     Sets the number of voltage sensors
//! \param[in] handle             The Clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
//
static inline void
setupClarke_V(CLARKE_Handle handle, const uint_least8_t numVoltageSensors)
{
    float32_t alpha_sf, beta_sf;

    //
    // initialize the Clarke transform module for voltage
    //
    if(3 == numVoltageSensors)
    {
        alpha_sf = MATH_ONE_OVER_THREE;
        beta_sf = MATH_ONE_OVER_SQRT_THREE;
    }
    else
    {
        alpha_sf = 0.0f;
        beta_sf = 0.0f;
    }

    //
    // set the parameters
    //
    CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
    CLARKE_setNumSensors(handle, numVoltageSensors);

    return;
} // end of setupClarke_V() function

//
//! \brief  Update the controllers
//
static inline void updateControllers(void)
{
    //
    // update the Id controller
    //
    PI_setGains(piHandle_Id, motorVars.Kp_Id, motorVars.Ki_Id);

    //
    // update the Iq controller
    //
    PI_setGains(piHandle_Iq, motorVars.Kp_Iq, motorVars.Ki_Iq);

    //
    // update the speed controller
    //
    PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);

    return;
}

//
//! \brief  Get the controllers Parameters
//
static inline void getControllers(void)
{
    //
    // Get the Id controller parameters
    //
    motorVars.Kp_Id = PI_getKp(piHandle_Id);
    motorVars.Ki_Id = PI_getKi(piHandle_Id);

    //
    // Get the Iq controller parameters
    //
    motorVars.Kp_Iq = PI_getKp(piHandle_Iq);
    motorVars.Ki_Iq = PI_getKi(piHandle_Iq);

    //
    // Get the speed controller parameters
    //
    motorVars.Kp_spd = PI_getKp(piHandle_spd);
    motorVars.Ki_spd = PI_getKi(piHandle_spd);

    return;
}

//
//! \brief  Sets up the current controllers
//
static inline void setupCurrentControllers(void)
{
    float32_t RoverL_Kp_sf = userParams.RoverL_Kp_sf;
    float32_t dcBus_nominal_V = userParams.dcBus_nominal_V;
    float32_t maxCurrent_A = userParams.maxCurrent_A;
    float32_t RoverL_min_rps = userParams.RoverL_min_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams.numCtrlTicksPerCurrentTick /
                    userParams.ctrlFreq_Hz;
    float32_t outMax_V = userParams.Vd_sf * userParams.maxVsMag_V;

    float32_t Kp = RoverL_Kp_sf * dcBus_nominal_V / maxCurrent_A;
    float32_t Ki = RoverL_min_rps * currentCtrlPeriod_sec;

    //
    // set the Id controller
    //
    PI_setGains(piHandle_Id, Kp, Ki);
    PI_setUi(piHandle_Id, 0.0f);
    PI_setRefValue(piHandle_Id, 0.0f);
    PI_setFbackValue(piHandle_Id, 0.0f);
    PI_setFfwdValue(piHandle_Id, 0.0f);
    PI_setMinMax(piHandle_Id, -outMax_V, outMax_V);

    //
    // set the Iq controller
    //
    PI_setGains(piHandle_Iq, Kp, Ki);
    PI_setUi(piHandle_Iq, 0.0f);
    PI_setRefValue(piHandle_Iq, 0.0f);
    PI_setFbackValue(piHandle_Iq, 0.0f);
    PI_setFfwdValue(piHandle_Iq, 0.0f);
    PI_setMinMax(piHandle_Iq, -outMax_V, outMax_V);

    return;
} // end of setupCurrentControllers() function


//
//! \brief  Sets up the controllers
//
static inline void setupControllers(void)
{
    float32_t Ls_d_H = userParams.Ls_d_H;
    float32_t Ls_q_H = userParams.Ls_q_H;
    float32_t Rs_Ohm = userParams.Rs_Ohm;
    float32_t RdoverLd_rps = Rs_Ohm / Ls_d_H;
    float32_t RqoverLq_rps = Rs_Ohm / Ls_q_H;
    float32_t BWc_rps = userParams.BWc_rps;
    float32_t currentCtrlPeriod_sec =
                (float32_t)userParams.numCtrlTicksPerCurrentTick /
                userParams.ctrlFreq_Hz;
    float32_t outMax_V = userParams.Vd_sf * userParams.maxVsMag_V;

    float32_t Kp_Id = Ls_d_H * BWc_rps;
    float32_t Ki_Id = RdoverLd_rps * currentCtrlPeriod_sec;

    float32_t Kp_Iq = Ls_q_H * BWc_rps;
    float32_t Ki_Iq = RqoverLq_rps * currentCtrlPeriod_sec;

    //
    // set the Id controller
    //
    PI_setGains(piHandle_Id, Kp_Id, Ki_Id);
    PI_setUi(piHandle_Id, 0.0f);
    PI_setRefValue(piHandle_Id, 0.0f);
    PI_setFbackValue(piHandle_Id, 0.0f);
    PI_setFfwdValue(piHandle_Id, 0.0f);
    PI_setMinMax(piHandle_Id, -outMax_V, outMax_V);

    //
    // set the Iq controller
    //
    PI_setGains(piHandle_Iq, Kp_Iq, Ki_Iq);
    PI_setUi(piHandle_Iq, 0.0f);
    PI_setRefValue(piHandle_Iq, 0.0f);
    PI_setFbackValue(piHandle_Iq, 0.0f);
    PI_setFfwdValue(piHandle_Iq, 0.0f);
    PI_setMinMax(piHandle_Iq, 0.0f, 0.0f);

#if(!defined(USER_MOTOR_INERTIA_Kgm2))
    //
    // set the speed controller
    //
    PI_setGains(piHandle_spd, motorVars.Kp_spd, motorVars.Ki_spd);
#else
    float32_t speedCtrlPeriod_sec =
        (float32_t)userParams.numCtrlTicksPerSpeedTick /
        userParams.ctrlFreq_Hz;

    float32_t BWdelta = userParams.BWdelta;
    float32_t Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                              userParams.motor_numPolePairs *
                              userParams.motor_ratedFlux_Wb /
                              (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

    float32_t Kp_spd =
            MATH_TWO_PI * BWc_rps / (BWdelta * Kctrl_Wb_p_kgm2);

    float32_t Ki_spd =
            BWc_rps * speedCtrlPeriod_sec / (BWdelta * BWdelta);

    //
    // set the speed controller
    //
    PI_setGains(piHandle_spd, Kp_spd, Ki_spd);
#endif

    //
    // set the speed controller
    //
    PI_setUi(piHandle_spd, 0.0f);
    PI_setRefValue(piHandle_spd, 0.0f);
    PI_setFbackValue(piHandle_spd, 0.0f);
    PI_setFfwdValue(piHandle_spd, 0.0f);
    PI_setMinMax(piHandle_spd,
                 (-userParams.maxCurrent_A),
                 userParams.maxCurrent_A);

    //
    // set the Id, Iq and speed controller parameters to motorVars
    //
    getControllers();

    return;
} // end of setupControllers() function

//
//! \brief      Updates the global motor variables
//
static inline void updateGlobalVariables(void)
{

    motorVars.speed_Hz = qep_sensor.SpeedHz;

    motorVars.speedFdbAbs_Hz = fabsf(motorVars.speed_Hz);

    motorVars.speed_krpm = motorVars.speed_Hz *
            (60.0f / 1000.0f / USER_MOTOR_NUM_POLE_PAIRS);

    motorVars.VdcBus_V = adcData.dcBus_V;

    // compute the vector voltage
    motorVars.Vs_V = sqrt((Vdq_out_V.value[0] * Vdq_out_V.value[0]) +
                          (Vdq_out_V.value[1] * Vdq_out_V.value[1]));

    return;
} // end of updateGlobalVariables() function


//
//! \brief      Updates the receive and transmit datas from CAN
//
extern void updateCANCmdFreq(void);


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

#endif // end of SERVO_MAIN_H definition
