//#############################################################################
//
// FILE:   user.c
//
// TITLE:  C28x InstaSPIN function for setting initialization data for the
//         CTRL, HAL, and EST modules (floating point)
//
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

#include "user.h"

#ifdef __TMS320C28XX_CLA__
#pragma CODE_SECTION(USER_setParams,"Cla1Prog2");
#endif

#ifdef _F28002x_
#pragma CODE_SECTION(USER_setParams, "user_code");
#endif

//*****************************************************************************
//
// USER_setParams
//
//*****************************************************************************
void USER_setParams(USER_Params *pUserParams)
{
    pUserParams->dcBus_nominal_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

    pUserParams->numIsrTicksPerCtrlTick = 1;
    pUserParams->numIsrTicksPerTrajTick = 1;

    pUserParams->numCtrlTicksPerCurrentTick = USER_NUM_ISR_TICKS_PER_CURRENT_TICK;
    pUserParams->numCtrlTicksPerSpeedTick = USER_NUM_ISR_TICKS_PER_SPEED_TICK;

    pUserParams->numCurrentSensors = USER_NUM_CURRENT_SENSORS;
    pUserParams->numVoltageSensors = USER_NUM_VOLTAGE_SENSORS;

    pUserParams->systemFreq_MHz = USER_SYSTEM_FREQ_MHz;

    pUserParams->voltage_sf = USER_VOLTAGE_SF;

    pUserParams->current_sf = USER_CURRENT_SF;


    pUserParams->offsetPole_rps = USER_OFFSET_POLE_rps;

    pUserParams->speedPole_rps = USER_SPEED_POLE_rps;


    pUserParams->maxVsMag_pu = USER_MAX_VS_MAG_PU;

    pUserParams->motor_type = USER_MOTOR_TYPE;

    pUserParams->motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;
    pUserParams->motor_numEncSlots = USER_MOTOR_NUM_ENC_SLOTS;

    pUserParams->motor_ratedFlux_Wb = USER_MOTOR_RATED_FLUX_VpHz / MATH_TWO_PI;

    pUserParams->maxCurrent_A = USER_MOTOR_MAX_CURRENT_A;

    pUserParams->Vd_sf = USER_VD_SF;
    pUserParams->maxVsMag_V = USER_NOMINAL_DC_BUS_VOLTAGE_V;

    /*LDRA_INSPECTED 139 S MR12 14.3 ""Its a compile time flag providing user a
       choice to use or bypass motor identification; so OK"*/
    if(USER_BYPASS_MOTOR_ID == 1)
    {

#if (USER_MOTOR == Estun_EMJ_04APB22_A)
        // Estun
        pUserParams->BWc_rps = MATH_TWO_PI * 100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0f * numPolesPairs * rotorFlux_Wb / (2.0f * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);
#elif (USER_MOTOR == Estun_EMJ_04APB22_B)
        // Estun
        pUserParams->BWc_rps = MATH_TWO_PI * 100.0f;
        pUserParams->BWdelta = (float32_t)8.0;

        // 3.0f * numPolesPairs * rotorFlux_Wb / (2.0f * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Marathon_N56PNRA10102)
        // Marathon
        pUserParams->BWc_rps = MATH_TWO_PI * 100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Anaheim_BLWS235D_160V)
        // Marathon
        pUserParams->BWc_rps = MATH_TWO_PI * 100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Anaheim_BLWS235D)
        // Anaheim_BLWS235D
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0f * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Anaheim_BLY172S_24V)
        // Anaheim_BLY172S_24V
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0f * numPolesPairs * rotorFlux_Wb / (2.0f * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);
#elif (USER_MOTOR == Teknic_M2310PLN04K)
        // M2310PLN04K
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)200.0f;
        pUserParams->BWdelta = (float32_t)20.0f;

        // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Marathon_5K33GN2A)
         pUserParams->BWc_rps = MATH_TWO_PI * 100.0f;
         pUserParams->BWdelta = (float32_t)8.0f;

         // 3.0 * numPolesPairs * rotorFlux_Wb / (2.0 * J_kg_m2);
         pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                        pUserParams->motor_numPolePairs *
                                        pUserParams->motor_ratedFlux_Wb /
                                     (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#elif (USER_MOTOR == Teknic_MxxxxPLNxxK)
        // MxxxxPLNxxK
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0f * numPolesPairs * rotorFlux_Wb / (2.0f * J_kg_m2);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       pUserParams->motor_ratedFlux_Wb /
                                    (float32_t) (2.0f * USER_MOTOR_INERTIA_Kgm2);

#else
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0f * pUserParams->motor_numPolePairs * 0.1f / (2.0f * 0.00001f);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       (float32_t)(0.001f) /
                                       (float32_t)(2.0f * 0.000001f);
#endif

    }
    else
    {
        pUserParams->BWc_rps = MATH_TWO_PI * (float32_t)100.0f;
        pUserParams->BWdelta = (float32_t)8.0f;

        // 3.0 * pUserParams->motor_numPolePairs * 0.1f / (2.0f * 0.00001f);
        pUserParams->Kctrl_Wb_p_kgm2 = (float32_t)3.0f *
                                       pUserParams->motor_numPolePairs *
                                       (float32_t)(0.001f) /
                                       (float32_t)(2.0f * 0.000001f);
    }

    pUserParams->angleDelayed_sf_sec = (float32_t)0.5f * USER_CTRL_PERIOD_sec;

    pUserParams->ctrlFreq_Hz = USER_CTRL_FREQ_Hz;
    pUserParams->trajFreq_Hz = USER_TRAJ_FREQ_Hz;

    pUserParams->pwmPeriod_usec = USER_PWM_PERIOD_usec;
    pUserParams->ctrlPeriod_sec = USER_CTRL_PERIOD_sec;

    pUserParams->maxAccel_Hzps = USER_MAX_ACCEL_Hzps;

    pUserParams->RoverL_Kp_sf = USER_R_OVER_L_KP_SF;
    pUserParams->RoverL_min_rps = MATH_TWO_PI * (float32_t)5.0f;
    pUserParams->RoverL_max_rps = MATH_TWO_PI * (float32_t)5000.0f;

    pUserParams->Ls_d_H = (float32_t)USER_MOTOR_Ls_d_H;
    pUserParams->Ls_q_H = (float32_t)USER_MOTOR_Ls_q_H;

    pUserParams->Rr_Ohm = (float32_t)USER_MOTOR_Rr_Ohm;

    pUserParams->Rs_Ohm = (float32_t)USER_MOTOR_Rs_Ohm;


    return;
} // end of USER_setParams() function


// end of file
