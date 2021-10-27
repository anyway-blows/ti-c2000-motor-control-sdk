//#############################################################################
//
// FILE:   userParams.h
//
// TITLE:  C28x InstaSPIN public interface for user initialization data for the
//         CTRL, DRV, and EST modules (floating point)
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

#ifndef USERPARAMS_H
#define USERPARAMS_H

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
//! \addtogroup USERPARAMS
//! @{
//
//*****************************************************************************

#include "libraries/math/include/math.h"

#include "motor.h"
#include "ctrl_states.h"


// **************************************************************************
// the defines

//*****************************************************************************
//
//! \brief Defines a structure for the user parameters
//
//*****************************************************************************
typedef struct _USER_Params_
{
    float32_t dcBus_nominal_V;            //!< Defines the nominal DC bus
                                        //!< voltage, V
    int_least16_t numIsrTicksPerCtrlTick;
                                        //!< Defines the number of Interrupt
                                        //!< Service Routine (ISR) clock ticks
                                        //!< per controller clock tick
    int_least16_t numIsrTicksPerTrajTick;
                                        //!< Defines the number of Interrupt
                                        //!< Service Routine (ISR) clock ticks
                                        //!< per controller clock tick
    int_least16_t numCtrlTicksPerCurrentTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per current controller
                                        //!< clock tick
    int_least16_t numCtrlTicksPerSpeedTick;
                                        //!< Defines the number of controller
                                        //!< clock ticks per speed controller
                                        //!< clock tick
    uint_least8_t numCurrentSensors;    //!< Defines the number of current
                                        //!< sensors
    uint_least8_t numVoltageSensors;    //!< Defines the number of voltage
                                        //!< sensors
    float32_t systemFreq_MHz;             //!< Defines the system clock
                                        //!< frequency, MHz
    float32_t pwmPeriod_usec;             //!< Defines the Pulse Width Modulation
                                        //!< (PWM) period, usec
    float32_t voltage_sf;                 //!< Defines the voltage scale factor
                                        //!< for the system
    float32_t current_sf;                 //!< Defines the current scale factor
                                        //!< for the system
    float32_t offsetPole_rps;             //!< Defines the pole location for the
                                        //!< voltage and current offset
                                        //!< estimation, rad/sec
    float32_t speedPole_rps;              //!< Defines the pole location for the
                                        //!< speed control filter, rad/sec
    float32_t maxVsMag_pu;                //!< Defines the maximum Vs magnitude
                                        //!< in per units
    MOTOR_Type_e motor_type;              //!< Defines the motor type
    uint_least16_t motor_numPolePairs;  //!< Defines the number of pole pairs
                                        //!< for the motor
    uint_least16_t motor_numEncSlots;   //!< Defines the number of encoder
                                        //!< slots if quadrature encoder is
                                        //!< connected
    float32_t motor_ratedFlux_Wb;         //!< Defines the rated flux of the
                                        //!< motor, Wb
    float32_t maxCurrent_A;               //!< Defines the maximum current value,
                                        //!< A
    float32_t IdRated_A;                  //!< Defines the Id rated current
                                        //!< value, A
    float32_t Vd_sf;                      //!< Defines the Vd scale factor to
                                        //!< prevent a Vd only component for
                                        //!< the Vdq vector
    float32_t maxVsMag_V;                 //!< Defines the maximum stator voltage
                                        //!< magnitude, V
    float32_t BWc_rps;                    //!< Defines the bandwidth of the
                                        //!< current controllers, rad/sec
    float32_t BWdelta;                    //!< Defines the bandwidth scaling to
                                        //!< maximize phase margin
    float32_t Kctrl_Wb_p_kgm2;            //!< Defines the speed controller
                                        //!< constant, Wb/(kg*m^2)
    float32_t angleDelayed_sf_sec;        //!< Defines the scale factor for
                                        //!< computing the angle considering
                                        //!< system delay, sec
    float32_t ctrlFreq_Hz;                //!< Defines the controller frequency,
                                        //!< Hz
    float32_t trajFreq_Hz;                //!< Defines the trajectory frequency,
                                        //!< Hz
    float32_t ctrlPeriod_sec;             //!< Defines the controller execution
                                        //!< period, sec
    float32_t maxAccel_Hzps;              //!< Defines the maximum acceleration
                                        //!< for the speed profiles, Hz/sec
    float32_t RoverL_Kp_sf;               //!< Defines the Kp scale factor used
                                        //!< during R/L, pu
    float32_t RoverL_min_rps;             //!< Defines the minimum estimated R/L
                                        //!< value allowed, rad/sec
    float32_t RoverL_max_rps;             //!< Defines the maximum estimated R/L
                                        //!< value allowed, rad/sec
    float32_t Ls_d_H;                     //!< Defines the default stator
                                        //!< inductance in the direct
                                        //!< direction, H
    float32_t Ls_q_H;                     //!< Defines the default stator
                                        //!< inductance in the quadrature
                                        //!< direction, H
    float32_t Rr_Ohm;                     //!< Defines the default rotor
                                        //!< resistance value, Ohm
    float32_t Rs_Ohm;                     //!< Defines the default stator
                                        //!< resistance value, Ohm
    // BELOW can be added userself varaibles
    float32_t maxFrequency_Hz;          //!< Defines the maximum frequency value,
                                        //!< Hz
} USER_Params;


// **************************************************************************
// the functions


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

#endif // end of USERPARAMS_H definition
