//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
//     Copyright (C) 2020 Texas Instruments Incorporated -
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef _USER_H_
#define _USER_H_

//! \file   solutions/drv8312_c2_kit/f28004x/drivers/user.h
//! \brief  Contains the user related definitions
//!


// **************************************************************************
// the includes

// modules
#include "userParams_updated.h"

//!
//!
//! \defgroup USER USER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

#define BOOSTX_to_J1_J2     0
#define BOOSTX_to_J5_J6     1

#define BOOST_to_LPD        BOOSTX_to_J1_J2
//#define BOOST_to_LPD        BOOSTX_to_J5_J6

//! \brief Defines the nominal DC bus voltage, V
//!
#define USER_NOMINAL_DC_BUS_VOLTAGE_V          ((float32_t)(24.0f))


//! \brief Defines the maximum voltage at the AD converter
//!
// Full scale voltage of AD converter, not the current voltage
#define USER_ADC_FULL_SCALE_VOLTAGE_V         ((float32_t)(57.52f))


//! \brief Defines the maximum current at the AD converter
//!
// DRV8323 motor control kit
#define USER_ADC_FULL_SCALE_CURRENT_A         ((float32_t)(23.57f))


//! \brief Defines the analog voltage filter pole location, Hz
//!
#define USER_VOLTAGE_FILTER_POLE_Hz            ((float32_t)(338.36f)) //((float32_t)(714.14f))


//! \brief ADC current offsets for A, B, and C phases
#define   IA_OFFSET_A    (11.78f)  // ~=0.5*USER_ADC_FULL_SCALE_CURRENT_A
#define   IB_OFFSET_A    (11.78f)  // ~=0.5*USER_ADC_FULL_SCALE_CURRENT_A
#define   IC_OFFSET_A    (11.78f)  // ~=0.5*USER_ADC_FULL_SCALE_CURRENT_A


#define   VA_OFFSET_V    (0.983381569f)   // ~=0.5
#define   VB_OFFSET_V    (0.983381569f)   // ~=0.5
#define   VC_OFFSET_V    (0.983381569f)   // ~=0.5


//! \brief Vbus used to calculate the voltage offsets A, B, and
//!
// =0.5*USER_NOMINAL_DC_BUS_VOLTAGE_VC
#define   VBUS_OFFSET_V  (0.5*USER_ADC_FULL_SCALE_VOLTAGE_V)


//! \brief Defines the maximum negative current to be applied in Id reference
//!
#define USER_MAX_NEGATIVE_ID_REF_CURRENT_A    ((float32_t)(-2.0f))


//! \brief Defines the number of pwm clock ticks per isr clock tick
//!        Note: Valid values are 1, 2 or 3 only
#define USER_NUM_PWM_TICKS_PER_ISR_TICK         (1)

//! \brief Defines the number of ISR clock ticks per current controller clock tick
//!
#define USER_NUM_ISR_TICKS_PER_CURRENT_TICK     (1)


//! \brief Defines the number of ISR clock ticks per speed controller clock tick
//!
#define USER_NUM_ISR_TICKS_PER_SPEED_TICK      (10)


//! \brief Defines the number of current sensors
//!
#define USER_NUM_CURRENT_SENSORS               (3)


//! \brief Defines the number of voltage sensors
//!
#define USER_NUM_VOLTAGE_SENSORS               (3)

//! \brief Defines the system maximum input frequency, MHz
//!
#define USER_MAXIMUM_SCALE_FREQ_Hz      ((float32_t)(1000.0f))

//! \brief Defines the system clock frequency, MHz
//!
#define USER_SYSTEM_FREQ_MHz            ((float32_t)(100.0f))


//! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
//!
//#define USER_PWM_FREQ_kHz          ((float32_t)(5.0f))              //5KHz PWM Frequnecy
#define USER_PWM_FREQ_kHz            ((float32_t)(10.0f))              //10KHz PWM Frequnecy
//#define USER_PWM_FREQ_kHz          ((float32_t)(12.0f))              //12KHz PWM Frequnecy
//#define USER_PWM_FREQ_kHz          ((float32_t)(15.0f))              //15KHz PWM Frequnecy
//#define USER_PWM_FREQ_kHz          ((float32_t)(20.0f))            //20KHz PWM Frequnecy

//! \brief Defines the Pulse Width Modulation (PWM) period, usec
//!
#define USER_PWM_PERIOD_usec       ((float32_t)1000.0/USER_PWM_FREQ_kHz)


//! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
//!
#define USER_ISR_FREQ_Hz           (USER_PWM_FREQ_kHz * (float32_t)1000.0f / (float32_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)


//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_ISR_PERIOD_usec       (USER_PWM_PERIOD_usec * (float32_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)


#ifdef _VSF_EN_
//! \brief Defines the timer frequency for estimator, kHz
//!
#define USER_EST_FREQ_kHz       ((float32_t)(20.0f))


//! \brief Defines the timer frequency for estimator, Hz
//!
#define USER_EST_FREQ_Hz        (USER_EST_FREQ_kHz * (float32_t)1000.0f)


//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_EST_PERIOD_usec    ((float32_t)1000000.0f/USER_EST_FREQ_Hz)


//! \brief Defines the timer frequency for controller, Hz
//!
#define USER_CTRL_FREQ_Hz        (USER_EST_FREQ_Hz)


//! \brief Defines the controller execution period, usec
//!
#define USER_CTRL_PERIOD_usec   (USER_EST_PERIOD_usec)


//! \brief Defines the controller execution period, sec
//!
#define USER_CTRL_PERIOD_sec    ((float32_t)USER_CTRL_PERIOD_usec/(float32_t)1000000.0)


//! \brief Defines the timer frequency for trajectory, Hz
//!
#define USER_TRAJ_FREQ_Hz       (USER_EST_FREQ_Hz)
#else

//! \brief Defines the timer frequency for estimator, kHz
//!
#define USER_EST_FREQ_Hz        (USER_ISR_FREQ_Hz)


//! \brief Defines the Interrupt Service Routine (ISR) period, usec
//!
#define USER_EST_PERIOD_usec    (USER_ISR_PERIOD_usec)


//! \brief Defines the timer frequency for controller, Hz
//!
#define USER_CTRL_FREQ_Hz        (USER_ISR_FREQ_Hz)


//! \brief Defines the controller execution period, usec
//!
#define USER_CTRL_PERIOD_usec   (USER_ISR_PERIOD_usec)


//! \brief Defines the controller execution period, sec
//!
#define USER_CTRL_PERIOD_sec    ((float32_t)USER_CTRL_PERIOD_usec/(float32_t)1000000.0)

//! \brief Defines the timer frequency for trajectory, Hz
//!
#define USER_TRAJ_FREQ_Hz       (USER_ISR_FREQ_Hz)
#endif  // _VPF_EN_

//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_VD_SF                 ((float32_t)(0.95f))


//! \brief Defines the voltage scale factor for the system
//!
#define USER_VOLTAGE_SF               (USER_ADC_FULL_SCALE_VOLTAGE_V / (float32_t)4096.0)            // 12 bit ADC, 2^12 = 4096
#define USER_DCBUS_VOLTAGE_SF         (USER_ADC_FULL_SCALE_DCBUS_VOLTAGE_V / (float32_t)4096.0)      // 12 bit ADC, 2^12 = 4096

//! \brief Defines the current scale factor for the system
//!
#define USER_CURRENT_SF               (USER_ADC_FULL_SCALE_CURRENT_A / (float32_t)4096.0)    // 12 bit ADC, 2^12 = 4096


//! \brief Defines the pole location for the DC bus filter, rad/sec
//!
#define USER_DCBUS_POLE_rps            ((float32_t)(100.0f))


//! \brief Defines the pole location for the voltage and current offset estimation, rad/s
//!
#define USER_OFFSET_POLE_rps            ((float32_t)(20.0f))


//! \brief Defines the pole location for the speed control filter, rad/sec
//!
#define USER_SPEED_POLE_rps        ((float32_t)(100.0f))


//! \brief Defines the analog voltage filter pole location, rad/s
//!
#define USER_VOLTAGE_FILTER_POLE_rps  (MATH_TWO_PI * USER_VOLTAGE_FILTER_POLE_Hz)


//! \brief Defines the maximum Vs magnitude in per units allowed
//!        This value sets the maximum magnitude for the output of the Id and
//!        Iq PI current controllers. The Id and Iq current controller outputs
//!        are Vd and Vq. The relationship between Vs, Vd, and Vq is:
//!        Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
//!        equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
//!        so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
//!
//!        Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
//!        SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
//!        is needed for this scenario.
//!
//!        Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
//!        with a peak at 100% duty cycle.  Current reconstruction
//!        will be needed for this scenario (Lab08).
//!
//!        Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
//!        voltage waveform.  Current reconstruction will be needed
//!        for this scenario (Lab08).
//!
//!        For space vector over-modulation, see lab08 for details on
//!        system requirements that will allow the SVM generator to
//!        go all the way to trapezoidal.
//!
//#define USER_MAX_VS_MAG_PU            (0.66f)
//#define USER_MAX_VS_MAG_PU            (0.57f)
#define USER_MAX_VS_MAG_PU              (0.5f)


//! \brief Defines the reference Vs magnitude in per units allowed
//! \      Set the value equal from 0.5 to 0.95 of the maximum Vs magnitude
#define USER_VS_REF_MAG_PU              ((float32_t)(0.8f) * USER_MAX_VS_MAG_PU)


//! \brief Defines the R/L excitation frequency, Hz
//!
#define USER_R_OVER_L_EXC_FREQ_Hz  ((float32_t)(300.0f))


//! \brief Defines the R/L Kp scale factor, pu
//! \brief Kp used during R/L is USER_R_OVER_L_KP_SF * USER_NOMINAL_DC_BUS_VOLTAGE_V / USER_MOTOR_MAX_CURRENT_A;
//!
#define USER_R_OVER_L_KP_SF        ((float32_t)(0.02f))


//! \brief Defines maximum acceleration for the estimation speed profiles, Hz/sec
//!
#define USER_MAX_ACCEL_Hzps        ((float32_t)(2.0f))


//! \brief Defines the IdRated delta to use during estimation
//!
#define USER_IDRATED_DELTA_A       ((float32_t)(0.0001f))


//! \brief Defines the forced angle frequency, Hz
//!
#define USER_FORCE_ANGLE_FREQ_Hz            ((float32_t)(1.0f))


//! \brief Defines the fraction of IdRated to use during inductance estimation
//!
#define USER_IDRATED_FRACTION_FOR_L_IDENT    ((float32_t)(0.5f))


//! \brief Defines the fraction of SpeedMax to use during inductance estimation
//!
#define USER_SPEEDMAX_FRACTION_FOR_L_IDENT  ((float32_t)(1.0f))


//! \brief Defines the Power Warp gain for computing Id reference
//! \brief If motor parameters are known, set this gain to:
//! \brief USER_PW_GAIN = SQRT(1.0 + USER_MOTOR_Rr_Ohm / USER_MOTOR_Rs_Ohm)
//!
#define USER_PW_GAIN                        ((float32_t)(1.0f))

//! \brief Defines the scale factor for the flux estimation
//! the default value is 1.0f, change the value between 0.1f and 1.25f
//!
#define USER_EST_FLUX_HF_SF                 ((float32_t)(0.125f))


//! \brief Defines the scale factor for the frequency estimation
//! the default value is 1.0f, change the value between 0.5f and 1.5f
//!
#define USER_EST_FREQ_HF_SF                 ((float32_t)(1.0f))


//! \brief Defines the scale factor for the bemf estimation
//! the default value is 1.0f, change the value between 0.50f and 1.25f
//!
#define USER_EST_BEMF_HF_SF                 ((float32_t)(0.50f))


//! \brief A flag to bypass motor identification (1/0 : true/false)
//!
#define USER_BYPASS_MOTOR_ID       (1)          // No motor parameters identification
//#define USER_BYPASS_MOTOR_ID        (0)           // Do motor parameters identification

#define USER_ENABLE_MOTOR_ID        0
#define USER_DISABLE_MOTOR_ID       1

//! brief Define the Kp gain for Field Weakening Control
#define USER_FWC_KP                 0.02f

//! brief Define the Ki gain for Field Weakening Control
#define USER_FWC_KI                 0.0001f

//! brief Define the maximum current vector angle for Field Weakening Control
#define USER_FWC_MAX_ANGLE          (float32_t)(-75.0f)                              // degree
#define USER_FWC_MAX_ANGLE_RAD      USER_FWC_MAX_ANGLE*MATH_PI/((float32_t)(180.0f))   // rad

//! brief Define the minimum current vector angle for Field Weakening Control
#define USER_FWC_MIN_ANGLE          (float32_t)(0.0f)                                // degree
#define USER_FWC_MIN_ANGLE_RAD      USER_FWC_MIN_ANGLE*MATH_PI/((float32_t)(180.0f))   // rad


// temporary define
#define TBD  NULL


//============================================================================================
// Motor defines

//************** Motor Parameters **************

// PMSM motors
#define Estun_EMJ_04APB22_A         101
#define Estun_EMJ_04APB22_B         102

#define Teknic_M2310PLN04K          121
#define Anaheim_BLY172S_24V         122
#define Anaheim_BLY341S_48V         123
#define Anaheim_BLY341S_24V         124

#define Traxxas_Velineon_380        131
#define Traxxas_Velineon_3500       132
#define Pacific_Scientific          133
#define Regal_Beloit_5SME39DL0756   134
#define AutoRadiatorFan             135

#define philips_respirator          141
#define tekin_redline_4600KV        142

// ACIM motors
#define Marathon_5K33GN2A           201
#define Marathon_56H17T2011A        202
#define Dayton_3N352C               203

#define my_motor_1                  301


//#define USER_MOTOR Estun_EMJ_04APB22_A
//#define USER_MOTOR Estun_EMJ_04APB22_B

#define USER_MOTOR Teknic_M2310PLN04K
//#define USER_MOTOR Anaheim_BLY172S_24V
//#define USER_MOTOR Anaheim_BLY341S_48V
//#define USER_MOTOR Anaheim_BLY341S_24V

//#define USER_MOTOR Traxxas_Velineon_380
//#define USER_MOTOR Traxxas_Velineon_3500
//#define USER_MOTOR Pacific_Scientific
//#define USER_MOTOR Regal_Beloit_5SME39DL0756
//#define USER_MOTOR AutoRadiatorFan

//#define USER_MOTOR philips_respirator
//#define USER_MOTOR tekin_redline_4600KV

//#define USER_MOTOR Marathon_5K33GN2A
//#define USER_MOTOR Marathon_56H17T2011A
//#define USER_MOTOR Dayton_3N352C

//#define USER_MOTOR my_motor_1
//#define USER_MOTOR my_motor_2

#if (USER_MOTOR == Estun_EMJ_04APB22_A)
#define USER_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (0.0)
#define USER_MOTOR_Rs_Ohm                 (2.303403)
#define USER_MOTOR_Ls_d_H                 (0.008464367)
#define USER_MOTOR_Ls_q_H                 (0.008464367)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.3821270569)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (3.82)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (2500.0)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR_VOLT_MAX_V             (200.0)         // Volt

#elif (USER_MOTOR == Estun_EMJ_04APB22_B)
#define USER_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (0.0)

// HV Kit with external OPA
#define USER_MOTOR_Rs_Ohm                 (2.98774099)
#define USER_MOTOR_Ls_d_H                 (0.008926632)
#define USER_MOTOR_Ls_q_H                 (0.008926632)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.445965141)

#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (3.82)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (2500.0)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR_VOLT_MAX_V             (200.0)         // Volt


#elif (USER_MOTOR == Teknic_M2310PLN04K)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.389923662)
#define USER_MOTOR_Ls_d_H                 (0.000186627527)
#define USER_MOTOR_Ls_q_H                 (0.000186627527)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.0410001911)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.5)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (6.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (1000)
#define USER_MOTOR_INERTIA_Kgm2           (7.06154e-06)

#define USER_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_MOTOR_RATED_SPEED_KRPM       (3.0)

#define USER_MOTOR_FREQ_MIN_HZ            (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (600.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (20.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt


#elif (USER_MOTOR == Anaheim_BLY172S_24V)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.417878777f)
#define USER_MOTOR_Ls_d_H                 (0.0007190173f)
#define USER_MOTOR_Ls_q_H                 (0.0007190173f)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.0327013217f)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.5f)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.5f)
#define USER_MOTOR_MAX_CURRENT_A          (5.0f)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0f)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_MOTOR_INERTIA_Kgm2           (7.06154e-06f)

//motorVars.Rr_Ohm    float   0.0               0x000002F4@Data
//motorVars.Rs_Ohm    float   0.427027732       0x000002F6@Data
//motorVars.Ls_d_H    float   0.000657772063    0x000002FA@Data
//motorVars.Ls_q_H    float   0.000657772063    0x000002FC@Data
//motorVars.flux_VpHz float   0.036639031       0x00000300@Data

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_MOTOR == Anaheim_BLY341S_48V)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.463800967)
#define USER_MOTOR_Ls_d_H                 (0.00114538975)
#define USER_MOTOR_Ls_q_H                 (0.00114538975)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.0978558362)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (2.5)
#define USER_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_MOTOR_MAX_CURRENT_A          (20.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_MOTOR == Anaheim_BLY341S_24V)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.4110007)
#define USER_MOTOR_Ls_d_H                 (0.0007092811)
#define USER_MOTOR_Ls_q_H                 (0.0007092811)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.03279636)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.5)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (10.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_MOTOR == Traxxas_Velineon_380)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (1)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (TBD)
#define USER_MOTOR_Ls_d_H                 (TBD)
#define USER_MOTOR_Ls_q_H                 (TBD)
#define USER_MOTOR_RATED_FLUX_VpHz        (TBD)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (4.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-0.5)
#define USER_MOTOR_MAX_CURRENT_A          (10.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == Traxxas_Velineon_3500)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (1)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.01822988)
#define USER_MOTOR_Ls_d_H                 (8.322238e-06)
#define USER_MOTOR_Ls_q_H                 (8.322238e-06)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.010249538)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (2.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == Pacific_Scientific)
#define USER_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (0.0)
#define USER_MOTOR_Rs_Ohm                 (0.9428042)
#define USER_MOTOR_Ls_d_H                 (0.002196057)
#define USER_MOTOR_Ls_q_H                 (0.002196057)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.3481677)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (8.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == Regal_Beloit_5SME39DL0756)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (3)
#define USER_MOTOR_Rr_Ohm                 (0.0)
#define USER_MOTOR_Rs_Ohm                 (4.581007)
#define USER_MOTOR_Ls_d_H                 (0.03727356)
#define USER_MOTOR_Ls_q_H                 (0.03727356)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.6589699)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (2.6)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == AutoRadiatorFan)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (0.0)
#define USER_MOTOR_Rs_Ohm                 (0.0)
#define USER_MOTOR_Ls_d_H                 (0.0)
#define USER_MOTOR_Ls_q_H                 (0.0)
#define USER_MOTOR_RATED_FLUX_VpHz        (TBD)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (10.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (50.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == philips_respirator)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (1)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.1690236)
#define USER_MOTOR_Ls_d_H                 (0.0001557156)
#define USER_MOTOR_Ls_q_H                 (0.0001557156)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.011538831)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == tekin_redline_4600KV)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (0.0181193)
#define USER_MOTOR_Ls_d_H                 (8.180002e-06)
#define USER_MOTOR_Ls_q_H                 (8.180002e-06)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.0041173688)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (3.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-3.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (60.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == Marathon_5K33GN2A)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_MOTOR_Rr_Ohm                 (5.508003)
#define USER_MOTOR_Rs_Ohm                 (10.71121)
#define USER_MOTOR_Ls_d_H                 (0.05296588)
#define USER_MOTOR_Ls_q_H                 (0.05296588)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.8165*220.0/60.0)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (1.378)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR_MAX_CURRENT_A          (3.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_MOTOR_VOLT_MAX_V             (200.0)         // Volt


#elif (USER_MOTOR == Marathon_56H17T2011A)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_MOTOR_Rr_Ohm                 (5.159403)
#define USER_MOTOR_Rs_Ohm                 (7.924815)
#define USER_MOTOR_Ls_d_H                 (0.03904648)
#define USER_MOTOR_Ls_q_H                 (0.03904648)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.8165*220.0/60.0)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (0.9941965)
#define USER_MOTOR_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR_MAX_CURRENT_A          (2.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == Dayton_3N352C)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_MOTOR_Rr_Ohm                 (2.428799)
#define USER_MOTOR_Rs_Ohm                 (2.863202)
#define USER_MOTOR_Ls_d_H                 (0.02391323)
#define USER_MOTOR_Ls_q_H                 (0.02391323)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR_MAX_CURRENT_A          (3.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_MOTOR_NUM_ENC_SLOTS          (NULL)

#elif (USER_MOTOR == my_motor_1)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (NULL)
#define USER_MOTOR_Ls_d_H                 (NULL)
#define USER_MOTOR_Ls_q_H                 (NULL)
#define USER_MOTOR_RATED_FLUX_VpHz        (NULL)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (2.0)
#define USER_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_MOTOR == my_motor_2)
#define USER_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_MOTOR_Rr_Ohm                 (NULL)
#define USER_MOTOR_Rs_Ohm                 (NULL)
#define USER_MOTOR_Ls_d_H                 (NULL)
#define USER_MOTOR_Ls_q_H                 (NULL)
#define USER_MOTOR_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_MOTOR_RES_EST_CURRENT_A      (0.5)
#define USER_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)

// Number of lines on the motor's quadrature encoder
#define USER_MOTOR_NUM_ENC_SLOTS          (1000)

#define USER_MOTOR_FREQ_MIN_HZ             (5.0)           // Hz
#define USER_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz

#define USER_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#else
#error No motor type specified
#endif


//! \brief Defines the maximum current slope for Id trajectory
//!
#define USER_MAX_CURRENT_DELTA_A        (USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz)


//! \brief Defines the maximum current slope for Id trajectory during power warp mode
//!
#define USER_MAX_CURRENT_DELTA_PW_A    (0.3 * USER_MOTOR_RES_EST_CURRENT_A / USER_ISR_FREQ_Hz)


#ifndef USER_MOTOR
#error Motor type is not defined in user.h
#endif

#ifndef USER_MOTOR_TYPE
#error The motor type is not defined in user.h
#endif

#ifndef USER_MOTOR_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user.h
#endif

#ifndef USER_MOTOR_Rr_Ohm
#error The rotor resistance is not defined in user.h
#endif

#ifndef USER_MOTOR_Rs_Ohm
#error The stator resistance is not defined in user.h
#endif

#ifndef USER_MOTOR_Ls_d_H
#error The direct stator inductance is not defined in user.h
#endif

#ifndef USER_MOTOR_Ls_q_H
#error The quadrature stator inductance is not defined in user.h
#endif

#ifndef USER_MOTOR_RATED_FLUX_VpHz
#error The rated flux of motor is not defined in user.h
#endif

#ifndef USER_MOTOR_MAGNETIZING_CURRENT_A
#error The magnetizing current is not defined in user.h
#endif

#ifndef USER_MOTOR_RES_EST_CURRENT_A
#error The resistance estimation current is not defined in user.h
#endif

#ifndef USER_MOTOR_IND_EST_CURRENT_A
#error The inductance estimation current is not defined in user.h
#endif

#ifndef USER_MOTOR_MAX_CURRENT_A
#error The maximum current is not defined in user.h
#endif

#ifndef USER_MOTOR_FLUX_EXC_FREQ_Hz
#error The flux excitation frequency is not defined in user.h
#endif

#if ((USER_NUM_CURRENT_SENSORS < 2) || (USER_NUM_CURRENT_SENSORS > 3))
#error The number of current sensors must be 2 or 3
#endif

#if (USER_NUM_VOLTAGE_SENSORS != 3)
#error The number of voltage sensors must be 3
#endif


// **************************************************************************
// the typedefs


// **************************************************************************
// the globals


// **************************************************************************
// the functions

//! \brief      Sets the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParams(USER_Params *pUserParams);


//! \brief      Sets the private user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParams_priv(USER_Params *pUserParams);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup
#endif // end of _USER_H_ definition

