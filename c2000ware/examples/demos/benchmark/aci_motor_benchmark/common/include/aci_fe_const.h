//#############################################################################
//
// FILE:   aci_fe_const.h
//
// TITLE:  ACI Motor Control Flux Estimator(FE) Constants.
//
//#############################################################################
// $TI Release: v3.04.00.00 $
// $Release Date: 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _ACI_FE_CONST_H
#define _ACI_FE_CONST_H

//
// Included Files
//
#include "parameter.h"

//
// Data structure for maintaining Flux estimator parameters
//
typedef struct
{ 
    float  Rs; 				//!< Input: Stator resistance (ohm) 
    float  Rr;				//!< Input: Rotor resistance (ohm) 
    float  Ls;				//!< Input: Stator inductance (H) 	  			      
    float  Lr;				//!< Input: Rotor inductance (H) 			
    float  Lm;				//!< Input: Magnetizing inductance (H) 
    float  Ib; 				//!< Input: Base phase current (amp) 
    float  Vb;				//!< Input: Base phase voltage (volt) 
    float  Ts;				//!< Input: Sampling period in sec   
    float  K1;				//!< Output: constant using in rotor flux calculation  
    float  K2;				//!< Output: constant using in rotor flux calculation  
    float  K3;				//!< Output: constant using in rotor flux calculation  
    float  K4;				//!< Output: constant using in stator current calculation  
    float  K5;				//!< Output: constant using in stator current calculation  
    float  K6;				//!< Output: constant using in stator current calculation  
    float  K7;				//!< Output: constant using in stator current calculation  
    float  K8;				//!< Output: constant using in torque calculation
} ACIFE_Const;
																																																																																																																																																																																																								

//
// Default initializer for the ACIFE_CONST object.
// These ACI parameters are based on WEG 1-hp induction motor
//
#define ACIFE_CONST_DEFAULTS {RS_VALUE, RR_VALUE, LS_VALUE, LR_VALUE, LM_VALUE, \
                             BASE_CURRENT, BASE_VOLTAGE, \
                             SAMPLING_PERIOD, \
 		          	         0,0,0,0,0,0,0,0, }

//
// Function Prototypes
//
void ACIFE_Const_calc(ACIFE_Const *);

#endif //_ACI_FE_CONST_H
