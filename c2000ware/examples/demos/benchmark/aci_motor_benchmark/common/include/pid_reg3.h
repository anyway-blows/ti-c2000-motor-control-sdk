//#############################################################################
//
// FILE:   pid_reg3.h
//
// TITLE:  PID Controller
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

#ifndef _PID_REG3_H
#define _PID_REG3_H

//
// Include files
//
#include "math.h"
#include "IQmathLib.h"

//
// PID Controller Parameters
//
typedef struct
{  
	_iq  pid_ref_reg3;   	//!< Input: Reference input 
	_iq  pid_fdb_reg3;   	//!< Input: Feedback input 
	_iq  e_reg3;			//!< Variable: Error   
	_iq  Kp_reg3;			//!< Parameter: Proportional gain 
	_iq  up_reg3;			//!< Variable: Proportional output 
	_iq  ui_reg3;			//!< Variable: Integral output   
	_iq  ud_reg3;			//!< Variable: Derivative output   	
	_iq  uprsat_reg3; 	    //!< Variable: Pre-saturated output 
	_iq  pid_out_max;		//!< Parameter: Maximum output   
	_iq  pid_out_min;		//!< Parameter: Minimum output   
	_iq  pid_out_reg3;   	//!< Output: PID output   
	_iq  saterr_reg3;		//!< Variable: Saturated difference
	_iq  Ki_reg3;			//!< Parameter: Integral gain  
	_iq  Kc_reg3;			//!< Parameter: Integral correction gain 
	_iq  Kd_reg3; 		    //!< Parameter: Derivative gain  
	_iq  up1_reg3;		    //!< History: Previous proportional output
} PIDREG3;	            

//
// Default initializer for the PIDREG3 object.
//
#define PIDREG3_DEFAULTS { 0, \
                           0, \
                           0, \
                           _IQ(1.3), \
                           0, \
                           0, \
                           0, \
                           0, \
                           _IQ(1), \
                           _IQ(-1), \
                           0, \
                           0, \
                           _IQ(0.02), \
                           _IQ(0.5), \
                           _IQ(1.05), \
                           0, \
              			 }

//
// PID Controller calculations
//
static inline void PIDREG3_calc(PIDREG3 *v)
{	
    _iq temp_out;

    v->e_reg3 = v->pid_ref_reg3 - v->pid_fdb_reg3;
    
    v->up_reg3 = _IQmpy(v->Kp_reg3,v->e_reg3);
    
    v->uprsat_reg3 = v->up_reg3 + v->ui_reg3 + v->ud_reg3;
    
    temp_out = v->uprsat_reg3;
    temp_out = (temp_out > v->pid_out_max)? v->pid_out_max:temp_out;
    temp_out = (temp_out < v->pid_out_min)? v->pid_out_min:temp_out;
    v->pid_out_reg3 = temp_out;
      
    v->saterr_reg3 = temp_out - v->uprsat_reg3;
    
    v->ui_reg3 = v->ui_reg3 + _IQmpy(v->Ki_reg3,v->up_reg3) + 
                _IQmpy(v->Kc_reg3,v->saterr_reg3);
    
    v->ud_reg3 = _IQmpy(v->Kd_reg3,(v->up_reg3 - v->up1_reg3));
     
    v->up1_reg3 = v->up_reg3;
}

#endif //_PID_REG3_H
