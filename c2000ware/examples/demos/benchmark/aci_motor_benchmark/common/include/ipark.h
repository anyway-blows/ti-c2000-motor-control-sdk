//#############################################################################
//
// FILE:   ipark.h
//
// TITLE:  Inverse Park Transform.
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

#ifndef _IPARK_H
#define _IPARK_H

#include "math.h"
#include "IQmathLib.h"
#include "FastFPUTrigLib.h"
#include "device.h"

//
// Inverse Park Transformation Parameters
//
typedef struct
{  
	_iq  ds; 	 	 //!<Output: stationary d-axis stator variable
	_iq  qs;		 //!<Output: stationary q-axis stator variable
	_iq  ang;		 //!<Input: rotating angle (pu)
	_iq  de;		 //!<Input: rotating d-axis stator variable
	_iq  qe;		 //!<Input: rotating q-axis stator variable
} IPARK;	            

//
// Default initalizer for the IPARK object.
//
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
					   }

//
// Constants
//
#define TWO_PI _IQ(6.28318530717959)

//
// Inverse Park transform calculation
//

static inline void IPARK_calc(IPARK *v)
{	
    #if USE_FAST_TRIG_LIB == 1
    sincos_t sin_cos_ang;
    #else // Use TMU
    _iq cos_ang, sin_ang;
    #endif    
   
    #if USE_FAST_TRIG_LIB == 1
    FASTPUSinCos(&sin_cos_ang,v->ang);

    v->ds = (v->de * sin_cos_ang.c) - (v->qe * sin_cos_ang.s);
    v->qs = (v->qe * sin_cos_ang.c) + (v->de * sin_cos_ang.s);
    #else  // Use TMU
    sin_ang = __sinpuf32(v->ang);
    cos_ang = __cospuf32(v->ang);

    v->ds = _IQmpy(v->de,cos_ang) - _IQmpy(v->qe,sin_ang);
    v->qs = _IQmpy(v->qe,cos_ang) + _IQmpy(v->de,sin_ang);  
    #endif
}


#endif //_IPARK_H
