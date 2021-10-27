//#############################################################################
//
// FILE:   clarke.h
//
// TITLE:  Inverse Clarke Transform.
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

#ifndef _ICLARKE_H
#define _ICLARKE_H

//
// Included Files
//
#include "math.h"
#include "IQmathLib.h"

//
// Clarke Transform Parameters for 3 phase stator current
//
typedef struct
{  
    _iq  Alpha;       //!< Input: stationary d-axis stator variable
    _iq  Beta;        //!< Input: stationary q-axis stator variable
    _iq  As;          //!< Output: phase-a stator variable
    _iq  Bs;          //!< Output: phase-b stator variable
    _iq  Cs;          //!< Output: phase-c stator variable
} ICLARKE;

//
//    Default initalizer for the CLARKE object.
//
#define ICLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0 }

//
// Constant
//
#define SQRT3 _IQ(1.732050808)

//
// Inverse Clarke transformation calculation
//
static inline void IClarke_calc(ICLARKE* v)
{
    v->As = v->Alpha;
    v->Bs = _IQmpy((-v->Alpha + _IQmpy(v->Beta, SQRT3)), 0.5);
    v->Cs = _IQmpy((-v->Alpha - _IQmpy(v->Beta, SQRT3)), 0.5);
}


#endif // _ICLARKE_H
