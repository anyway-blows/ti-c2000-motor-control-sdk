//#############################################################################
//
// FILE:   FastFPUTrigLib.h
//
// TITLE:  FastRTS Trignometric Cosine, Sine and Atan2 Functions.
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

#ifndef _FASTFPUTRIGLIB_H
#define _FASTFPUTRIGLIB_H

//
// Included Files
//
#include    "fastrts.h"

//
// Constants
//
#define PI            (3.141592653589793F)
#define TWOPI         (6.283185307179586F)

//
// Primitive for the sincos()
//
typedef struct{
    float c;   //!< The Cosine
    float s;   //!< The Sine
}sincos_t;

//
// Function Prototypes
//
extern void sincosf (float radian, float* PtrSin, float* PtrCos);
extern float atan2f (float Y, float X);


//
// This function computes a 32-bit floating point sin & cos given a
// Per Unit input (-1.0 = -2pi, +1.0 = 2pi).
// Note: Input is wrapped around -1.0 to 1.0
//
inline void FASTPUSinCos(sincos_t *output, float pu_value)
{
    float radian_value = pu_value * TWOPI;

    // Assembly version optimized for C28x
    sincosf (radian_value, &(output->s), &(output->c));
}

//
// This function computes a 32-bit floating point atan2 given a
// Y, X input.  This function uses the FPU math tables to
// compute the atan2 resulting angle.
//
inline float FASTAtan2(float y_vect, float x_vect)
{
    // Assembly version optimized for C28x
    return atan2f(y_vect, x_vect);
}


#endif //_FASTFPUTRIGLIB_H
