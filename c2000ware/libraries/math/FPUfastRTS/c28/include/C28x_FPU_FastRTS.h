//###########################################################################
//  This software is licensed for use with Texas Instruments C28x
//  family DSCs.  This license was provided to you prior to installing
//  the software.  You may review this license by consulting a copy of
//  the agreement in the doc directory of this library.
// ------------------------------------------------------------------------
//          Copyright (C) 2007 Texas Instruments, Incorporated.
//                          All Rights Reserved.
// ==========================================================================
//
// FILE:   C28x_FPU_FastRTS.h
//
// TITLE:  Prototypes and Definitions for the C28x Fast RTS Library
//
// DESCRIPTION:
//
//         These prototypes are for functions not found in the standard
//         RTS library.
//
//         For standard functions, include the appropriate header file as
//         usual.  For example: math.h
//
//###########################################################################
// $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
// $Release Date: Feb 12, 2021 $
//###########################################################################

#ifndef C28X_FPU_FAST_RTS_H
#define C28X_FPU_FAST_RTS_H


#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------
// Standard C28x Data Types
//-----------------------------------------------------------------------------


#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int                 int16;
typedef long                int32;
typedef long long           int64;
typedef unsigned int        Uint16;
typedef unsigned long       Uint32;
typedef unsigned long long  Uint64;
typedef float               float32;
typedef long double         float64;
#endif


//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

float32 isqrt(float32 X);
void sincos(float32 radian, float32* PtrSin, float32* PtrCos);

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   // - end of C28X_FPU_FAST_RTS_H

//===========================================================================
// End of file.
//===========================================================================
