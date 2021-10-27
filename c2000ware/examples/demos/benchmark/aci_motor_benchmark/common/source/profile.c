//#############################################################################
//
// FILE:   profile.c
//
// TITLE:  Data structures for recording benchmarking count.
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

//
// Include Files
//
#include "profile.h"

//
// Application execution benchmark datastructures
//
volatile unsigned long Bmrk_Start;
volatile unsigned long Bmrk_End;
volatile unsigned long Bmrk_Cyc[BMRK_INSTANCES];
volatile unsigned long Bmrk_Sum[BMRK_INSTANCES];
volatile unsigned long Bmrk_Max[BMRK_INSTANCES];
volatile unsigned long Bmrk_Min[BMRK_INSTANCES];
volatile unsigned long Bmrk_Count[BMRK_INSTANCES];
volatile unsigned long Bmrk_Adjust;

//
// IO Response benchmark datastructures
//
volatile unsigned long IOBmrk_End;
volatile unsigned long IOBmrk_Cyc;
volatile unsigned long IOBmrk_Sum;
volatile unsigned long IOBmrk_Max;
volatile unsigned long IOBmrk_Min;
volatile unsigned long IOBmrk_Count;

//
// Total cycle count
//
volatile unsigned int Bmrk_TotalAvg = 0;
volatile unsigned int Bmrk_TotalMin = 0;
volatile unsigned int Bmrk_TotalMax = 0;
