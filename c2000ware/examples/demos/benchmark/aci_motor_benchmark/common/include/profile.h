//#############################################################################
//
// FILE:   profile.h
//
// TITLE:  Application execution and IO response benchmarking
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

#ifndef _PROFILE_H
#define _PROFILE_H

//
// Include files
//
#include    "string.h"
#include    "stdio.h"
#include    "stdint.h"
#include    "device_profile.h"

//
// Instances of application benchmarking datastructures
//
#define BMRK_INSTANCES 10

//
// Benchmarking datastructure declarations
//
extern volatile unsigned long Bmrk_Start;
extern volatile unsigned long Bmrk_End;
extern volatile unsigned long Bmrk_Cyc[BMRK_INSTANCES];
extern volatile unsigned long Bmrk_Sum[BMRK_INSTANCES];
extern volatile unsigned long Bmrk_Max[BMRK_INSTANCES];
extern volatile unsigned long Bmrk_Min[BMRK_INSTANCES];
extern volatile unsigned long Bmrk_Count[BMRK_INSTANCES];
extern volatile unsigned long Bmrk_Adjust;

extern volatile unsigned long IOBmrk_End;
extern volatile unsigned long IOBmrk_Cyc;
extern volatile unsigned long IOBmrk_Sum;
extern volatile unsigned long IOBmrk_Max;
extern volatile unsigned long IOBmrk_Min;
extern volatile unsigned long IOBmrk_Count;

extern volatile unsigned int Bmrk_TotalAvg;
extern volatile unsigned int Bmrk_TotalMin;
extern volatile unsigned int Bmrk_TotalMax;

//
// Function definitions
// These device specific implementations
//
extern void Bmrk_start(void);
extern void Bmrk_end(void);
extern void Bmrk_calibrate(void);
extern void Bmrk_init(void);

extern void IOBmrk_end(void);
extern void IOBmrk_init(void);

//
// Defines for printing benchmark results
//
#define BMRK_READ_ADC_CONV_FLOAT 1
#define BMRK_CLARKE_TRANSFORM 2
#define BMRK_PID_TRANSFORM 3
#define BMRK_IPARK_TRANSFORM 4
#define BMRK_ACI_MODEL 5
#define BMRK_FE_TRANSFORM 6
#define BMRK_SE_TRANSFORM 7
#define BMRK_PARK_TRANSFORM 8
#define BMRK_SVGEN 9
#define BMRK_PWM_WRITE 10

#define PRINT_HEADER "\n                                    \t AVG \t MAX \t MIN"
#define PRINT_ST_0 "\n INT Response (trigger to ISR entry) :\t %d  \t %d  \t %d"
#define PRINT_ST_1 "\n Read 2 ADC, convert float           :\t %d  \t %d  \t %d"
#define PRINT_ST_2 "\n Clarke Transform                    :\t %d  \t %d  \t %d"
#define PRINT_ST_3 "\n 3 PID Controller Transforms         :\t %d  \t %d  \t %d"
#define PRINT_ST_4 "\n Inverse Park Transform              :\t %d  \t %d  \t %d"
#define PRINT_ST_5 "\n ACI Motor Modeling                  :\t %d  \t %d  \t %d"
#define PRINT_ST_6 "\n Flux Estimator                      :\t %d  \t %d  \t %d"
#define PRINT_ST_7 "\n Speed Estimator                     :\t %d  \t %d  \t %d"
#define PRINT_ST_8 "\n Park Transform                      :\t %d  \t %d  \t %d"
#define PRINT_ST_9 "\n SVGen Transform                     :\t %d  \t %d  \t %d"
#define PRINT_ST_10 "\n Write 3 PWM                         :\t %d  \t %d  \t %d"
#define PRINT_ST_TOTAL "\n Total                               :\t %d  \t %d  \t %d"
#define PRINT_ST_LINE  "\n-------------------------------------------------------------------------"

#define IOBmrk_print() printf(PRINT_ST_0, (int) ((((float) IOBmrk_Sum) / ((float)IOBmrk_Count))), (int) IOBmrk_Max, (int) IOBmrk_Min);
#define Bmrk_print(n) printf(PRINT_ST_##n, (int) (((float) Bmrk_Sum[n-1]) / ((float)Bmrk_Count[n-1])), (int) Bmrk_Max[n-1], (int) Bmrk_Min[n-1]);

#define Bmrk_print_total()  printf(PRINT_ST_LINE); \
                                         printf(PRINT_ST_TOTAL, Bmrk_TotalAvg, Bmrk_TotalMax, Bmrk_TotalMin); \
                                         printf(PRINT_ST_LINE);

#define Bmrk_add_IO()   Bmrk_TotalAvg += (int)((float)IOBmrk_Sum / (float)IOBmrk_Count); \
                        Bmrk_TotalMax += IOBmrk_Max; \
                        Bmrk_TotalMin += IOBmrk_Min;

#define Bmrk_add(n)  Bmrk_TotalAvg += (int)((float)Bmrk_Sum[n-1] / (float)Bmrk_Count[n-1]); \
                     Bmrk_TotalMax += Bmrk_Max[n-1]; \
                     Bmrk_TotalMin += Bmrk_Min[n-1];

#define Bmrk_printHeader()    printf(PRINT_ST_LINE); \
                              printf(PRINT_HEADER);

#endif  // _PROFILE_H
