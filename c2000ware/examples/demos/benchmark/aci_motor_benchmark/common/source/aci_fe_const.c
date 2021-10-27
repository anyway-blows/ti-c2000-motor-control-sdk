//#############################################################################
//
// FILE:   aci_fe_const.c
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

//
// Included Files
//
#include "math.h"
#include "aci_fe_const.h"

//
// Calculate Flux Estimator(FE) output constants
//
void ACIFE_Const_calc(ACIFE_Const *v)
{	
   float Tr;
   
   //
   // Rotor time constant (sec)
   //
   Tr = v->Lr/v->Rr;

   v->K1 = Tr/(Tr+v->Ts); 
   v->K2 = v->Ts/(Tr+v->Ts); 
   v->K3 = v->Lm/v->Lr;
   v->K4 = (v->Ls*v->Lr-v->Lm*v->Lm)/(v->Lr*v->Lm);   
   v->K5 = v->Ib*v->Rs/v->Vb; 
   v->K6 = v->Vb*v->Ts/(v->Lm*v->Ib); 
   v->K7 = v->Lr/v->Lm;   
   v->K8 = (v->Ls*v->Lr-v->Lm*v->Lm)/(v->Lm*v->Lm);
   
}
