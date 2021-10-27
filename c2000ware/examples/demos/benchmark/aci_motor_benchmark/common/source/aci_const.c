//#############################################################################
//
// FILE:   aci_const.c
//
// TITLE:  ACI Motor Control Model Output Constants.
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
#include "aci_const.h"

//
// Calculate the ACI motor output constants
//
void ACI_Const_calc(ACI_Const *v)
{	
	float sigma,gamma,alpha,beta;
	
	sigma = 1 - (v->Lm*v->Lm)/(v->Ls*v->Lr);
	gamma = (v->Lm*v->Lm*v->Rr + v->Lr*v->Lr*v->Rs)/(sigma*v->Ls*v->Lr*v->Lr);
	alpha = v->Rr/v->Lr;
	beta = v->Lm/(sigma*v->Ls*v->Lr);
	
    v->K1 = v->Ts*alpha;
    v->K2 = v->Ts*v->Wb;
    v->K3 = v->Ts*alpha*v->Lm*(v->Ib/v->Lb);  
    v->K4 = v->Ts*alpha*beta*(v->Lb/v->Ib);
    v->K5 = v->Ts*beta*(v->Lb*v->Wb/v->Ib);
    v->K6 = v->Ts*gamma;  
    v->K7 = v->Ts*(1/(sigma*v->Ls))*(v->Vb/v->Ib);
    v->K8 = 1.5*(v->p/2)*(v->Lm/v->Lr)*(v->Lb*v->Ib/v->Tb);
    v->K9 = v->Ts*(v->B/v->J);  
    v->K10 = v->Ts*(v->p/2)*(1/v->J)*(v->Tb/v->Wb);   
}

//
// End of File
//
