//#############################################################################
//
// FILE:   aci_se.h
//
// TITLE:  ACI Motor Control Speed Estimator(SE) calculation.
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

#ifndef _ACI_SE_H
#define _ACI_SE_H

//
// Included Files
//
#include "math.h"
#include "IQmathLib.h"

//
// Parameters for calculating Speed Estimator
//
typedef struct
{
	_iq  i_qs_se;  		 //!< Input: Stationary q-axis stator current  
	_iq  psi_dr_se;  	 //!< Input: Stationary d-axis rotor flux  
	_iq  i_ds_se;		 //!< Input: Stationary d-axis stator current  
	_iq  psi_qr_se;		 //!< Input: Stationary q-axis rotor flux  		
	_iq  K1_se;			 //!< Parameter: Constant using in speed computation  
	_iq  psi_r_2;     	 //!< Variable: Squared rotor flux    
	_iq  theta_r_se;  	 //!< Input: Rotor flux angle      		  
	_iq  K2_se;			 //!< Parameter: Constant using in differentiator  
	_iq  theta_r_old; 	 //!< Variable: Previous rotor flux angle      		  
	_iq  K3_se;			 //!< Parameter: Constant using in low-pass filter   
	_iq  wr_psi_r;		 //!< Variable: Synchronous rotor flux speed in per-unit  
	_iq  K4_se;			 //!< Parameter: Constant using in low-pass filter  
	_iq  wr_hat_se;		 //!< Output: Estimated speed in per unit  
	long  base_rpm_se; 	 //!< Parameter: Base rpm speed (Q0) - \
						                 independently with global Q 		 	 	  
	long  wr_hat_rpm_se; //!< Output: Estimated speed in rpm (Q0) - \
						              independently with global Q
} ACISE;	            
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																			

//
// Default initalizer for the ACISE object.
//
#define ACISE_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          _IQ(0.1), \
                          0, \
                          0, \
                          _IQ(0.1), \
                          0, \
                          _IQ(0.1), \
                          0, \
                          _IQ(0.1), \
                          0, \
                          3600, \
              			  0 }


//
// Constants
//
#define DIFF_MAX_LIMIT  	_IQ(0.97)
#define DIFF_MIN_LIMIT  	_IQ(0.03)


//
// ACI rotor speed estimation calculation
//
static inline void ACISE_calc(ACISE *v)
{	
   _iq w_slip, w_syn;
					
   //
   // Slip computation
   //
   v->psi_r_2 = _IQmpy(v->psi_dr_se,v->psi_dr_se) + 
   				_IQmpy(v->psi_qr_se,v->psi_qr_se);

   w_slip = _IQmpy(v->K1_se,(_IQmpy(v->psi_dr_se,v->i_qs_se) - 
   							 _IQmpy(v->psi_qr_se,v->i_ds_se)));

   w_slip = _IQdiv(w_slip,v->psi_r_2);
   
   //
   // Synchronous speed computation
   //
   if ((v->theta_r_se < DIFF_MAX_LIMIT)&(v->theta_r_se > DIFF_MIN_LIMIT))
   		w_syn = _IQmpy(v->K2_se,(v->theta_r_se - v->theta_r_old));
   else  w_syn = v->wr_psi_r;
   
   //
   // low-pass filter
   //
   v->wr_psi_r = _IQmpy(v->K3_se,v->wr_psi_r) + _IQmpy(v->K4_se,w_syn);
    
   v->theta_r_old = v->theta_r_se;
   v->wr_hat_se = w_syn - w_slip;
 
   //
   // Saturate
   //
   v->wr_hat_se = (v->wr_hat_se > _IQ( 1.0f))? _IQ( 1.0f):v->wr_hat_se;
   v->wr_hat_se = (v->wr_hat_se < _IQ(-1.0f))? _IQ(-1.0f):v->wr_hat_se;
   
   //
   // This operation performs long = iq * long
   //
   v->wr_hat_rpm_se = _IQmpyI32int(v->wr_hat_se, v->base_rpm_se);
}


#endif //_ACI_SE_H

