//#############################################################################
//
// FILE:   aci_fe.h
//
// TITLE:  ACI Motor Control Flux Estimator(FE).
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

#ifndef _ACI_FE_H
#define _ACI_FE_H

//
// Included Files
//
#include "math.h"
#include "IQmathLib.h"
#include "FastFPUTrigLib.h"
#include "device.h"

//
// Parameters needed for Flux Estimator calculations
//
typedef struct
{  
    _iq  theta_r_fe; //!< Output: Rotor flux angle 
    _iq  i_qs_fe;	 //!< Input: Stationary q-axis stator current  
    _iq  i_ds_fe;    //!< Input: Stationary d-axis stator current  
    _iq  K1_fe;      //!< Parameter: Constant using in current model 
    _iq  flx_dr_e;   //!< Variable: Rotating d-axis rotor flux (current model)  
    _iq  K2_fe;      //!< Parameter: Constant using in current model  
    _iq  flx_qr_s;   //!< Variable: Stationary q-axis rotor flux (current model)  
    _iq  flx_dr_s;   //!< Variable: Stationary d-axis rotor flux (current model)  
    _iq  K3_fe;      //!< Parameter: Constant using in stator flux computation  
    _iq  K4_fe;      //!< Parameter: Constant using in stator flux computation 
    _iq  flx_ds_s;   //!< Variable: Stationary d-axis stator flux (current model)  
    _iq  flx_qs_s;   //!< Variable: Stationary q-axis stator flux (current model)  
    _iq  psi_ds_fe;  //!< Variable: Stationary d-axis stator flux (voltage model)  
    _iq  Kp_fe;      //!< Parameter: PI proportionnal gain  
    _iq  ui_ds;      //!< Variable: Stationary d-axis _iqegral term  
    _iq  ucomp_ds;   //!< Variable: Stationary d-axis compensated voltage  
    _iq  Ki_fe;      //!< Parameter: PI _iqegral gain  
    _iq  psi_qs_fe;  //!< Variable: Stationary q-axis stator flux (voltage model)  
    _iq  ui_qs;      //!< Variable: Stationary q-axis _iqegral term  
    _iq  ucomp_qs;   //!< Variable: Stationary q-axis compensated voltage  
    _iq  emf_ds;     //!< Variable: Stationary d-axis back emf  
    _iq  u_ds_fe;    //!< Input: Stationary d-axis stator voltage  
    _iq  K5_fe;      //!< Parameter: Constant using in back emf computation  
    _iq  K6_fe;      //!< Parameter: Constant using in back emf computation  
    _iq  emf_qs;     //!< Variable: Stationary q-axis back emf  
    _iq  u_qs_fe;    //!< Input: Stationary q-axis stator voltage  
    _iq  K8_fe;      //!< Parameter: Constant using in rotor flux computation  
    _iq  K7_fe;      //!< Parameter: Constant using in rotor flux computation  
    _iq  psi_dr_fe;	 //!< Output: Stationary d-axis estimated rotor flux 
    _iq  psi_qr_fe;	 //!< Output: Stationary q-axis estimated rotor flux
} ACIFE;	            


//
// Default initalizer for the ACIFE object.
//
#define ACIFE_DEFAULTS {  0,    /*  theta_r_fe  */  \
	                      0,    /*  i_qs_fe  */     \
	                      0,    /*  i_ds_fe  */     \
	                      0,    /*  K1_fe */       \
	                      0,    /*  flx_dr_e  */    \
	                      0,    /*  K2_fe  */       \
	                      0,    /*  flx_dr_s  */    \
	                      0,    /*  flx_qr_s  */    \
	                      0,    /*  K3_fe  */       \
	                      0,    /*  K4_fe  */       \
	                      0,    /*  flx_ds_s  */    \
	                      0,    /*  flx_qs_s  */    \
	 		              0,    /*  psi_ds_fe  */   \
	                      0,    /*  Kp_fe  */       \
	                      0,    /*  ui_ds  */    \
	                      0,    /*  ucomp_ds  */    \
	                      0,    /*  Ki_fe  */ \
	                      0,    /*  psi_qs_fe  */   \
 	                      0,    /*  ui_qs  */    \
	                      0,    /*  ucomp_qs  */    \
	                      0,    /*  emf_ds  */      \
                          0,    /*  u_ds_fe  */     \
	                      0,    /*  K5_fe  */       \
	                      0,    /*  K6_fe  */       \
	                      0,    /*  emf_qs  */      \
                          0,    /*  u_qs_fe  */     \
	                      0,    /*  K8_fe  */       \
	                      0,    /*  K7_fe  */       \
		                  0,    /*  psi_dr_fe  */   \
		                  0,	/*  psi_qr_fe  */   }

//
// Constants
//
#define TWO_PI _IQ(6.28318530717959)
#define INV_2PI  _IQ(0.15915494309190)

//
// ACI motor flux estimator calculation
//

static inline void ACIFE_calc(ACIFE *v)
{	
   #if USE_FAST_TRIG_LIB == 1
   sincos_t sin_cos_ang;
   #endif  

   _iq i_ds_e, error, emf_old;

   //
   // Park transformation on the measured stator current
   //
   #if USE_FAST_TRIG_LIB == 1
   FASTPUSinCos(&sin_cos_ang,v->theta_r_fe);
   i_ds_e  = v->i_qs_fe * sin_cos_ang.s;
   i_ds_e += v->i_ds_fe * sin_cos_ang.c;
   #else // Use TMU32
   i_ds_e  = v->i_qs_fe * __sinpuf32(v->theta_r_fe);
   i_ds_e += v->i_ds_fe * __cospuf32(v->theta_r_fe);
   #endif
   
   //
   // The current model section (Classical Rotor Flux Vector Control Equation)
   //
   v->flx_dr_e = _IQmpy(v->K1_fe,v->flx_dr_e) - _IQmpy(v->K2_fe,i_ds_e);	

   //
   // Inverse park transformation on the rotor flux from the current model
   //
   #if USE_FAST_TRIG_LIB == 1
   FASTPUSinCos(&sin_cos_ang,v->theta_r_fe);
   v->flx_dr_s = v->flx_dr_e * sin_cos_ang.c;
   v->flx_qr_s = v->flx_dr_e * sin_cos_ang.s;
   #else // Use TMU32
   v->flx_dr_s = v->flx_dr_e * __cospuf32(v->theta_r_fe);
   v->flx_qr_s = v->flx_dr_e * __sinpuf32(v->theta_r_fe);
   #endif

   //
   // Compute the stator flux based on the rotor flux from current model
   //
   v->flx_ds_s = _IQmpy(v->K3_fe,v->flx_dr_s) + _IQmpy(v->K4_fe,v->i_ds_fe);	
   v->flx_qs_s = _IQmpy(v->K3_fe,v->flx_qr_s) + _IQmpy(v->K4_fe,v->i_qs_fe);

   //
   // Conventional PI controller section
   //
   error =  v->psi_ds_fe - v->flx_ds_s;
   v->ucomp_ds = _IQmpy(v->Kp_fe,error) + v->ui_ds;
   v->ui_ds = _IQmpy(v->Kp_fe,_IQmpy(v->Ki_fe,error)) + v->ui_ds;
   
   error =  v->psi_qs_fe - v->flx_qs_s;
   v->ucomp_qs = _IQmpy(v->Kp_fe,error) + v->ui_qs;
   v->ui_qs = _IQmpy(v->Kp_fe,_IQmpy(v->Ki_fe,error)) + v->ui_qs;    

   //
   // Compute the estimated stator flux based on the integral of back emf
   //
   emf_old = v->emf_ds;
   v->emf_ds = v->u_ds_fe - v->ucomp_ds - _IQmpy(v->K5_fe,v->i_ds_fe);
   v->psi_ds_fe = v->psi_ds_fe + 
                  _IQmpy(_IQ(0.5),_IQmpy(v->K6_fe,(v->emf_ds + emf_old))); 

   emf_old = v->emf_qs;
   v->emf_qs = v->u_qs_fe - v->ucomp_qs - _IQmpy(v->K5_fe,v->i_qs_fe);
   v->psi_qs_fe = v->psi_qs_fe +
                  _IQmpy(_IQ(0.5),_IQmpy(v->K6_fe,(v->emf_qs + emf_old))); 

   //
   // Compute the estimated rotor flux based on the stator flux
   // from the integral of back emf
   //
   v->psi_dr_fe = _IQmpy(v->K7_fe,v->psi_ds_fe) - _IQmpy(v->K8_fe,v->i_ds_fe);  
   v->psi_qr_fe = _IQmpy(v->K7_fe,v->psi_qs_fe) - _IQmpy(v->K8_fe,v->i_qs_fe);  

   //
   // Compute the rotor flux angle
   //
   #if USE_FAST_TRIG_LIB == 1
   v->theta_r_fe = FASTAtan2(v->psi_qr_fe,v->psi_dr_fe) * INV_2PI; 
   #else // Use TMU
   v->theta_r_fe = _IQmpy(_IQatan2(v->psi_qr_fe,v->psi_dr_fe),INV_2PI); 
   #endif

   //
   // (PI,-PI)/(2*PI) -> (0, 2*PI)/(2*PI)
   //
   if (v->theta_r_fe < _IQ(0))
     v->theta_r_fe = _IQ(1) + v->theta_r_fe;

}

#endif //_ACI_FE_H
