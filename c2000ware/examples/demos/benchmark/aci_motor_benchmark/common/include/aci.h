//#############################################################################
//
// FILE:   aci.h
//
// TITLE:  ACI Motor Control Model.
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

#ifndef _ACI_H
#define _ACI_H

#include "math.h"
#include "IQmathLib.h"

//
// ACI Model parameters
//

typedef struct
{ 
	_iq  ualfa; 		//!< Input: alfa-axis phase voltage at k  
	_iq  ubeta;			//!< Input: beta-axis phase voltage at k  
	_iq  load_torque;	//!< Input: load torque  	  			      
	_iq  ialfa;			//!< Output: alfa-axis phase current at k  			
	_iq  ibeta;			//!< Output: beta-axis phase current at k  
	_iq  psi_r_alfa; 	//!< Output: alfa-axis rotor flux at k  		
	_iq  psi_r_beta;	//!< Output: beta-axis rotor flux at k  					  
	_iq  torque; 		//!< Output: electromagnetic torque at k  		
	_iq  wr;			//!< Output: electrically angular velocity of motor  
	long  wr_rpm; 		//!< Output: motor speed in rpm  (Q0) - \
									 independently with global Q 
	_iq  K1;			//!< Parameter: constant using in rotor flux calculation  
	_iq  K2;			//!< Parameter: constant using in rotor flux calculation  
	_iq  K3;			//!< Parameter: constant using in rotor flux calculation  
	_iq  K4;			//!< Parameter: constant using in stator current \
										calculation  
	_iq  K5;			//!< Parameter: constant using in stator current \
										calculation  
	_iq  K6;			//!< Parameter: constant using in stator current \
										calculation  
	_iq  K7;			//!< Parameter: constant using in stator current \
										calculation  
	_iq  K8;			//!< Parameter: constant using in torque calculation  
	_iq  K9;			//!< Parameter: constant using in rotor speed calculation  
	_iq  K10;			//!< Parameter: constant using in rotor speed calculation  			      
	long  base_rpm;		//!< Parameter: base motor speed in rpm (Q0) - \
										independently with global Q 	  			      
	_iq  alpha;			//!< Parameter: trapezoidal integration parameter
} ACI;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																				

//
// Default initalizer for the ACI object.
//
#define ACI_DEFAULTS { 0,0,0, \
                       0,0,0,0,0,0,0, \
 		          	   _IQ(0.000595097),_IQ(0.037699112),_IQ(0.000994357),_IQ(0.003227603),_IQ(0.204467), \
 		          	   _IQ(0.010596),_IQ(0.093444628),_IQ(1.094054741),_IQ(0.005),_IQ(0.00816532), \
 		          	   3600, _IQ(0.01) }

//
// ACI motor modelling calculations
//

static inline void ACI_calc(ACI *v)
{	
    _iq psi_r_alfa_p, psi_r_beta_p, ialfa_p, ibeta_p;
	_iq dpsi_r_alfa_p, dpsi_r_beta_p, dialfa_p, dibeta_p;
	_iq dpsi_r_alfa, dpsi_r_beta, dialfa, dibeta;
	_iq wr_p, dwr_p, dwr;

    //
    // Rotor flux/Stator current calculation
    //

	//
    // Predictor
	//
    psi_r_beta_p = v->psi_r_beta - _IQmpy(v->K1,v->psi_r_beta) + _IQmpy(_IQmpy(v->K2,v->wr),v->psi_r_alfa) + _IQmpy(v->K3,v->ibeta);
    psi_r_alfa_p = v->psi_r_alfa - _IQmpy(v->K1,v->psi_r_alfa) - _IQmpy(_IQmpy(v->K2,v->wr),v->psi_r_beta) + _IQmpy(v->K3,v->ialfa);
    ibeta_p = v->ibeta + _IQmpy(v->K4,v->psi_r_beta) - _IQmpy(_IQmpy(v->K5,v->wr),v->psi_r_alfa) - _IQmpy(v->K6,v->ibeta) + _IQmpy(v->K7,v->ubeta);
    ialfa_p = v->ialfa + _IQmpy(v->K4,v->psi_r_alfa) + _IQmpy(_IQmpy(v->K5,v->wr),v->psi_r_beta) - _IQmpy(v->K6,v->ialfa) + _IQmpy(v->K7,v->ualfa);

    //
    // Corrector
    //
    dpsi_r_beta_p = - _IQmpy(v->K1,psi_r_beta_p) + _IQmpy(_IQmpy(v->K2,v->wr),psi_r_alfa_p) + _IQmpy(v->K3,ibeta_p);
    dpsi_r_alfa_p = - _IQmpy(v->K1,psi_r_alfa_p) - _IQmpy(_IQmpy(v->K2,v->wr),psi_r_beta_p) + _IQmpy(v->K3,ialfa_p);
    dibeta_p = _IQmpy(v->K4,psi_r_beta_p) - _IQmpy(_IQmpy(v->K5,v->wr),psi_r_alfa_p) - _IQmpy(v->K6,ibeta_p) + _IQmpy(v->K7,v->ubeta);
    dialfa_p = _IQmpy(v->K4,psi_r_alfa_p) + _IQmpy(_IQmpy(v->K5,v->wr),psi_r_beta_p) - _IQmpy(v->K6,ialfa_p) + _IQmpy(v->K7,v->ualfa);
  
    dpsi_r_beta = psi_r_beta_p - v->psi_r_beta;
    dpsi_r_alfa = psi_r_alfa_p - v->psi_r_alfa;
    dibeta = ibeta_p - v->ibeta;
    dialfa = ialfa_p - v->ialfa;

    v->psi_r_beta = v->psi_r_beta + _IQmpy(_IQ(0.5),(_IQmpy((_IQ(1)+v->alpha),dpsi_r_beta_p) + _IQmpy((_IQ(1)-v->alpha),dpsi_r_beta)));
    v->psi_r_alfa = v->psi_r_alfa + _IQmpy(_IQ(0.5),(_IQmpy((_IQ(1)+v->alpha),dpsi_r_alfa_p) + _IQmpy((_IQ(1)-v->alpha),dpsi_r_alfa)));

    v->ibeta = v->ibeta + _IQmpy(_IQ(0.5),(_IQmpy((_IQ(1)+v->alpha),dibeta_p) + _IQmpy((_IQ(1)-v->alpha),dibeta)));
    v->ialfa = v->ialfa + _IQmpy(_IQ(0.5),(_IQmpy((_IQ(1)+v->alpha),dialfa_p) + _IQmpy((_IQ(1)-v->alpha),dialfa)));

    //
    // Electromagnetic torque calculation
    //
    v->torque = _IQmpy(v->K8,(_IQmpy(v->psi_r_alfa,v->ibeta) - _IQmpy(v->psi_r_beta,v->ialfa)));

    //
    // Rotor speed calculation
    //
    wr_p = v->wr - _IQmpy(v->K9,v->wr) + _IQmpy(v->K10,(v->torque - v->load_torque));

    dwr_p = - _IQmpy(v->K9,wr_p) + _IQmpy(v->K10,(v->torque - v->load_torque));

    dwr = wr_p - v->wr;

    v->wr = v->wr + _IQmpy(_IQ(0.5),(_IQmpy((_IQ(1)+v->alpha),dwr_p) + _IQmpy((_IQ(1)-v->alpha),dwr)));
    
    v->wr_rpm = _IQmpyI32int(v->wr,v->base_rpm);    // This operation performs long = iq * long
}

#endif //_ACI_H

