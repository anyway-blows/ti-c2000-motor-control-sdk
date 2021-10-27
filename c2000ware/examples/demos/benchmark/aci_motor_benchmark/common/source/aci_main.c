//#############################################################################
//
// FILE:   aci_main.c
//
// TITLE:  ACI Motor Control Benchmark Application
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
// Include files
//

#include "math.h"
#include "IQmathLib.h"
#include "profile.h"
#include "aci_main.h"
#include "FastFPUTrigLib.h"
#include "device.h"

//
// Set simulation parameters
//

//
// PU (per unit radians) = (angle in radians) / (2*pi)
//
_iq idRef      = _IQ(0.4);           	// Id reference (pu)
_iq speedRef   = _IQ(0.5);         	// Speed reference (pu)
_iq T           = _IQ(SAMPLING_PERIOD); // Samping period (sec)

//
// Initialize ACI model system simulation defaults
//
ACI_Const aci1Const = ACI_CONST_DEFAULTS;
ACIFE_Const fe1Const = ACIFE_CONST_DEFAULTS;
ACISE_Const se1Const = ACISE_CONST_DEFAULTS;

ACI aci1 = ACI_DEFAULTS;
ACIFE fe1 = ACIFE_DEFAULTS;
ACISE se1 = ACISE_DEFAULTS;

PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;

PIDREG3 pid1Id = PIDREG3_DEFAULTS;
PIDREG3 pid1Iq = PIDREG3_DEFAULTS;
PIDREG3 pid1Spd = PIDREG3_DEFAULTS;

CLARKE clarke1 = CLARKE_DEFAULTS;
ICLARKE iclarke1 = ICLARKE_DEFAULTS;

SVGEN svgen1 = SVGEN_DEFAULTS;

// 
// Data logging buffers for two channels (or graphs):
//
#define DLOG_NUMBER_SAMPLES  1024
_iq	DLogCh1[DLOG_NUMBER_SAMPLES];
_iq	DLogCh2[DLOG_NUMBER_SAMPLES];

//
// Global variables
//
volatile int ctrlLoopCount = 0;
extern uint32_t PWM_HALF_MAX;
_iq speedEndOfTest = 0;

//
// Start main program:
//
int main(void)
{
    int i = 0;

    //
    // Print start of benchmarking test
    //
    printf("\nACI Benchmark Test:\n");
    
    //
    // Initialize benchmarking resources
    //
    Bmrk_init();
    IOBmrk_init();

    //
    // Initialize data logging buffers
    //
    for(i = 0; i < DLOG_NUMBER_SAMPLES; i++) 
    {
      DLogCh1[i] = 0;
      DLogCh2[i] = 0;
    }

    //
    // Initialize control loop execution count
    //
    ctrlLoopCount = 0;

    //
    // Initialize the ACI module
    //
    ACI_Const_calc(&aci1Const);
 	
    aci1.K1 = _IQ(aci1Const.K1);
    aci1.K2 = _IQ(aci1Const.K2);
    aci1.K3 = _IQ(aci1Const.K3);
    aci1.K4 = _IQ(aci1Const.K4);
    aci1.K5 = _IQ(aci1Const.K5);
    aci1.K6 = _IQ(aci1Const.K6);
    aci1.K7 = _IQ(aci1Const.K7);
    aci1.K8 = _IQ(aci1Const.K8);
    aci1.K9 = _IQ(aci1Const.K9);
    aci1.K10 = _IQ(aci1Const.K10);
    aci1.base_rpm = 120*BASE_FREQ/P_VALUE;
    aci1.load_torque = _IQ(TL_VALUE/BASE_TORQUE);

    //
    // Initialize the ACI_FE module
    //
    ACIFE_Const_calc(&fe1Const);
 	
    fe1.K1_fe = _IQ(fe1Const.K1);
    fe1.K2_fe = _IQ(fe1Const.K2);
    fe1.K3_fe = _IQ(fe1Const.K3);
    fe1.K4_fe = _IQ(fe1Const.K4);
    fe1.K5_fe = _IQ(fe1Const.K5);
    fe1.K6_fe = _IQ(fe1Const.K6);
    fe1.K7_fe = _IQ(fe1Const.K7);
    fe1.K8_fe = _IQ(fe1Const.K8);
    fe1.Kp_fe = _IQ(0.055);
    fe1.Ki_fe = _IQ(0.002);
 
    //
    // Initialize the ACI_SE module
    //
    ACISE_Const_calc(&se1Const);
 	
    se1.K1_se = _IQ(se1Const.K1);
    se1.K2_se = _IQ(se1Const.K2);
    se1.K3_se = _IQ(se1Const.K3);
    se1.K4_se = _IQ(se1Const.K4);
    se1.base_rpm_se = 120*BASE_FREQ/P_VALUE;

    //
    // Initialize the PID_REG3 module for Id
    //
    pid1Id.Kp_reg3 = _IQ(0.0541);
    pid1Id.Ki_reg3 = _IQ(0.5);
    pid1Id.Kd_reg3 = _IQ(0.2);
    pid1Id.Kc_reg3 = _IQ(0.1);
    pid1Id.pid_out_max = _IQ(0.71);
    pid1Id.pid_out_min = _IQ(-0.71);
 
    //
    // Initialize the PID_REG3 module for Iq
    //
    pid1Iq.Kp_reg3 = _IQ(0.0541);
    pid1Iq.Ki_reg3 = _IQ(0.5);
    pid1Iq.Kd_reg3 = _IQ(0.2);
    pid1Iq.Kc_reg3 = _IQ(0.1);
    pid1Iq.pid_out_max = _IQ(0.71);
    pid1Iq.pid_out_min = _IQ(-0.71);

    //
    // Initialize the PID_REG3 module for speed
    //
    pid1Spd.Kp_reg3 = _IQ(7.2);
    pid1Spd.Ki_reg3 = _IQ(0.05);
    pid1Spd.Kd_reg3 = _IQ(0.2);
    pid1Spd.Kc_reg3 = _IQ(0.9);
    pid1Spd.pid_out_max = _IQ(1);
    pid1Spd.pid_out_min = _IQ(-1);

    //
    // Setup device
    //
    Device_setup();

#if SIGNAL_CHAIN == 1 // Full signal chain benchmark

    //
    // Enable ADC interrupt
    //
    ADCInt_enable();

    //
    // Execute Control Loop ISR for the specified number of times
    //
    while(ctrlLoopCount < DLOG_NUMBER_SAMPLES)
    {
    }

    //
    // Disable interrupts
    //
    ADCInt_disable();
    ADCInt_ack();
    
#else  // Control alogrithm benchmark only
    
    while(ctrlLoopCount < DLOG_NUMBER_SAMPLES)
    {
      aci_ctrlLoop();
    }

#endif


    //
    // Print profile numbers
    //
    printf("\nExecution in cycle counts (avg, max, min) over %d iterations\n", DLOG_NUMBER_SAMPLES);
    Bmrk_printHeader();

#if SIGNAL_CHAIN == 1
    //
    // Print numbers pertaining to signal chain elements (IO response, ADC etc)
    //
    IOBmrk_print();
    Bmrk_add_IO();

    Bmrk_print(1);
    Bmrk_add(1);

    Bmrk_print(2);
    Bmrk_add(2);

#endif
    
    Bmrk_print(3);
    Bmrk_add(3);

    Bmrk_print(4);
    Bmrk_add(4);

    Bmrk_print(6);
    Bmrk_add(6);

    Bmrk_print(7);
    Bmrk_add(7);

    Bmrk_print(8);
    Bmrk_add(8);

#if SIGNAL_CHAIN == 1
    //
    // Print numbers pertaining to signal chain elements (PWM writes etc)
    //
    Bmrk_print(9);
    Bmrk_add(9);

    Bmrk_print(10);
    Bmrk_add(10);
#endif
    

    //
    // Print total numbers
    //
    Bmrk_print_total();

    //
    // Calculate end of test speed by averaging last few samples
    //
    speedEndOfTest = 0;
    for (i=0; i<50; i++)
    {
        speedEndOfTest += DLogCh2[1023-i];
    }
    speedEndOfTest = speedEndOfTest/50;

    printf("\n\nMotor Speed at end of test (expected 0.5) : %lf", speedEndOfTest);

    printf("\n\nTest program end.\n\n");

    return 0;
}


//
// ACI Control Loop Implementation
//
#if SIGNAL_CHAIN == 1
interrupt
#endif
void aci_ctrlLoop(void)
{

#if SIGNAL_CHAIN == 1
    //
    // Benchmark IO response
    //
    IOBmrk_end();
    IOBmrk_calc();
    
    //
    // Read ADC and convert to float
    //
    Bmrk_calibrate();
    asm("BMRK_ADC_READ_START:");
    Bmrk_start();

    asm("ADC_READ_START:");
    _iq Ia = _IQ12toIQ(ADC_getIa());
    _iq Ib = _IQ12toIQ(ADC_getIb());
    asm("ADC_READ_END:");

    Bmrk_end();
    asm("BMRK_ADC_READ_END:");
    Bmrk_calc(BMRK_READ_ADC_CONV_FLOAT);

    //
    //  Convert the ADC measurement to phase currents
    //
    Ia = ADJUST_Ia_INPUT(Ia);
    Ib = ADJUST_Ib_INPUT(Ib);

    //
    // Perform Clarke transform
    //
    Bmrk_calibrate();
    asm("BMRK_CLARKE_CALC_START:");
    Bmrk_start();

    clarke1.As = Ia; // Phase A curr.
    clarke1.Bs = Ib; // Phase B curr.
    asm("CLARKE_CALC_START:");
    Clarke_calc(&clarke1);
    asm("CLARKE_CALC_END:");

    Bmrk_end();
    asm("BMRK_CLARKE_CALC_END:");
    Bmrk_calc(BMRK_CLARKE_TRANSFORM);

    //
    // Apply ADC derived phase current to park transform
    //
    _iq ialpha = clarke1.Alpha;
    _iq ibeta = clarke1.Beta;

#else

    //
    // Apply ACI model derived phase current to park transform
    //
    _iq ialpha = aci1.ialfa;
    _iq ibeta = aci1.ibeta;

#endif

    //
    // Perform Park transform
    //
    Bmrk_calibrate();

    asm("BMRK_PARK_CALC_START:");
    Bmrk_start();

    park1.ds = ialpha;
    park1.qs = ibeta;
    park1.ang = fe1.theta_r_fe;
    asm("PARK_CALC_START:");
    PARK_calc(&park1);
    asm("PARK_CALC_END:");

    Bmrk_end();
    asm("BMRK_PARK_CALC_END:");
    Bmrk_calc(BMRK_PARK_TRANSFORM);

    //
    // Perform PID Controller transform
    //

    Bmrk_calibrate();
    Bmrk_start();

    // Forward Control
    pid1Spd.pid_ref_reg3 = speedRef;
    pid1Spd.pid_fdb_reg3 = se1.wr_hat_se;
    asm("PID_1_CALC_START:");
    PIDREG3_calc(&pid1Spd);
    asm("PID_1_CALC_END:");

    pid1Iq.pid_ref_reg3 = pid1Spd.pid_out_reg3;
    pid1Iq.pid_fdb_reg3 = park1.qe;
    asm("PID_2_CALC_START:");
    PIDREG3_calc(&pid1Iq);
    asm("PID_2_CALC_END:");

    pid1Id.pid_ref_reg3 = idRef;
    pid1Id.pid_fdb_reg3 = park1.de;
    asm("PID_3_CALC_START:");
    PIDREG3_calc(&pid1Id);
    asm("PID_3_CALC_END:");

    Bmrk_end();
    Bmrk_calc(BMRK_PID_TRANSFORM);

    //
    // Perform Inverse Park Transform
    //
    Bmrk_calibrate();
    Bmrk_start();
    ipark1.de = pid1Id.pid_out_reg3;
    ipark1.qe = pid1Iq.pid_out_reg3;
    ipark1.ang = fe1.theta_r_fe;
    asm("IPARK_CALC_START:");
    IPARK_calc(&ipark1);
    asm("IPARK_CALC_END:");
    Bmrk_end();
    Bmrk_calc(BMRK_IPARK_TRANSFORM);


    //
    // Perform ACI Motor Modeling
    //

    Bmrk_calibrate();
    Bmrk_start();
    aci1.ualfa = ipark1.ds;
    aci1.ubeta = ipark1.qs;
    asm("ACI_CALC_START:");
    ACI_calc(&aci1);
    asm("ACI_CALC_END:");
    Bmrk_end();
    Bmrk_calc(BMRK_ACI_MODEL);


    //
    // Save computed values to log buffer for plotting
    //
    DLogCh1[ctrlLoopCount] = aci1.ialfa;     // Current Plot
    DLogCh2[ctrlLoopCount] = se1.wr_hat_se;  // Speed   Plot

#if SIGNAL_CHAIN == 1
    //
    // Output modeled phase current to DAC so it can be
    // routed to ADC to complete the loop
    //

    asm("BMRK_ICLARKE_CALC_START:");
    iclarke1.Alpha = aci1.ialfa;
    iclarke1.Beta = aci1.ibeta;
    IClarke_calc(&iclarke1);
    asm("BMRK_ICLARKE_CALC_END:");

    asm("BMRK_DAC_WRITE_START:");

    //
    // Adjust the phase current values (between -1.0 abd 1.0) to a DAC value
    // that can be output and read correctly by ADC
    //

    DAC_setIa(ADJUST_Ia_OUTPUT(iclarke1.As));
    DAC_setIb(ADJUST_Ib_OUTPUT(iclarke1.Bs));

    asm("BMRK_DAC_WRITE_END:");
#endif

    //
    // Perform Flux Estimation
    //

    Bmrk_calibrate();
    Bmrk_start();
    // Feedback Control:
    fe1.u_ds_fe = ipark1.ds;
    fe1.u_qs_fe = ipark1.qs;
    fe1.i_ds_fe = aci1.ialfa;
    fe1.i_qs_fe = aci1.ibeta;
    asm("ACIFE_CALC_START:");
    ACIFE_calc(&fe1);
    asm("ACIFE_CALC_END:");
    Bmrk_end();
    Bmrk_calc(BMRK_FE_TRANSFORM);

    //
    // Perform Speed Estimation
    //
    Bmrk_calibrate();
    Bmrk_start();
    se1.i_ds_se = ipark1.ds;
    se1.i_qs_se = ipark1.qs;
    se1.psi_dr_se = fe1.psi_dr_fe;
    se1.psi_qr_se = fe1.psi_qr_fe;
    se1.theta_r_se = fe1.theta_r_fe;
    asm("ACISE_CALC_START:");
    ACISE_calc(&se1);
    asm("ACISE_CALC_END:");
    Bmrk_end();
    Bmrk_calc(BMRK_SE_TRANSFORM);


#if SIGNAL_CHAIN == 1
    //
    // Perform SVGen Transform
    //

    Bmrk_calibrate();
    asm("BMRK_SVGEN_CALC_START:");
    Bmrk_start();
    svgen1.Ualpha = ipark1.ds;
    svgen1.Ubeta  = ipark1.qs;
    asm("SVGEN_CALC_START:");
    SVGEN_calc(&svgen1);
    asm("SVGEN_CALC_END:");

    uint32_t Ua = _IQmpy(PWM_HALF_MAX,svgen1.Ta)+ PWM_HALF_MAX;
    uint32_t Ub = _IQmpy(PWM_HALF_MAX,svgen1.Tb)+ PWM_HALF_MAX;
    uint32_t Uc = _IQmpy(PWM_HALF_MAX,svgen1.Tc)+ PWM_HALF_MAX;

    Bmrk_end();
    asm("BMRK_SVGEN_CALC_END:");
    Bmrk_calc(BMRK_SVGEN);

    //
    // Write to the three PWMs. The writes simulate a real execution where
    // PWMs control the 3 phase voltage.
    //

    Bmrk_calibrate();
    asm("BMRK_PWM_WRITE_START:");
    Bmrk_start();

    asm("PWM_WRITE_START:");
    PWM_setUa(Ua);
    PWM_setUb(Ub);
    PWM_setUc(Uc);
    asm("PWM_WRITE_END:");

    Bmrk_end();
    asm("BMRK_PWM_WRITE_END:");
    Bmrk_calc(BMRK_PWM_WRITE);

#endif

    //
    // Increment control loop execution count
    //
    ++ctrlLoopCount;
    
#if SIGNAL_CHAIN == 1

    //
    // Disable interrupt if control loop execution count has
    // reached the required count
    //
    if(ctrlLoopCount == DLOG_NUMBER_SAMPLES)        ADCInt_disable();

    //
    // Acknowledge the interrupt
    //
    ADCInt_ack();

#endif
   
}
