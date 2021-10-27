//#############################################################################
//
// FILE:    multi_axis_lead_user_ctrl.c
//
// TITLE:   Initialize the parameter variables for controller
//
//
// Group:   C2000
//
// Target Family: F2837x/F28004x
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated - http://www.ti.com/
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
// includes
//
#include "multi_axis_lead_ctrl_main.h"

//
// initSysParameters()
//
void initSysParameters(SYS_Vars_t *pSys)
{
    pSys->isrTicker = 0;
    pSys->Ts = (0.001F / M_ISR_FREQUENCY);

    pSys->focExecutionTime_us = 0;

    pSys->focClrCntr = 0;
    pSys->focCycleCountMax = 0;
    pSys->focCycleCount = 0;

    pSys->speedSet = 0.2F;

    pSys->speedLoopPrescaler = 2;
    pSys->speedLoopCount = 0;

    pSys->fsiNode = SYS_NODE1;
    pSys->fsiNodePrev = SYS_NODE1;
    pSys->ctrlNode = SYS_NODE1;
    pSys->ctrlNodePrev = SYS_NODE1;
    pSys->ctrlSynSet = CTRL_SYN_ENABLE;
    pSys->ctrlStateSet = CTRL_STOP;
    pSys->ctrlModeSet = CTRL_MODE_SPEED;

//    pSys->ecatCtrlSet = ECAT_CTRL_ENABLE;
    pSys->ecatCtrlSet = ECAT_CTRL_DISABLE;

#if(BUILDLEVEL == FCL_LEVEL5)       // Verify FSI
    // sets up all nodes parameters
#endif  // (BUILDLEVEL == FCL_LEVEL5)

    return;
}

//
// initCtrlParameters()
//
void initCtrlParameters(CTRL_Vars_t *pCtrl)
{
    pCtrl->posArray[0] = 6.5;
    pCtrl->posArray[1] = -6.5;
    pCtrl->posArray[2] = 8.5;
    pCtrl->posArray[3] = -8.5;

    pCtrl->posSlewRate = 0.001;

    pCtrl->baseFreq = M_BASE_FREQ;
    pCtrl->curLimit = M_MAXIMUM_CURRENT;        // 5.0A

    pCtrl->IdRefStart = M_ID_START;
    pCtrl->IqRefStart = M_ID_START;
    pCtrl->ctrlIdRef = 0.0;
    pCtrl->ctrlIqRef = 0.0;
    pCtrl->ctrlSpeedRef = 0.0;
    pCtrl->ctrlPosRef = 0.0;

    pCtrl->ctrlSpdOut = 0.0;
    pCtrl->ctrlPosOut = 0.0;
    pCtrl->ctrlSpdMinOut = 0.0;
    pCtrl->ctrlSpdMaxOut = 0.95;
    pCtrl->ctrlPosMaxOut = 0.95;

#if(BUILDLEVEL == FCL_LEVEL5)   // Verify FSI
    pCtrl->speedWeDelta = 0.001;
    pCtrl->posMechThetaDelta = 0.001;
#else   // (BUILDLEVEL != FCL_LEVEL5)
    pCtrl->speedWeDelta = 0.020;
    pCtrl->posMechThetaDelta = 0.010;
#endif  // (BUILDLEVEL != FCL_LEVEL5)

    pCtrl->speedSet = M_SPEED_REF;
    pCtrl->positionSet = 2.0;

    pCtrl->speedRef = M_SPEED_REF;
    pCtrl->positionRef = 0.0;

    pCtrl->posElecTheta = 0.0;
    pCtrl->posMechTheta = 0.0;
    pCtrl->speedWe = 0.0;
    pCtrl->speedMech = 0.0;

    pCtrl->rc.RampDelayMax = 1;
    pCtrl->rc.RampLowLimit = -1.0;
    pCtrl->rc.RampHighLimit = 1.0;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    //
    // PI Controllers Configuration
    // Initialize the PI module for position
    pCtrl->pi_pos.Kp = 0.2;             //0.2;
    pCtrl->pi_pos.Ki = 0.001;           //T*speedLoopPrescaler/0.3;
    pCtrl->pi_pos.Umax = 1.0;
    pCtrl->pi_pos.Umin = -1.0;

    // Initialize the PID module for speed
    pCtrl->pid_spd.param.Kp   = 0.36;
    pCtrl->pid_spd.param.Ki   = 0.0018;
    pCtrl->pid_spd.param.Kd   = 0.0;
    pCtrl->pid_spd.param.Kr   = 1.0;
    pCtrl->pid_spd.param.Umax = 0.95;
    pCtrl->pid_spd.param.Umin = -0.95;
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    //
    // initialize DCL controller for position
    //
    pCtrl->dcl_pos.Kp = 0.2;
    pCtrl->dcl_pos.Ki = 0.001;
    pCtrl->dcl_pos.Umax = 1.0;
    pCtrl->dcl_pos.Umin = -1.0;

    //
    // initialize DCL controller for speed
    //
    pCtrl->dcl_spd.Kp = 0.36;
    pCtrl->dcl_spd.Ki = 0.0018;
    pCtrl->dcl_spd.Umax = 0.95;
    pCtrl->dcl_spd.Umin = -0.95;
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    pCtrl->posRampMax = 1000;
    pCtrl->posRampCntr = 0;
    pCtrl->posBufMax = 4;
    pCtrl->posBufPtr = 0;

    pCtrl->faultFlag = 0;
    pCtrl->runState = 0;
    pCtrl->fsiState = 0;

    pCtrl->ctrlModeSet = CTRL_MODE_SPEED;
    pCtrl->ctrlModeCom = CTRL_MODE_SPEED;

    pCtrl->ctrlStateSet = CTRL_STOP;
    pCtrl->ctrlStateCom = CTRL_STOP;
    pCtrl->ctrlStateFdb = CTRL_STOP;

    return;
}

//
// End of Code
//
