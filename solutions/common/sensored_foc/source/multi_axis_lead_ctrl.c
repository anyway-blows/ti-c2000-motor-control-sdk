//#############################################################################
//
// FILE:    multi_axis_lead_ctrl.c
//
// TITLE:   multi-axis motor drive on the related kits
//
// Group:   C2000
//
// Target Family: F2838x
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
#include "motor_ctrl_settings.h"
#include "multi_axis_lead_ctrl_main.h"

//
//  Prototype statements for Local Functions
//
#pragma INTERRUPT (motorControlISR, LPI)

#ifdef _FLASH
#pragma CODE_SECTION(motorControlISR, ".TI.ramfunc");
#endif

//
// Global variables used in this system
//
CTRL_Vars_t ctrlVars[SYS_NODE_NUM] = {  CTRL_DEFAULTS,                         \
                                        CTRL_DEFAULTS,                         \
                                        CTRL_DEFAULTS,                         \
                                        CTRL_DEFAULTS,                         \
                                        CTRL_DEFAULTS                          \
                                      };
SYS_Vars_t  sysVars = SYS_DEFAULTS;

HAL_Handle    halHandle;    //!< the handle for the hardware abstraction layer
HAL_Obj       hal;          //!< the hardware abstraction layer object

// flag variables
volatile uint16_t tempPIEIER;

// variables for Datalog module
#ifdef DLOG_ENABLE
#pragma DATA_SECTION(DBUFF_6CH, "DLOG_Data")
float32_t DBUFF_6CH[DLOG_CH_NUM][DLOG_BUF_SIZE];

float32_t dlog6ChData[DLOG_CH_NUM];

// Create an instance of DATALOG Module
DLOG_6CH_F dlog_6ch;
#endif  // DLOG_ENABLE

// Variables for SFRA module
SFRATest_e      sfraTestLoop;  //speedLoop;
uint32_t        sfraCollectStart;
float32_t       sfraNoiseD;
float32_t       sfraNoiseQ;
float32_t       sfraNoiseW;

#ifdef DACOUT_EN
DAC_DataView_t dacDataView[2];
#endif   // DACOUT_EN

//
//   Various Incremental Build levels
//
static inline void switchActiveNode(void)
{
    switch(sysVars.ctrlNode)
    {
        case SYS_NODEM:
            sysVars.ctrlNode = SYS_NODE1;
            break;

        case SYS_NODE1:
            sysVars.ctrlNode = SYS_NODE2;
            break;
        case SYS_NODE2:
            sysVars.ctrlNode = SYS_NODE3;

            break;
        case SYS_NODE3:
            sysVars.ctrlNode = SYS_NODE4;

            break;
        case SYS_NODE4:
            sysVars.ctrlNode = SYS_NODEM;

            break;
        default:
            sysVars.ctrlNode = SYS_NODE1;
    }
    return;
}

//****************************************************************************
// INCRBUILD 1
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL1)
// FCL_LEVEL1: Verify HAL
// build level 1 subroutine for all nodes
static inline void buildLevel1(void)
{
    setInterruptPriority();

    //
    // Connect inputs of the PI module and call the PID speed controller module
    //
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        sysVars.speedLoopCount = 0;
    }

    ctrlVars[sysVars.ctrlNode].IdRef = ctrlVars[sysVars.ctrlNode].IdRefSet;
    ctrlVars[sysVars.ctrlNode].IqRef = ctrlVars[sysVars.ctrlNode].IqRefSet;

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL1)

//****************************************************************************
// INCRBUILD 2
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL2)
// FCL_LEVEL2: Speed or position loop over IPC
// build level 2 subroutine for main node
static void buildLevel2(void)
{
    setInterruptPriority();

#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;

        ctrlVars[sysVars.ctrlNode].rc.SetpointValue = 0.0;
    }
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);
        ctrlVars[sysVars.ctrlNode].rc.SetpointValue = 0.0;
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                ctrlVars[sysVars.ctrlNode].speedRef;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {
        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;
        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        }
    }
    else
    {
        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;
        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
    }

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL2)

//****************************************************************************
// INCRBUILD 3
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL3)
// FCL_LEVEL3: Verify IPC & EtherCAT
// build level 3 subroutine for main node
static void buildLevel3(void)
{
    setInterruptPriority();

    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        sysVars.speedLoopCount = 0;
    }

    // Send back the received data for verify the EtherCat
    if(dataBufferFromCM.ctrlSynchron == CTRL_SYN_ENABLE)
    {
        dataBufferToCM.statusNode[sysVars.ctrlNode].state =
                dataBufferFromCM.ctrlNode[SYS_NODEM].command;
        dataBufferToCM.statusNode[sysVars.ctrlNode].speed =
                dataBufferFromCM.ctrlNode[SYS_NODEM].speedRef;
        dataBufferToCM.statusNode[sysVars.ctrlNode].position =
                dataBufferFromCM.ctrlNode[SYS_NODEM].positionRef;
    }
    else
    {
        dataBufferToCM.statusNode[sysVars.ctrlNode].state =
                dataBufferFromCM.ctrlNode[sysVars.ctrlNode].command;
        dataBufferToCM.statusNode[sysVars.ctrlNode].speed =
                dataBufferFromCM.ctrlNode[sysVars.ctrlNode].speedRef;
        dataBufferToCM.statusNode[sysVars.ctrlNode].position =
                dataBufferFromCM.ctrlNode[sysVars.ctrlNode].positionRef;
    }

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL3)

//****************************************************************************
// INCRBUILD 4, 11
//****************************************************************************
#if((BUILDLEVEL == FCL_LEVEL4) || (BUILDLEVEL == FCL_LEVEL11))
// FCL_LEVEL4, 11: Speed or position loop over FSI, IPC & EtherCAT
// build level 4 or 11 subroutine for all nodes (main & node1~4)
static void buildLevel4_11(void)
{
    setInterruptPriority();

#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlModeSet == CTRL_MODE_SPEED)
        {
            if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
            {
                ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                        ctrlVars[sysVars.ctrlNode].speedRef;
            }

            fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

            ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                     ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        }
        else if(ctrlVars[sysVars.ctrlNode].ctrlModeSet == CTRL_MODE_POSITION)
        {
            if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
            {
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                        ctrlVars[sysVars.ctrlNode].posMechTheta;
            }
            else
            {
                ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                        refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                                  &ctrlVars[sysVars.ctrlNode]);

                ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

                // Rolling in angle within 0 to 1pu
                if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0)
                {
                    ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
                }
            }

            ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                    ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
            ctrlVars[sysVars.ctrlNode].pi_pos.Ref =
                    ctrlVars[sysVars.ctrlNode].ctrlPosRef;
            ctrlVars[sysVars.ctrlNode].pi_pos.Fbk =
                    ctrlVars[sysVars.ctrlNode].posMechTheta;

            runPIPos(&ctrlVars[sysVars.ctrlNode].pi_pos);

            ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                    ctrlVars[sysVars.ctrlNode].pi_pos.Out;
        }

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef >= 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                   -ctrlVars[sysVars.ctrlNode].ctrlSpdMinOut;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMinOut;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;

        ctrlVars[sysVars.ctrlNode].pi_pos.ui = 0;
        ctrlVars[sysVars.ctrlNode].pi_pos.i1 = 0;

        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0;
        ctrlVars[sysVars.ctrlNode].rc.SetpointValue = 0;

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut = 0.0;
    }
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlModeSet == CTRL_MODE_SPEED)
        {
            if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
            {
                ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                        ctrlVars[sysVars.ctrlNode].speedRef;
            }

            fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

            ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                     ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        }
        else if(ctrlVars[sysVars.ctrlNode].ctrlModeSet == CTRL_MODE_POSITION)
        {
            if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
            {
                ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                    refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                              &ctrlVars[sysVars.ctrlNode]);

                ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                    (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

                // Rolling in angle within 0.0 to 1.0pu
                if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0.0)
                {
                    ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
                }

                ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                        ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
            }

            ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                    DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_pos,
                                 ctrlVars[sysVars.ctrlNode].ctrlPosRef,
                                 ctrlVars[sysVars.ctrlNode].posMechTheta);
        }

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_pos);
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);

        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlSpdOut = 0.0;
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)


    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;

        ctrlVars[sysVars.ctrlNode].ctrlModeCom =
                ctrlVars[sysVars.ctrlNode].ctrlModeSet;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {

        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef >= 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
    }
    else    // (ctrlVars[sysVars.ctrlNode].ctrlStateSet == CTRL_MODE_STOP)
    {

        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0;
        ctrlVars[sysVars.ctrlNode].rc.SetpointValue = 0;

        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef = 0;
        ctrlVars[sysVars.ctrlNode].ctrlSpdOut = 0.0;

        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_STOP;
    }

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL4, 11)

//****************************************************************************
// INCRBUILD 5
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL5)
// FCL_LEVEL5: Verify FSI
// build level 5 subroutine for all slave nodes (node1~4)
static inline void buildLevel5(void)
{
    setInterruptPriority();

    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        sysVars.speedLoopCount = 0;
    }

    ctrlVars[sysVars.ctrlNode].ctrlStateCom = CTRL_STOP;
    ctrlVars[sysVars.ctrlNode].IdRef = ctrlVars[sysVars.ctrlNode].IdRefSet;
    ctrlVars[sysVars.ctrlNode].IqRef = ctrlVars[sysVars.ctrlNode].IqRefSet;

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL5)


//****************************************************************************
// INCRBUILD 6
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL6)
// FCL_LEVEL6: Verify torque current control over FSI
// build level 6 subroutine for all slave nodes (node1~4)
static inline void buildLevel6(void)
{
    setInterruptPriority();

    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        sysVars.speedLoopCount = 0;
    }

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;
    ctrlVars[sysVars.ctrlNode].IdRef = ctrlVars[sysVars.ctrlNode].IdRefSet;
    ctrlVars[sysVars.ctrlNode].IqRef = ctrlVars[sysVars.ctrlNode].IqRefSet;

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL6)

//****************************************************************************
// INCRBUILD 7, and 9
//****************************************************************************
#if((BUILDLEVEL == FCL_LEVEL7) || (BUILDLEVEL == FCL_LEVEL9))
// FCL_LEVEL6: Verify speed loop over FSI
// FCL_LEVEL8: SFRA integration to verify speed loop bandwidth
// build level 6&8 subroutine for motor
static inline void buildLevel7_9(void)
{
    setInterruptPriority();

#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

    #if(BUILDLEVEL == FCL_LEVEL9)   // enables SFRA
        // SFRA Noise injection in speed loop
        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue + sfraNoiseW;
    #else  // (BUILDLEVEL == FCL_LEVEL9)
        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
    #endif // (BUILDLEVEL == FCL_LEVEL7)

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;
    }
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                    ctrlVars[sysVars.ctrlNode].speedRef;
        }

        //
        //  Connect inputs of the RMP module and call the ramp control module
        //
        fclRampControl(&ctrlVars[sysVars.ctrlNode].rc);

    #if(BUILDLEVEL == FCL_LEVEL9)   // enables SFRA
        // SFRA Noise injection in speed loop
        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue + sfraNoiseW;
    #else  // (BUILDLEVEL == FCL_LEVEL9)
        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
    #endif // (BUILDLEVEL == FCL_LEVEL7)

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {

        ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                ctrlVars[sysVars.ctrlNode].speedRef;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {

        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {

        ctrlVars[sysVars.ctrlNode].rc.TargetValue = 0.0;

        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
    }
    else
    {

        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_STOP;
    }

    return;
}

#endif // ((BUILDLEVEL==FCL_LEVEL7) || (BUILDLEVEL == FCL_LEVEL9))


//****************************************************************************
// INCRBUILD 8
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL8)
// FCL_LEVEL8: verifies the position control
// build level 8 subroutine for motor for all slave nodes (node1~4)
static void buildLevel8(void)
{
    setInterruptPriority();

#if(SPD_CNTLR == SPD_PID_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].posMechTheta;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                    refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                              &ctrlVars[sysVars.ctrlNode]);

            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

            // Rolling in angle within 0 to 1pu
            if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0)
            {
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
            }
        }

        ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;
        ctrlVars[sysVars.ctrlNode].pi_pos.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlPosRef;
        ctrlVars[sysVars.ctrlNode].pi_pos.Fbk =
                ctrlVars[sysVars.ctrlNode].posMechTheta;

        runPIPos(&ctrlVars[sysVars.ctrlNode].pi_pos);

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                ctrlVars[sysVars.ctrlNode].pi_pos.Out;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Ref =
                ctrlVars[sysVars.ctrlNode].ctrlSpeedRef;

        ctrlVars[sysVars.ctrlNode].pid_spd.term.Fbk =
                ctrlVars[sysVars.ctrlNode].speedWe;

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].pid_spd.param.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        runPID(&ctrlVars[sysVars.ctrlNode].pid_spd);

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                ctrlVars[sysVars.ctrlNode].pid_spd.term.Out;

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.d2 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.i1 = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ud = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.ui = 0;
        ctrlVars[sysVars.ctrlNode].pid_spd.data.up = 0;

        ctrlVars[sysVars.ctrlNode].pi_pos.ui = 0;
        ctrlVars[sysVars.ctrlNode].pi_pos.i1 = 0;

    }

#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        switchActiveNode();

        if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
        {
            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                    ctrlVars[sysVars.ctrlNode].posMechTheta;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].rc.TargetValue =
                refPosGen(ctrlVars[sysVars.ctrlNode].rc.TargetValue,
                          &ctrlVars[sysVars.ctrlNode]);

            ctrlVars[sysVars.ctrlNode].rc.SetpointValue =
                ctrlVars[sysVars.ctrlNode].rc.TargetValue -
                (float32_t)((int32_t)ctrlVars[sysVars.ctrlNode].rc.TargetValue);

            // Rolling in angle within 0.0 to 1.0pu
            if(ctrlVars[sysVars.ctrlNode].rc.SetpointValue < 0.0)
            {
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue += 1.0;
            }
        }

        ctrlVars[sysVars.ctrlNode].ctrlPosRef =
                ctrlVars[sysVars.ctrlNode].rc.SetpointValue;

        ctrlVars[sysVars.ctrlNode].ctrlSpeedRef =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_pos,
                             ctrlVars[sysVars.ctrlNode].ctrlPosRef,
                             ctrlVars[sysVars.ctrlNode].posMechTheta);

        if(ctrlVars[sysVars.ctrlNode].ctrlSpeedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax =
                    ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin = 0.0;
        }
        else
        {
            ctrlVars[sysVars.ctrlNode].dcl_spd.Umax = 0.0;

            ctrlVars[sysVars.ctrlNode].dcl_spd.Umin =
                    -ctrlVars[sysVars.ctrlNode].ctrlSpdMaxOut;
        }

        ctrlVars[sysVars.ctrlNode].ctrlSpdOut =
                DCL_runPI_C1(&ctrlVars[sysVars.ctrlNode].dcl_spd,
                             ctrlVars[sysVars.ctrlNode].ctrlSpeedRef,
                             ctrlVars[sysVars.ctrlNode].speedWe);

        sysVars.speedLoopCount = 0;
    }

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb != CTRL_RUN)
    {
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_pos);
        DCL_resetPI(&ctrlVars[sysVars.ctrlNode].dcl_spd);
    }

#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].ctrlIdRef;

        ctrlVars[sysVars.ctrlNode].IqRef =
                ctrlVars[sysVars.ctrlNode].ctrlSpdOut;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_FAULT)
    {
        ctrlVars[sysVars.ctrlNode].IdRef = 0.0;
        ctrlVars[sysVars.ctrlNode].IqRef = 0.0;
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_FAULT;
    }
    else if(ctrlVars[sysVars.ctrlNode].ctrlStateCom == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].IdRef =
                ctrlVars[sysVars.ctrlNode].IdRefStart;

        if(ctrlVars[sysVars.ctrlNode].speedRef > 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
        else if(ctrlVars[sysVars.ctrlNode].speedRef < 0.0)
        {
            ctrlVars[sysVars.ctrlNode].IqRef =
                    -ctrlVars[sysVars.ctrlNode].IqRefStart;
        }
    }

    return;
}

#endif // (BUILDLEVEL==FCL_LEVEL8)


//****************************************************************************
// INCRBUILD 10
//****************************************************************************
#if(BUILDLEVEL == FCL_LEVEL10)
// FCL_LEVEL10: Verify EtherCAT & FSI without control loop
// build level 10 subroutine
static void buildLevel10(void)
{
    setInterruptPriority();

    sysVars.speedLoopCount++;

    if(sysVars.speedLoopCount >= sysVars.speedLoopPrescaler)
    {
        // Update sysVars.ctrlNode
        switchActiveNode();

        sysVars.speedLoopCount = 0;
    }

    ctrlVars[sysVars.ctrlNode].ctrlStateCom =
            ctrlVars[sysVars.ctrlNode].ctrlStateSet;
    ctrlVars[sysVars.ctrlNode].IdRef = ctrlVars[sysVars.ctrlNode].IdRefSet;
    ctrlVars[sysVars.ctrlNode].IqRef = ctrlVars[sysVars.ctrlNode].IqRefSet;

    if(ctrlVars[sysVars.ctrlNode].ctrlStateFdb == CTRL_RUN)
    {
        ctrlVars[sysVars.ctrlNode].ctrlModeCom =
                ctrlVars[sysVars.ctrlNode].ctrlModeSet;
    }
    else
    {
        ctrlVars[sysVars.ctrlNode].ctrlModeCom = CTRL_MODE_STOP;
    }

    return;
}
#endif // (BUILDLEVEL==FCL_LEVEL10)


__interrupt void motorControlISR(void)
{

#if(BUILDLEVEL == FCL_LEVEL1)
    buildLevel1();

#elif(BUILDLEVEL == FCL_LEVEL2)
    buildLevel2();

#elif(BUILDLEVEL == FCL_LEVEL3)
    buildLevel3();

#elif(BUILDLEVEL == FCL_LEVEL4)
    buildLevel4_11();

#elif(BUILDLEVEL == FCL_LEVEL5)
    buildLevel5();

#elif(BUILDLEVEL == FCL_LEVEL6)
    buildLevel6();

#elif(BUILDLEVEL == FCL_LEVEL7)
    buildLevel7_9();

#elif(BUILDLEVEL == FCL_LEVEL8)
    buildLevel8();

#elif(BUILDLEVEL == FCL_LEVEL9)
    buildLevel7_9();

#elif(BUILDLEVEL == FCL_LEVEL10)
    buildLevel10();

#elif(BUILDLEVEL == FCL_LEVEL11)
    buildLevel4_11();

#endif  //(BUILDLEVEL == FCL_LEVELxx)


//------------------------------------------------------------------------------
// Variable display on DACs
//------------------------------------------------------------------------------
#ifdef DACOUT_EN
    DAC_setShadowValue(hal.dacHandle[0], dacConvertData(&dacDataView[0]));
    DAC_setShadowValue(hal.dacHandle[1], dacConvertData(&dacDataView[1]));
#endif   // DACOUT_EN

// ----------------------------------------------------------------------------
//    Call the DATALOG update function.
// ----------------------------------------------------------------------------
#ifdef DLOG_ENABLE
// -----------------------------------------------------------------------------
//    Connect inputs of the DATALOG module
// -----------------------------------------------------------------------------
    dlog6ChData[0] = ctrlVars[0].ctrlSpeedRef;

    dlog6ChData[1] = ctrlVars[0].speedWe;
    dlog6ChData[2] = ctrlVars[1].speedWe;
    dlog6ChData[3] = ctrlVars[2].speedWe;
    dlog6ChData[4] = ctrlVars[3].speedWe;
    dlog6ChData[5] = ctrlVars[4].speedWe;

//    dlog6ChData[1] = ctrlVars[0].posMechTheta;
//    dlog6ChData[2] = ctrlVars[1].posMechTheta;
//    dlog6ChData[3] = ctrlVars[2].posMechTheta;
//    dlog6ChData[4] = ctrlVars[3].posMechTheta;
//    dlog6ChData[5] = ctrlVars[4].posMechTheta;

    DLOG_6CH_F_FUNC(&dlog_6ch);
#endif  // DLOG_ENABLE

    sysVars.isrTicker++;

    // Acknowledges an interrupt
    HAL_ackInt(halHandle);

    restoreInterruptRegisters();

} // motor1ControlISR Ends Here

//
// run the controller
//

void runController(SysNode_e node)
{
#if((BUILDLEVEL >= FCL_LEVEL5) && (BUILDLEVEL <= FCL_LEVEL9))
    if(sysVars.ctrlSynSet == CTRL_SYN_ENABLE)
    {
        ctrlVars[node].ctrlStateSet = sysVars.ctrlStateSet;
        ctrlVars[node].speedRef = sysVars.speedSet;
        ctrlVars[node].positionSet = sysVars.positionSet;
    }
    else
    {
        ctrlVars[node].speedRef = ctrlVars[node].speedSet;
        ctrlVars[node].positionRef = ctrlVars[node].positionSet;
    }
#endif  // ((BUILDLEVEL >= FCL_LEVEL5) && (BUILDLEVEL <= FCL_LEVEL9))

    return;
}

//
// reset the controller
//
void resetControllerVars(CTRL_Vars_t *pCtrl)
{
    pCtrl->ctrlStateCom = CTRL_STOP;
    pCtrl->ctrlStateFdb = CTRL_STOP;

#if(SPD_CNTLR == SPD_PID_CNTLR)
    pCtrl->pid_spd.data.d1 = 0;
    pCtrl->pid_spd.data.d2 = 0;
    pCtrl->pid_spd.data.i1 = 0;
    pCtrl->pid_spd.data.ud = 0;
    pCtrl->pid_spd.data.ui = 0;
    pCtrl->pid_spd.data.up = 0;
#endif  // (SPD_CNTLR == SPD_PID_CNTLR)

#if(SPD_CNTLR == SPD_DCL_CNTLR)
    DCL_resetPI(&pCtrl->dcl_spd);
#endif  // (SPD_CNTLR == SPD_DCL_CNTLR)

    return;
}

//
// POSITION LOOP UTILITY FUNCTIONS
//

// slew programmable ramper
float32_t ramper(float32_t in, float32_t out, float32_t rampDelta)
{
    float32_t err;

    err = in - out;

    if(err > rampDelta)
    {
        return(out + rampDelta);
    }
    else if(err < -rampDelta)
    {
        return(out - rampDelta);
    }
    else
    {
        return(in);
    }
}

//
// Reference Position Generator for position loop
//
float32_t refPosGen(float32_t out, CTRL_Vars_t *pCtrl)
{
    float32_t in = pCtrl->posArray[pCtrl->posBufPtr];

    out = ramper(in, out, pCtrl->posSlewRate);

    if(in == out)
    {
        pCtrl->posRampCntr++;

        if(pCtrl->posRampCntr > pCtrl->posRampMax)
        {
            pCtrl->posRampCntr = 0;

            pCtrl->posBufPtr++;

            if(pCtrl->posBufPtr >= pCtrl->posBufMax)
            {
                pCtrl->posBufPtr = 0;
            }
        }
    }

    return(out);
}

// *************************************************************************
// Using SFRA tool :
// =================
//      - INJECT noise
//      - RUN the controller
//      - CAPTURE or COLLECT the controller output
// From a controller analysis standpoint, this sequence will reveal the
// output of controller for a given input, and therefore, good for analysis
// *************************************************************************
void injectSFRA(void)
{
    sfraNoiseW = SFRA_F32_inject(0.0);
    return;
}

// ****************************************************************************
void collectSFRA(CTRL_Vars_t *pCtrl)
{
    SFRA_F32_collect(&pCtrl->ctrlSpdOut, &pCtrl->speedWe);
    return;
}

//
// End of Code
//
