//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
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
//! \file   /solutions/fast_uni_lab/common/source/sys_main.c.c
//!
//! \brief  This project is used to implement motor control with FAST, eSMO
//!         Encoder, and Hall sensors based sensored/sensorless-FOC.
//!         Supports multiple TI EVM board with F28002x
//!
//

//
// include the related header files
//

#include "sys_settings.h"
#include "sys_main.h"

//
volatile SYSTEM_Vars_t systemVars;
#pragma DATA_SECTION(systemVars,"sys_data");

#ifdef CPUTIME_ENABLE
// define CPU time for performance test
CPU_TIME_Obj     cpuTime;
CPU_TIME_Handle  cpuTimeHandle;
#pragma DATA_SECTION(cpuTime,"sys_data");
#pragma DATA_SECTION(cpuTimeHandle,"sys_data");
#endif  // CPUTIME_ENABLE

#if defined(EPWMDAC_MODE)
HAL_PWMDACData_t pwmDACData;
#pragma DATA_SECTION(pwmDACData,"sys_data");
#endif  // EPWMDAC_MODE

#if defined(DAC128S_ENABLE)
DAC128S_Handle   dac128sHandle;        //!< the DAC128S interface handle
DAC128S_Obj      dac128s;              //!< the DAC128S interface object
#pragma DATA_SECTION(dac128sHandle,"sys_data");
#pragma DATA_SECTION(dac128s,"sys_data");
#endif  // DAC128S_ENABLE

#if defined(SFRA_ENABLE)
float32_t   sfraNoiseId;
float32_t   sfraNoiseIq;
float32_t   sfraNoiseSpd;
float32_t   sfraNoiseOut;
float32_t   sfraNoiseFdb;
SFRATest_e  sfraTestLoop;        //speedLoop;
bool        sfraCollectStart;
#endif  // SFRA_ENABLE

// **************************************************************************
// the functions
// !!! Please make sure that you had went through the user guide, and follow the
// !!! guide to set up the kit and load the right code
void main(void)
{
    //Clear memory for system and controller
    // If the link command file (.cmd) is changed, the following xxx_RAM_START_ADDR
    // and  xxx_RAM_VARS_SIZE must be reset accordingly !!!!
    HAL_clearDataRAM((void *)USER_RAM_START_ADDR, (uint16_t)USER_RAM_VARS_SIZE);
    HAL_clearDataRAM((void *)CTRL_RAM_START_ADDR, (uint16_t)CTRL_RAM_VARS_SIZE);
    HAL_clearDataRAM((void *)MTRS_RAM_START_ADDR, (uint16_t)MTRS_RAM_VARS_SIZE);

#if defined(BSXL8323RS_REVA)
    systemVars.boardKit = BOARD_BSXL8323RS_REVA;    // BSXL8323RS_REVA
#elif defined(BSXL8323RH_REVB)
    systemVars.boardKit = BOARD_BSXL8323RH_REVB;    // BSXL8323RH_REVB
#elif defined(BSXL8353RS_REVA)
    systemVars.boardKit = BOARD_BSXL8353RS_REVA;    // BSXL8353RS_REVA
#elif defined(BSXL3PHGAN_REVA)
    systemVars.boardKit = BOARD_BSXL3PHGAN_REVA;    // BSXL3PHGAN_REVA
#elif defined(HVMTRPFC_REV1P1)
    systemVars.boardKit = BOARD_HVMTRPFC_REV1P1;    // HVMTRPFC_REV1P1
#elif defined(BSXL8316RT_REVA)
    systemVars.boardKit = BOARD_BSXL8316RT_REVA;    // BSXL8316RT_REVA
#else
#error Not select a right board for this project
#endif

#if defined(MOTOR1_ISBLDC) && (defined(MOTOR1_FAST) || \
    defined(MOTOR1_ESMO) || defined(MOTOR1_ENC) || defined(MOTOR1_HALL))
#error ISBLDC can't work with other estimaor simultaneously
#elif defined(MOTOR1_ENC) && defined(MOTOR1_HALL)
#error Can't support ENC and HALL simultaneously
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_HALL)
#error Can't support ESMO and HALL simultaneously
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_FAST_ENC;     // the estimator is FAST and ENC
#elif defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    systemVars.estType = EST_TYPE_FAST_ESMO;    // the estimator is FAST and ESMO
#elif defined(MOTOR1_FAST) && defined(MOTOR1_HALL)
    systemVars.estType = EST_TYPE_FAST_HALL;    // the estimator is FAST and HALL
#elif defined(MOTOR1_ESMO) && defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ESMO_ENC;     // the estimator is ESMO and ENC
#elif defined(MOTOR1_FAST)
    systemVars.estType = EST_TYPE_FAST;         // the estimator is only FAST
#elif defined(MOTOR1_ESMO)
    systemVars.estType = EST_TYPE_ESMO;         // the estimator is only ESMO
#elif defined(MOTOR1_ENC)
    systemVars.estType = EST_TYPE_ENC;          // the estimator is only ENC
#elif defined(MOTOR1_HALL)
    systemVars.estType = EST_TYPE_HALL;         // the estimator is only HALL
#elif defined(MOTOR1_ISBLDC)
    systemVars.estType = EST_TYPE_ISBLDC;       // the estimator is only ISBLDC
#else
#error Not select a right estimator for this project
#endif  // MOTOR1_FAST->MOTOR1_ENC

#if defined(_SOFT_LIB) && defined(_FLASH)
    // the FAST software library and run in Flash
    systemVars.fastType = FAST_TYPE_SOFTLIB_FLASH;
#elif defined(_ROM_SYMBOLS) && defined(_FLASH)
    // the FAST ROM library and run in Flash
    systemVars.fastType = FAST_TYPE_ROMLIB_FLASH;
#elif defined(_ROM_SYMBOLS) && defined(_RAM)
    // the FAST ROM library and run in RAM
    systemVars.fastType = FAST_TYPE_ROMLIB_RAM;
#elif defined(_FLASH)
    // the non FAST and run in Flash
    systemVars.fastType = FAST_TYPE_NONFAST_FLASH;
#elif defined(_RAM)
    // the non FAST and run in RAM
    systemVars.fastType = FAST_TYPE_NONFAST_RAM;
#else
#error Not select a right FAST type for this project
#endif  // FAST_Type

#if defined(MOTOR1_DCLINKSS) || defined(MOTOR1_ISBLDC)
    systemVars.currentSenseType = CURSEN_TYPE_SINGLE_SHUNT;
#elif defined(BSXL3PHGAN_REVA)
    systemVars.currentSenseType = CURSEN_TYPE_INLINE_SHUNT;
#else
    systemVars.currentSenseType = CURSEN_TYPE_THREE_SHUNT;
#endif  // Current Sense Type

// ** below codes are only for checking the settings
#if defined(MOTOR1_ENC) && !defined(QEP_ENABLE)
#error Must set QEP_ENABLE pre-defined symbols if uses MOTOR1_ENC
#endif  // MOTOR1_ENC & QEP_ENABLE

#if defined(MOTOR1_HALL) && !defined(HALL_ENABLE)
#error Must set HALL_ENABLE if uses MOTOR1_HALL
#endif  // MOTOR1_HALL & HALL_ENABLE

#if defined(MOTOR1_DCLINKSS) && \
    (!defined(BSXL8323RH_REVB) && !defined(BSXL8323RS_REVA))
#error Only modificated BSXL8323RS_REVA and BSXL8323RS_REVA support single shunt
#endif  // MOTOR1_HALL & HALL_ENABLE

#if defined(MOTOR1_ISBLDC) && (!defined(BSXL8323RH_REVB) && \
        !defined(BSXL8323RS_REVA))
#error Only modificated BSXL8323RS_REVA and BSXL8323RS_REVA support ISBLDC
#endif  // MOTOR1_HALL & HALL_ENABLE

#if defined(DATALOGF2_EN) && defined(STEP_RP_EN)
#error DATALOG and GRAPH_STEP_RESPONSE can't be used simultaneously on this device
#endif  // DATALOGF2_EN && STEP_RP_EN

#if defined(MOTOR1_ISBLDC) && defined(MOTOR1_DCLINKSS)
#error Don't need to enable single shunt pre-define name if use instaspin-bldc
#endif  // MOTOR1_ISBLDC & MOTOR1_DCLINKSS

#if (defined(MOTOR1_SSIPD) || defined(MOTOR1_OVM)) && defined(MOTOR1_DCLINKSS)
#error Don't enable SSIPD and OVM if enable single shunt
#endif  // (MOTOR1_SSIPD | MOTOR1_OVM) & (MOTOR1_DCLINKSS

#if defined(MOTOR1_ISBLDC) && (defined(MOTOR1_OVM) || defined(MOTOR1_FWC) || \
        defined(MOTOR1_MTPA) || defined(MOTOR1_SSIPD))
#error Don't need to enable these functions if use instaspin-bldc
#endif  // MOTOR1_ISBLDC & (MOTOR1_OVM | MOTOR1_FWC | MOTOR1_MTPA | MOTOR1_SSIPD)
// ** above codes are only for checking the settings, not occupy the memory

    // initialize the driver
    halHandle = HAL_init(&hal, sizeof(hal));

    // set the driver parameters
    HAL_setParams(halHandle);

    // set the control parameters for motor 1
    motorHandle_M1 = (MOTOR_Handle)(&motorVars_M1);

    // set the reference speed, this can be replaced or removed
    motorVars_M1.speedRef_Hz = 40.0f;

    // false - enables identification, true - disables identification
    userParams_M1.flag_bypassMotorId = true;

    initMotor1Handles(motorHandle_M1);

    // initialize motor control parameters
    initMotor1CtrlParameters(motorHandle_M1);

#if defined(CMD_POT_EN)
    setExtCmdPotParams(motorHandle_M1);
#endif  // CMD_POT_EN

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
    setExtCmdCapParams(motorHandle_M1);
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
    setExtCmdSwitchParams(motorHandle_M1);
#endif  //CMD_SWITCH_EN

#ifdef CPUTIME_ENABLE
    // initialize the CPU usage module
    cpuTimeHandle = CPU_TIME_init(&cpuTime, sizeof(cpuTime));
    CPU_TIME_reset(cpuTimeHandle);
#endif  // CPUTIME_ENABLE

#if defined(EPWMDAC_MODE)
    // set DAC parameters
    pwmDACData.periodMax =
            PWMDAC_getPeriod(halHandle->pwmDACHandle[PWMDAC_NUMBER_1]);

    pwmDACData.ptrData[0] = &motorVars_M1.angleFOC_rad;             // PWMDAC1

#if defined(MOTOR1_FAST)
    pwmDACData.ptrData[1] = &motorVars_M1.angleEST_rad;             // PWMDAC2
#elif defined(MOTOR1_ESMO)
    pwmDACData.ptrData[1] = &motorVars_M1.anglePLL_rad;             // PWMDAC2
#elif defined(MOTOR1_ENC)
    pwmDACData.ptrData[1] = &motorVars_M1.angleENC_rad;             // PWMDAC2
#elif defined(MOTOR1_HALL)
    pwmDACData.ptrData[1] = &motorVars_M1.angleHall_rad;            // PWMDAC2
#endif  // MOTOR1_FAST|| MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_HALL

#if defined(MOTOR1_ENC)
    pwmDACData.ptrData[2] = &motorVars_M1.angleENC_rad;             // PWMDAC3
#elif defined(MOTOR1_ESMO)
    pwmDACData.ptrData[2] = &motorVars_M1.anglePLL_rad;             // PWMDAC3
#elif defined(MOTOR1_HALL)
    pwmDACData.ptrData[2] = &motorVars_M1.angleHall_rad;            // PWMDAC3
#elif defined(MOTOR1_FAST)
    pwmDACData.ptrData[2] = &motorVars_M1.angleEST_rad;             // PWMDAC3
#endif  // MOTOR1_FAST|| MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_HALL

    pwmDACData.ptrData[3] = &motorVars_M1.adcData.I_A.value[0];     // PWMDAC4

    pwmDACData.offset[0] = 0.5f;
    pwmDACData.offset[1] = 0.5f;
    pwmDACData.offset[2] = 0.5f;
    pwmDACData.offset[3] = 0.5f;

    pwmDACData.gain[0] = 1.0f / MATH_TWO_PI;
    pwmDACData.gain[1] = 1.0f / MATH_TWO_PI;
    pwmDACData.gain[2] = 1.0f / MATH_TWO_PI;
    pwmDACData.gain[3] = 4096.0f / USER_MOTOR1_OVER_CURRENT_A;
#endif  // EPWMDAC_MODE

#if defined(DATALOGF2_EN)
    // Initialize Datalog
    datalogHandle = DATALOGIF_init(&datalog, sizeof(datalog));
    DATALOG_Obj *datalogObj = (DATALOG_Obj *)datalogHandle;

    HAL_setupDMAforDLOG(halHandle, 0, &datalogBuff1[0], &datalogBuff1[1]);
    HAL_setupDMAforDLOG(halHandle, 1, &datalogBuff2[0], &datalogBuff2[1]);

#if (DMC_BUILDLEVEL <= DMC_LEVEL_2)
    // set datalog parameters
    datalogObj->iptr[0] = &motorVars_M1.adcData.I_A.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_3)
    datalogObj->iptr[0] = &motorVars_M1.adcData.V_V.value[0];
    datalogObj->iptr[1] = &motorVars_M1.adcData.V_V.value[1];
#elif (DMC_BUILDLEVEL == DMC_LEVEL_4)
#if defined(MOTOR1_FAST) && defined(MOTOR1_ESMO)
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.angleEST_rad;
#else
    datalogObj->iptr[0] = &motorVars_M1.angleFOC_rad;
    datalogObj->iptr[1] = &motorVars_M1.adcData.I_A.value[0];
#endif  // MOTOR1_FAST & MOTOR1_ESMO
#endif  // DMC_BUILDLEVEL = DMC_LEVEL_1/2/3/4
#endif  // DATALOGI4_EN || DATALOGF2_EN

#if defined(DAC128S_ENABLE)
    // initialize the DAC128S
    dac128sHandle = DAC128S_init(&dac128s);

    // setup SPI for DAC128S
    DAC128S_setupSPI(dac128sHandle);
#if defined(MOTOR1_ISBLDC)
    dac128s.ptrData[0] = &bldc_M1.IdcInFilter;                      // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.V_V.value[0];        // CH_B
    dac128s.ptrData[2] = &isbldc_M1.bemfInt;                        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.V_V.value[1];        // CH_D
    dac128s.ptrData[4] = &motorVars_M1.adcData.V_V.value[2];        // CH_E
    dac128s.ptrData[5] = &isbldc_M1.Vabcn.value[0];                 // CH_F
    dac128s.ptrData[6] = &isbldc_M1.Vabcn.value[1];                 // CH_G
    dac128s.ptrData[7] = &isbldc_M1.Vabcn.value[2];                 // CH_H

    dac128s.gain[0] = 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[1] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[4] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[5] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[6] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[6] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[7] = 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[4] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[5] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[6] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[7] = (uint16_t)(0.5f * 4096.0f);
#elif (DMC_BUILDLEVEL >= DMC_LEVEL_4)   // !MOTOR1_ISBLDC
    dac128s.ptrData[0] = &motorVars_M1.angleFOC_rad;                // CH_A

#if defined(MOTOR1_FAST)
    dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_B
#elif defined(MOTOR1_ESMO)
    dac128s.ptrData[1] = &motorVars_M1.anglePLL_rad;                // CH_B
#elif defined(MOTOR1_ENC)
    dac128s.ptrData[1] = &motorVars_M1.angleENC_rad;                // CH_B
#elif defined(MOTOR1_HALL)
    dac128s.ptrData[1] = &motorVars_M1.angleHall_rad;               // CH_B
#endif  // MOTOR1_FAST|| MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_HALL

#if defined(MOTOR1_ENC)
    dac128s.ptrData[2] = &motorVars_M1.angleENC_rad;                // CH_C
#elif defined(MOTOR1_ESMO)
    dac128s.ptrData[2] = &motorVars_M1.anglePLL_rad;                // CH_C
#elif defined(MOTOR1_HALL)
    dac128s.ptrData[2] = &motorVars_M1.angleHall_rad;               // CH_C
#elif defined(MOTOR1_FAST)
    dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_C
#endif  // MOTOR1_ESMO || MOTOR1_ENC || MOTOR1_HALL || MOTOR1_FAST

    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[0];        // CH_D
    dac128s.ptrData[4] = &motorVars_M1.adcData.I_A.value[1];        // CH_E
    dac128s.ptrData[5] = &motorVars_M1.adcData.I_A.value[2];        // CH_F
    dac128s.ptrData[6] = &motorVars_M1.adcData.V_V.value[0];        // CH_G
    dac128s.ptrData[7] = &motorVars_M1.adcData.V_V.value[0];        // CH_H

    dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[1] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[2] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[3] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[4] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[5] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[6] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[7] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[4] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[5] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[6] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[7] = (uint16_t)(0.5f * 4096.0f);
#elif (DMC_BUILDLEVEL <= DMC_LEVEL_3)
/*
    // Build_Level_2, verify the current sampling value
    dac128s.ptrData[0] = &motorVars_M1.adcData.I_A.value[0];        // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.I_A.value[1];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.I_A.value[2];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.angleGen_rad;                // CH_D
    dac128s.ptrData[4] = &motorVars_M1.angleEST_rad;                // CH_E, N/A
    dac128s.ptrData[5] = &motorVars_M1.adcData.V_V.value[0];        // CH_F, N/A
    dac128s.ptrData[6] = &motorVars_M1.adcData.V_V.value[1];        // CH_G, N/A
    dac128s.ptrData[7] = &motorVars_M1.adcData.V_V.value[2];        // CH_H, N/A

    dac128s.gain[0] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[1] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[2] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[3] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[4] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[5] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[6] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[7] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
*/
/*
    // Build_Level_2, verify the voltage sampling value
    dac128s.ptrData[0] = &motorVars_M1.adcData.V_V.value[0];        // CH_A
    dac128s.ptrData[1] = &motorVars_M1.adcData.V_V.value[1];        // CH_B
    dac128s.ptrData[2] = &motorVars_M1.adcData.V_V.value[2];        // CH_C
    dac128s.ptrData[3] = &motorVars_M1.angleGen_rad;                // CH_D
    dac128s.ptrData[4] = &motorVars_M1.angleEST_rad;                // CH_E, N/A
    dac128s.ptrData[5] = &motorVars_M1.adcData.I_A.value[0];        // CH_F, N/A
    dac128s.ptrData[6] = &motorVars_M1.adcData.I_A.value[1];        // CH_G, N/A
    dac128s.ptrData[7] = &motorVars_M1.adcData.I_A.value[2];        // CH_H, N/A

    dac128s.gain[0] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[1] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[2] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[3] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[4] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[5] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[6] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[7] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
*/

    // Build_Level_2 or Level_3, verify the estimator
    dac128s.ptrData[0] = &motorVars_M1.angleGen_rad;                // CH_A
    dac128s.ptrData[1] = &motorVars_M1.angleEST_rad;                // CH_B
    dac128s.ptrData[2] = &motorVars_M1.anglePLL_rad;                // CH_C
    dac128s.ptrData[3] = &motorVars_M1.adcData.I_A.value[0];        // CH_D
    dac128s.ptrData[4] = &motorVars_M1.adcData.V_V.value[0];        // CH_E, N/A
    dac128s.ptrData[5] = &motorVars_M1.adcData.I_A.value[1];        // CH_F, N/A
    dac128s.ptrData[6] = &motorVars_M1.adcData.I_A.value[2];        // CH_G, N/A
    dac128s.ptrData[7] = &motorVars_M1.adcData.V_V.value[1];        // CH_H, N/A

    dac128s.gain[0] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[1] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[2] = 4096.0f / MATH_TWO_PI;
    dac128s.gain[3] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[4] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;
    dac128s.gain[5] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[6] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_CURRENT_A;
    dac128s.gain[7] = 2.0f * 4096.0f / USER_M1_ADC_FULL_SCALE_VOLTAGE_V;

    dac128s.offset[0] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[1] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[2] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[3] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[4] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[5] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[6] = (uint16_t)(0.5f * 4096.0f);
    dac128s.offset[7] = (uint16_t)(0.5f * 4096.0f);
#endif  // (DMC_BUILDLEVEL <= DMC_LEVEL_3)

    DAC128S_writeCommand(dac128sHandle);
#endif  // DAC128S_ENABLE

#if defined(SFRA_ENABLE)
    // Plot GH & H plots using SFRA_GUI, GH & CL plots using SFRA_GUI_MC
    configureSFRA(SFRA_GUI_PLOT_GH_H, USER_M1_ISR_FREQ_Hz);

    sfraNoiseId = 0.0f;
    sfraNoiseIq = 0.0f;
    sfraNoiseSpd = 0.0f;
    sfraNoiseOut = 0.0f;
    sfraNoiseFdb = 0.0f;
    sfraTestLoop = SFRA_TEST_D_AXIS;
    sfraCollectStart = false;
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
    GRAPH_init(&stepRPVars,
               &motorVars_M1.speedRef_Hz, &motorVars_M1.speed_Hz,
               &motorVars_M1.IdqRef_A.value[0], &motorVars_M1.Idq_in_A.value[0],
               &motorVars_M1.IdqRef_A.value[1], &motorVars_M1.Idq_in_A.value[1]);
#endif  // STEP_RP_EN

    systemVars.flagEnableSystem = true;

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC/PWM interrupts for control
    // enable interrupts to trig DMA
    HAL_enableCtrlInts(halHandle);

#if defined(CMD_CAN_EN)
    // setup the CAN
    HAL_setupCANA(halHandle);

    // setup the CAN interrupt
    HAL_enableCANInts(halHandle);

    // initialize the CANCOM
    initCANCOM();

    motorVars_M1.cmdCAN.speedSet_Hz = 40.0f;

    motorVars_M1.cmdCAN.flagEnablCmd = true;
    motorVars_M1.cmdCAN.flagEnablSyncLead = false;
#endif // CMD_CAN_EN

    motorVars_M1.flagEnableOffsetCalc = true;

    // run offset calibration for motor 1
    runMotor1OffsetsCalculation(motorHandle_M1);

    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    systemVars.powerRelayWaitTime_ms = POWER_RELAY_WAIT_TIME_ms;

    // Waiting for enable system flag to be set
    while(systemVars.flagEnableSystem == false)
    {
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            systemVars.timerBase_1ms++;

            if(systemVars.timerBase_1ms > systemVars.powerRelayWaitTime_ms)
            {
                systemVars.flagEnableSystem = true;
                systemVars.timerBase_1ms = 0;
            }
        }
    }

    while(systemVars.flagEnableSystem == true)
    {
        // loop while the enable system flag is true
        systemVars.mainLoopCnt++;

        // 1ms time base
        if(HAL_getCPUTimerStatus(halHandle, HAL_CPU_TIMER0))
        {
            HAL_clearCPUTimerFlag(halHandle, HAL_CPU_TIMER0);

            // toggle status LED
            systemVars.counterLED++;

            if(systemVars.counterLED > (uint16_t)(LED_BLINK_FREQ_Hz * 1000))
            {
                HAL_toggleLED(halHandle, HAL_GPIO_LED2C);

                systemVars.counterLED = 0;
            }

            systemVars.timerBase_1ms++;

            switch(systemVars.timerBase_1ms)
            {
                case 1:     // motor 1 protection check
                    runMotorMonitor(motorHandle_M1);
                    break;
                case 2:
                    calculateRMSData(motorHandle_M1);
                    break;
                case 3:
                    break;
                case 4:     // calculate motor protection value
                    calcMotorOverCurrentThreshold(motorHandle_M1);
                    break;
                case 5:     // system control
                    systemVars.timerBase_1ms = 0;
                    systemVars.timerCnt_5ms++;
                    break;
            }

#if defined(CMD_CAN_EN)
            updateCANCmdFreq(motorHandle_M1);

            if((motorVars_M1.cmdCAN.flagEnablCmd == true) && (motorVars_M1.faultMtrUse.all == 0))
            {
                canComVars.flagCmdTxRun = motorVars_M1.cmdCAN.flagCmdRun;
                canComVars.speedSet_Hz = motorVars_M1.cmdCAN.speedSet_Hz;

                if(motorVars_M1.cmdCAN.flagEnablSyncLead == true)
                {
                    motorVars_M1.flagEnableRunAndIdentify = motorVars_M1.cmdCAN.flagCmdRun;
                    motorVars_M1.speedRef_Hz = motorVars_M1.cmdCAN.speedSet_Hz;
                }
                else
                {
                    motorVars_M1.flagEnableRunAndIdentify = canComVars.flagCmdRxRun;
                    motorVars_M1.speedRef_Hz = canComVars.speedRef_Hz;
                }
            }
#endif // CMD_CAN_EN

#if defined(CMD_POT_EN)
            updateExtCmdPotFreq(motorHandle_M1);
#endif  // CMD_POT_EN

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(CMD_CAP_EN)
            updateExtCmdCapFreq(motorHandle_M1,
                                HAL_calcCAPCount(motorHandle_M1->halMtrHandle));
#endif  // CMD_CAP_EN

#if defined(CMD_SWITCH_EN)
            updateCmdSwitch(motorHandle_M1);
#endif  //CMD_SWITCH_EN

#if defined(SFRA_ENABLE)
            // SFRA test
            SFRA_F32_runBackgroundTask(&sfra1);
            SFRA_GUI_runSerialHostComms(&sfra1);
#endif  // SFRA_ENABLE

#if defined(STEP_RP_EN)
            // Generate Step response
            GRAPH_generateStepResponse(&stepRPVars);
#endif  // STEP_RP_EN

        }       // 1ms Timer

#if defined(CMD_SWITCH_EN)
        outputCmdState(motorHandle_M1);
#endif  //CMD_SWITCH_EN
        // run control for motor 1
        runMotor1Control(motorHandle_M1);

#if defined(BSXL8320RS_REVA) || defined(BSXL8323RS_REVA) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA)
        HAL_writeDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
        HAL_readDRVData(motorHandle_M1->halMtrHandle, &drvicVars_M1);
#endif  // BSXL8320RS_REVA || BSXL8323RS_REVA || BSXL8353RS_REVA  ||
        // BSXL8316RT_REVA

    } // end of while() loop


    // disable the PWM
    HAL_disablePWM(motorHandle_M1->halMtrHandle);

} // end of main() function


//
//-- end of this file ----------------------------------------------------------
//
