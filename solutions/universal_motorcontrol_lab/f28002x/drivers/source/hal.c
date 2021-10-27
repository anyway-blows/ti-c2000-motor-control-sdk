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
//! \file   solutions/fast_uni_lab/f28002x/drivers/hal_all.c
//! \brief  Contains the various functions related to the HAL object
//!
//

//
// the includes
//
#include "user.h"

//
// drivers
//

// modules

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalogIF.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

void HAL_disableGlobalInts(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    return;
} // end of HAL_disableGlobalInts() function

void HAL_disableWdog(HAL_Handle halHandle)
{
    // disable watchdog
    SysCtl_disableWatchdog();

    return;
} // end of HAL_disableWdog() function

void HAL_enableCtrlInts(HAL_Handle handle)
{
    // Acknowledge interrupt from PIE group
    Interrupt_clearACKGroup(MTR1_INT_ACK_GROUP);

    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(MTR1_PIE_INT_NUM);    // motor_1

    // enable the ADC interrupts for motor_1
    ADC_enableInterrupt(MTR1_ADC_INT_BASE, MTR1_ADC_INT_NUM);

    // enable the cpu interrupt for PWM and interrupts
    Interrupt_enableInCPU(MTR1_CPU_INT_NUM);        // motor_1

    return;
} // end of HAL_enableCtrlInts() function

// No HAL_enableADCIntsToTriggerCLA() in this device

void HAL_enableDebugInt(HAL_Handle handle)
{

    // enable debug events
    ERTM;

    return;
} // end of HAL_enableDebugInt() function

void HAL_enableGlobalInts(HAL_Handle handle)
{

    // enable global interrupts
    Interrupt_enableMaster();

    return;
} // end of HAL_enableGlobalInts() function


HAL_Handle HAL_init(void *pMemory,const size_t numBytes)
{
    HAL_Handle handle;
    HAL_Obj *obj;

    if(numBytes < sizeof(HAL_Obj))
    {
        return((HAL_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_Handle)pMemory;

    // assign the object
    obj = (HAL_Obj *)handle;

    // disable watchdog
    SysCtl_disableWatchdog();

    // Two ADC modules in this device
    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCCRESULT_BASE;

    // No DAC modules in this device
    // No CLA module in this device

    // initialize SCI handle
    obj->sciHandle = SCIA_BASE;             //!< the SCIA handle

    // initialize CAN handle
    obj->canHandle = CANA_BASE;             //!< the SCIA handle

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;              //!< the DMA handle

    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_CH3_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_CH4_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_CH5_BASE;     //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_CH6_BASE;     //!< the DMA Channel handle

    // initialize timer handles
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

#ifdef EPWMDAC_MODE
    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWMDAC1_BASE;
    obj->pwmDACHandle[1] = EPWMDAC2_BASE;
    obj->pwmDACHandle[2] = EPWMDAC3_BASE;
    obj->pwmDACHandle[3] = EPWMDAC3_BASE;
#endif  // EPWMDAC_MODE

    return(handle);
} // end of HAL_init() function

HAL_MTR_Handle HAL_MTR1_init(void *pMemory, const size_t numBytes)
{
    HAL_MTR_Handle handle;
    HAL_MTR_Obj *obj;

    if(numBytes < sizeof(HAL_MTR_Obj))
    {
        return((HAL_MTR_Handle)NULL);
    }

    // assign the handle
    handle = (HAL_MTR_Handle)pMemory;

    // assign the object
    obj = (HAL_MTR_Obj *)handle;

    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = MTR1_PWM_U_BASE;        //!< the PWM handle
    obj->pwmHandle[1] = MTR1_PWM_V_BASE;        //!< the PWM handle
    obj->pwmHandle[2] = MTR1_PWM_W_BASE;        //!< the PWM handle

    // initialize CMPSS handle
#if defined(MOTOR1_ISBLDC)
    obj->cmpssHandle[0] = MTR1_CMPSS_IDC_BASE;  //!< the CMPSS handle
#elif defined(MOTOR1_DCLINKSS)
    obj->cmpssHandle[0] = MTR1_CMPSS_IDC_BASE;  //!< the CMPSS handle
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    obj->cmpssHandle[0] = MTR1_CMPSS_U_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = MTR1_CMPSS_V_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = MTR1_CMPSS_W_BASE;    //!< the CMPSS handle
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
    // initialize CAP handles for Motor 1
    obj->capHandle[0] = MTR1_CAP_U_BASE;        //!< the CAP handle
    obj->capHandle[1] = MTR1_CAP_V_BASE;        //!< the CAP handle
    obj->capHandle[2] = MTR1_CAP_W_BASE;        //!< the CAP handle
#elif defined(CMD_CAP_EN)
    obj->capHandle = MTR1_CAP_FREQ_BASE;        //!< the CAP handle
#endif // HALL_ENABLE || CMD_CAP_EN

    // No PGA modules in this device

    // Assign gateEnableGPIO
#if defined(BSXL8353RS_REVA)
    // initialize drv8353 interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    // BSXL8353RS_REVA
#elif defined(BSXL8316RT_REVA)
    // initialize drv8316 interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->drvOffGPIO = MTR1_DRV_OFF_GPIO;
    // BSXL8316RT_REVA
#elif defined(BSXL8323RS_REVA)
    // initialize drv8323 interface
    obj->drvicHandle = DRVIC_init(&obj->drvic);

    // initialize SPI handle
    obj->spiHandle = MTR1_SPI_BASE;             //!< the SPI handle

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    obj->gateCalGPIO = MTR1_GATE_CAL_GPIO;
    // BSXL8323RS_REVA
#elif defined(BSXL8323RH_REVB)
    obj->gateModeGPIO = MTR1_GATE_MODE_GPIO;
    obj->gateGainGPIO = MTR1_GATE_GAIN_GPIO;
    obj->gateCalGPIO = MTR1_GATE_CAL_GPIO;
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
    // BSXL8323RH_REVB
#elif defined(BSXL3PHGAN_REVA)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
#elif defined(HVMTRPFC_REV1P1)
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;
#endif  // Assign gateEnableGPIO

    #ifdef QEP_ENABLE
    // initialize QEP driver
    obj->qepHandle = MTR1_QEP_BASE;             // the QEP handle
    #endif  // QEP_ENABLE

#ifdef SST_ENABLE
    obj->sstpwmHandle = EPWM4_BASE;             // the PWM handle for SST
#endif  //SST_ENABLE

    obj->motorNum = MTR_1;

    return(handle);
} // end of HAL_MTR1_init() function

void HAL_setParams(HAL_Handle handle)
{
    // disable global interrupts
    Interrupt_disableMaster();

    // Disable the watchdog
    SysCtl_disableWatchdog();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    memcpy(&ctrlfuncsRunStart, &ctrlfuncsLoadStart, (size_t)&ctrlfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    // Enable temperature sensor
    ASysCtl_enableTemperatureSensor();

    // initialize the interrupt controller
    Interrupt_initModule();

    // init vector table
    Interrupt_initVectorTable();

#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
    defined(BSXL8353RS_REVA) || defined(BSXL3PHGAN_REVA) || \
    defined(BSXL8316RT_REVA)
    // External 20M Crystal on LaunchXL
    // Set up PLL control and clock dividers
    // CPU Clock Frequency = 100MHz
    // PLLSYSCLK = 20MHz (XTAL_OSC) * 30 (IMULT) / (2 (REFDIV) * 3 (ODIV) * 1(SYSDIV))
    SysCtl_setClock(SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(30) |
                    SYSCTL_REFDIV(2) | SYSCTL_ODIV(3) |
                    SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE |
                    SYSCTL_DCC_BASE_0);
#elif defined(HVMTRPFC_REV1P1)
    // External 20M Crystal on ControlCard
    // Set up PLL control and clock dividers
    // CPU Clock Frequency = 100MHz
    // PLLSYSCLK = 20MHz (XTAL_OSC) * 30 (IMULT) / (2 (REFDIV) * 3 (ODIV) * 1(SYSDIV))
    SysCtl_setClock(SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(30) |
                    SYSCTL_REFDIV(2) | SYSCTL_ODIV(3) |
                    SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE |
                    SYSCTL_DCC_BASE_0);
#else
#error Select the right clock of PLL for the board
#endif  // PLL Selection

    // Make sure the LSPCLK divider is set to divide by 2
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_2); // 50MHz for SFRA, SPI&SCI

    // These asserts will check that the #defines for the clock rates in
    // device.h match the actual rates that have been configured. If they do
    // not match, check that the calculations of DEVICE_SYSCLK_FREQ and
    // DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
    // expected if these are not correct.
    //
    ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == DEVICE_LSPCLK_FREQ);

#ifndef _FLASH
    //
    // Call Device_cal function when run using debugger
    // This function is called as part of the Boot code. The function is called
    // in the Device_init function since during debug time resets, the boot code
    // will not be executed and the gel script will reinitialize all the
    // registers and the calibrated values will be lost.
    // Sysctl_deviceCal is a wrapper function for Device_Cal
    //
    SysCtl_deviceCal();
#endif

    // setup the peripheral clocks
    HAL_setupPeripheralClks(handle);

    //
    // Lock VREGCTL Register
    // The register VREGCTL is not supported in this device. It is locked to
    // prevent any writes to this register
    //
    ASysCtl_lockVREG();


    // setup the GPIOs
    HAL_setupGPIOs(handle);


#if defined(EPWMDAC_MODE)
    // setup the PWM DACs
    HAL_setupPWMDACs(handle, USER_SYSTEM_FREQ_MHz);
#endif  //EPWMDAC_MODE

    // setup the ADCs
    HAL_setupADCs(handle);

    // Sets up the CPU timer for time base
    HAL_setupTimeBaseTimer(handle, USER_TIME_BASE_FREQ_Hz);

    // Sets up the timers for CPU usage diagnostics
    HAL_setupCPUUsageTimer(handle);

#if !defined(SFRA_ENABLE)
    // setup the sci
    HAL_setupSCIA(handle);
#endif

    // setup the i2c
    HAL_setupI2CA(handle);

    // setup the DMA
    HAL_setupDMA();

    return;
} // end of HAL_setParams() function

void HAL_MTR_setParams(HAL_MTR_Handle handle, USER_Params *pUserParams)
{
    HAL_setNumCurrentSensors(handle, pUserParams->numCurrentSensors);
    HAL_setNumVoltageSensors(handle, pUserParams->numVoltageSensors);

    // setup the PWMs
    HAL_setupPWMs(handle);

    // setup the CMPSSs
    HAL_setupCMPSSs(handle);

#if defined(HALL_ENABLE) || defined(CMD_CAP_EN)
    // setup the CAPs
    HAL_setupCAPs(handle);
#endif  // HALL_ENABLE || CMD_CAP_EN

    // Setup Gate Enable
#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
    // setup the spi for drv8323/drv8353/drv8316
    HAL_setupSPI(handle);

    // setup the drv8323s/drv8353s/drv8316s interface
    HAL_setupGate(handle);
    // BSXL8323RS_REVA || BSXL8353RS_REVA || BSXL8316RT_REVA
#elif defined(BSXL8323RH_REVB) || defined(BSXL3PHGAN_REVA)
    // setup the drv8323h interface
    HAL_setupGate(handle);
#elif defined(HVMTRPFC_REV1P1)
    HAL_setupGate(handle);
#endif  // Setup Gate Enable

    #ifdef QEP_ENABLE
    // setup the EQEP
    HAL_setupQEP(handle);
    #endif  // QEP_ENABLE

    return;
} // end of HAL_MTR_setParams() function

void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    SysCtl_delay(100U);
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
    defined(BSXL8353RS_REVA) || defined(BSXL3PHGAN_REVA) || \
    defined(BSXL8316RT_REVA)
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
#elif defined(HVMTRPFC_REV1P1)
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
#else
#error Select the right clock of PLL for the board
#endif  // ADC Reference

    SysCtl_delay(100U);

    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);

    // set priority of SOCs
    ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);

//    ADC_setSOCPriority(PFC_IAC_ADC_BASE, ADC_PRI_SOC0_HIPRI);

    // enable the ADCs
    ADC_enableConverter(obj->adcHandle[0]);
    ADC_enableConverter(obj->adcHandle[1]);

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);

    //-------------------------------------------------------------------------
#if defined(MOTOR1_ISBLDC)
    // configure the interrupt sources
    // Interrupt for motor 1
    ADC_setInterruptSource(MTR1_ADC_INT_BASE,
                           MTR1_ADC_INT_NUM, MTR1_ADC_INT_SOC);

    // Idc 1st
    ADC_setupSOC(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_SOC_NUM, MTR1_IDC_TIGGER_SOC,
                 MTR1_IDC1_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, MTR1_IDC1_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);

    // Idc 2nd
    ADC_setupSOC(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_SOC_NUM, MTR1_IDC_TIGGER_SOC,
                 MTR1_IDC2_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, MTR1_IDC2_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);

    // VSEN_A_M1
    ADC_setupSOC(MTR1_VU_ADC_BASE, MTR1_VU_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VU_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // VSEN_B_M1
    ADC_setupSOC(MTR1_VV_ADC_BASE, MTR1_VV_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VV_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // VSEN_C_M1
    ADC_setupSOC(MTR1_VW_ADC_BASE, MTR1_VW_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VW_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // VSEN_DCBUS_M1-->Trig Interrupt
    ADC_setupSOC(MTR1_VDC_ADC_BASE, MTR1_VDC_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VDC_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

#if defined(CMD_POT_EN)
    // POT_M1
    ADC_setupSOC(MTR1_POT_ADC_BASE, MTR1_POT_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_POT_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);
#endif  // CMD_POT_EN

#else // !MOTOR1_ISBLDC
    // configure the SOCs for M1
#if defined(MOTOR1_DCLINKSS)
    // configure the interrupt sources
    // Interrupt for motor 1
    ADC_setInterruptSource(MTR1_ADC_INT_BASE,
                           MTR1_ADC_INT_NUM, MTR1_ADC_INT_SOC);

    // Idc 1st
    ADC_setupSOC(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_SOC_NUM, MTR1_IDC1_TIGGER_SOC,
                 MTR1_IDC1_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, MTR1_IDC1_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC1_ADC_BASE, MTR1_IDC1_ADC_PPB_NUM, 0);

    // Idc 2nd
    ADC_setupSOC(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_SOC_NUM, MTR1_IDC2_TIGGER_SOC,
                 MTR1_IDC2_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, MTR1_IDC2_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC2_ADC_BASE, MTR1_IDC2_ADC_PPB_NUM, 0);

    // Idc 3rd
    ADC_setupSOC(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_SOC_NUM, MTR1_IDC3_TIGGER_SOC,
                 MTR1_IDC3_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, MTR1_IDC3_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC3_ADC_BASE, MTR1_IDC3_ADC_PPB_NUM, 0);

    // Idc 4th
    ADC_setupSOC(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_SOC_NUM, MTR1_IDC4_TIGGER_SOC,
                 MTR1_IDC4_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, MTR1_IDC4_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IDC4_ADC_BASE, MTR1_IDC4_ADC_PPB_NUM, 0);

#else   // !(MOTOR1_DCLINKSS)
    // configure the interrupt sources
    // Interrupt for motor 1
    ADC_setInterruptSource(MTR1_ADC_INT_BASE,
                           MTR1_ADC_INT_NUM, MTR1_ADC_INT_SOC);
    // ISEN_A_M1
    ADC_setupSOC(MTR1_IU_ADC_BASE, MTR1_IU_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_IU_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, MTR1_IU_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IU_ADC_BASE, MTR1_IU_ADC_PPB_NUM, 0);

    // ISEN_B_M1
    ADC_setupSOC(MTR1_IV_ADC_BASE, MTR1_IV_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_IV_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCC_SOC0
    ADC_setupPPB(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, MTR1_IV_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IV_ADC_BASE, MTR1_IV_ADC_PPB_NUM, 0);

    // ISEN_C_M1
    ADC_setupSOC(MTR1_IW_ADC_BASE, MTR1_IW_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_IW_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // Configure PPB to eliminate subtraction related calculation
    // PPB is associated with ADCA_SOC0
    ADC_setupPPB(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, MTR1_IW_ADC_SOC_NUM);

    // Write zero to this for now till offset calibration complete
    ADC_setPPBCalibrationOffset(MTR1_IW_ADC_BASE, MTR1_IW_ADC_PPB_NUM, 0);
#endif   // !(MOTOR1_DCLINKSS)

#if defined(MOTOR1_FAST) || defined(MOTOR1_ISBLDC)
    // VSEN_A_M1
    ADC_setupSOC(MTR1_VU_ADC_BASE, MTR1_VU_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VU_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // VSEN_B_M1
    ADC_setupSOC(MTR1_VV_ADC_BASE, MTR1_VV_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VV_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);

    // VSEN_C_M1
    ADC_setupSOC(MTR1_VW_ADC_BASE, MTR1_VW_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VW_ADC_CH_NUM, MTR1_ADC_I_SAMPLEWINDOW);
#endif  // MOTOR1_FAST || MOTOR1_ISBLDC

    // VSEN_DCBUS-->Trig Interrupt
    ADC_setupSOC(MTR1_VDC_ADC_BASE, MTR1_VDC_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_VDC_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);

#if defined(CMD_POT_EN)
    // POT_M1
    ADC_setupSOC(MTR1_POT_ADC_BASE, MTR1_POT_ADC_SOC_NUM, MTR1_ADC_TIGGER_SOC,
                 MTR1_POT_ADC_CH_NUM, MTR1_ADC_V_SAMPLEWINDOW);
#endif  // CMD_POT_EN
#endif  // !MOTOR1_ISBLDC

    return;
} // end of HAL_setupADCs() function


#if defined(HALL_ENABLE) && defined(CMD_CAP_EN)
#error HALL and CMD_CAP can't be enabled at the same time
#elif defined(HALL_ENABLE)
void HAL_setupCAPs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Disable ,clear all capture flags and interrupts
        ECAP_disableInterrupt(obj->capHandle[cnt],
                              (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                               ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                               ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                               ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                               ECAP_ISR_SOURCE_COUNTER_COMPARE));

        ECAP_clearInterrupt(obj->capHandle[cnt],
                            (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                             ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                             ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                             ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                             ECAP_ISR_SOURCE_COUNTER_COMPARE));

        // Disable CAP1-CAP4 register loads
        ECAP_disableTimeStampCapture(obj->capHandle[cnt]);

        // Configure eCAP
        //    Enable capture mode.
        //    One shot mode, stop capture at event 3.
        //    Set polarity of the events to rising, falling, rising edge.
        //    Set capture in time difference mode.
        //    Select input from XBAR4/5/6.
        //    Enable eCAP module.
        //    Enable interrupt.
        ECAP_stopCounter(obj->capHandle[cnt]);
        ECAP_enableCaptureMode(obj->capHandle[cnt]);

        ECAP_setCaptureMode(obj->capHandle[cnt], ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_3);

        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
        ECAP_setEventPolarity(obj->capHandle[cnt], ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_1);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_2);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_3);
        ECAP_enableCounterResetOnEvent(obj->capHandle[cnt], ECAP_EVENT_4);

        ECAP_enableLoadCounter(obj->capHandle[cnt]);
        ECAP_setSyncOutMode(obj->capHandle[cnt], ECAP_SYNC_OUT_SYNCI);
        ECAP_startCounter(obj->capHandle[cnt]);
        ECAP_enableTimeStampCapture(obj->capHandle[cnt]);
        ECAP_reArm(obj->capHandle[cnt]);
    }

    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_CAP_U_XBAR, MTR1_HALL_U_GPIO);
    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_CAP_V_XBAR, MTR1_HALL_V_GPIO);
    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_CAP_W_XBAR, MTR1_HALL_W_GPIO);

    ECAP_selectECAPInput(obj->capHandle[0], MTR1_CAP_U_INSEL);
    ECAP_selectECAPInput(obj->capHandle[1], MTR1_CAP_V_INSEL);
    ECAP_selectECAPInput(obj->capHandle[2], MTR1_CAP_W_INSEL);

    return;
}

void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        ECAP_setAPWMPeriod(obj->capHandle[cnt], 0);
        ECAP_setAPWMCompare(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowPeriod(obj->capHandle[cnt], 0x01FFFFFF);
        ECAP_setAPWMShadowCompare(obj->capHandle[cnt], 0x01FFFFFF);
    }

    return;
}
#elif defined(CMD_CAP_EN)
void HAL_setupCAPs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    // Disable ,clear all capture flags and interrupts
    ECAP_disableInterrupt(obj->capHandle,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));

    ECAP_clearInterrupt(obj->capHandle,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    // Disable CAP1-CAP4 register loads
    ECAP_disableTimeStampCapture(obj->capHandle);

    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 3.
    //    Set polarity of the events to rising, falling, rising edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR4/5/6.
    //    Enable eCAP module.
    //    Enable interrupt.
    ECAP_stopCounter(obj->capHandle);
    ECAP_enableCaptureMode(obj->capHandle);

    ECAP_setCaptureMode(obj->capHandle, ECAP_CONTINUOUS_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_1, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_3, ECAP_EVNT_FALLING_EDGE);
    ECAP_setEventPolarity(obj->capHandle, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(obj->capHandle, ECAP_EVENT_4);

    ECAP_enableLoadCounter(obj->capHandle);
    ECAP_setSyncOutMode(obj->capHandle, ECAP_SYNC_OUT_SYNCI);
    ECAP_startCounter(obj->capHandle);
    ECAP_enableTimeStampCapture(obj->capHandle);
    ECAP_reArm(obj->capHandle);

    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_CAP_FREQ_XBAR, MTR1_CAP_FREQ_GPIO);

    ECAP_selectECAPInput(obj->capHandle, MTR1_CAP_FREQ_INSEL);

    return;
}

void HAL_resetCAPTimeStamp(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;

    ECAP_setAPWMPeriod(obj->capHandle, 0);
    ECAP_setAPWMCompare(obj->capHandle, 0x01FFFFFF);
    ECAP_setAPWMShadowPeriod(obj->capHandle, 0x01FFFFFF);
    ECAP_setAPWMShadowCompare(obj->capHandle, 0x01FFFFFF);

    return;
}
#endif  // HALL_ENABLE || CMD_CAP_EN

// HAL_setupCMPSSs
void HAL_setupCMPSSs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
    // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    uint16_t cmpsaDACL = MTR1_CMPSS_DACL_VALUE;

    ASysCtl_selectCMPLPMux(MTR1_IDC_CMPLP_SEL, MTR1_IDC_CMPLP_MUX);

    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[0]);

    // NEG signal from DAC for COMP-L
    CMPSS_configLowComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);

    // Configure the output signals. Both CTRIPH/L and CTRIPOUTH/L will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[0],
                           CMPSS_TRIP_FILTER |
                           CMPSS_TRIPOUT_FILTER |
                           CMPSS_INV_INVERTED);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    // Initialize the filter logic and start filtering
    CMPSS_configFilterLow(obj->cmpssHandle[0], 32, 32, 30);
    CMPSS_initFilterLow(obj->cmpssHandle[0]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[0],
               CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[0], cmpsaDACL);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
    uint16_t cmpsaDACH = MTR1_CMPSS_DACH_VALUE;
    uint16_t cmpsaDACL = MTR1_CMPSS_DACL_VALUE;

#if defined(HVMTRPFC_REV1P1)
    ASysCtl_selectCMPHPMux(MTR1_IU_CMPHP_SEL, MTR1_IU_CMPHP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IV_CMPHP_SEL, MTR1_IV_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IV_CMPLP_SEL, MTR1_IV_CMPLP_MUX);

    ASysCtl_selectCMPLPMux(MTR1_IW_CMPLP_SEL, MTR1_IW_CMPLP_MUX);

    //----- U Phase ------------------------------------------------------------
    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[0]);

    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[0], CMPSS_INSRC_DAC);


    // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[0],
                            CMPSS_TRIP_FILTER |
                            CMPSS_TRIPOUT_FILTER);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    CMPSS_configFilterHigh(obj->cmpssHandle[0], 32, 32, 30);
    CMPSS_initFilterHigh(obj->cmpssHandle[0]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[0], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[0],
               CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], cmpsaDACH);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    //---------------- V pHase -------------------------------------------------
    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[1]);

    // NEG signal from DAC for COMP-H
    CMPSS_configHighComparator(obj->cmpssHandle[1], CMPSS_INSRC_DAC);

    // NEG signal from DAC for COMP-L
    CMPSS_configLowComparator(obj->cmpssHandle[1], CMPSS_INSRC_DAC);

    // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
    CMPSS_configOutputsHigh(obj->cmpssHandle[1],
                            CMPSS_TRIP_FILTER |
                            CMPSS_TRIPOUT_FILTER);

    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[1],
                           CMPSS_TRIP_FILTER |
                           CMPSS_TRIPOUT_FILTER |
                           CMPSS_INV_INVERTED);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    CMPSS_configFilterHigh(obj->cmpssHandle[1], 32, 32, 30);
    CMPSS_initFilterHigh(obj->cmpssHandle[1]);

    // Initialize the filter logic and start filtering
    CMPSS_configFilterLow(obj->cmpssHandle[1], 32, 32, 30);
    CMPSS_initFilterLow(obj->cmpssHandle[1]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[1], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[1],
               CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-H to allowed MAX +ve current
    CMPSS_setDACValueHigh(obj->cmpssHandle[1], cmpsaDACH);

    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[1], cmpsaDACL);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    //---------------- W Phase -------------------------------------------------
    // Enable CMPSS and configure the negative input signal to come from the DAC
    CMPSS_enableModule(obj->cmpssHandle[2]);

    // NEG signal from DAC for COMP-L
    CMPSS_configLowComparator(obj->cmpssHandle[2], CMPSS_INSRC_DAC);

    // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
    // the asynchronous comparator output.
    // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
    CMPSS_configOutputsLow(obj->cmpssHandle[2],
                           CMPSS_TRIP_FILTER |
                           CMPSS_TRIPOUT_FILTER |
                           CMPSS_INV_INVERTED);

    // Configure digital filter. For this example, the maxiumum values will be
    // used for the clock prescale, sample window size, and threshold.
    // Initialize the filter logic and start filtering
    CMPSS_configFilterLow(obj->cmpssHandle[2], 32, 32, 30);
    CMPSS_initFilterLow(obj->cmpssHandle[2]);

    // Set up COMPHYSCTL register
    // COMP hysteresis set to 2x typical value
    CMPSS_setHysteresis(obj->cmpssHandle[2], 1);

    // Use VDDA as the reference for the DAC and set DAC value to midpoint for
    // arbitrary reference
    CMPSS_configDAC(obj->cmpssHandle[2],
               CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

    // Set DAC-L to allowed MAX -ve current
    CMPSS_setDACValueLow(obj->cmpssHandle[2], cmpsaDACL);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
#elif defined(BSXL8323RH_REVB) || defined(BSXL8323RS_REVA) || \
      defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA) || \
      defined(BSXL3PHGAN_REVA)

    uint16_t  cnt;

    ASysCtl_selectCMPHPMux(MTR1_IU_CMPHP_SEL, MTR1_IU_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IU_CMPLP_SEL, MTR1_IU_CMPLP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IV_CMPHP_SEL, MTR1_IV_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IV_CMPLP_SEL, MTR1_IV_CMPLP_MUX);

    ASysCtl_selectCMPHPMux(MTR1_IW_CMPHP_SEL, MTR1_IW_CMPHP_MUX);
    ASysCtl_selectCMPLPMux(MTR1_IW_CMPLP_SEL, MTR1_IW_CMPLP_MUX);

    for(cnt=0; cnt<3; cnt++)
    {
        // Enable CMPSS and configure the negative input signal to come from the DAC
        CMPSS_enableModule(obj->cmpssHandle[cnt]);

        // NEG signal from DAC for COMP-H
        CMPSS_configHighComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);

        // NEG signal from DAC for COMP-L
        CMPSS_configLowComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);

        // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed by
        // the asynchronous comparator output.
        // Dig filter output ==> CTRIPH, Dig filter output ==> CTRIPOUTH
        CMPSS_configOutputsHigh(obj->cmpssHandle[cnt],
                                CMPSS_TRIP_FILTER |
                                CMPSS_TRIPOUT_FILTER);

        // Dig filter output ==> CTRIPL, Dig filter output ==> CTRIPOUTL
        CMPSS_configOutputsLow(obj->cmpssHandle[cnt],
                               CMPSS_TRIP_FILTER |
                               CMPSS_TRIPOUT_FILTER |
                               CMPSS_INV_INVERTED);

        // Configure digital filter. For this example, the maxiumum values will be
        // used for the clock prescale, sample window size, and threshold.
        CMPSS_configFilterHigh(obj->cmpssHandle[cnt], 32, 32, 30);
        CMPSS_initFilterHigh(obj->cmpssHandle[cnt]);

        // Initialize the filter logic and start filtering
        CMPSS_configFilterLow(obj->cmpssHandle[cnt], 32, 32, 30);
        CMPSS_initFilterLow(obj->cmpssHandle[cnt]);

        // Set up COMPHYSCTL register
        // COMP hysteresis set to 2x typical value
        CMPSS_setHysteresis(obj->cmpssHandle[cnt], 1);

        // Use VDDA as the reference for the DAC and set DAC value to midpoint for
        // arbitrary reference
        CMPSS_configDAC(obj->cmpssHandle[cnt],
                   CMPSS_DACREF_VDDA | CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

        // Set DAC-H to allowed MAX +ve current
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], cmpsaDACH);

        // Set DAC-L to allowed MAX -ve current
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], cmpsaDACL);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);
    }
#endif  // !HVMTRPFC_REV1P1
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    return;
} // end of HAL_setupCMPSSs() function

// HAL_setupGate & HAL_enableDRV
#if defined(BSXL8323RS_REVA) || defined(BSXL8353RS_REVA) || \
    defined(BSXL8316RT_REVA)
void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    DRVIC_enable(obj->drvicHandle);

    return;
}  // end of HAL_enableDRV() function

void HAL_writeDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_writeData(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_writeDRVData() function


void HAL_readDRVData(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_readData(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_readDRVData() function

void HAL_setupDRVSPI(HAL_MTR_Handle handle, DRVIC_VARS_t *drvicVars)
{
    HAL_MTR_Obj  *obj = (HAL_MTR_Obj *)handle;

    DRVIC_setupSPI(obj->drvicHandle, drvicVars);

    return;
}  // end of HAL_setupDRVSPI() function

void HAL_setupGate(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    DRVIC_setSPIHandle(obj->drvicHandle, obj->spiHandle);

    DRVIC_setGPIOCSNumber(obj->drvicHandle, MTR1_DRV_SPI_CS_GPIO);
    DRVIC_setGPIOENNumber(obj->drvicHandle, MTR1_GATE_EN_GPIO);

    return;
} // HAL_setupGate() function

void HAL_setupSPI(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj   *obj = (HAL_MTR_Obj *)handle;

    // Must put SPI into reset before configuring it
    SPI_disableModule(obj->spiHandle);

    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size, 25MHz LSPCLK
    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 400000, 16);

    SPI_disableLoopback(obj->spiHandle);

    SPI_setEmulationMode(obj->spiHandle, SPI_EMULATION_FREE_RUN);

    SPI_enableFIFO(obj->spiHandle);
    SPI_setTxFifoTransmitDelay(obj->spiHandle, 0x10);

    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // Configuration complete. Enable the module.
    SPI_enableModule(obj->spiHandle);

    return;
}  // end of HAL_setupSPI() function
// BSXL8323RS_REVA || BSXL8353RS_REVA || BSXL8316RT_REVA
#elif defined(BSXL8323RH_REVB)
void HAL_setupGate(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    obj->gateModeGPIO = MTR1_GATE_MODE_GPIO;
    obj->gateGainGPIO = MTR1_GATE_GAIN_GPIO;
    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;

    return;
} // HAL_setupGate() function

void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_ISBLDC)
    // Set EN_GATE to high for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 1);

    // Set MODE to low for setting 3-PWM mode (has a 47k pull down)
    GPIO_setDirectionMode(obj->gateModeGPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(obj->gateModeGPIO, GPIO_PIN_TYPE_STD);

    // disable calibrate mode
    GPIO_writePin(obj->gateCalGPIO, 0);
#else  // !MOTOR1_ISBLDC
    // Set EN_GATE to high for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 1);

    // Set MODE to low for setting 6-PWM mode
    GPIO_writePin(obj->gateModeGPIO, 0);

    // disable calibrate mode
    GPIO_writePin(obj->gateCalGPIO, 0);
#endif  // !MOTOR1_ISBLDC

    return;
} // HAL_setupGate() function
// BSXL8323RH_REVB
#elif defined(BSXL3PHGAN_REVA)
void HAL_setupGate(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;

    return;
} // HAL_setupGate() function

void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    return;
} // HAL_setupGate() function
// HVMTRPFC_REV1P1
#elif defined(HVMTRPFC_REV1P1)
void HAL_setupGate(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    obj->gateEnableGPIO = MTR1_GATE_EN_GPIO;

    return;
} // HAL_setupGate() function

void HAL_enableDRV(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

    // Set EN_GATE to low for enabling the DRV
    GPIO_writePin(obj->gateEnableGPIO, 0);

    return;
} // HAL_setupGate() function
// HVMTRPFC_REV1P1
#else
#error No HAL_setupGate or HAL_enableDRV
#endif  // HAL_setupGate & HAL_enableDRV

void HAL_setupMtrFaults(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;
    uint16_t cnt;

    // Configure TRIP 7 to OR the High and Low trips from both
    // comparator 5, 3 & 1, clear everything first
    EALLOW;
    HWREG(XBAR_EPWM_CFG_REG_BASE + MTR1_XBAR_TRIP_ADDRL) = 0;
    HWREG(XBAR_EPWM_CFG_REG_BASE + MTR1_XBAR_TRIP_ADDRH) = 0;
    EDIS;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
    // Configure TRIP7 to be CTRIP5H and CTRIP5L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IDC_XBAR_EPWM_MUX);

    // Disable all the mux first
    XBAR_disableEPWMMux(MTR1_XBAR_TRIP, 0xFFFF);

    // Enable Mux 0  OR Mux 4 to generate TRIP
    XBAR_enableEPWMMux(MTR1_XBAR_TRIP, MTR1_IDC_XBAR_MUX);
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    // Configure TRIP7 to be CTRIP5H and CTRIP5L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IU_XBAR_EPWM_MUX);

    // Configure TRIP7 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IV_XBAR_EPWM_MUX);

    // Configure TRIP7 to be CTRIP3H and CTRIP3L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(MTR1_XBAR_TRIP, MTR1_IW_XBAR_EPWM_MUX);

    // Disable all the mux first
    XBAR_disableEPWMMux(MTR1_XBAR_TRIP, 0xFFFF);

    // Enable Mux 0  OR Mux 4 to generate TRIP
    XBAR_enableEPWMMux(MTR1_XBAR_TRIP, MTR1_IU_XBAR_MUX |
                                       MTR1_IV_XBAR_MUX | MTR1_IW_XBAR_MUX);
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    // configure the input x bar for TZ2 to GPIO, where Over Current is connected
    XBAR_setInputPin(INPUTXBAR_BASE, MTR1_XBAR_INPUT1, MTR1_PM_nFAULT_GPIO);
    XBAR_lockInput(INPUTXBAR_BASE, MTR1_XBAR_INPUT1);

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot fault trip zone
    // These trips need to be repeated for EPWM1 ,2 & 3

    for(cnt=0; cnt<3; cnt++)
    {
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], MTR1_TZ_OSHT1);


        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                              MTR1_DCTRIPIN, EPWM_DC_TYPE_DCAH);

        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                              MTR1_DCTRIPIN, EPWM_DC_TYPE_DCBH);

        // Trigger event when DCAH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        // Trigger event when DCBH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_B1,
                                                     EPWM_TZ_EVENT_DCXL_HIGH);

        // Configure the DCA path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_A,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        // Configure the DCB path to be un-filtered and asynchronous
        EPWM_setDigitalCompareEventSource(obj->pwmHandle[cnt],
                                          EPWM_DC_MODULE_B,
                                          EPWM_DC_EVENT_1,
                                          EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_A,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        EPWM_setDigitalCompareEventSyncMode(obj->pwmHandle[cnt],
                                            EPWM_DC_MODULE_B,
                                            EPWM_DC_EVENT_1,
                                            EPWM_DC_EVENT_INPUT_NOT_SYNCED);

        // Enable DCA as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Enable DCB as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCBEVT1);

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZA,
                               EPWM_TZ_ACTION_LOW);

        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZB,
                               EPWM_TZ_ACTION_LOW);
    }

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
#if defined(HVMTRPFC_REV1P1)
    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);

    // Clear any high comparator digital filter output latch
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);

    // Clear any low comparator digital filter output latch
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
#else // !HVMTRPFC_REV1P1
    for(cnt=0; cnt<3; cnt++)
    {
        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZFLAG_INTERRUPT_ALL);
    }
#endif  // !HVMTRPFC_REV1P1
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    // Clear any spurious fault
    EPWM_clearTripZoneFlag(obj->pwmHandle[0], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[1], HAL_TZFLAG_INTERRUPT_ALL);
    EPWM_clearTripZoneFlag(obj->pwmHandle[2], HAL_TZFLAG_INTERRUPT_ALL);

    return;
} // end of HAL_setupMtrFaults() function

void HAL_setupGPIOs(HAL_Handle handle)
{
//------------------------------------------------------------------------------
#if defined(BSXL8323RS_REVA)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

#if defined(MOTOR1_ISBLDC)
    // GPIO1->M1_UL*
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_writePin(1, 0);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#else
    // GPIO1->EPWM1B->M1_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

#if defined(MOTOR1_ISBLDC)
    // GPIO3->M1_VL*
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_writePin(3, 0);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#else
    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->Connect to GPIO5 using a jumper wire->M1_DRV_SCS
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->Connect to GPIO5 using a jumper wire->M1_DRV_nSCS
#ifdef DRV_CS_GPIO
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_writePin(8, 1);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
#endif

    // GPIO09->M1_DRV_SCLK*
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // GPIO10->SPIA_SOMI->M1_DRV_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);

    // GPIO11->SPIA_SIMO->M1_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#else
    // GPIO14->QEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#endif  // ENC_QEP2

#if defined(MOTOR1_ISBLDC)
    // GPIO15->M1_WL*
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_writePin(15, 0);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#else
    // GPIO15->EPWM3B->M1_WL*
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

#if !defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
    // GPIO16->SCIA_TX for SFRA
    GPIO_setPinConfig(GPIO_16_SCIA_TX);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SCIA_RX for SFRA
    GPIO_setPinConfig(GPIO_17_SCIA_RX);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#elif defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
#error SFRA can't be supported in this case
#else  // !SFRA_ENABLE
    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // !SFRA_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->M1_DRV_LED
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_writePin(19, 1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22->SPIB_CLK->M2_DRV_SCLK/DAC128S_SDI
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);

#if defined(CMD_CAN_EN) && defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO23->M2_DRV_VDS
    GPIO_setPinConfig(GPIO_23_SPIB_STE);
    GPIO_writePin(23, 1);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
#else   // Command Switch
    // GPIO23->Command Switch Button
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 4);
#endif  // CMD_CAN_EN & DAC128S_ENABLE & DAC128S_SPIB

    // GPIO24->M2_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_writePin(24, 1);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#else
    // GPIO25->QEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#else
    // GPIO26->QEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#endif  // ENC_QEP2

    // GPIO27->M1_DRV_CAL
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_writePin(27, 0);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->M1_DRV_ENABLE
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_writePin(28, 1);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->M1_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_writePin(29, 1);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->SPIB_SIMO->DAC128S_SDI*
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);

    // GPIO31->SPIB_SOMI->DAC128S_SDO*
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);

#if defined(CMD_CAN_EN)
    // GPIO33->CAN_TX
    GPIO_setPinConfig(GPIO_32_CANA_TX);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(32, GPIO_QUAL_ASYNC);
#else   // !(CMD_CAN_EN)
    // GPIO32->M2_DRV_nSCS/GAIN
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
#endif  // !(CMD_CAN_EN)

#if defined(CMD_CAN_EN)
    // GPIO33->CAN_RX
    GPIO_setPinConfig(GPIO_33_CANA_RX);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(33, GPIO_QUAL_ASYNC);
#elif defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_writePin(33, 1);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO34->M1_DRV_nFAULT*
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_writePin(35, 1);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#else
    // GPIO37->HALL_V
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#endif  // ENC_QEP2

    // GPIO39->Run State
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 1);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->Freq Capture
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->M1_DRV_MODE#
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#else
    // GPIO43->HALL_W
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#else
    // GPIO44->HALL_U
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#endif  // ENC_QEP2

    // GPIO45->M2_DRV_MODE#, N/A
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL8323RS_REVA

//------------------------------------------------------------------------------
#elif defined(BSXL8323RH_REVB)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
#if defined(MOTOR1_ISBLDC)
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_writePin(1, 0);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL*
#if defined(MOTOR1_ISBLDC)
    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_writePin(3, 0);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#else
    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO5->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
#else
    // GPIO5->Reserve GPIO
    GPIO_setPinConfig(GPIO_5_GPIO5);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->M1_DRV_nSCS/GAIN (has a 47k pull down)
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO09->SPIA_CLK->DAC12S_SCLK
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO9->Reserve GPIO
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO10->SPIA_SOMI->DAC128S_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO10->Reserve GPIO
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO11->SPIA_SIMO->DAC128S_SDI
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO11->Reserve GPIO
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#else
    // GPIO14->QEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#endif  // ENC_QEP2

    // GPIO15->EPWM3B->M1_WL*
#if defined(MOTOR1_ISBLDC)
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_writePin(15, 0);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

#if !defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
    // GPIO16->SCIA_TX for SFRA
    GPIO_setPinConfig(GPIO_16_SCIA_TX);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SCIA_RX for SFRA
    GPIO_setPinConfig(GPIO_17_SCIA_RX);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#elif defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
#error SFRA can't be supported in this case
#else  // !SFRA_ENABLE
    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // !SFRA_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->M1_DRV_LED
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_writePin(19, 1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO22->SPIB_CLK->DAC128S_SCLK
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO22->Reserve GPIO
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(CMD_CAN_EN) && defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO23->M2_DRV_VDS
    GPIO_setPinConfig(GPIO_23_SPIB_STE);
    GPIO_writePin(23, 1);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
#else   // Command Switch
    // GPIO23->M2_DRV_VDS / Command Switch
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 4);
#endif  // CMD_CAN_EN & DAC128S_ENABLE & DAC128S_SPIB

    // GPIO24->M2_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_writePin(24, 1);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_PULLUP);

#if !defined(ENC_QEP2)
    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#else
    // GPIO25->QEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#else
    // GPIO26->QEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#endif  // ENC_QEP2

    // GPIO27->M1_DRV_CAL
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_writePin(27, 0);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->HAL_GPIO_ISR_M1
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_writePin(28, 1);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->M1_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_writePin(29, 1);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO30->SPIB_SIMO->DAC128S_SDI*
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO30->
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO31->SPIB_SOMI->DAC128S_SDO*
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(CMD_CAN_EN)
    // GPIO33->CAN_TX
    GPIO_setPinConfig(GPIO_32_CANA_TX);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
#else   // !(CMD_CAN_EN)
    // GPIO32->M2_DRV_nSCS/GAIN
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
#endif  // !(CMD_CAN_EN)

#if defined(CMD_CAN_EN)
    // GPIO33->CAN_RX
    GPIO_setPinConfig(GPIO_33_CANA_RX);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#elif defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_writePin(33, 1);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO34->M1_DRV_nFAULT*
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#else
    // GPIO37->HALL_V
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#endif  // ENC_QEP2

    // GPIO39->HAL_GPIO_ISR_M1
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->CMD_FREQ_CAP
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->M1_DRV_MODE->6-PWM-Mode (GND)
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_writePin(42, 0);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#else
    // GPIO43->HALL_W
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#else
    // GPIO44->HALL_U
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#endif  // ENC_QEP2

    // GPIO45->M2_DRV_MODE->6-PWM-Mode (GND)
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_writePin(45, 0);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->M2_DRV_CAL->Low
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_writePin(46, 0);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end ofBSXL8323RH_REVB

//------------------------------------------------------------------------------
#elif defined(BSXL8353RS_REVA)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
#if defined(MOTOR1_ISBLDC)
    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_writePin(1, 0);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

#if defined(MOTOR1_ISBLDC)
    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_GPIO3);
    GPIO_writePin(3, 0);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#else
    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->M1_DRV_SCS (Use a wire->J2-18)
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->M1_DRV_nSCS
#ifdef DRV_CS_GPIO
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_writePin(8, 1);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_PULLUP);
#else   // M1_DRV_SCS (Use a wire->J2-18)
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
#endif

    // GPIO09->M1_DRV_SCLK*
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // GPIO10->SPIA_SOMI->M1_DRV_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);

    // GPIO11->SPIA_SIMO->M1_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#else
    // GPIO14->QEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#endif  // ENC_QEP2

    // GPIO15->EPWM3B->M1_WL*
#if defined(MOTOR1_ISBLDC)
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_writePin(15, 0);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);
#endif // !MOTOR1_ISBLDC

#if !defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
    // GPIO16->SCIA_TX for SFRA
    GPIO_setPinConfig(GPIO_16_SCIA_TX);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SCIA_RX for SFRA
    GPIO_setPinConfig(GPIO_17_SCIA_RX);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#elif defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
#error SFRA can't be supported in this case
#else  // !SFRA_ENABLE
    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // !SFRA_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->M1_DRV_LED
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_writePin(19, 1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22->SPIB_CLK->M2_DRV_SCLK
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);

#if defined(CMD_CAN_EN) && defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO23->M2_DRV_VDS
    GPIO_setPinConfig(GPIO_23_SPIB_STE);
    GPIO_writePin(23, 1);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
#else   // Command Switch
    // GPIO23->M2_DRV_VDS / Command Switch
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 4);
#endif  // CMD_CAN_EN & DAC128S_ENABLE & DAC128S_SPIB

    // GPIO24->M2_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_writePin(24, 1);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#else
    // GPIO25->QEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#else
    // GPIO26->QEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#endif  // ENC_QEP2

    // GPIO27->M1_DRV_CAL
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_writePin(27, 0);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->M1_DRV_ENABLE
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_writePin(28, 1);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->M1_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_writePin(29, 1);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->SPIB_SIMO->M2_DRV_SDI*
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);

    // GPIO31->SPIB_SOMI->M2_DRV_SDO*
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);

    // GPIO32->M2_DRV_nSCS*
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 1);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

#if defined(DAC128S_ENABLE)
    // GPIO33->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_writePin(33, 1);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif

    // GPIO34->M1_DRV_nFAULT*
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#else
    // GPIO37->HALL_V
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#endif  // ENC_QEP2

    // GPIO39->nFault
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->CMD_FREQ_CAP
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->M1_DRV_MODE#
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#else
    // GPIO43->HALL_W
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#else
    // GPIO44->HALL_U
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#endif  // ENC_QEP2

    // GPIO45->M2_DRV_MODE#, N/A
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL8353RS_REVA
//-----------------------------------------------------------------------------
#elif defined(BSXL8316RT_REVA)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->M1_DRV_SCS (Use a wire->J2-18)
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->M1_DRV_nSCS
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO09->M1_DRV_SCLK*
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // GPIO10->SPIA_SOMI->M1_DRV_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);

    // GPIO11->SPIA_SIMO->M1_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#else
    // GPIO14->QEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(14, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(14, 2);
#endif  // ENC_QEP2

#if !defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
    // GPIO16->SCIA_TX for SFRA
    GPIO_setPinConfig(GPIO_16_SCIA_TX);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SCIA_RX for SFRA
    GPIO_setPinConfig(GPIO_17_SCIA_RX);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#elif defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
#error SFRA can't be supported in this case
#else  // !SFRA_ENABLE
    // GPIO15->EPWM3B->M1_WL*
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // !SFRA_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22->SPIB_CLK->M2_DRV_SCLK/DAC128S_SDI
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);

    // GPIO23->M1_DRV_OFF, 1- Disable, 0-Enable
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // GPIO24->M2_DRV_ENABLE*
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_writePin(24, 1);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#else
    // GPIO25->QEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#else
    // GPIO26->QEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 4);
#endif  // ENC_QEP2

    // GPIO27->M1_LED
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_writePin(27, 0);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->M1_DRV_ENABLE
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_writePin(28, 1);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->M1_nSLEEP, 1-Active, 0-Low Power Sleep Mode
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_writePin(29, 1);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->SPIB_SIMO->M2_DRV_SDI*/DAC128S_SDI
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);

    // GPIO31->SPIB_SOMI->M2_DRV_SDO*/DAC128S_SDI
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);

    // GPIO32->M2_DRV_nSCS*
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 1);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

#if defined(DAC128S_ENABLE)
    // GPIO33->DAC128S_SYNC
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_writePin(33, 1);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif

    // GPIO34->M1_DRV_nFAULT*
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#else
    // GPIO37->HALL_V
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(37, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(37, 2);
#endif  // ENC_QEP2

    // GPIO39->M2_DRV_nFault
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // GPIO40->Reserve
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->M1_DRV_nFAULT
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->M1_DRV_MODE#
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#else
    // GPIO43->HALL_W
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(43, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(43, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#else
    // GPIO44->HALL_U
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(44, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(44, 2);
#endif  // ENC_QEP2

    // GPIO45->M2_DRV_MODE#, N/A
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL8316RT_REVA
//------------------------------------------------------------------------------
#elif defined(BSXL3PHGAN_REVA)
    // GPIO0->EPWM1A->M1_UH*
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL*
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH*
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL*
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_WH*
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO5->SPIA_STE
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO5->
    GPIO_setPinConfig(GPIO_5_GPIO5);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO6->EPWM4A->M2_VH*
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->M2_VL*
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->Reserve
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO09->SPI_SCLK*
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO9->Reserve
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO10->SPIA_SOMI->M1_DRV_SDO*
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO10->Reserve
    GPIO_setPinConfig(GPIO_10_GPIO10);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIA)
    // GPIO11->SPIA_SIMO->M1_DRV_SDI*
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
#else
    // GPIO11->Reserve
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIA

    // GPIO12->EPWM7A->M2_UH*
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->EPWM7B->M2_UL*
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

#if defined(LPD_SITE_J5_J8)
    // GPIO14->EQEP2_A
    GPIO_setPinConfig(GPIO_14_EQEP2_A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);
#else
    //GPIO14->HALL_U
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_PULLUP);
#endif  // LPD_SITE_J5_J8

    // GPIO15->EPWM3B->M1_WL*
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

#if !defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
    // GPIO16->SCIA_TX for SFRA
    GPIO_setPinConfig(GPIO_16_SCIA_TX);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SCIA_RX for SFRA
    GPIO_setPinConfig(GPIO_17_SCIA_RX);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#elif defined(LPD_SITE_J5_J8) && defined(SFRA_ENABLE)
#error SFRA can't be supported in this case
#else  // !SFRA_ENABLE
    // GPIO16->EPWM5A->M2_WH*
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B->M2_WL*
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);
#endif  // !SFRA_ENABLE

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO22->SPIB_CLK->M2_DRV_SCLK
    GPIO_setPinConfig(GPIO_22_SPIB_CLK);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(20, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO23-> (site 1)
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 1);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);

    // GPIO24->Reserve
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

#if defined(LPD_SITE_J5_J8)
    // GPIO25->EQEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
#else
    // GPIO25->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
#endif  // LPD_SITE_J5_J8

#if defined(LPD_SITE_J5_J8)
    // GPIO26->EQEP2_INDEX
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
#else
    // GPIO26->HALL_W
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_PULLUP);
#endif  // LPD_SITE_J5_J8

    // GPIO27->Reserve
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIA_RX->XDS110
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_PULLUP);

    // GPIO29->SCIA_TX->XDS110
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO30->SPIB_SIMO->M2_DRV_SDI*
    GPIO_setPinConfig(GPIO_30_SPIB_SIMO);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO31->SPIB_SOMI->M2_DRV_SDO*
    GPIO_setPinConfig(GPIO_31_SPIB_SOMI);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);
#else
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO32->Reserve*
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->SPIB_STE
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

    // GPIO34->LPD_LED5
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 1);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // GPIO35->Reserve
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

#if defined(DAC128S_ENABLE) && defined(DAC128S_SPIB)
    // GPIO33->SPIB_STE
    GPIO_setPinConfig(GPIO_33_SPIB_STE);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
#endif  // DAC128S_ENABLE && DAC128S_SPIB

#if defined(LPD_SITE_J1_J4)
    // GPIO37->EQEP1_B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
#endif  // LPD_SITE_J1_J4

    // GPIO39->EN_GATE (site 2)
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_writePin(39, 1);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);

    // GPIO40->Reserve
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // GPIO41->reserve
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->OT (Site 1)
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_PULLUP);

#if defined(LPD_SITE_J1_J4)
    // GPIO43->EQEP1_INDEX!
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
#endif  // LPD_SITE_J1_J4

#if defined(LPD_SITE_J1_J4)
    // GPIO44->EQEP1_A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
#else
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);
#endif  // LPD_SITE_J1_J4

    // GPIO45->OT (site 2)
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_PULLUP);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of BSXL3PHGAN_REVA
//------------------------------------------------------------------------------
#elif defined(HVMTRPFC_REV1P1)
    // GPIO0->EPWM1A->M1_UH
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // GPIO1->EPWM1B->M1_UL
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // GPIO2->EPWM2A->M1_VH
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // GPIO3->EPWM2B->M1_VL
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // GPIO4->EPWM3A->M1_WH
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // GPIO5->EPWM3B->M1_WL
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // GPIO6->EPWM4A->PFC_1H
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // GPIO7->EPWM4B->PFC_1L
    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // GPIO8->Reserve
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // GPIO09->CLR-Fault
    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_writePin(9, 1);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_PULLUP);

    // PWM6A->DAC1
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // PWM6B->DAC2
    GPIO_setPinConfig(GPIO_11_EPWM6_B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // PWM7A->DAC3
    GPIO_setPinConfig(GPIO_12_EPWM7_A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // PWM7B->DAC4
    GPIO_setPinConfig(GPIO_13_EPWM7_B);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->LED2 on HVKIT
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_writePin(14, 1);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // GPIO15->Reserve
    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->EPWM5A
    GPIO_setPinConfig(GPIO_16_EPWM5_A);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->EPWM5B
    GPIO_setPinConfig(GPIO_17_EPWM5_B);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // GPIO18->Reserve
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO19->Reserve
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // GPIO22->Reserve
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

#if !defined(ENC_QEP2)
    // GPIO23->EQEP1_I
    GPIO_setPinConfig(GPIO_23_EQEP1_INDEX);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 2);
#else   // ENC_QEP2
    // GPIO23->CAP1->HALL_W
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(23, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(23, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO24->CAP1->HALL_U
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(24, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(24, 2);
#else   // ENC_QEP2
    // GPIO24->EQEP2_A
    GPIO_setPinConfig(GPIO_24_EQEP2_A);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(24, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(24, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO25->CAP2->HALL_V
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 1);
#else   // ENC_QEP2
    // GPIO25->EQEP2_B
    GPIO_setPinConfig(GPIO_25_EQEP2_B);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(25, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(25, 1);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO26->CAP3->HALL_U
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 1);
#else   // ENC_QEP2
    // GPIO26->EQEP2_I
    GPIO_setPinConfig(GPIO_26_EQEP2_INDEX);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(26, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(26, 1);
#endif  // ENC_QEP2

    // GPIO27->PFC ISR Executing Time
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIA_RX
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO28->SCIA_TX
    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);

    // GPIO30->CAN-RX
    GPIO_setPinConfig(GPIO_30_CANA_RX);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // GPIO31->CAN-TX
    GPIO_setPinConfig(GPIO_31_CANA_TX);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

    // GPIO32->Reserve
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

    // GPIO33->Reserve
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->LED-D2 on Control Card
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 1);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // TDI
    GPIO_setPinConfig(GPIO_35_TDI);

    // TDO
    GPIO_setPinConfig(GPIO_37_TDO);

    // GPIO39->nFault
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(39, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(39, 1);

#if !defined(ENC_QEP2)
    // GPIO40->EQEP1_A
    GPIO_setPinConfig(GPIO_40_EQEP1_A);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(40, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(40, 2);
#else   // ENC_QEP2
    // GPIO40->CAP1->HALL_U
    GPIO_setPinConfig(GPIO_40_EQEP1_A);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(40, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(40, 2);
#endif  // ENC_QEP2

#if !defined(ENC_QEP2)
    // GPIO41->EQEP1_B
    GPIO_setPinConfig(GPIO_41_EQEP1_B);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(41, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(41, 2);
#else   // ENC_QEP2
    // GPIO41->CAP2->HALL_V
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(41, GPIO_QUAL_3SAMPLE);
    GPIO_setQualificationPeriod(41, 2);
#endif  // ENC_QEP2

    // GPIO42->LED1 on HVKIT
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_writePin(42, 1);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO43->Reserve
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->Reserve
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->Reserve
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->Reserve
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    // end of HVMTRPFC_REV1P1
//------------------------------------------------------------------------------
#else
#error The GPIOs is not configured for motor control board
#endif

    return;
}  // end of HAL_setupGPIOs() function

void HAL_setupPeripheralClks(HAL_Handle handle)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_HRPWM);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINA);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_LINB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);

    return;
} // end of HAL_setupPeripheralClks() function

void HAL_setupPWMs(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj    *obj = (HAL_MTR_Obj *)handle;
    uint16_t  cnt;

    uint16_t       pwmPeriodCycles = (uint16_t)(USER_M1_PWM_TBPRD_NUM);
    uint16_t       numPWMTicksPerISRTick = USER_M1_NUM_PWM_TICKS_PER_ISR_TICK;

    // disable the ePWM module time base clock sync signal
    // to synchronize all of the PWMs
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB) || \
    defined(BSXL8353RS_REVA) || defined(BSXL8316RT_REVA) || \
    defined(BSXL3PHGAN_REVA) || defined(HVMTRPFC_REV1P1)

    for(cnt=0; cnt<3; cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);

        EPWM_disablePhaseShiftLoad(obj->pwmHandle[cnt]);

        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);

        EPWM_enableSyncOutPulseSource(obj->pwmHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

        EPWM_setClockPrescaler(obj->pwmHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                                 EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setCountModeAfterSync(obj->pwmHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);

        EPWM_setEmulationMode(obj->pwmHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

#if defined(MOTOR1_ISBLDC)
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        // setup the Action-qualifier Continuous Software Force Register (AQCSFRC)
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                 EPWM_AQ_OUTPUT_B,
                                                 EPWM_AQ_SW_OUTPUT_HIGH);

        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, false);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, false);

#else   //!MOTOR1_ISBLDC


#if defined(MOTOR1_DCLINKSS)
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
#else  // !(MOTOR1_DCLINKSS)
        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

#endif  // !(MOTOR1_DCLINKSS)

        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_RED, true);
        EPWM_setDeadBandDelayMode(obj->pwmHandle[cnt], EPWM_DB_FED, true);

        // select EPWMA as the input to the dead band generator
        EPWM_setRisingEdgeDeadBandDelayInput(obj->pwmHandle[cnt],
                                             EPWM_DB_INPUT_EPWMA);

        // configure the right polarity for active high complementary config.
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_RED,
                                      EPWM_DB_POLARITY_ACTIVE_HIGH);
        EPWM_setDeadBandDelayPolarity(obj->pwmHandle[cnt],
                                      EPWM_DB_FED,
                                      EPWM_DB_POLARITY_ACTIVE_LOW);

        // setup the Dead-Band Rising Edge Delay Register (DBRED)
        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt], MTR1_PWM_DBRED_CNT);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt], MTR1_PWM_DBFED_CNT);

#endif  //!MOTOR1_ISBLDC

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmHandle[cnt], HAL_TZSEL_SIGNALS_ALL);
    }

    // BSXL8323RS_REVA || BSXL8323RH_REVB || \
    // BSXL8353RS_REVA || BSXL8316RT_REVA || \
    // BSXL3PHGAN_REVA || HVMTRPFC_REV1P1
#else
#error The PWM is not configured for motor_1 control
#endif  // boards

#if defined(MOTOR1_ISBLDC)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);

    EPWM_disableInterrupt(obj->pwmHandle[0]);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                              EPWM_SOC_B, EPWM_SOC_TBCTR_U_CMPB);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_B);
#elif defined(MOTOR1_DCLINKSS)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);

    EPWM_enableInterrupt(obj->pwmHandle[0]);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    // ADC SOC trigger for the 1st dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_U_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_A);

    // ADC SOC trigger for the 2nd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[1],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_U_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[1], EPWM_SOC_B);

    // ADC SOC trigger for the 3rd dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                                 EPWM_SOC_A,
                                 EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_A);

    // ADC SOC trigger for the 4th dc-link current sampling
    EPWM_setADCTriggerSource(obj->pwmHandle[2],
                             EPWM_SOC_B,
                             EPWM_SOC_TBCTR_D_CMPD);

    EPWM_enableADCTrigger(obj->pwmHandle[2], EPWM_SOC_B);
#else   //!(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_ZERO);

    EPWM_enableInterrupt(obj->pwmHandle[0]);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 15);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B, 15);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B, 15);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A, 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B, 15);
#endif  //MOTOR1_DCLINKSS
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 1);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B, 1);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B, 1);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A, 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B, 1);
#endif  //MOTOR1_DCLINKSS
    }
    else
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);

#if defined(MOTOR1_DCLINKSS)
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[1], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);

        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[2], EPWM_SOC_B,
                                        numPWMTicksPerISRTick);
#endif  //MOTOR1_DCLINKSS
    }

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_B);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], pwmPeriodCycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], pwmPeriodCycles);

    // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 10);

    // write the PWM data value  for ADC trigger
#if defined(MOTOR1_DCLINKSS)
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[1], EPWM_SOC_B);

    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_A);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[2], EPWM_SOC_B);

    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[1],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles>>1);

    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_C, pwmPeriodCycles>>1);
    EPWM_setCounterCompareValue(obj->pwmHandle[2],
                                EPWM_COUNTER_COMPARE_D, pwmPeriodCycles>>1);

#endif  //MOTOR1_DCLINKSS

#ifdef SST_ENABLE
    // setup the Time-Base Control Register (TBCTL)
    EPWM_setTimeBaseCounterMode(obj->sstpwmHandle,
                                EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disablePhaseShiftLoad(obj->sstpwmHandle);

    EPWM_setPeriodLoadMode(obj->sstpwmHandle, EPWM_PERIOD_DIRECT_LOAD);

    EPWM_enableSyncOutPulseSource(obj->sstpwmHandle,
                                  EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

    EPWM_setClockPrescaler(obj->sstpwmHandle, EPWM_CLOCK_DIVIDER_1,
                             EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setCountModeAfterSync(obj->sstpwmHandle,
                               EPWM_COUNT_MODE_UP_AFTER_SYNC);

    EPWM_setEmulationMode(obj->sstpwmHandle, EPWM_EMULATION_FREE_RUN);

    // setup the Timer-Based Phase Register (TBPHS)
    EPWM_setPhaseShift(obj->sstpwmHandle, 0);

    // setup the Time-Base Counter Register (TBCTR)
    EPWM_setTimeBaseCounter(obj->sstpwmHandle, 0);

    // setup the Time-Base Period Register (TBPRD)
    // set to zero initially
    EPWM_setTimeBasePeriod(obj->sstpwmHandle, 0);

    // setup the Counter-Compare Control Register (CMPCTL)
    EPWM_setCounterCompareShadowLoadMode(obj->sstpwmHandle,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(obj->sstpwmHandle,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(obj->sstpwmHandle,
                                         EPWM_COUNTER_COMPARE_C,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_setCounterCompareShadowLoadMode(obj->sstpwmHandle,
                                         EPWM_COUNTER_COMPARE_D,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // setup the Action-Qualifier Output A Register (AQCTLB)
    EPWM_setActionQualifierAction(obj->sstpwmHandle,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(obj->sstpwmHandle,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

    EPWM_setActionQualifierAction(obj->sstpwmHandle,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    // setup the Dead-Band Generator Control Register (DBCTL)
    EPWM_setDeadBandDelayMode(obj->sstpwmHandle, EPWM_DB_RED, false);
    EPWM_setDeadBandDelayMode(obj->sstpwmHandle, EPWM_DB_FED, false);

    // setup the PWM-Chopper Control Register (PCCTL)
    EPWM_disableChopper(obj->sstpwmHandle);

    // setup the Trip Zone Select Register (TZSEL)
    EPWM_disableTripZoneSignals(obj->sstpwmHandle, HAL_TZSEL_SIGNALS_ALL);

    EPWM_setTimeBasePeriod(obj->sstpwmHandle, pwmPeriodCycles);
#endif  //SST_ENABLE

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}  // end of HAL_setupPWMs() function


#if defined(EPWMDAC_MODE)
void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;

    // PWMDAC frequency = 100kHz, calculate the period for pwm
    uint16_t  period_cycles = (uint16_t)(systemFreq_MHz *
                                  (float32_t)(1000.0f / HA_PWMDAC_FREQ_KHZ));
    uint16_t  cnt;

    for(cnt = 0; cnt < 4; cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmDACHandle[cnt],
                                    EPWM_COUNTER_MODE_UP);

        EPWM_disablePhaseShiftLoad(obj->pwmDACHandle[cnt]);

        EPWM_setPeriodLoadMode(obj->pwmDACHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);

        EPWM_enableSyncOutPulseSource(obj->pwmDACHandle[cnt],
                                      EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

        EPWM_setClockPrescaler(obj->pwmDACHandle[cnt], EPWM_CLOCK_DIVIDER_1,
                                 EPWM_HSCLOCK_DIVIDER_1);

        EPWM_setCountModeAfterSync(obj->pwmDACHandle[cnt],
                                   EPWM_COUNT_MODE_UP_AFTER_SYNC);

        EPWM_setEmulationMode(obj->pwmDACHandle[cnt], EPWM_EMULATION_FREE_RUN);

        // setup the Timer-Based Phase Register (TBPHS)
        EPWM_setPhaseShift(obj->pwmDACHandle[cnt], 0);

        // setup the Time-Base Counter Register (TBCTR)
        EPWM_setTimeBaseCounter(obj->pwmDACHandle[cnt], 0);

        // setup the Time-Base Period Register (TBPRD)
        // set to zero initially
        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], 0);

        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(obj->pwmDACHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        // setup the Action-Qualifier Output B Register (AQCTLB)
        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                      EPWM_AQ_OUTPUT_B,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


        // setup the Dead-Band Generator Control Register (DBCTL)
        EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_RED, false);

        EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_FED, false);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmDACHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmDACHandle[cnt], HAL_TZSEL_SIGNALS_ALL);

        // since the PWM is configured as an up/down counter, the period
        // register is set to one-half of the desired PWM period
        EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], period_cycles);
    }

    return;
}  // end of HAL_setupPWMDACs() function
#endif // EPWMDAC_MODE


#ifdef QEP_ENABLE
void HAL_setupQEP(HAL_MTR_Handle handle)
{
    HAL_MTR_Obj   *obj = (HAL_MTR_Obj *)handle;

    // Configure the decoder for quadrature count mode, counting both
    // rising and falling edges (that is, 2x resolution), QDECCTL
    EQEP_setDecoderConfig(obj->qepHandle, (EQEP_CONFIG_2X_RESOLUTION |
                                           EQEP_CONFIG_QUADRATURE |
                                           EQEP_CONFIG_NO_SWAP) );

    EQEP_setEmulationMode(obj->qepHandle, EQEP_EMULATIONMODE_RUNFREE);

    // Configure the position counter to be latched on a unit time out
    // and latch on rising edge of index pulse
    EQEP_setLatchMode(obj->qepHandle, (EQEP_LATCH_RISING_INDEX |
                                       EQEP_LATCH_UNIT_TIME_OUT) );

    EQEP_setPositionCounterConfig(obj->qepHandle, EQEP_POSITION_RESET_MAX_POS,
                             (uint32_t)((4 * USER_MOTOR1_NUM_ENC_SLOTS) - 1));

#if defined(ENC_UVW)
    EQEP_setInitialPosition(obj->qepHandle, USER_MOTOR1_ENC_POS_OFFSET);

    EQEP_setPositionInitMode(obj->qepHandle, EQEP_INIT_RISING_INDEX);
#endif

    // Enable the unit timer, setting the frequency to 10KHz
    // QUPRD, QEPCTL
    EQEP_enableUnitTimer(obj->qepHandle, (USER_M1_QEP_UNIT_TIMER_TICKS - 1));

    // Disables the eQEP module position-compare unit
    EQEP_disableCompare(obj->qepHandle);

    // Configure and enable the edge-capture unit. The capture clock divider is
    // SYSCLKOUT/128. The unit-position event divider is QCLK/32.
    EQEP_setCaptureConfig(obj->qepHandle, EQEP_CAPTURE_CLK_DIV_128,
                                          EQEP_UNIT_POS_EVNT_DIV_32);

    // Enable QEP edge-capture unit
    EQEP_enableCapture(obj->qepHandle);

    // Enable UTO on QEP
    EQEP_enableInterrupt(obj->qepHandle, EQEP_INT_UNIT_TIME_OUT);

    // Enable the eQEP module
    EQEP_enableModule(obj->qepHandle);

    return;
}
#endif  // QEP_ENABLE


void HAL_setupSCIA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Initialize SCIA and its FIFO.
    SCI_performSoftwareReset(obj->sciHandle);

    // Configure SCIA for echoback.
    SCI_setConfig(obj->sciHandle, DEVICE_LSPCLK_FREQ, 9600,
                        ( SCI_CONFIG_WLEN_8 |
                          SCI_CONFIG_STOP_ONE |
                          SCI_CONFIG_PAR_NONE ) );

    SCI_resetChannels(obj->sciHandle);

    SCI_resetRxFIFO(obj->sciHandle);

    SCI_resetTxFIFO(obj->sciHandle);

    SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_TXFF | SCI_INT_RXFF);

    SCI_enableFIFO(obj->sciHandle);

    SCI_enableModule(obj->sciHandle);

    SCI_performSoftwareReset(obj->sciHandle);

    return;
}  // end of DRV_setupSci() function


void HAL_setupI2CA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Must put I2C into reset before configuring it
    I2C_disableModule(obj->i2cHandle);

    // I2C configuration. Use a 400kHz I2CCLK with a 50% duty cycle.
    I2C_initMaster(obj->i2cHandle, DEVICE_SYSCLK_FREQ, 400000, I2C_DUTYCYCLE_50);
    I2C_setConfig(obj->i2cHandle, I2C_MASTER_SEND_MODE);
    I2C_setSlaveAddress(obj->i2cHandle, I2C_SLAVE_ADDRESS);
    I2C_disableLoopback(obj->i2cHandle);
    I2C_setBitCount(obj->i2cHandle, I2C_BITCOUNT_8);
    I2C_setDataCount(obj->i2cHandle, 2);
    I2C_setAddressMode(obj->i2cHandle, I2C_ADDR_MODE_7BITS);

    // Enable stop condition and register-access-ready interrupts
//    I2C_enableInterrupt(obj->i2cHandle, I2C_INT_STOP_CONDITION |
//                                        I2C_INT_REG_ACCESS_RDY);

    I2C_enableInterrupt(obj->i2cHandle, I2C_INT_ADDR_SLAVE |
                                        I2C_INT_ARB_LOST |
                                        I2C_INT_NO_ACK |
                                        I2C_INT_STOP_CONDITION);

    // FIFO configuration
    I2C_enableFIFO(obj->i2cHandle);
    I2C_setFIFOInterruptLevel(obj->i2cHandle, I2C_FIFO_TXEMPTY, I2C_FIFO_RX2);

//    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_RXFF | I2C_INT_TXFF);
    I2C_clearInterruptStatus(obj->i2cHandle, I2C_INT_ARB_LOST | I2C_INT_NO_ACK);

    // Configuration complete. Enable the module.
    I2C_setEmulationMode(obj->i2cHandle, I2C_EMULATION_FREE_RUN);
    I2C_enableModule(obj->i2cHandle);

    return;
}  // end of HAL_setupI2CA() function


void HAL_setupTimeBaseTimer(HAL_Handle handle, const float32_t timeBaseFreq_Hz)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;

    uint32_t timerPeriod = (uint32_t)((USER_SYSTEM_FREQ_MHz * 1000.0f *1000.0f) /
                                      timeBaseFreq_Hz);

    // use timer 0 for CPU usage diagnostics
    CPUTimer_setPreScaler(obj->timerHandle[0], 0);

    CPUTimer_setEmulationMode(obj->timerHandle[0],
                              CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_setPeriod(obj->timerHandle[0], timerPeriod);

    CPUTimer_startTimer(obj->timerHandle[0]);

    return;
}  // end of HAL_setupTimeBaseTimer() function


void HAL_setupADCTriggerTimer(HAL_Handle handle, const float32_t adcTriggerFreq_Hz)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;

    uint32_t timerPeriod = (uint32_t)((USER_SYSTEM_FREQ_MHz * 1000.0f *1000.0f) /
                                      adcTriggerFreq_Hz);

    // use timer 1 for CPU usage diagnostics
    CPUTimer_setPreScaler(obj->timerHandle[1], 0);

    CPUTimer_setEmulationMode(obj->timerHandle[1],
                              CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_setPeriod(obj->timerHandle[1], timerPeriod);

    CPUTimer_enableInterrupt(obj->timerHandle[1]);

    CPUTimer_startTimer(obj->timerHandle[1]);

    return;
}  // end of HAL_setupADCTriggerTimer() function


void HAL_setupCPUUsageTimer(HAL_Handle handle)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;

    // use timer 2 for CPU usage diagnostics
    CPUTimer_setPreScaler(obj->timerHandle[2], 0);

    CPUTimer_setEmulationMode(obj->timerHandle[2],
                              CPUTIMER_EMULATIONMODE_RUNFREE);

    CPUTimer_setPeriod(obj->timerHandle[2], 0xFFFFFFFF);

    CPUTimer_startTimer(obj->timerHandle[2]);

    return;
}  // end of HAL_setupCPUUsageTimer() function

#if defined(DATALOGF2_EN)
void HAL_setupDMAforDLOG(HAL_Handle handle, const uint16_t dmaChNum,
                          const void *destAddr, const void *srcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    DMA_configAddresses(obj->dmaChHandle[dmaChNum], destAddr, srcAddr);

    // configure DMA Channel
    DMA_configBurst(obj->dmaChHandle[dmaChNum], DLOG_BURST_SIZE, 2, 2);
//    DMA_configBurst(obj->dmaChHandle[dmaChNum], DLOG_BURST_SIZE, 1, 1);
    DMA_configTransfer(obj->dmaChHandle[dmaChNum], DLOG_TRANSFER_SIZE, 1, 1);
    DMA_configWrap(obj->dmaChHandle[dmaChNum], 0xFFFF, 0, 0xFFFF, 0);

    DMA_configMode(obj->dmaChHandle[dmaChNum], DMA_TRIGGER_SOFTWARE,
                   DMA_CFG_ONESHOT_ENABLE | DMA_CFG_CONTINUOUS_ENABLE |
                   DMA_CFG_SIZE_32BIT);

    DMA_setInterruptMode(obj->dmaChHandle[dmaChNum], DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[dmaChNum]);
    DMA_disableInterrupt(obj->dmaChHandle[dmaChNum]);

    return;
}    //end of HAL_setupDMAforDLOG() function
#endif  // DATALOGF2_EN

void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }
}   //end of HAL_clearDataRAM() function


void HAL_setupDMA(void)
{
    // Initializes the DMA controller to a known state
    DMA_initController();

    // Sets DMA emulation mode, Continue DMA operation
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);

    return;
}    //end of HAL_setupDMA() function


void HAL_setMtrCMPSSDACValue(HAL_MTR_Handle handle,
                             const uint16_t dacValH, const uint16_t dacValL)
{
    HAL_MTR_Obj *obj = (HAL_MTR_Obj *)handle;

#if defined(MOTOR1_ISBLDC) || defined(MOTOR1_DCLINKSS)
#if defined(BSXL8323RS_REVA) || defined(BSXL8323RH_REVB)
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);
#else
#error This board doesn't support single shunt
#endif  // BSXL8323RS_REVA || BSXL8323RH_REVB
#else   // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)
#if defined(HVMTRPFC_REV1P1)
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
#else   //!HVMTRPFC_REV1P1
    CMPSS_setDACValueHigh(obj->cmpssHandle[0], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[0], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[1], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[1], dacValL);

    CMPSS_setDACValueHigh(obj->cmpssHandle[2], dacValH);
    CMPSS_setDACValueLow(obj->cmpssHandle[2], dacValL);
#endif  //!HVMTRPFC_REV1P1
#endif  // !(MOTOR1_ISBLDC || MOTOR1_DCLINKSS)

    return;
}


void HAL_setTriggerPrams(HAL_PWMData_t *pPWMData, const float32_t systemFreq_MHz,
                   const float32_t deadband_us, const float32_t noiseWindow_us)
{
    uint16_t deadband =  (uint16_t)(deadband_us * systemFreq_MHz);
    uint16_t noiseWindow =  (uint16_t)(noiseWindow_us * systemFreq_MHz);

    pPWMData->deadband = deadband;
    pPWMData->noiseWindow = noiseWindow;

    pPWMData->minCMPValue = deadband + noiseWindow + 33;

    return;
}

//*****************************************************************************
//
// Error handling function to be called when an ASSERT is violated
//
//*****************************************************************************
void __error__(char *filename, uint32_t line)
{
    //
    // An ASSERT condition was evaluated as false. You can use the filename and
    // line parameters to determine what went wrong.
    //
    ESTOP0;
}
// end of file
