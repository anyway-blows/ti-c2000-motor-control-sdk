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

//! \file   solutions/boostxl_drv8320rs/f28004x/drivers/hal.c
//! \brief  Contains the various functions related to the HAL object
//!


// **************************************************************************
// the includes

// drivers

// modules
#include "user.h"

// platforms
#include "hal.h"
#include "hal_obj.h"

// libraries
#include "datalog.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals
extern uint32_t Cla1ProgLoadStart;
extern uint32_t Cla1ProgRunStart;
extern uint32_t Cla1ProgLoadSize;

extern uint32_t Cla1Prog2LoadStart;
extern uint32_t Cla1Prog2RunStart;
extern uint32_t Cla1Prog2LoadSize;

extern uint32_t Cla1ConstLoadStart;
extern uint32_t Cla1ConstRunStart;
extern uint32_t Cla1ConstLoadSize;


// **************************************************************************
// the functions
void HAL_cal(HAL_Handle handle)
{
    SysCtl_deviceCal();

    return;
} // end of HAL_cal() function

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

void HAL_enableADCInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(INT_ADCB1);        //RB2

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER1);

    // enable the cpu interrupt for ADC interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(INT_ADCC1);        //RC2

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[2], ADC_INT_NUMBER1);

    // enable the cpu interrupt for ADC interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);
#endif

#ifdef _VSF_EN_
    // enable the cpu timer 0 interrupt for FAST estimator
    Interrupt_enable(INT_TIMER0);
#endif  // _VSF_EN_

    return;
} // end of HAL_enableADCInts() function

void HAL_enableADCIntsToTriggerCLA(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(obj->adcHandle[1], ADC_INT_NUMBER1);

    // Acknowledge interrupt from PIE group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // set up trigger source for CLA task 2
    CLA_setTriggerSource(CLA_TASK_2, CLA_TRIGGER_ADCB1);

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[1], ADC_INT_NUMBER1);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    // clear the ADC interrupt flag
    ADC_clearInterruptStatus(obj->adcHandle[2], ADC_INT_NUMBER1);

    // Acknowledge interrupt from PIE group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // set up trigger source for CLA task 2
    CLA_setTriggerSource(CLA_TASK_2, CLA_TRIGGER_ADCC1);

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[2], ADC_INT_NUMBER1);
#endif

    return;
} // end of HAL_enableADCIntsToTriggerCLA() function


void HAL_enableDebugInt(HAL_Handle handle)
{

    // enable debug events
    ERTM;

    return;
} // end of HAL_enableDebugInt() function


void HAL_enableDRV(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  DRV8320_enable(obj->drv8320Handle);

  return;
}  // end of HAL_enableDRV() function

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

    // initialize the ADC handles
    obj->adcHandle[0] = ADCA_BASE;
    obj->adcHandle[1] = ADCB_BASE;
    obj->adcHandle[2] = ADCC_BASE;

    // initialize the ADC results
    obj->adcResult[0] = ADCARESULT_BASE;
    obj->adcResult[1] = ADCBRESULT_BASE;
    obj->adcResult[2] = ADCCRESULT_BASE;

    // initialize CLA handle
    obj->claHandle = CLA1_BASE;

    // initialize SCI handle
    obj->sciHandle[0] = SCIA_BASE;        //!< the SCIA handle
    obj->sciHandle[1] = SCIB_BASE;        //!< the SCIB handle

    // initialize SPI handle
    obj->spiHandle[0] = SPIA_BASE;        //!< the SPIA handle
    obj->spiHandle[1] = SPIB_BASE;        //!< the SPIB handle

    // initialize DMA handle
    obj->dmaHandle = DMA_BASE;            //!< the DMA handle

    // initialize DMA channel handle
    obj->dmaChHandle[0] = DMA_CH1_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[1] = DMA_CH2_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[2] = DMA_CH3_BASE;   //!< the DMA Channel handle
    obj->dmaChHandle[3] = DMA_CH4_BASE;   //!< the DMA Channel handle

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = EPWM6_BASE;       //!< the PWM handle, GPIO10/GPIO11
    obj->pwmHandle[1] = EPWM5_BASE;       //!< the PWM handle, GPIO8/GPIO9
    obj->pwmHandle[2] = EPWM3_BASE;       //!< the PWM handle, GPIO4/GPIO5

    // initialize PGA handle
    obj->pgaHandle[0] = PGA5_BASE;        //!< the PGA handle
    obj->pgaHandle[1] = PGA3_BASE;        //!< the PGA handle
    obj->pgaHandle[2] = PGA1_BASE;        //!< the PGA handle

    // initialize CMPSS handle
    obj->cmpssHandle[0] = CMPSS5_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = CMPSS3_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CMPSS1_BASE;    //!< the CMPSS handle
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    // initialize PWM handles for Motor 1
    obj->pwmHandle[0] = EPWM1_BASE;       //!< the PWM handle, GPIO0/GPIO1
    obj->pwmHandle[1] = EPWM4_BASE;       //!< the PWM handle, GPIO6/GPIO7
    obj->pwmHandle[2] = EPWM2_BASE;       //!< the PWM handle, GPIO2/GPIO3

    // initialize PGA handle
    obj->pgaHandle[0] = PGA2_BASE;        //!< the PGA handle
    obj->pgaHandle[1] = PGA4_BASE;        //!< the PGA handle
    obj->pgaHandle[2] = PGA6_BASE;        //!< the PGA handle

    // initialize CMPSS handle
    obj->cmpssHandle[0] = CMPSS2_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[1] = CMPSS4_BASE;    //!< the CMPSS handle
    obj->cmpssHandle[2] = CMPSS6_BASE;    //!< the CMPSS handle
#endif

    // initialize DAC handle
    obj->dacHandle[0] = DACB_BASE;        //!< the DAC handle
    obj->dacHandle[1] = DACA_BASE;        //!< the DAC handle

    // initialize timer handles
    obj->timerHandle[0] = CPUTIMER0_BASE;
    obj->timerHandle[1] = CPUTIMER1_BASE;
    obj->timerHandle[2] = CPUTIMER2_BASE;

    // initialize pwmdac handles
    obj->pwmDACHandle[0] = EPWM7_BASE;
    obj->pwmDACHandle[1] = EPWM7_BASE;
    obj->pwmDACHandle[2] = EPWM8_BASE;
    obj->pwmDACHandle[3] = EPWM8_BASE;

    // initialize drv8320 interface
    obj->drv8320Handle = DRV8320_init(&obj->drv8320);

#ifdef _EQEP_EN_
    // initialize QEP driver
    obj->qepHandle[0] = EQEP1_BASE;         // EQEP1
    obj->qepHandle[1] = EQEP2_BASE;         // EQEP2
#endif

  return(handle);
} // end of HAL_init() function

void HAL_setParams(HAL_Handle handle)
{
    HAL_setNumCurrentSensors(handle,USER_NUM_CURRENT_SENSORS);
    HAL_setNumVoltageSensors(handle,USER_NUM_VOLTAGE_SENSORS);

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
#endif

    // Disable DC-DC controller
    ASysCtl_disableDCDC();

    // Enable temperature sensor
    ASysCtl_enableTemperatureSensor();

    // initialize the interrupt controller
    Interrupt_initModule();

    // init vector table
    Interrupt_initVectorTable();

    // Set up PLL control and clock dividers
    // PLLSYSCLK = 20MHz (XTAL_OSC) * 10 (IMULT) * 1 (FMULT) / 2 (PLLCLK_BY_2)
//    SysCtl_setClock(SYSCTL_OSCSRC_XTAL |
//                    SYSCTL_IMULT(10) |
//                    SYSCTL_FMULT_NONE |
//                    SYSCTL_SYSDIV(2) |
//                    SYSCTL_PLL_ENABLE);

    SysCtl_setClock(SYSCTL_OSCSRC_XTAL |
                    SYSCTL_IMULT(10) |
                    SYSCTL_FMULT_NONE |
                    SYSCTL_SYSDIV(2) |
                    SYSCTL_PLL_ENABLE);

    // These asserts will check that the #defines for the clock rates in
    // device.h match the actual rates that have been configured. If they do
    // not match, check that the calculations of DEVICE_SYSCLK_FREQ and
    // DEVICE_LSPCLK_FREQ are accurate. Some examples will not perform as
    // expected if these are not correct.
    //
    ASSERT(SysCtl_getClock(DEVICE_OSCSRC_FREQ) == DEVICE_SYSCLK_FREQ);
    ASSERT(SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) == DEVICE_LSPCLK_FREQ);


    // run the device calibration
    HAL_cal(handle);

    // setup the peripheral clocks
    HAL_setupPeripheralClks(handle);

#ifdef CLA
    // setup CLA
    HAL_setupCLA(handle);
#endif

    // setup the GPIOs
    HAL_setupGPIOs(handle);

#ifdef _FLASH
    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

#ifdef PWMDAC_ENABLE
    // setup the PWM DACs
    HAL_setupPWMDACs(handle, USER_SYSTEM_FREQ_MHz);
#endif

    // setup the ADCs
    HAL_setupADCs(handle);

    // setup the PGAs
    HAL_setupPGAs(handle);

    // setup the Dacs
    HAL_setupDACs(handle);

    // setup the CMPSSs
    HAL_setupCMPSSs(handle);

    // setup the PWMs
    HAL_setupPWMs(handle,
                  USER_SYSTEM_FREQ_MHz,
                  USER_PWM_PERIOD_usec,
                  USER_NUM_PWM_TICKS_PER_ISR_TICK);

    // set the current scale factor
    HAL_setCurrentScaleFactor(handle,USER_CURRENT_SF);

    // set the voltage scale factor
    HAL_setVoltageScaleFactor(handle,USER_VOLTAGE_SF);

    // setup the timers
    HAL_setupTimers(handle, USER_SYSTEM_FREQ_MHz);

    // setup the sci
    HAL_setupSCIA(handle);

    // setup the spiB for DRV8320_Kit_RevD
    HAL_setupSPIA(handle);

    // setup the spiB for DRV8301_Kit_RevD
    HAL_setupSPIB(handle);

    // setup the drv8320 interface
    HAL_setupGate(handle);

#ifdef _EQEP_EN_
    // setup the eqep
    HAL_setupQEP(handle, HAL_QEP_QEP1);
#endif

#ifdef _VSF_EN_
    // setup the timer for estimator
    HAL_setupEstTimer(handle, USER_SYSTEM_FREQ_MHz, USER_EST_FREQ_Hz);
#endif  // _VSF_EN_

  return;
} // end of HAL_setParams() function

void HAL_setupADCs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    SysCtl_delay(100U);
    ADC_setVREF(obj->adcHandle[2], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    SysCtl_delay(100U);

    // Configure internal reference as 1.65V*2 = 3.3V
    ASysCtl_setAnalogReference1P65(ASYSCTL_VREFHIA |
                                   ASYSCTL_VREFHIB |
                                   ASYSCTL_VREFHIC);

    // Enable internal voltage reference
    ASysCtl_setAnalogReferenceInternal(ASYSCTL_VREFHIA |
                                       ASYSCTL_VREFHIB |
                                       ASYSCTL_VREFHIC);

    // Set main clock scaling factor (50MHz max clock for the ADC module)
    ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);
    ADC_setPrescaler(obj->adcHandle[2], ADC_CLK_DIV_2_0);

    // set the ADC interrupt pulse generation to end of conversion
    ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(obj->adcHandle[2], ADC_PULSE_END_OF_CONV);

    // enable the ADCs
    ADC_enableConverter(obj->adcHandle[0]);
    ADC_enableConverter(obj->adcHandle[1]);
    ADC_enableConverter(obj->adcHandle[2]);

    // set priority of SOCs
    ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);
    ADC_setSOCPriority(obj->adcHandle[2], ADC_PRI_ALL_HIPRI);

    // delay to allow ADCs to power up
    SysCtl_delay(1000U);

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    // configure the interrupt sources
    // configure the ample window to 15 system clock cycle wide by assigning 14
    // to the ACQPS of ADCSOCxCTL Register.
    // RB2/B1
    ADC_setInterruptSource(obj->adcHandle[1], ADC_INT_NUMBER1, ADC_SOC_NUMBER2);

    // configure the SOCs for hvkit_rev1p1
    // ISENA - PGA5->A14->RA0
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN14, HAL_ADC_SAMPLE_WINDOW);

    // ISENB - PGA3->C7->RC0
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN7, HAL_ADC_SAMPLE_WINDOW);

    // ISENC - PGA1->B7->RB0
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN7, HAL_ADC_SAMPLE_WINDOW);

    // VSENA - A5->RA1
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN5, HAL_ADC_SAMPLE_WINDOW);

    // VSENB - B0->RB1
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN0, HAL_ADC_SAMPLE_WINDOW);

    // VSENC - C2->RC1
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN2, HAL_ADC_SAMPLE_WINDOW);

    // VSENVM - B1->RB2. hvkit board has capacitor on Vbus feedback, so
    // the sampling doesn't need to be very long to get an accurate value
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN1, HAL_ADC_SAMPLE_WINDOW);

    // Vthrottle - B3->RB3. hvkit board has capacitor on Vbus feedback, so
    // the sampling doesn't need to be very long to get an accurate value
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM6_SOCA,
                 ADC_CH_ADCIN3, HAL_ADC_SAMPLE_WINDOW);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    // configure the interrupt sources
    // configure the ample window to 15 system clock cycle wide by assigning 14
    // to the ACQPS of ADCSOCxCTL Register.
    // RC2/C1
    ADC_setInterruptSource(obj->adcHandle[2], ADC_INT_NUMBER1, ADC_SOC_NUMBER2);

    // configure the SOCs for hvkit_rev1p1
    // ISENA - PGA2->B9->RB0
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN9, HAL_ADC_SAMPLE_WINDOW);

    // ISENB - PGA4->A15->RA0
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN15, HAL_ADC_SAMPLE_WINDOW);

    // ISENC - PGA1->C9->RC0
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN9, HAL_ADC_SAMPLE_WINDOW);

    // VSENA - A6->RA1
    ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN6, HAL_ADC_SAMPLE_WINDOW);

    // VSENB - B6/A2->RB1
    ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN6, HAL_ADC_SAMPLE_WINDOW);

    // VSENC - C14->RC1
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN14, HAL_ADC_SAMPLE_WINDOW);

    // VSENVM - C1->RC2. hvkit board has capacitor on Vbus feedback, so
    // the sampling doesn't need to be very long to get an accurate value
    ADC_setupSOC(obj->adcHandle[2], ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1, HAL_ADC_SAMPLE_WINDOW);
#endif

  return;
} // end of HAL_setupADCs() function

void HAL_setupPGAs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    // For Motor_1/Motor_2
    for(cnt = 0; cnt < 3; cnt++)
    {
        // Set a gain of 12 to PGA1/3/5
        PGA_setGain(obj->pgaHandle[cnt], PGA_GAIN_12);

        // No filter resistor for output
        PGA_setFilterResistor(obj->pgaHandle[cnt], PGA_LOW_PASS_FILTER_DISABLED);

        // Enable PGA1/3/5
        PGA_enable(obj->pgaHandle[cnt]);
    }

    return;
} // end of HAL_setupPGAs() function

// HAL_setupCMPSSs
void HAL_setupCMPSSs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    // Set the initial value to half of ADC range
    uint16_t cmpsaDACH = 2048 + 1024;
    uint16_t cmpsaDACL = 2048 - 1024;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // Enable CMPSS and configure the negative input signal to come
        // from the DAC
        CMPSS_enableModule(obj->cmpssHandle[cnt]);

        CMPSS_configHighComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);
        CMPSS_configLowComparator(obj->cmpssHandle[cnt], CMPSS_INSRC_DAC);

        // Use VDDA as the reference for the DAC and set DAC value to midpoint
        // for arbitrary reference
        CMPSS_configDAC(obj->cmpssHandle[cnt], CMPSS_DACREF_VDDA |
                        CMPSS_DACVAL_SYSCLK | CMPSS_DACSRC_SHDW);

        // define these numbers in hal.h
        //
        CMPSS_setDACValueHigh(obj->cmpssHandle[cnt], cmpsaDACH);
        CMPSS_setDACValueLow(obj->cmpssHandle[cnt], cmpsaDACL);

        // Configure digital filter. For this example, the maxiumum values will
        // be used for the clock prescale, sample window size, and threshold.
        CMPSS_configFilterHigh(obj->cmpssHandle[cnt], 4, 3, 2);
        CMPSS_configFilterLow(obj->cmpssHandle[cnt], 4, 3, 2);

        // Initialize the filter logic and start filtering
        CMPSS_initFilterHigh(obj->cmpssHandle[cnt]);
        CMPSS_initFilterLow(obj->cmpssHandle[cnt]);

        // Configure the output signals. Both CTRIPH and CTRIPOUTH will be fed
        // by the asynchronous comparator output. CMPSS_INV_INVERTED |
        CMPSS_configOutputsHigh(obj->cmpssHandle[cnt], CMPSS_TRIP_FILTER |
                                CMPSS_TRIPOUT_FILTER |
                                CMPSS_OR_ASYNC_OUT_W_FILT);

        CMPSS_configOutputsLow(obj->cmpssHandle[cnt], CMPSS_TRIP_FILTER |
                               CMPSS_TRIPOUT_FILTER |
                               CMPSS_INV_INVERTED);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);
    }

    //
    // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
    // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
    //
#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_5, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_5, 4);

    // Configure TRIP9 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP9, XBAR_EPWM_MUX08_CMPSS5_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP9, XBAR_MUX08);

    // Configure TRIP7 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX00_CMPSS1_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX00);

    // Configure TRIP8 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP8, XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP8, XBAR_MUX04);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_2, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_2, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_4, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_4, 4);

    ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_6, 4);
    ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_6, 4);

    // Configure TRIP11 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP9, XBAR_EPWM_MUX06_CMPSS4_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP9, XBAR_MUX06);

    // Configure TRIP10 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX02_CMPSS2_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX02);

    // Configure TRIP12 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
    XBAR_setEPWMMuxConfig(XBAR_TRIP8, XBAR_EPWM_MUX10_CMPSS6_CTRIPH_OR_L);
    XBAR_enableEPWMMux(XBAR_TRIP8, XBAR_MUX10);
#endif

    return;
} // end of HAL_setupCMPSSs() function

void HAL_setupDACs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    // Set the DAC gain to 2
    DAC_setGainMode(obj->dacHandle[0], DAC_GAIN_TWO);

    // Set ADC voltage reference
    DAC_setReferenceVoltage(obj->dacHandle[0], DAC_REF_ADC_VREFHI);

    // Set load mode for DAC on next SYSCLK
    DAC_setLoadMode(obj->dacHandle[0], DAC_LOAD_SYSCLK);

    // Enable DAC output
    DAC_enableOutput(obj->dacHandle[0]);

    // Set the DAC Shadow Output Value
    // Set the initial value to half of ADC range for 1.65V output
    DAC_setShadowValue(obj->dacHandle[0], 2048U);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    // Set the DAC gain to 2
    DAC_setGainMode(obj->dacHandle[1], DAC_GAIN_TWO);

    // Set ADC voltage reference
    DAC_setReferenceVoltage(obj->dacHandle[1], DAC_REF_ADC_VREFHI);

    // Set load mode for DAC on next SYSCLK
    DAC_setLoadMode(obj->dacHandle[1], DAC_LOAD_SYSCLK);

    // Enable DAC output
    DAC_enableOutput(obj->dacHandle[1]);

    // Set the DAC Shadow Output Value
    // Set the initial value to half of ADC range for 1.65V output
    DAC_setShadowValue(obj->dacHandle[1], 2048U);
#endif

    return;
} // end of HAL_setupDACs() function


void HAL_writeDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_writeData(obj->drv8320Handle,drv8320SPIVars);

  return;
}  // end of HAL_writeDRVData() function


void HAL_readDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_readData(obj->drv8320Handle,drv8320SPIVars);

  return;
}  // end of HAL_readDRVData() function

void HAL_setupDRVSPI(HAL_Handle handle, DRV8320_SPIVars_t *drv8320SPIVars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_setupSPI(obj->drv8320Handle, drv8320SPIVars);

  return;
}  // end of HAL_setupDRVSPI() function

void HAL_setupFaults(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;
    uint16_t cnt;

    // Configure Trip Mechanism for the Motor control software
    // -Cycle by cycle trip on CPU halt
    // -One shot fault trip zone
    // These trips need to be repeated for EPWM1 ,2 & 3

    // configure the input x bar for TZ2 to GPIO, where Over Current is connected
    XBAR_setInputPin(XBAR_INPUT2, HAL_PM_nFAULT_GPIO);
    XBAR_lockInput(XBAR_INPUT2);

    for(cnt=0;cnt<3;cnt++)
    {
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_CBC6);

        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                   EPWM_TZ_SIGNAL_OSHT2);

        //enable DC TRIP combinational input
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
                EPWM_DC_TYPE_DCAH);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
                EPWM_DC_TYPE_DCAL);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
               EPWM_DC_TYPE_DCBH);
        EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
               (EPWM_DC_COMBINATIONAL_TRIPIN7 |
                EPWM_DC_COMBINATIONAL_TRIPIN8 |
                EPWM_DC_COMBINATIONAL_TRIPIN9),
               EPWM_DC_TYPE_DCBL);

        // Trigger event when DCAH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_A1,
                                                     EPWM_TZ_EVENT_DCXH_HIGH);

        // Trigger event when DCBH is High
        EPWM_setTripZoneDigitalCompareEventCondition(obj->pwmHandle[cnt],
                                                     EPWM_TZ_DC_OUTPUT_B1,
                                                     EPWM_TZ_EVENT_DCXL_HIGH);

        // Enable DCA as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCAEVT1);

        // Enable DCB as OST
        EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_DCBEVT1);

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

        // What do we want the OST/CBC events to do?
        // TZA events can force EPWMxA
        // TZB events can force EPWMxB
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZA,
                               EPWM_TZ_ACTION_LOW);
        EPWM_setTripZoneAction(obj->pwmHandle[cnt],
                               EPWM_TZ_ACTION_EVENT_TZB,
                               EPWM_TZ_ACTION_LOW);

        // Clear any high comparator digital filter output latch
        CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

        // Clear any low comparator digital filter output latch
        CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

        // Clear any spurious fault
        EPWM_clearTripZoneFlag(obj->pwmHandle[cnt],
                                        HAL_TZ_INTERRUPT_ALL);
    }

    return;
} // end of HAL_setupFaults() function


void HAL_setupGate(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)
    DRV8320_setSPIHandle(obj->drv8320Handle, obj->spiHandle[0]);
    DRV8320_setGPIOCSNumber(obj->drv8320Handle, HAL_DRV_SPI_CS_GPIO);
    DRV8320_setGPIONumber(obj->drv8320Handle, HAL_DRV_EN_GATE_GPIO);
#endif

#if (BOOST_to_LPD == BOOSTX_to_J5_J6)
    DRV8320_setSPIHandle(obj->drv8320Handle, obj->spiHandle[1]);
    DRV8320_setGPIOCSNumber(obj->drv8320Handle, HAL_DRV_SPI_CS_GPIO);
    DRV8320_setGPIONumber(obj->drv8320Handle, HAL_DRV_EN_GATE_GPIO);
#endif

    return;
} // HAL_setupGate() function


void HAL_setupGPIOs(HAL_Handle handle)
{
    // EPWM1A->UH for J5/J6 Connection
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // EPWM1B->UL for J5/J6 Connection
    GPIO_setMasterCore(1, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // EPWM2A->WH for J5/J6 Connection
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // EPWM2B->WL for J5/J6 Connection
    GPIO_setMasterCore(3, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->WH for J1/J2 Connection
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    // EPWM3B->WL for J1/J2 Connection
    GPIO_setMasterCore(5, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);

    // EPWM4A->VH for J5/J6 Connection
    GPIO_setMasterCore(6, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_6_EPWM4A);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);

    // EPWM4B->VL for J5/J6 Connection
    GPIO_setMasterCore(7, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_7_EPWM4B);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // EPWM5A->VH for J1/J2 Connection
    GPIO_setMasterCore(8, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_8_EPWM5A);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    // EPWM5B->VL for J1/J2 Connection
    GPIO_setMasterCore(9, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_9_EPWM5B);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    // EPWM6A->UH for J1/J2 Connection
    GPIO_setMasterCore(10, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_10_EPWM6A);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);

    // EPWM6B->UL for J1/J2 Connection
    GPIO_setMasterCore(11, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_11_EPWM6B);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // EPWM8B->PWM-DAC3
    GPIO_setMasterCore(12, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_12_EPWM7A);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->J1/J2-DRV_EN
    GPIO_setMasterCore(13, GPIO_CORE_CPU1);
    GPIO_writePin(13, 1);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_PULLUP);

    // EPWM8A->PWM-DAC1
    GPIO_setMasterCore(14, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_14_EPWM8A);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // EPWM8B->PWM-DAC2
    GPIO_setMasterCore(15, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_15_EPWM8B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // GPIO16->SPIA-SDI for J1/J2 connection
    GPIO_setMasterCore(16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_SPISIMOA);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // GPIO17->SPIA-SDO for J1/J2 connection
    GPIO_setMasterCore(17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_SPISOMIA);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_PULLUP);

    // GPIO18->Reserve (N/A for GPIO)
    GPIO_setMasterCore(18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_GPIO18);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // GPIO22->SPIB-CLK for J5/J6 connection
    GPIO_setMasterCore(22, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_22_SPICLKB);
    GPIO_writePin(22, 0);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
    GPIO_setAnalogMode(22, GPIO_ANALOG_DISABLED);

    // GPIO23->LaunchPad LED5
    #ifdef CLA
    GPIO_setMasterCore(23, GPIO_CORE_CPU1_CLA1);
    #else
    GPIO_setMasterCore(23, GPIO_CORE_CPU1);
    #endif
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_writePin(23, 0);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setAnalogMode(23, GPIO_ANALOG_DISABLED);

    // GPIO24->SPIB-SDI for J5/J6 connection
    GPIO_setMasterCore(24, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_24_SPISIMOB);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // GPIO25->LED for J1/J2 connection
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_writePin(25, 0);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // GPIO26->Reserve (N/A)
    GPIO_setMasterCore(26, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);

    // GPIO27->SPIB-CS for J5/J6 connection
    GPIO_setMasterCore(27, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_27_SPISTEB);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // GPIO28->DRV-EN for J5/J6 connection
    GPIO_setMasterCore(28, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_writePin(28, 1);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->nFAULT for J5/J6 connection
    GPIO_setMasterCore(29, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

    // GPIO30->Reserve (N/A), START/STOP
    #ifdef CLA
    GPIO_setMasterCore(30, GPIO_CORE_CPU1_CLA1);
    #else
    GPIO_setMasterCore(30, GPIO_CORE_CPU1_CLA1);
    #endif
    GPIO_setMasterCore(30, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_PULLUP);

    // GPIO31->SPIB-SDO for J5/J6 connection
    GPIO_setMasterCore(31, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_31_SPISOMIB);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_PULLUP);

    // GPIO32->LED for J5/J6 connection
    GPIO_setMasterCore(32, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_writePin(32, 0);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

    // GPIO33->Reserve (N/A)
    GPIO_setMasterCore(33, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->LaunchPad LED5
    #ifdef CLA
    GPIO_setMasterCore(34, GPIO_CORE_CPU1_CLA1);
    #else
    GPIO_setMasterCore(34, GPIO_CORE_CPU1);
    #endif

    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_writePin(34, 0);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // TDI
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_35_TDI);

    // TDO
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_37_TDO);

    // GPIO39->Reserve (N/A)
    GPIO_setMasterCore(39, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_PULLUP);

    // GPIO40->nFAULT for J1/J2 connection
    GPIO_setMasterCore(40, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_PULLUP);

    // GPIO41->Reserve (N/A)
    GPIO_setMasterCore(41, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // GPIO42->Reserve (N/A)
    GPIO_setMasterCore(42, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO43->Reserve (N/A)
    GPIO_setMasterCore(43, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO44->Reserve (N/A)
    GPIO_setMasterCore(44, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_44_GPIO44);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // GPIO45->Reserve (N/A)
    GPIO_setMasterCore(45, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // GPIO46->Reserve (N/A)
    GPIO_setMasterCore(46, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);

    // GPIO47->Reserve (N/A)
    GPIO_setMasterCore(47, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_47_GPIO47);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(47, GPIO_PIN_TYPE_STD);

    // GPIO48->Reserve (N/A)
    GPIO_setMasterCore(48, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_48_GPIO48);
    GPIO_setDirectionMode(48, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(48, GPIO_PIN_TYPE_STD);

    // GPIO49->Reserve (N/A)
    GPIO_setMasterCore(49, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(49, GPIO_PIN_TYPE_STD);

    // GPIO50->Reserve (N/A)
    GPIO_setMasterCore(50, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_50_GPIO50);
    GPIO_setDirectionMode(50, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(50, GPIO_PIN_TYPE_STD);

    // GPIO51->Reserve (N/A)
    GPIO_setMasterCore(51, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_51_GPIO51);
    GPIO_setDirectionMode(51, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(51, GPIO_PIN_TYPE_STD);

    // GPIO52->Reserve (N/A)
    GPIO_setMasterCore(52, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_52_GPIO52);
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);

    // GPIO53->Reserve (N/A)
    GPIO_setMasterCore(53, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_53_GPIO53);
    GPIO_setDirectionMode(53, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(53, GPIO_PIN_TYPE_STD);

    // GPIO54->Reserve (N/A)
    GPIO_setMasterCore(54, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_54_GPIO54);
    GPIO_setDirectionMode(54, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_STD);

    // GPIO55->Reserve (N/A)
    GPIO_setMasterCore(55, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_55_GPIO55);
    GPIO_setDirectionMode(55, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_STD);

    // GPIO56->SPIA-CLK for J1/J2 connection
    GPIO_setMasterCore(56, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_56_SPICLKA);
    GPIO_setDirectionMode(56, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(56, GPIO_PIN_TYPE_STD);

    // GPIO57->SPIA-CS for J1/J2 connection
    GPIO_setMasterCore(57, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_57_SPIA_STE);
    GPIO_setDirectionMode(57, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);

    // GPIO58->Reserve (N/A)
    GPIO_setMasterCore(58, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_58_GPIO58);
    GPIO_setDirectionMode(58, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);

    // GPIO59->Reserve (N/A)
    GPIO_setMasterCore(59, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_59_GPIO59);
    GPIO_setDirectionMode(59, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_STD);

    return;
}  // end of HAL_setupGPIOs() function

#ifdef CLA
void HAL_setupCLA(HAL_Handle handle)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;
    uint32_t tmp_vec;

#ifdef _FLASH
    //
    // Copy CLA code from its load address (FLASH) to CLA program RAM
    //
    // Note: during debug the load and run addresses can be
    // the same as Code Composer Studio can load the CLA program
    // RAM directly.
    //
    // The ClafuncsLoadStart, ClafuncsLoadEnd, and ClafuncsRunStart
    // symbols are created by the linker.
    //
    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
            (uint32_t)&Cla1ProgLoadSize);

    memcpy((uint32_t *)&Cla1Prog2RunStart, (uint32_t *)&Cla1Prog2LoadStart,
            (uint32_t)&Cla1Prog2LoadSize);

//    memcpy((uint32_t *)&Cla1ConstRunStart, (uint32_t *)&Cla1ConstLoadStart,
//            (uint32_t)&Cla1ConstLoadSize);

#endif //_FLASH

    // configure LS memory through LSxMSEL register to allow sharing between CPU and CLA
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS3, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS6, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS7, MEMCFG_LSRAMMASTER_CPU_CLA1);

    // configure what memory is for CLA program through the LSxCLAPGM register
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS2, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS3, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS6, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS7, MEMCFG_CLA_MEM_DATA);

    tmp_vec = (uint32_t)(&task_initModules);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_1, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&task_mainISR);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_2, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&task_mainLoop);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_3, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task4);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_4, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task5);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_5, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task6);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_6, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task7);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_7, (uint16_t)tmp_vec);

    tmp_vec = (uint32_t)(&cla1Task8);
    CLA_mapTaskVector(obj->claHandle, CLA_MVECT_8, (uint16_t)tmp_vec);

    CLA_enableTasks(obj->claHandle, CLA_TASKFLAG_ALL);
    CLA_clearTaskFlags(obj->claHandle, CLA_TASKFLAG_ALL);

    CLA_enableBackgroundTask(obj->claHandle);

    CLA_disableHardwareTrigger(obj->claHandle);

    tmp_vec = (uint32_t)(&cla_EST_run_BackgroundTask);
    CLA_mapBackgroundTaskVector(obj->claHandle, (uint16_t)tmp_vec);

    CLA_setTriggerSource(CLA_TASK_1, CLA_TRIGGER_SOFTWARE);
    CLA_setTriggerSource(CLA_TASK_2, CLA_TRIGGER_SOFTWARE);
    CLA_setTriggerSource(CLA_TASK_3, CLA_TRIGGER_SOFTWARE);

    return;
} // end of HAL_setupCLA() function
#endif

void HAL_setupPeripheralClks(HAL_Handle handle)
{

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_HRPWM);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_SD1);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_I2CA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PGA7);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_LINA);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);

    return;
} // end of HAL_setupPeripheralClks() function

void HAL_setupPWMs(HAL_Handle handle,
                   const float32_t systemFreq_MHz,
                   const float32_t pwmPeriod_usec,
                   const uint_least16_t numPWMTicksPerISRTick)
{
    HAL_Obj   *obj = (HAL_Obj *)handle;
    uint16_t  halfPeriod_cycles = (uint16_t)(systemFreq_MHz *
                                  pwmPeriod_usec / (float32_t)2.0);
    uint16_t  cnt;

    // disable the ePWM module time base clock sync signal
    // to synchronize all of the PWMs
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    for(cnt=0;cnt<3;cnt++)
    {
        // setup the Time-Base Control Register (TBCTL)
        EPWM_setTimeBaseCounterMode(obj->pwmHandle[cnt],
                                    EPWM_COUNTER_MODE_UP_DOWN);
        EPWM_disablePhaseShiftLoad(obj->pwmHandle[cnt]);
        EPWM_setPeriodLoadMode(obj->pwmHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);
        EPWM_setSyncOutPulseMode(obj->pwmHandle[cnt],
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

        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B);

                                                   //
        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_C);

        //
        EPWM_disableCounterCompareShadowLoadMode(obj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_D);

        // setup the Action-Qualifier Output A Register (AQCTLA)
        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        EPWM_setActionQualifierAction(obj->pwmHandle[cnt],
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

        // setup the Action-qualifier Continuous Software Force Register
        // (AQCSFRC)
        EPWM_setActionQualifierContSWForceAction(obj->pwmHandle[cnt],
                                                 EPWM_AQ_OUTPUT_B,
                                                 EPWM_AQ_SW_OUTPUT_HIGH);

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
        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt],HAL_PWM_DBRED_CNT);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt],HAL_PWM_DBFED_CNT);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmHandle[cnt],
                                    EPWM_TZ_SIGNAL_CBC1 |
                                    EPWM_TZ_SIGNAL_CBC2 |
                                    EPWM_TZ_SIGNAL_CBC3 |
                                    EPWM_TZ_SIGNAL_CBC4 |
                                    EPWM_TZ_SIGNAL_CBC5 |
                                    EPWM_TZ_SIGNAL_CBC6 |
                                    EPWM_TZ_SIGNAL_DCAEVT2 |
                                    EPWM_TZ_SIGNAL_DCBEVT2 |
                                    EPWM_TZ_SIGNAL_OSHT1 |
                                    EPWM_TZ_SIGNAL_OSHT2 |
                                    EPWM_TZ_SIGNAL_OSHT3 |
                                    EPWM_TZ_SIGNAL_OSHT4 |
                                    EPWM_TZ_SIGNAL_OSHT5 |
                                    EPWM_TZ_SIGNAL_OSHT6 |
                                    EPWM_TZ_SIGNAL_DCAEVT1 |
                                    EPWM_TZ_SIGNAL_DCBEVT1);
    }

    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_disableInterrupt(obj->pwmHandle[0]);
    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A,
                             EPWM_SOC_TBCTR_D_CMPC);

    EPWM_enableADCTrigger(obj->pwmHandle[0], EPWM_SOC_A);

    // setup the Event Trigger Prescale Register (ETPS)
    if(numPWMTicksPerISRTick > 15)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 15);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 15);
    }
    else if(numPWMTicksPerISRTick < 1)
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], 1);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A, 1);
    }
    else
    {
        EPWM_setInterruptEventCount(obj->pwmHandle[0], numPWMTicksPerISRTick);
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0],
                                        EPWM_SOC_A,
                                        numPWMTicksPerISRTick);
    }

    // setup the Event Trigger Clear Register (ETCLR)
    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);
    EPWM_clearADCTriggerFlag(obj->pwmHandle[0], EPWM_SOC_A);

    // since the PWM is configured as an up/down counter, the period register is
    // set to one-half of the desired PWM period
    EPWM_setTimeBasePeriod(obj->pwmHandle[0], halfPeriod_cycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[1], halfPeriod_cycles);
    EPWM_setTimeBasePeriod(obj->pwmHandle[2], halfPeriod_cycles);

      // write the PWM data value  for ADC trigger
    EPWM_setCounterCompareValue(obj->pwmHandle[0],
                                EPWM_COUNTER_COMPARE_C,
                                2);

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}  // end of HAL_setupPWMs() function

void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  // PWMDAC frequency = 100kHz, calculate the period for pwm
  uint16_t halfPeriod_cycles = (uint16_t)(systemFreq_MHz *
                               (float32_t)(1000.0/100.0/2.0));
  uint16_t  cnt;

  // disable the ePWM module time base clock sync signal
  // to synchronize all of the PWMs
  SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

  for(cnt=0;cnt<4;cnt++)
    {
      // setup the Time-Base Control Register (TBCTL)
      EPWM_setTimeBaseCounterMode(obj->pwmDACHandle[cnt],
                                  EPWM_COUNTER_MODE_UP_DOWN);
      EPWM_disablePhaseShiftLoad(obj->pwmDACHandle[cnt]);
      EPWM_setPeriodLoadMode(obj->pwmDACHandle[cnt], EPWM_PERIOD_DIRECT_LOAD);
      EPWM_setSyncOutPulseMode(obj->pwmDACHandle[cnt],
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
                                    EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_A,
                                    EPWM_AQ_OUTPUT_LOW,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);


      // setup the Action-Qualifier Output B Register (AQCTLB)
      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_B,
                                    EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

      EPWM_setActionQualifierAction(obj->pwmDACHandle[cnt],
                                    EPWM_AQ_OUTPUT_B,
                                    EPWM_AQ_OUTPUT_LOW,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


      // setup the Dead-Band Generator Control Register (DBCTL)
      EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_RED, false);
      EPWM_setDeadBandDelayMode(obj->pwmDACHandle[cnt], EPWM_DB_FED, false);

      // setup the PWM-Chopper Control Register (PCCTL)
      EPWM_disableChopper(obj->pwmDACHandle[cnt]);

      // setup the Trip Zone Select Register (TZSEL)
      EPWM_disableTripZoneSignals(obj->pwmDACHandle[cnt],
                                  EPWM_TZ_SIGNAL_CBC1 |
                                  EPWM_TZ_SIGNAL_CBC2 |
                                  EPWM_TZ_SIGNAL_CBC3 |
                                  EPWM_TZ_SIGNAL_CBC4 |
                                  EPWM_TZ_SIGNAL_CBC5 |
                                  EPWM_TZ_SIGNAL_CBC6 |
                                  EPWM_TZ_SIGNAL_DCAEVT2 |
                                  EPWM_TZ_SIGNAL_DCBEVT2 |
                                  EPWM_TZ_SIGNAL_OSHT1 |
                                  EPWM_TZ_SIGNAL_OSHT2 |
                                  EPWM_TZ_SIGNAL_OSHT3 |
                                  EPWM_TZ_SIGNAL_OSHT4 |
                                  EPWM_TZ_SIGNAL_OSHT5 |
                                  EPWM_TZ_SIGNAL_OSHT6 |
                                  EPWM_TZ_SIGNAL_DCAEVT1 |
                                  EPWM_TZ_SIGNAL_DCBEVT1);

      // since the PWM is configured as an up/down counter, the period register
      // is set to one-half of the desired PWM period
      EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], halfPeriod_cycles);
    }

  // enable the ePWM module time base clock sync signal
  SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

  return;
}  // end of HAL_setupPWMs() function


#ifdef _EQEP_EN_
void HAL_setupQEP(HAL_Handle handle, HAL_QEPSelect_e qep)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  //
  // Configure the decoder for quadrature count mode
  //
  EQEP_setDecoderConfig(obj->qepHandle[qep], (EQEP_CONFIG_1X_RESOLUTION |
                                     EQEP_CONFIG_QUADRATURE |
                                     EQEP_CONFIG_NO_SWAP));

  EQEP_setEmulationMode(obj->qepHandle[qep], EQEP_EMULATIONMODE_RUNFREE);

  //
  // Configure the position counter to reset on an index event
  //
  EQEP_setPositionCounterConfig(obj->qepHandle[qep], EQEP_POSITION_RESET_IDX,
                                0xFFFFFFFF);

  //
  // Enable the unit timer, setting the frequency to 100 Hz
  //
  EQEP_enableUnitTimer(obj->qepHandle[qep], (DEVICE_SYSCLK_FREQ / 100));

  //
  // Configure the position counter to be latched on a unit time out
  //
  EQEP_setLatchMode(obj->qepHandle[qep], EQEP_LATCH_UNIT_TIME_OUT);

  //
  // Enable the eQEP module
  //
  EQEP_enableModule(obj->qepHandle[qep]);

  //
  // Configure and enable the edge-capture unit. The capture clock divider is
  // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
  //
  EQEP_setCaptureConfig(obj->qepHandle[qep], EQEP_CAPTURE_CLK_DIV_64,
                        EQEP_UNIT_POS_EVNT_DIV_32);

  EQEP_enableCapture(obj->qepHandle[qep]);

  return;
}
#endif


void HAL_setupSCIA(HAL_Handle halHandle)
{
  HAL_Obj *obj = (HAL_Obj *)halHandle;

  // Initialize SCIA and its FIFO.
  SCI_performSoftwareReset(obj->sciHandle[0]);

  // Configure SCIA for echoback.
  SCI_setConfig(obj->sciHandle[0], DEVICE_LSPCLK_FREQ, 9600,
                                                      (SCI_CONFIG_WLEN_8 |
                                                      SCI_CONFIG_STOP_ONE |
                                                      SCI_CONFIG_PAR_NONE));
  SCI_resetChannels(obj->sciHandle[0]);

  SCI_resetRxFIFO(obj->sciHandle[0]);

  SCI_resetTxFIFO(obj->sciHandle[0]);

  SCI_clearInterruptStatus(obj->sciHandle[0], SCI_INT_TXFF | SCI_INT_RXFF);

  SCI_enableFIFO(obj->sciHandle[0]);

  SCI_enableModule(obj->sciHandle[0]);

  SCI_performSoftwareReset(obj->sciHandle[0]);

}  // end of DRV_setupSci() function


void HAL_setupSPIA(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  // Must put SPI into reset before configuring it
  SPI_disableModule(obj->spiHandle[0]);

  // SPI configuration. Use a 1MHz SPICLK and 16-bit word size. 500kbps
  SPI_setConfig(obj->spiHandle[0], DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                SPI_MODE_MASTER, 400000, 16);

  SPI_disableLoopback(obj->spiHandle[0]);

  SPI_setEmulationMode(obj->spiHandle[0], SPI_EMULATION_FREE_RUN);

  SPI_enableFIFO(obj->spiHandle[0]);

  HWREGH((obj->spiHandle[0])+SPI_O_FFCT) = 0x0018;

  SPI_clearInterruptStatus(obj->spiHandle[0], SPI_INT_TXFF);

  // Configuration complete. Enable the module.
  SPI_enableModule(obj->spiHandle[0]);

  return;
}  // end of HAL_setupSPIA() function


void HAL_setupSPIB(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  // Must put SPI into reset before configuring it
  SPI_disableModule(obj->spiHandle[1]);

  // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
  SPI_setConfig(obj->spiHandle[1], DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                SPI_MODE_MASTER, 400000, 16);

  SPI_disableLoopback(obj->spiHandle[1]);

  SPI_setEmulationMode(obj->spiHandle[1], SPI_EMULATION_FREE_RUN);

  SPI_enableFIFO(obj->spiHandle[1]);
  HWREGH((obj->spiHandle[1])+SPI_O_FFCT) = 0x0018;

  SPI_clearInterruptStatus(obj->spiHandle[1], SPI_INT_TXFF);

  // Configuration complete. Enable the module.
  SPI_enableModule(obj->spiHandle[1]);

  return;
}  // end of HAL_setupSPIB() function


void HAL_setupEstTimer(HAL_Handle handle,
                       const float32_t systemFreq_MHz,
                       const float32_t estFreq_Hz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  uint32_t temp;

  //
  // calculate timer period value
  //
  temp = ((uint32_t)(systemFreq_MHz *1000000.0f / estFreq_Hz)) - 1;

  //
  // use CPU timer 0 for estimator ISR in variable pwm frequency project
  //
  CPUTimer_setPreScaler(obj->timerHandle[0], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[0],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[0], temp);

  CPUTimer_enableInterrupt(obj->timerHandle[0]);

  // Starts CPU-Timer 0.
  CPUTimer_startTimer(obj->timerHandle[0]);

  return;
}  // end of HAL_setupEstTimer() function


void HAL_setupTimers(HAL_Handle handle, const float32_t systemFreq_MHz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  //
  // 1ms calculation
  //
  uint32_t timerPeriod_1ms = (uint32_t)(systemFreq_MHz *
                              (float32_t)1000.0f) - 1;

  //
  // use CPU timer 0 for estimator ISR in variable pwm frequency project
  //
  CPUTimer_setPreScaler(obj->timerHandle[0], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[0],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[0], timerPeriod_1ms);

  //
  // 1ms, use CPU timer 1 as a software timer for system control
  //
  CPUTimer_setPreScaler(obj->timerHandle[1], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[1],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[1], timerPeriod_1ms);

  //
  // use CPU timer 2 for CPU usage diagnostics
  //
  CPUTimer_setPreScaler(obj->timerHandle[2], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[2],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[2], 0xFFFFFFFF);

  return;
}  // end of HAL_setupTimers() function


void HAL_setupVSFPWMMode(HAL_Handle halHandle)
{
    HAL_Obj *halObj = (HAL_Obj *)halHandle;

    // setup the Counter-Compare Control Register (CMPCTL)
    uint16_t cnt;

    for(cnt = 0; cnt < 3; cnt++)
    {
        // setup the Counter-Compare Control Register (CMPCTL)
        EPWM_setCounterCompareShadowLoadMode(halObj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_A,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setCounterCompareShadowLoadMode(halObj->pwmHandle[cnt],
                                             EPWM_COUNTER_COMPARE_B,
                                             EPWM_COMP_LOAD_ON_CNTR_ZERO);

        EPWM_setPeriodLoadMode(halObj->pwmHandle[cnt],
                               EPWM_PERIOD_DIRECT_LOAD);
    }

    return;
}  // end of HAL_setupVSFPWMMode() function


void HAL_setPWMDACParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pPWMDACData->periodMax =
            PWMDAC_getPeriod(obj->pwmDACHandle[PWMDAC_NUMBER_1]);

    pPWMDACData->offset[0] = 0.0;
    pPWMDACData->offset[1] = 0.0;
    pPWMDACData->offset[2] = 0.0;
    pPWMDACData->offset[3] = 0.0;

    pPWMDACData->gain[0] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[1] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[2] = MATH_ONE_OVER_TWO_PI;
    pPWMDACData->gain[3] = MATH_ONE_OVER_TWO_PI;

    return;
}   //end of HAL_setPWMDACParameters() function


void HAL_setupDlogWithDMA(HAL_Handle handle,
                          const uint16_t dmaChannel,
                          const void *dlogDestAddr,
                          const void *dlogSrcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    const void *destAddr;
    const void *srcAddr;
    destAddr = (const void *)dlogDestAddr;
    srcAddr  = (const void *)dlogSrcAddr;

    //
    // configure DMA Channel
    //
    DMA_configAddresses(obj->dmaChHandle[dmaChannel], destAddr, srcAddr);
    DMA_configBurst(obj->dmaChHandle[dmaChannel], DLOG_BURST_SIZE, 2, 2);
    DMA_configTransfer(obj->dmaChHandle[dmaChannel], DLOG_TRANSFER_SIZE, 1, 1);
    DMA_configMode(obj->dmaChHandle[dmaChannel], DMA_TRIGGER_SOFTWARE,
                   (DMA_CFG_ONESHOT_ENABLE +
                    DMA_CFG_CONTINUOUS_ENABLE +
                    DMA_CFG_SIZE_32BIT));
    DMA_setInterruptMode(obj->dmaChHandle[dmaChannel],DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[dmaChannel]);
    DMA_disableInterrupt(obj->dmaChHandle[dmaChannel]);

    return;
}    //end of HAL_initDlogDMA() function


void HAL_clearDataRAM(void *pMemory, const uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t loopCount, loopLength;

    pMemoryStart = pMemory;
    loopLength = lengthMemory;

    for(loopCount = 0; loopCount < loopLength; loopCount++)
    {
        *(pMemoryStart + loopCount) = 0x0000;
    }

    return;
}   //end of HAL_clearDataRAM() function


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
