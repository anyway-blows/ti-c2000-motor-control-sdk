//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
//     Copyright (C) 2020 Texas Instruments Incorporated -
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//! \file   solutions/drv8312_c2_kit/f28004x/drivers/hal.c
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
//#include "datalog.h"
#include "device.h"

#include "communication.h"

#ifdef _FLASH
#pragma CODE_SECTION(Flash_initModule, ".TI.ramfunc");
#endif

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions
//#ifdef _FLASH
void HAL_cal(HAL_Handle handle)
{
    SysCtl_deviceCal();

    return;
} // end of HAL_cal() function
//#endif

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

#if defined(CMD_CAN_ENABLE)
void HAL_enableCANInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the CAN interrupts
    CAN_enableInterrupt(obj->canHandle, CAN_INT_IE0 | CAN_INT_ERROR |
                            CAN_INT_STATUS);

    // enable the PIE interrupts associated with the CAN interrupts
    Interrupt_enable(INT_CANA0);

    CAN_enableGlobalInterrupt(obj->canHandle, CAN_GLOBAL_INT_CANINT0);

    // enable the cpu interrupt for CAN interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT9);

    return;
} // end of HAL_enableCANInts() function
#endif  // CMD_CAN_ENABLE

void HAL_enableADCInts(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    // enable the PIE interrupts associated with the ADC interrupts
    Interrupt_enable(INT_ADCA1);

    // enable the ADC interrupts
    ADC_enableInterrupt(obj->adcHandle[0], ADC_INT_NUMBER1);

    // enable the cpu interrupt for ADC interrupts
    Interrupt_enableInCPU(INTERRUPT_CPU_INT1);

    return;
} // end of HAL_enableADCInts() function

void HAL_enableDebugInt(HAL_Handle handle)
{

    // enable debug events
    ERTM;

    return;
} // end of HAL_enableDebugInt() function

void HAL_enableDRV(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;


#ifdef DRV8320_SPI
  DRV8320_enable(obj->drv8320Handle);
#endif

#ifdef DRV8323_SPI
  DRV8323_enable(obj->drv8323Handle);
#endif

  return;
}  // end of HAL_enableDRV() function

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
    return((HAL_Handle)NULL);

  // assign the handle
  handle = (HAL_Handle)pMemory;

  // assign the object
  obj = (HAL_Obj *)handle;

  // disable watchdog
  SysCtl_disableWatchdog();

  // initialize the ADC handles
  obj->adcHandle[0] = ADCA_BASE;
  obj->adcHandle[1] = ADCC_BASE;

  // initialize the ADC results
  obj->adcResult[0] = ADCARESULT_BASE;
  obj->adcResult[1] = ADCCRESULT_BASE;

  // initialize SCI handle
  obj->sciHandle = SCIA_BASE;        //!< the SCIA handle

  // initialize CAN handle
  obj->canHandle = CANA_BASE;        //!< the CANA handle

#ifdef SPI_ENABLE
  // initialize SPI handle
  obj->spiHandle[0] = SPIA_BASE;            //!< the SPIA handle
  obj->spiHandle[1] = SPIB_BASE;            //!< the SPIB handle
#endif

#ifdef DATALOG_ENABLE
  // initialize DMA handle
  obj->dmaHandle = DMA_BASE;            //!< the DMA handle

  // initialize DMA channel handle
  obj->dmaChHandle[0] = DMA_CH1_BASE;   //!< the DMA Channel handle
  obj->dmaChHandle[1] = DMA_CH2_BASE;   //!< the DMA Channel handle
  obj->dmaChHandle[2] = DMA_CH3_BASE;   //!< the DMA Channel handle
  obj->dmaChHandle[3] = DMA_CH4_BASE;   //!< the DMA Channel handle
#endif

  // initialize PWM handles for Motor 1
  obj->pwmHandle[0] = EPWM1_BASE;
  obj->pwmHandle[1] = EPWM2_BASE;
  obj->pwmHandle[2] = EPWM3_BASE;

  // initialize CMPSS handle
  obj->cmpssHandle[0] = CMPSS1_BASE;    //!< the CMPSS handle
  obj->cmpssHandle[1] = CMPSS2_BASE;    //!< the CMPSS handle
  obj->cmpssHandle[2] = CMPSS3_BASE;    //!< the CMPSS handle


  // initialize timer handles
  obj->timerHandle[0] = CPUTIMER0_BASE;
  obj->timerHandle[1] = CPUTIMER1_BASE;
  obj->timerHandle[2] = CPUTIMER2_BASE;

#ifdef PWMDAC_ENABLE
  // initialize pwmdac handles
  obj->pwmDACHandle[0] = EPWM6_BASE;
  obj->pwmDACHandle[1] = EPWM6_BASE;
  obj->pwmDACHandle[2] = EPWM5_BASE;
  obj->pwmDACHandle[3] = EPWM5_BASE;
#endif

  obj->drv8323Handle = DRV8323_init(&obj->drv8323);

#ifdef EQEP_ENABLE
  // initialize QEP driver
  obj->qepHandle[0] = EQEP1_BASE;           // EQEP1
  obj->qepHandle[1] = EQEP2_BASE;           // EQEP2
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

  // Enable temperature sensor
  ASysCtl_enableTemperatureSensor();

  //Device_initGPIO();

  // initialize the interrupt controller
  Interrupt_initModule();

  // init vector table
  Interrupt_initVectorTable();

  // Set up PLL control and clock dividers
  // PLLSYSCLK = 20MHz (XTAL_OSC) * 30 (IMULT) / (2 (REFDIV) * 3 (ODIV) * 1(SYSDIV))
  SysCtl_setClock(SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(30) |
                  SYSCTL_REFDIV(2) | SYSCTL_ODIV(3) |
                  SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE |
                  SYSCTL_DCC_BASE_0);

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

  // Make sure the LSPCLK divider is set to divide by 2
  SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_2); // 50MHz for SFRA, SPI&SCI

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
#endif  // PWMDAC_ENABLE

  // setup the ADCs
  HAL_setupADCs(handle);        //

  // setup the CMPSSs
  HAL_setupCMPSSs(handle);      //

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

  // setup the drv8320 interface
  HAL_setupGate(handle);

#if defined(SCI_ENABLE)
  // setup the sci
  HAL_setupSCIA(handle);
#endif // SCI_ENABLE

#if defined(CMD_CAN_ENABLE)
  // setup the CANA
  HAL_setupCANA(handle);
#endif  // CMD_CAN_ENABLE


#ifdef SPI_ENABLE
  // setup the spiB for DRV8320_Kit_RevD
  HAL_setupSPIA(handle);
#endif

#ifdef EQEP_ENABLE
  // setup the eqep
  HAL_setupQEP(handle, HAL_QEP_QEP1);
#endif

  return;
} // end of HAL_setParams() function

void HAL_setupADCs(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  SysCtl_delay(100U);
  ADC_setVREF(obj->adcHandle[0], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
  ADC_setVREF(obj->adcHandle[1], ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
  SysCtl_delay(100U);

  // Set main clock scaling factor (50MHz max clock for the ADC module)
  ADC_setPrescaler(obj->adcHandle[0], ADC_CLK_DIV_2_0);
  ADC_setPrescaler(obj->adcHandle[1], ADC_CLK_DIV_2_0);

  // set the ADC interrupt pulse generation to end of conversion
  ADC_setInterruptPulseMode(obj->adcHandle[0], ADC_PULSE_END_OF_CONV);
  ADC_setInterruptPulseMode(obj->adcHandle[1], ADC_PULSE_END_OF_CONV);

  // enable the ADCs
  ADC_enableConverter(obj->adcHandle[0]);
  ADC_enableConverter(obj->adcHandle[1]);

  // set priority of SOCs
  ADC_setSOCPriority(obj->adcHandle[0], ADC_PRI_ALL_HIPRI);
  ADC_setSOCPriority(obj->adcHandle[1], ADC_PRI_ALL_HIPRI);

  // delay to allow ADCs to power up
  SysCtl_delay(1000U);

  // configure the interrupt sources    //RA3
  ADC_setInterruptSource(obj->adcHandle[1], ADC_INT_NUMBER1, ADC_SOC_NUMBER3);

  // configure the SOCs for drv8312kit_revD
  // IA-FB - A1/RA0
  ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN0, HAL_ADC_SAMPLE_WINDOW);

  // IB-FB - C1/RC0
  ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN14, HAL_ADC_SAMPLE_WINDOW);

  // IC-FB - A3/RA1
  ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN7, HAL_ADC_SAMPLE_WINDOW);

  // ADC-Vhb1 - C11/RC1
  ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN6, HAL_ADC_SAMPLE_WINDOW);

  // ADC-Vhb2 - A7/RA2
  ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN5, HAL_ADC_SAMPLE_WINDOW);

  // ADC-Vhb3 - C7/RC2
  ADC_setupSOC(obj->adcHandle[0], ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN2, HAL_ADC_SAMPLE_WINDOW);

  // VDCBUS -   A14/RA3. DRV8312 board does not have capacitor on Vbus feedback, so
  // the sampling needs to be very long to get an accurate value
  ADC_setupSOC(obj->adcHandle[1], ADC_SOC_NUMBER3, ADC_TRIGGER_EPWM1_SOCA,
               ADC_CH_ADCIN6, HAL_ADC_SAMPLE_WINDOW);

  return;
} // end of HAL_setupADCs() function

// HAL_setupCMPSSs
void HAL_setupCMPSSs(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

        uint16_t  cnt;
        uint16_t cmpsaDACH;
        uint16_t cmpsaDACL;

        // Refer to the Table 9-2 in Chapter 9 of TMS320F28004x
        // Technical Reference Manual (SPRUI33B), to configure the ePWM X-Bar
        cmpsaDACH = CMPSS_DACH_VALUE;
        cmpsaDACL = CMPSS_DACL_VALUE;

        ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, IA_CMPHP_MUX);
        ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, IA_CMPLP_MUX);

        ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_3, IB_CMPHP_MUX);
        ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_3, IB_CMPLP_MUX);

        ASysCtl_selectCMPHPMux(ASYSCTL_CMPHPMUX_SELECT_1, IC_CMPHP_MUX);
        ASysCtl_selectCMPLPMux(ASYSCTL_CMPLPMUX_SELECT_1, IC_CMPLP_MUX);

        for(cnt=0;cnt<3;cnt++)
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

        return;


} // end of HAL_setupCMPSSs() function

#ifdef DRV8320_SPI
void HAL_writeDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_writeData(obj->drv8320Handle,drv8320Vars);

  return;
}  // end of HAL_writeDRVData() function


void HAL_readDRVData(HAL_Handle handle, DRV8320_SPIVars_t *drv8320Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_readData(obj->drv8320Handle,drv8320Vars);

  return;
}  // end of HAL_readDRVData() function

void HAL_setupDRVSPI(HAL_Handle handle, DRV8320_SPIVars_t *drv8320Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8320_setupSPI(obj->drv8320Handle, drv8320Vars);

  return;
}  // end of HAL_setupDRVSPI() function
#endif

#ifdef DRV8323_SPI
void HAL_writeDRVData(HAL_Handle handle, DRV8323_SPIVars_t *drv8323Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8323_writeData(obj->drv8323Handle,drv8323Vars);

  return;
}  // end of HAL_writeDRVData() function


void HAL_readDRVData(HAL_Handle handle, DRV8323_SPIVars_t *drv8323Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8323_readData(obj->drv8323Handle,drv8323Vars);

  return;
}  // end of HAL_readDRVData() function

void HAL_setupDRVSPI(HAL_Handle handle, DRV8323_SPIVars_t *drv8323Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8323_setupSPI(obj->drv8323Handle, drv8323Vars);

  return;
}  // end of HAL_setupDRVSPI() function
#endif

void HAL_setupFaults(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;
  uint_least8_t cnt;

  // Configure TRIP 7 to OR the High and Low trips from both
     // comparator 5, 3 & 1, clear everything first
     EALLOW;
     HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP7MUX0TO15CFG) = 0;
     HWREG(XBAR_EPWM_CFG_REG_BASE + XBAR_O_TRIP7MUX16TO31CFG) = 0;
     EDIS;

     // Configure TRIP7 to be CTRIP5H and CTRIP5L using the ePWM X-BAR
     XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX00_CMPSS1_CTRIPH);

     // Configure TRIP7 to be CTRIP1H and CTRIP1L using the ePWM X-BAR
     XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX04_CMPSS3_CTRIPH_OR_L);

     // Configure TRIP7 to be CTRIP3H and CTRIP3L using the ePWM X-BAR
     XBAR_setEPWMMuxConfig(XBAR_TRIP7, XBAR_EPWM_MUX01_CMPSS1_CTRIPL);

     // Disable all the mux first
     XBAR_disableEPWMMux(XBAR_TRIP7, 0xFFFF);

     // Enable Mux 0  OR Mux 4 to generate TRIP
     XBAR_enableEPWMMux(XBAR_TRIP7, XBAR_MUX00 |
                        XBAR_MUX04 | XBAR_MUX01);

     // configure the input x bar for TZ2 to GPIO, where Over Current is connected
     XBAR_setInputPin(INPUTXBAR_BASE, XBAR_INPUT1, HAL_PM_nFAULT_GPIO);
     XBAR_lockInput(INPUTXBAR_BASE, XBAR_INPUT1);



     for(cnt=0;cnt<3;cnt++)
     {
         EPWM_enableTripZoneSignals(obj->pwmHandle[cnt], EPWM_TZ_SIGNAL_OSHT1);
     }

     // Configure Trip Mechanism for the Motor control software
     // -Cycle by cycle trip on CPU halt
     // -One shot fault trip zone
     // These trips need to be repeated for EPWM1 ,2 & 3

     for(cnt=0;cnt<3;cnt++)
     {
         EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                    EPWM_TZ_SIGNAL_CBC6);

         EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                    EPWM_TZ_SIGNAL_OSHT2);

         EPWM_enableTripZoneSignals(obj->pwmHandle[cnt],
                                    EPWM_TZ_SIGNAL_OSHT3);

         //enable DC TRIP combinational input
         EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                                       EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCAH);

         EPWM_enableDigitalCompareTripCombinationInput(obj->pwmHandle[cnt],
                                                       EPWM_DC_COMBINATIONAL_TRIPIN7, EPWM_DC_TYPE_DCBH);

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

         // Clear any high comparator digital filter output latch
         CMPSS_clearFilterLatchHigh(obj->cmpssHandle[cnt]);

         // Clear any low comparator digital filter output latch
         CMPSS_clearFilterLatchLow(obj->cmpssHandle[cnt]);

         // Clear any spurious fault
         EPWM_clearTripZoneFlag(obj->pwmHandle[cnt], HAL_TZ_INTERRUPT_ALL);
     }

     return;

} // end of HAL_setupFaults() function

void HAL_setupGate(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

#if (BOOST_to_LPD == BOOSTX_to_J1_J2)

#ifdef DRV8320_SPI
    DRV8320_setSPIHandle(obj->drv8320Handle, obj->spiHandle[0]);
    DRV8320_setGPIOCSNumber(obj->drv8320Handle, HAL_DRV_SPI_CS_GPIO);
    DRV8320_setGPIONumber(obj->drv8320Handle, HAL_DRV_EN_GATE_GPIO);
#endif

#ifdef DRV8323_SPI
    DRV8323_setSPIHandle(obj->drv8323Handle, obj->spiHandle[0]);
    DRV8323_setGPIOCSNumber(obj->drv8323Handle, HAL_DRV_SPI_CS_GPIO);
    DRV8323_setGPIONumber(obj->drv8323Handle, HAL_DRV_EN_GATE_GPIO);
#endif

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
    // EPWM1A->UH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);

    // EPWM1B->LH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);

    // EPWM2A->UH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);

    // EPWM2B->LH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);

    // EPWM3A->UH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);

    //// SPI-CS
    GPIO_setPinConfig(GPIO_5_SPIA_STE);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_PULLUP);

#ifdef _FLASH
    // No Connection
    GPIO_setPinConfig(GPIO_6_GPIO6);
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
#endif

    // STOP Push Button
    GPIO_setPinConfig(GPIO_7_GPIO7);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);

    // DRV_SCS
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);

    //SPIA
    //SPI-CLK
    GPIO_setPinConfig(GPIO_9_SPIA_CLK);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);

    //// SPI-SOMI
    GPIO_setPinConfig(GPIO_10_SPIA_SOMI);
    GPIO_setDirectionMode(10, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_PULLUP);

    //// SPI-SIMO
    GPIO_setPinConfig(GPIO_11_SPIA_SIMO);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_STD);

    // GPIO13->OCTw
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_STD);

    // GPIO14->nFAULT
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_STD);

    // EPWM3B->LH for J1/J2 Connection
    GPIO_setPinConfig(GPIO_15_EPWM3_B);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_16_GPIO16);
    GPIO_setDirectionMode(16, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(16, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_17_GPIO17);
    GPIO_setDirectionMode(17, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(17, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_18_GPIO18_X2);
    GPIO_setDirectionMode(18, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(18, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_19_GPIO19_X1);
    GPIO_setDirectionMode(19, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(19, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);

    // CAP1
    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);

    // CAP2
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);

    // CAP3
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);

    // GPIO29->DRV-EN for J5/J6 connection
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_writePin(29, 1);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_PULLUP);

    // No Connection
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);

    // LED1 on Launchxl-f280025C
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_writePin(31, 0);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);

    // CAN-TX
    GPIO_setPinConfig(GPIO_32_CANA_TX);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_PULLUP);

    // CAN-RX
    GPIO_setPinConfig(GPIO_33_CANA_RX);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);

    // GPIO34->nFAULT
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);

    // GPIO->EQEP1B
    GPIO_setPinConfig(GPIO_37_EQEP1_B);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_40_GPIO40);
    GPIO_setDirectionMode(40, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(40, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_41_GPIO41);
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);

    // GPIO->EQEP1I
    GPIO_setPinConfig(GPIO_43_EQEP1_INDEX);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);

    // GPIO->EQEP1A
    GPIO_setPinConfig(GPIO_44_EQEP1_A);
    GPIO_setDirectionMode(44, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(44, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_45_GPIO45);
    GPIO_setDirectionMode(45, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(45, GPIO_PIN_TYPE_STD);

    // No Connection
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);

    return;
}  // end of HAL_setupGPIOs() function


void HAL_setupPeripheralClks(HAL_Handle handle)
{
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

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_I2CA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);

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
    HAL_Obj       *obj = (HAL_Obj *)handle;

    uint16_t  cnt;

    uint16_t halfPeriod_cycles = (uint16_t)(systemFreq_MHz *
                                  pwmPeriod_usec / (float32_t)2.0);

    // disable the ePWM module time base clock sync signal
    // to synchronize all of the PWMs
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    // turns off the outputs of the EPWM peripherals which will put the power
    // switches into a high impedance state.
    EPWM_forceTripZoneEvent(obj->pwmHandle[0], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[1], EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(obj->pwmHandle[2], EPWM_TZ_FORCE_EVENT_OST);

    uint16_t       pwmDBRED = HAL_PWM_DBRED_CNT;
    uint16_t       pwmDBFED = HAL_PWM_DBFED_CNT;

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
        EPWM_setRisingEdgeDelayCount(obj->pwmHandle[cnt], pwmDBRED);

        // setup the Dead-Band Falling Edge Delay Register (DBFED)
        EPWM_setFallingEdgeDelayCount(obj->pwmHandle[cnt], pwmDBFED);

        // setup the PWM-Chopper Control Register (PCCTL)
        EPWM_disableChopper(obj->pwmHandle[cnt]);

        // setup the Trip Zone Select Register (TZSEL)
        EPWM_disableTripZoneSignals(obj->pwmHandle[cnt], HAL_TZSEL_SIGNALS_ALL);
    }

    // setup the Event Trigger Selection Register (ETSEL)
    EPWM_setInterruptSource(obj->pwmHandle[0], EPWM_INT_TBCTR_PERIOD);

    EPWM_disableInterrupt(obj->pwmHandle[0]);

    EPWM_setADCTriggerSource(obj->pwmHandle[0],
                             EPWM_SOC_A, EPWM_SOC_TBCTR_D_CMPC);

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
        EPWM_setADCTriggerEventPrescale(obj->pwmHandle[0], EPWM_SOC_A,
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
    EPWM_setCounterCompareValue(obj->pwmHandle[0], EPWM_COUNTER_COMPARE_C, 3);

    // BSXL8323RH_REVB || BSXL8323RS_REVA || BSXL8353RS_REVA

    // enable the ePWM module time base clock sync signal
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    return;
}  // end of HAL_setupPWMs() function

#ifdef PWMDAC_ENABLE
void HAL_setupPWMDACs(HAL_Handle handle,
                   const float32_t systemFreq_MHz)
{
  HAL_Obj       *obj = (HAL_Obj *)handle;
  uint16_t       halfPeriod_cycles = (uint16_t)(systemFreq_MHz*(float32_t)(1000.0/100.0/2.0));    //100kHz
  uint_least8_t  cnt;

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

      // since the PWM is configured as an up/down counter, the period register is
      // set to one-half of the desired PWM period
      EPWM_setTimeBasePeriod(obj->pwmDACHandle[cnt], halfPeriod_cycles);
    }

  // enable the ePWM module time base clock sync signal
  SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

  return;
}  // end of HAL_setupPWMs() function
#endif

#ifdef EQEP_ENABLE
void HAL_setupQEP(HAL_Handle handle, HAL_QEPSelect_e qep)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  //
  // Configure the decoder for quadrature count mode
  //
  EQEP_setDecoderConfig(obj->qepHandle[qep], (EQEP_CONFIG_2X_RESOLUTION |
                                     EQEP_CONFIG_QUADRATURE |
                                     EQEP_CONFIG_NO_SWAP));

  EQEP_setEmulationMode(obj->qepHandle[qep], EQEP_EMULATIONMODE_RUNFREE);

  //
  // Configure the position counter to be latched on a unit time out
  //

  EQEP_setLatchMode(obj->qepHandle[qep], (EQEP_LATCH_RISING_INDEX|EQEP_LATCH_UNIT_TIME_OUT));


  //
  // Configure the position counter to reset on an index event
  //

  EQEP_setPositionCounterConfig(obj->qepHandle[qep], EQEP_POSITION_RESET_IDX,
                                4*USER_MOTOR_NUM_ENC_SLOTS - 1);

  // setup strobe
  //EQEP_setStrobeSource(EQEP_STROBE_OR_ADCSOCA,EQEP_STROBE_OR_ADCSOCA);

   //
  // Enable the unit timer, setting the frequency to 100 Hz
  //

  EQEP_enableUnitTimer(obj->qepHandle[qep], (DEVICE_SYSCLK_FREQ / 100));

  // Disables the eQEP module position-compare unit
  EQEP_disableCompare(obj->qepHandle[qep]);


  //
  // Configure and enable the edge-capture unit. The capture clock divider is
  // SYSCLKOUT/64. The unit-position event divider is QCLK/32.
  //

  EQEP_setCaptureConfig(obj->qepHandle[qep], EQEP_CAPTURE_CLK_DIV_128,
                        EQEP_UNIT_POS_EVNT_DIV_32);

  EQEP_enableCapture(obj->qepHandle[qep]);

  // Enable UTO on QEP
  EQEP_enableInterrupt(obj->qepHandle[qep], EQEP_INT_UNIT_TIME_OUT);

  //
  // Enable the eQEP module
  //

  EQEP_enableModule(obj->qepHandle[qep]);

  return;
}
#endif

#if defined(SCI_ENABLE)
void HAL_setupSCIA(HAL_Handle halHandle)
{
  HAL_Obj *obj = (HAL_Obj *)halHandle;

  // Initialize SCIA and its FIFO.
  SCI_performSoftwareReset(obj->sciHandle);

  // Configure SCIA for echoback.
  SCI_setConfig(obj->sciHandle, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8 |
                                                      SCI_CONFIG_STOP_ONE |
                                                      SCI_CONFIG_PAR_NONE));
  SCI_resetChannels(obj->sciHandle);

  SCI_resetRxFIFO(obj->sciHandle);

  SCI_resetTxFIFO(obj->sciHandle);

  SCI_clearInterruptStatus(obj->sciHandle, SCI_INT_TXFF | SCI_INT_RXFF);

  SCI_enableFIFO(obj->sciHandle);

  SCI_enableModule(obj->sciHandle);

  SCI_performSoftwareReset(obj->sciHandle);

}  // end of HAL_setupSCIA() function
#endif  // SCI_ENABLE

#if defined(CMD_CAN_ENABLE)
void HAL_setupCANA(HAL_Handle halHandle)
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    // Initialize the CAN controller
    CAN_initModule(obj->canHandle);

    // Set up the CAN bus bit rate to 200kHz
    // Refer to the Driver Library User Guide for information on how to set
    // tighter timing control. Additionally, consult the device data sheet
    // for more information about the CAN module clocking.
    CAN_setBitRate(obj->canHandle, DEVICE_SYSCLK_FREQ, 500000, 16);

    // Initialize the transmit message object used for sending CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 1
    //      Message Identifier: 0x1
    //      Message Frame: Standard
    //      Message Type: Transmit
    //      Message ID Mask: 0x0
    //      Message Object Flags: Transmit Interrupt
    //      Message Data Length: 8 Bytes
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE,
                           MSG_DATA_LENGTH);

    // Initialize the receive message object used for receiving CAN messages.
    // Message Object Parameters:
    //      Message Object ID Number: 2
    //      Message Identifier: 0x1
    //      Message Frame: Standard
    //      Message Type: Receive
    //      Message ID Mask: 0x0
    //      Message Object Flags: Receive Interrupt
    //      Message Data Length: 8 Bytes
    CAN_setupMessageObject(obj->canHandle, RX_MSG_OBJ_ID, 0x1, CAN_MSG_FRAME_STD,
                           CAN_MSG_OBJ_TYPE_RX, 0, CAN_MSG_OBJ_RX_INT_ENABLE,
                           MSG_DATA_LENGTH);

    // Start CAN module operations
    CAN_startModule(obj->canHandle);

    return;
}
#endif  // CMD_CAN_ENABLE

#ifdef SPI_ENABLE
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
#endif

void HAL_setupTimers(HAL_Handle handle, const float32_t systemFreq_MHz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  //1ms
  uint32_t timerPeriod_1ms = (uint32_t)(systemFreq_MHz *
                              (float32_t)1000.0) - 1;

  // use timer 0 for CPU usage diagnostics
  CPUTimer_setPreScaler(obj->timerHandle[0], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[0],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[0], timerPeriod_1ms);

  //10ms
  uint32_t timerPeriod_10ms = (uint32_t)(systemFreq_MHz *
                              (float32_t)10000.0) - 1;

  // use timer 1 for CPU usage diagnostics
  CPUTimer_setPreScaler(obj->timerHandle[1], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[1],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[1], timerPeriod_10ms);

  // use timer 2 for CPU usage diagnostics
  CPUTimer_setPreScaler(obj->timerHandle[2], 0);
  CPUTimer_setEmulationMode(obj->timerHandle[2],
                            CPUTIMER_EMULATIONMODE_RUNFREE);
  CPUTimer_setPeriod(obj->timerHandle[2], 0xFFFFFFFF);

  return;
}  // end of HAL_setupTimers() function


//------------------------------------------------------------------
bool HAL_getTimerStatus(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
   HAL_Obj   *obj = (HAL_Obj *)halHandle;
   bool cpuTimerStatus;

   cpuTimerStatus = CPUTimer_getTimerOverflowStatus(obj->timerHandle[cpuTimerNumber]);
   return cpuTimerStatus;
}   // end of HAL_getTimerStatus() function


void HAL_clearTimerFlag(HAL_Handle halHandle, const uint16_t cpuTimerNumber)
{
   HAL_Obj   *obj = (HAL_Obj *)halHandle;

   CPUTimer_clearOverflowFlag(obj->timerHandle[cpuTimerNumber]);
}   // end of HAL_clearTimerFlag() function


#ifdef PWMDAC_ENABLE
void HAL_setPwmDacParameters(HAL_Handle handle, HAL_PWMDACData_t *pPWMDACData)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    pPWMDACData->periodMax = PWMDAC_getPeriod(obj->pwmDACHandle[PWMDAC_NUMBER_1]);

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
#endif

#ifdef DATALOG_ENABLE
void HAL_setupDlogWithDMA(HAL_Handle handle, const uint16_t DMAChannel, const void *dlogDestAddr, const void *dlogSrcAddr)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    const void *destAddr;
    const void *srcAddr;
    destAddr = (const void *)dlogDestAddr;
    srcAddr  = (const void *)dlogSrcAddr;

    //
    // configure DMA Channel
    //
    DMA_configAddresses(obj->dmaChHandle[DMAChannel], destAddr, srcAddr);
    DMA_configBurst(obj->dmaChHandle[DMAChannel],DLOG_BURST,2,2);
    DMA_configTransfer(obj->dmaChHandle[DMAChannel],DLOG_TRANSFER,1,1);
    DMA_configMode(obj->dmaChHandle[DMAChannel],DMA_TRIGGER_SOFTWARE,
                   DMA_CFG_ONESHOT_ENABLE+DMA_CFG_CONTINUOUS_ENABLE+DMA_CFG_SIZE_32BIT);
    DMA_setInterruptMode(obj->dmaChHandle[DMAChannel],DMA_INT_AT_END);
    DMA_enableTrigger(obj->dmaChHandle[DMAChannel]);
    DMA_disableInterrupt(obj->dmaChHandle[DMAChannel]);

    return;
}    //end of HAL_initDlogDMA() function
#endif  //  DATALOG_ENABLE

#ifdef _FLASH
void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory)
{
    uint16_t *pMemoryStart;
    uint16_t LoopCount, LoopLength;

    pMemoryStart = pMemory;
    LoopLength = lengthMemory;

    for(LoopCount=0; LoopCount<LoopLength; LoopCount++)
        *(pMemoryStart+LoopCount) = 0x0000;
}   //end of HAL_clearDataRAM() function
#endif

// end of file
