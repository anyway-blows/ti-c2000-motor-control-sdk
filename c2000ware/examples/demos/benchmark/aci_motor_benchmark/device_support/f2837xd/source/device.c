//#############################################################################
//
// FILE:   device.c
//
// TITLE:  Device Specific Initialization
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
// Include Files
//

#include <parameter.h>
#include <device.h>
#include "string.h"
#include "stdio.h"

//
// Bitfield definition for ADCA results reg
//
#ifdef __cplusplus
#pragma DATA_SECTION("AdcaResultRegsFile")
#else
#pragma DATA_SECTION(AdcaResult,"AdcaResultRegsFile");
#endif
struct ADC_RESULT_REGS AdcaResult;

//
// ADC constants
//
#define EPWMCLK (DEVICE_SYSCLK_FREQ/2)
#define ADC_SAMPLE_PERIOD (EPWMCLK * SAMPLING_PERIOD)
#define ADC_ACQWIN 15

//
// PWM constants
//
uint32_t PWM_PERIOD_MAX = ADC_SAMPLE_PERIOD/2;
uint32_t PWM_HALF_MAX = ADC_SAMPLE_PERIOD/4;

//
// Function declarations
//
void Device_init(void);
void Device_initGPIO(void);
void Device_enableAllPeripherals(void);
void Device_configureTMXAnalogTrim(void);
void Device_enableUnbondedGPIOPullups(void);
void Device_enableUnbondedGPIOPullups(void);

//*****************************************************************************
//
// Function to initialize the device. Primarily initializes system control to a
// known state by disabling the watchdog, setting up the SYSCLKOUT frequency,
// and enabling the clocks to the peripherals.
// The function also configures the GPIO pins 22 and 23 in digital mode.
// To configure these pins as analog pins, use the function GPIO_setAnalogMode
//
//*****************************************************************************
void Device_init(void)
{
    //
    // Disable the watchdog
    //
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

    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    //
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif
    #ifdef CPU1

        //
        // Configure Analog Trim in case of untrimmed or TMX sample
        //
        if((SysCtl_getDeviceParametric(SYSCTL_DEVICE_QUAL) == 0x0U)       &&
           (HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMA) == 0x0U))
        {
            Device_configureTMXAnalogTrim();
        }

        //
    // Set up PLL control and clock dividers
    //
    SysCtl_setClock(DEVICE_SETCLOCK_CFG);

    //
    // Make sure the LSPCLK divider is set to the default (divide by 4)
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_4);

    //
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
//    SysCtl_deviceCal();
#endif

    #endif
    //
    // Turn on all peripherals
    //
    Device_enableAllPeripherals();

}

//*****************************************************************************
//
// Function to turn on all peripherals, enabling reads and writes to the
// peripherals' registers.
//
// Note that to reduce power, unused peripherals should be disabled.
//
//*****************************************************************************
void Device_enableAllPeripherals(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    #ifdef CPU1
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRPWM);
    #endif
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    #ifdef CPU1
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF2);
    #endif

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM9);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM10);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM11);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM12);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP3);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD2);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCID);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANB);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCBSPA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCBSPB);

    #ifdef CPU1
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_USBA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UPPA);
    #endif

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCD);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS8);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACC);
}

//*****************************************************************************
//
// Function to disable pin locks and enable pullups on GPIOs.
//
//*****************************************************************************
void Device_initGPIO(void)
{
    //
    // Disable pin locks.
    //
    GPIO_unlockPortConfig(GPIO_PORT_A, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_B, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_C, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_D, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_E, 0xFFFFFFFF);
    GPIO_unlockPortConfig(GPIO_PORT_F, 0xFFFFFFFF);

    //
    // Enable GPIO Pullups
    //
    Device_enableUnbondedGPIOPullups();
}

void Device_enableUnbondedGPIOPullupsFor176Pin(void)
{
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPCPUD) = ~0x80000000U;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPDPUD) = ~0xFFFFFFF7U;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPEPUD) = ~0xFFFFFFDFU;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPFPUD) = ~0x000001FFU;
    EDIS;
}

//*****************************************************************************
//
// Function to enable pullups for the unbonded GPIOs on the 100PZ package:
// GPIOs     Grp Bits
// 0-1       A   1:0
// 5-9       A   9:5
// 22-40     A   31:22
//           B   8:0
// 44-57     B   25:12
// 67-68     C   4:3
// 74-77     C   13:10
// 79-83     C   19:15
// 93-168    C   31:29
//           D   31:0
//           E   31:0
//           F   8:0
//
//*****************************************************************************
void Device_enableUnbondedGPIOPullupsFor100Pin(void)
{
    EALLOW;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPAPUD) = ~0xFFC003E3U;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPBPUD) = ~0x03FFF1FFU;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPCPUD) = ~0xE10FBC18U;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPDPUD) = ~0xFFFFFFF7U;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPEPUD) = ~0xFFFFFFFFU;
    HWREG(GPIOCTRL_BASE + GPIO_O_GPFPUD) = ~0x000001FFU;
    EDIS;
}

void Device_enableUnbondedGPIOPullups(void)
{
    //
    // bits 8-10 have pin count
    //
    uint16_t pinCount = ((HWREG(DEVCFG_BASE + SYSCTL_O_PARTIDL) &
                          (uint32_t)SYSCTL_PARTIDL_PIN_COUNT_M) >>
                         SYSCTL_PARTIDL_PIN_COUNT_S);

    /*
     * 5 = 100 pin
     * 6 = 176 pin
     * 7 = 337 pin
     */
    if(pinCount == 5)
    {
        Device_enableUnbondedGPIOPullupsFor100Pin();
    }
    else if(pinCount == 6)
    {
        Device_enableUnbondedGPIOPullupsFor176Pin();
    }
    else
    {
        //
        // Do nothing - this is 337 pin package
        //
    }
}



void initADC()
{
    //
    // Configure the ADC-A base registers
    //
    ADC_disableConverter(ADCA_BASE);                                    // Power down ADC for configuration
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);                       // ADC clock prescaler = CPUCLK/4
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    //
    // Configure INT pulse generation at end of acquisition (early interrupt)
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_ACQ_WIN);

    //
    // SOC configuration - Trigger using ePWM1-ADCSOCA
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, ADC_ACQWIN);
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, ADC_ACQWIN);

    //
    // No ADC interrupt triggers SOC0 (TRIGSEL determined by SOC and not ADCINT1 or ADCINT2)
    //
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    //
    // All SOCs handled in round-robin mode
    //
    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);

    //
    // ADCA1 interrupt configuration
    //
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);                       // Interrupt pulses regardless of flag state
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);        // SOC0 triggers the interrupt

    //
    // Enable ADCA1 interrupt and specify interrupt handler (ISR)
    //
    Interrupt_register(INT_ADCA1, aci_ctrlLoop);

    //
    // Power up ADC
    //
    ADC_enableConverter(ADCA_BASE);

    //
    // Wait 1 ms after power-up before using the ADC
    //
    DEVICE_DELAY_US(1000);

}

//*****************************************************************************
//
// Helper function to configure PWM
//
//*****************************************************************************
void initPWMx(uint32_t base)
{

    //
    // Configure PWM in up down mode
    //
    EPWM_setSyncOutPulseMode(base,EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
    EPWM_enablePhaseShiftLoad(base);
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);                         // Ignore emulation suspend
    EPWM_setPeriodLoadMode(base, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);                 // Enable the timer in count up down mode
    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);   // TBCLK = EPWMCLK
    EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    //
    // Configure timer poeriod
    //
    EPWM_setTimeBasePeriod(base, PWM_PERIOD_MAX);
    EPWM_setPhaseShift(base, 0);

    //
    // Set counter to reload on 0
    //
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Setup action qualifier
    //
    EPWM_setActionQualifierActionComplete(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA | EPWM_AQ_OUTPUT_LOW_UP_CMPA);
}

//*****************************************************************************
//
// Initialize the PWM modules (1, 2, 3)
//
//*****************************************************************************
void initPWM(void)
{
    //
    // Must disable the clock to the ePWM modules to have all ePWM modules synchronized
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Reset all PWMs
    //
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM1);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM2);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM3);

    //
    // Initialize all PWMs
    //
    initPWMx(EPWM1_BASE);
    initPWMx(EPWM2_BASE);
    initPWMx(EPWM3_BASE);

    //
    // Enable PWM1 to trigger ADCA SOC
    //
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);                               // Enable ADC SOCA event
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);     // Set SOCA on PRD event
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);                  // Generate SOCA on first event
}

//*****************************************************************************
//
// Initialize the DAC modules (A, B)
//
//*****************************************************************************
void initDAC(void)
{

    //
    // Note: DAC is being configured to use the VREFHI internal voltage reference. For proper operation,
    // the reference modes for ADC-A and ADC-B must be configured. When using the internal reference of
    // 3.3V, the DAC gain of x2 should be set.
    //

    //ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    //ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //
    // Set VREFHI as as the DAC reference voltage
    //
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);

    //
    // Set X 2 gain
    //
    //DAC_setGainMode(DACB_BASE, DAC_GAIN_TWO);
    //DAC_setGainMode(DACA_BASE, DAC_GAIN_TWO);

    //
    // Set load mode as load on next SYSCLK
    //
    DAC_setLoadMode(DACB_BASE, DAC_LOAD_SYSCLK);
    DAC_setLoadMode(DACA_BASE, DAC_LOAD_SYSCLK);

    //
    // Enable DAC output
    //
    DAC_enableOutput(DACB_BASE);                    // Enable DAC output
    DAC_enableOutput(DACA_BASE);                    // Enable DAC output

    //
    // Initialize DAC output to 0
    //
    DAC_setShadowValue(DACB_BASE, ADJUST_Ia_OUTPUT(0));
    DAC_setShadowValue(DACA_BASE, ADJUST_Ib_OUTPUT(0));

    //
    // Required delay after enabling the DAC (delay for DAC to power up)
    //
    DEVICE_DELAY_US(10);
}


//*****************************************************************************
//
// This function configures the device and the relevant peripherals like
// ADC, PWM and DAC for this application.
//
//*****************************************************************************
void Device_setup()
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Read the system clock
    //
    uint32_t clk = SysCtl_getClock(DEVICE_OSCSRC_FREQ);

    //
    // Output the Device details to console
    //
    printf ("F2837x CPUCLK = %ld Hz ", clk);

#ifdef _FLASH
        printf ("from FLASH WS = %d, ", (unsigned)DEVICE_FLASH_WAITSTATES);
#else
        printf ("from RAM, ");
#endif

    #if USE_FAST_TRIG_LIB
        printf ("using FastRTS\n\n");
    #else
        printf ("using TMU\n\n");
    #endif

    //
    // Initialize peripherals
    //
    initADC();
    initDAC();
    initPWM();

    //
    // Enable global interrupts and real-time debug
    //
    EINT;
    ERTM;
}


#ifdef CPU1
//*****************************************************************************
//
// Function to implement Analog trim of TMX devices
//
//*****************************************************************************
void Device_configureTMXAnalogTrim(void)
{
    //
    // Enable ADC clock
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCD);

    //
    // Configure ADC reference trim for TMX devices
    //
    EALLOW;
    HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMA) = 0x7BDDU;
    HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMB) = 0x7BDDU;
    HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMC) = 0x7BDDU;
    HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_ANAREFTRIMD) = 0x7BDDU;

    //
    // Configure ADC offset trim. The user should generate the trim values
    // by following the instructions in the "ADC Zero Offset Calibration"
    // section in device TRM. The below lines needs to be uncommented and
    // updated with the correct trim values.
    //
//    HWREGH(ADCA_BASE + ADC_O_OFFTRIM) = 0x0U;
//    HWREGH(ADCB_BASE + ADC_O_OFFTRIM) = 0x0U;
//    HWREGH(ADCC_BASE + ADC_O_OFFTRIM) = 0x0U;
//    HWREGH(ADCD_BASE + ADC_O_OFFTRIM) = 0x0U;

    //
    // Configure internal oscillator trim. If the internal oscillator trim
    // contains all zeros, the user can adjust the lowest 10 bits of the
    // oscillator trim register between 1 (minimum) and 1023 (maximum)
    // while observing the system clock on the XCLOCKOUT pin. The below
    // lines needs to be uncommented and updated with the correct trim values.
    //
//    if(HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM) == 0x0U)
//    {
//        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC1TRIM) = 0x0U;
//    }
//    if( HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM) = 0x0U)
//    {
//        HWREGH(ANALOGSUBSYS_BASE + ASYSCTL_O_INTOSC2TRIM) = 0x0U;
//    }

    EDIS;

    //
    // Disable ADC clock
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_ADCD);
}
#endif

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
