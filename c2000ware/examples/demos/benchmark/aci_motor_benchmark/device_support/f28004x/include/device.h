//#############################################################################
//
// FILE:   device.h
//
// TITLE:  Device Specific Implementation
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

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "f28004x_device.h"
#include "driverlib.h"
#include "stdint.h"

//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************
//
// 20MHz XTAL on controlCARD. For use with SysCtl_getClock().
//
#define DEVICE_OSCSRC_FREQ          20000000U
//
// 100MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 10 * 1) / 2)

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 20MHz (XTAL_OSC) * 10 (IMULT) * 1 (FMULT) / 2 (PLLCLK_BY_2)
//
#define DEVICE_SETCLOCK_CFG         (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(10) |  \
                                     SYSCTL_FMULT_NONE | SYSCTL_SYSDIV(2) |   \
                                     SYSCTL_PLL_ENABLE)


//*****************************************************************************
//
// Macro to call SysCtl_delay() to achieve a delay in microseconds. The macro
// will convert the desired delay in microseconds to the count value expected
// by the function. \b x is the number of microseconds to delay.
//
//*****************************************************************************
#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)



//*****************************************************************************
//
// Adjust the modeled current so that it can be output by DAC and read from
// ADC correctly as the modeled current is between 1.0 and -1.0
//
//*****************************************************************************
#define Ia_ADJUST_FACTOR 1000
#define Ia_ADJUST_BASELINE 2700

#define ADJUST_Ia_OUTPUT(x) ( ((x) * Ia_ADJUST_FACTOR) + Ia_ADJUST_BASELINE )
#define ADJUST_Ia_INPUT(x)  ( (x - Ia_ADJUST_BASELINE) / Ia_ADJUST_FACTOR )

#define Ib_ADJUST_FACTOR 500
#define Ib_ADJUST_BASELINE 1500

#define ADJUST_Ib_OUTPUT(x) ( ((x) * Ib_ADJUST_FACTOR) + Ib_ADJUST_BASELINE )
#define ADJUST_Ib_INPUT(x)  ( (x - Ib_ADJUST_BASELINE) / Ib_ADJUST_FACTOR )

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
extern void Device_setup(void);

//
// Bitfield datastructure definition
//
extern struct ADC_RESULT_REGS AdcaResult;

//*****************************************************************************
//
// ADC functions
//
//*****************************************************************************

//
// ADC read functions
//
inline uint16_t ADC_getIa()
{
    return AdcaResult.ADCRESULT0;
}


inline uint16_t ADC_getIb()
{
    return AdcaResult.ADCRESULT1;
}

//
// Interrupt handling functions
//
inline void ADCInt_ack()
{
    //
    // Enable more interrupts from this timer
    //
    *(uint16_t *)(ADCA_BASE+ADC_O_INTFLG) |= ADC_INTFLG_ADCINT1;

    //
    // Acknowledge interrupt to recieve more interrupts from PIE group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

inline void ADCInt_enable()
{
    //
    // Enable ADC interrupt
    //
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_enable(INT_ADCA1);

    //
    // Enable the clock to synchronously enable all the ePWMs
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

inline void ADCInt_disable()
{
    //
    // Disable PWMs by disabling clocks
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Disable ADC interrupt
    //
    Interrupt_disable(INT_ADCA1);
    ADC_disableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
//
// PWM functions
//
//*****************************************************************************
inline void PWM_setUa(uint16_t Ua)
{
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, Ua);
}

inline void PWM_setUb(uint16_t Ub)
{
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, Ub);
}

inline void PWM_setUc(uint16_t Uc)
{
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, Uc);
}

//*****************************************************************************
//
// DAC functions
//
//*****************************************************************************
inline void DAC_setIa(uint16_t value)
{
    DAC_setShadowValue(DACA_BASE, value);
}

inline void DAC_setIb(uint16_t value)
{
    DAC_setShadowValue(DACB_BASE, value);
}

//*****************************************************************************
//
// Defines, Globals, and Header Includes related to Flash Support
//
//*****************************************************************************
#ifdef _FLASH
#include <stddef.h>

extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;

#endif

#define DEVICE_FLASH_WAITSTATES 4

#endif //__DEVICE_H__
