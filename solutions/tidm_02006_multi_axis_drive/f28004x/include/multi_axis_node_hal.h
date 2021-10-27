//#############################################################################
//
// FILE:    multi_axis_node_hal.h
//
// TITLE:   header file of device handle configuration
//
// Group:   C2000
//
// Target Family: F28002x
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

#ifndef MULTI_AXIS_NODE_HAL_H
#define MULTI_AXIS_NODE_HAL_H

//
// Include project specific include files.
//
#include <math.h>
#include <stdlib.h>
#include <limits.h>

#include "device.h"

#include "motor_drive_user.h"

// interrupt variables
extern volatile uint16_t tempPIEIER;

//
// Define .bss memory
//
#ifdef _FLASH
#define RAMMS_START_ADDRESS     0x00000400
#define RAMMS_SIZE_LENGTH       0x00000000

#define RAMLS_START_ADDRESS     0x00009000      // RAMLS2 & RAMLS3
#define RAMLS_SIZE_LENGTH       0x00000FF8

#define RAMGS_START_ADDRESS     0x00012000      // RAMGS3
#define RAMGS_SIZE_LENGTH       0x00001FF8
#else
#define RAMMS_START_ADDRESS     0x00000400
#define RAMMS_SIZE_LENGTH       0x00000000

#define RAMLS_START_ADDRESS     0x00009000      // RAMLS3
#define RAMLS_SIZE_LENGTH       0x00000FF8

#define RAMGS_START_ADDRESS     0x00012000      // RAMGS3
#define RAMGS_SIZE_LENGTH       0x00001FF8
#endif

//
// Define the system frequency (MHz)
//
#define SYSTEM_FREQUENCY    (DEVICE_SYSCLK_FREQ / 1000000U)

//
// Timer definitions based on System Clock
//
#define     MICROSEC         SYSTEM_FREQUENCY
#define     MICROSEC_50       50 * MICROSEC     // 50 uS
#define     MICROSEC_100     100 * MICROSEC     // 0.1 mS
#define     MICROSEC_150     150 * MICROSEC     // 0.15 mS
#define     MILLISEC        1000 * MICROSEC     // 1 mS

#define     MILSEC_0_5       0.5 * MILLISEC     // 0.5 mS
#define     MILSEC_1         1.0 * MILLISEC     // 1.0 mS
#define     MILSEC_2         2.0 * MILLISEC     // 2.0 mS
#define     MILSEC_5         5.0 * MILLISEC     // 5.0 mS
#define     MILSEC_7_5       7.5 * MILLISEC     // 7.5 mS
#define     MILSEC_10         10 * MILLISEC     // 10 mS
#define     MILSEC_20         20 * MILLISEC     // 20 mS
#define     MILSEC_50         50 * MILLISEC     // 50 mS
#define     MILSEC_100       100 * MILLISEC     // 100 mS
#define     MILSEC_500       500 * MILLISEC     // 500 mS
#define     MILSEC_1000     1000 * MILLISEC     // 1000 mS


#ifdef _LAUNCHXL_F280049C
#define  CCARD_GPIO_LED1        23        // NC: Set up based board
#define  CCARD_GPIO_LED2        34        // NC: Set up based board

#define  CCARD_LED1_WAIT_TIME   4000
#define  CCARD_LED2_WAIT_TIME   2000

//
// define deadband delay cout for rising edge
//
#define EPWM_DB_DELAY_RISE      60     // 60*10ns=0.6us, EPWMCLK=100MHz

//
// define deadband delay cout for falling edge
//
#define EPWM_DB_DELAY_FALL      60     // 60*10ns=0.6us, EPWMCLK=100MHz


#else // _TMDSCNCD_F280049C


#define  CCARD_GPIO_LED1        31        // NC: Set up based board
#define  CCARD_GPIO_LED2        34        // NC: Set up based board

#define  CCARD_LED1_WAIT_TIME   2000
#define  CCARD_LED2_WAIT_TIME   1000

//
// define deadband delay cout for rising edge
//
#define EPWM_DB_DELAY_RISE      200     // 200*10ns=2us, EPWMCLK=100MHz

//
// define deadband delay cout for falling edge
//
#define EPWM_DB_DELAY_FALL      200     // 200*10ns=2us, EPWMCLK=100MHz


#endif // _LAUNCHXL_F280049C/

//! \brief    Defines the hardware abstraction layer (HAL) data
//! \details  The HAL object contains all handles to peripherals. When accessing
//!           a peripheral on a processor, use a HAL function along
//!           with the HAL handle for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
    uint32_t  claHandle;          //!< the CLA handle

    uint32_t  clbHandle;          //!< the CLB handle

    uint32_t  timerHandle[3];     //<! the timer handles

    uint32_t  adcHandle[3];       //!< the ADC handles
    uint32_t  adcResult[3];       //!< the ADC Result handles

    uint32_t  dacHandle[2];       //!< the DAC handle

    uint32_t  pwmHandle[6];       //<! the PWM handles
    uint32_t  cmpssHandle[4];     //!< the CMPSS handle
    uint32_t  pgaHandle[3];       //!< the PGA handle

    uint32_t  spiHandle;          //!< the SPI handle
    uint32_t  sciHandle;          //!< the SCI handle

    uint32_t  fsiTxHandle;        //!< the FSI TX handle
    uint32_t  fsiRxHandle;        //!< the FSI RX handle

    uint32_t  dmaHandle;          //!< the DMA handle
    uint32_t  dmaChHandle[4];     //!< the DMA Channel handle

    uint32_t  qepHandle;          //!< the QEP handle

    bool      flagEnablePWM;      //<! the pwm enable flag
} HAL_Obj;

//! \brief    Defines the HAL handle
//! \details  The HAL handle is a pointer to a HAL object.  In all HAL
//!           functions the HAL handle is passed so that the function knows what
//!           peripherals are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;

extern HAL_Handle    halHandle;
extern HAL_Obj       hal;

//
// the function prototypes
//

//! \brief     Acknowledges an interrupt so that another INT can happen again.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_ackInt(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[0]);

    //
    // clear ADCINT1 INT and ack PIE INT
    //
    ADC_clearInterruptStatus(M_IW_ADC_BASE, ADC_INT_NUMBER1);

    //
    // ACK the PWM interrupts
    //
//    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3 |
//                            INTERRUPT_ACK_GROUP7 |
//                            INTERRUPT_ACK_GROUP11);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
    return;
}

//! \brief     Clear TZFLAG of EPWM modules.
//! \param[in] handle     The hardware abstraction layer (HAL) handle
static inline void HAL_clearTZFlag(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    //
    // clear OST & DCAEVT1 flags
    //
    EPWM_clearTripZoneFlag(obj->pwmHandle[0],
                           (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

    EPWM_clearTripZoneFlag(obj->pwmHandle[1],
                           (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

    EPWM_clearTripZoneFlag(obj->pwmHandle[2],
                           (EPWM_TZ_FLAG_OST | EPWM_TZ_FLAG_DCAEVT1));

    //
    // clear HLATCH - (not in TRIP gen path)
    //
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchHigh(obj->cmpssHandle[3]);

    //
    // clear LLATCH - (not in TRIP gen path)
    //
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[0]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[1]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[2]);
    CMPSS_clearFilterLatchLow(obj->cmpssHandle[3]);

    return;
}

//! \brief     Enables the interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_enableInterrupts(HAL_Handle handle);


//! \brief      Initializes the hardware abstraction layer (HAL) object
//! \details    Initializes all handles to the microcontroller peripherals.
//!             Returns a handle to the HAL object.
//! \param[in]  pMemory   A pointer to the memory for the HAL object
//! \param[in]  numBytes  The number of bytes allocated for the HAL object
//! \return     The hardware abstraction layer (HAL) object handle
extern HAL_Handle HAL_init(void *pMemory, const size_t numBytes);

//! \brief     Sets up the CPU timers
//! \param[in] base         The cpu timer base
//! \param[in] periodCount  The cpu timer period count
extern void HAL_setupCpuTimer(uint32_t base, uint32_t periodCount);

//! \brief     Sets up the DAC
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupDACs(HAL_Handle handle);

//! \brief     Sets up the FSI
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSI(HAL_Handle handle);

//! \brief     Sets up the FSI interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSIInterrupts(HAL_Handle handle);

//! \brief     Sets up the DMA for FSIRX
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSIRXDMA(HAL_Handle handle);

//! \brief     Sets up the DMA for FSITX
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSITXDMA(HAL_Handle handle);

//! \brief     Sets up the GPIO (General Purpose I/O) pins
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupGPIOs(HAL_Handle handle);

//! \brief     Sets up the interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupInterrupts(HAL_Handle handle);

//! \brief     Sets up the PWMs (Pulse Width Modulators) for motor
//! \param[in] handle  The hardware abstraction layer (HAL) handle for motor
extern void HAL_setupMotorPWMs(HAL_Handle handle);

//! \brief     Sets up the ADC
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupADCs(HAL_Handle handle);

//! \brief     Sets up the CLA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCLA(HAL_Handle handle);

//! \brief     Sets up the CLB
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCLB(HAL_Handle handle);

//! \brief     Sets up the CMPSS
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCMPSS(HAL_Handle handle);

//! \brief      Sets up the PGAs (Programmable Gain Amplifiers)
//! \param[in]  handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupPGAs(HAL_Handle handle);

//! \brief     Sets up the CMPSS
//! \param[in] handle  The hardware abstraction layer (HAL) handle for motor
//! \param[in] curHi  DAC value for CMPSS High threshold
//! \param[in] curLo  DAC value for CMPSS Low threshold
extern void HAL_setupCMPSS_DACValue(HAL_Handle handle,
                                    const uint16_t curHi, const uint16_t curLo);

//! \brief     Sets up the DAC
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupDACs(HAL_Handle handle);

//! \brief     Sets up the GATE object
//! \param[in] handle       The hardware abstraction layer (HAL) handle
extern void HAL_setupGate(HAL_Handle handle);

//! \brief     Sets up the QEP peripheral
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupQEP(HAL_Handle handle);

//! \brief     Sets up fault protection for motor
//! \param[in] handle  The hardware abstraction layer (HAL) handle for motor
//! \param[in] currentLimit  over current limitation value
extern void  HAL_setupMotorFaultProtection(HAL_Handle handle,
                                           const float32_t currentLimit);


//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//
// software prioritized interrupts
//
static inline void setInterruptPriority(void)
{
#ifdef FSI_DMA_ENABLE
    //
    // Set interrupt priority:
    //
    tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER7);

    // set the global and group priority to allow CPU interrupts
    // with higher priority
    IER = INTERRUPT_CPU_INT7;

    HWREGH(PIECTRL_BASE + PIE_O_IER7) = PIE_IER7_INTX13;

    // enable Interrupts
    Interrupt_clearACKGroup(0xFFFFU);

    __asm("  NOP");
    EINT;
#else
    //
    // Set interrupt priority:
    //
    tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER7);

    // set the global and group priority to allow CPU interrupts
    // with higher priority
    IER = INTERRUPT_CPU_INT7;

    HWREGH(PIECTRL_BASE + PIE_O_IER7) = PIE_IER7_INTX11 | PIE_IER7_INTX13;

    // enable Interrupts
    Interrupt_clearACKGroup(0xFFFFU);

    __asm("  NOP");
    EINT;
#endif  // FSI_DMA_ENABLE
}

static inline void restoreInterruptRegisters(void)
{
    //
    // Restore registers saved
    //
    DINT;

    __asm("  NOP");

    HWREGH(PIECTRL_BASE + PIE_O_IER7) = tempPIEIER;
}

//! \brief      clear the memory
//! \param[in]  pMemory   A pointer to the start address memory
//! \param[in]  lengthMemory  the length of memory
extern void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);


#endif  // end of MULTI_AXIS_NODE_HAL_H definition
