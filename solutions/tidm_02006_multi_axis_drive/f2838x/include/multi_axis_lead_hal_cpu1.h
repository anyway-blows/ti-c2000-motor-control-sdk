//#############################################################################
//
// FILE:    multi_axis_lead_hal_cpu1.h
//
// TITLE:   header file of device handle configuration
//
// Group:   C2000
//
// Target Family: F2837x
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
//! \file  solutions/multi_axis_drive/f2838x/include/multi_axis_lead_hal_cpu1.h
//! \brief header file to be included in all labs
//!
//
#ifndef MULTI_AXIS_LEAD_HAL_CPU1_H
#define MULTI_AXIS_LEAD_HAL_CPU1_H

//
//! \defgroup MASTER_DRIVE
//! @{
//

//
// includes
//
#include <math.h>
#include <stdlib.h>
#include <limits.h>

#include "device.h"

#include "motor_ctrl_user.h"

// flag variables
extern volatile uint16_t tempPIEIER;

//
// Define .bss memory
//
#ifdef _FLASH
#define RAMMS_START_ADDRESS     0x00000400
#define RAMMS_SIZE_LENGTH       0x00000000

#define RAMLS_START_ADDRESS     0x00009000
#define RAMLS_SIZE_LENGTH       0x00000FF8

#define RAMGS_START_ADDRESS     0x0000F000
#define RAMGS_SIZE_LENGTH       0x00001FF8
#else
#define RAMMS_START_ADDRESS     0x00000400
#define RAMMS_SIZE_LENGTH       0x00000000

#define RAMLS_START_ADDRESS     0x00009000
#define RAMLS_SIZE_LENGTH       0x00000FF8

#define RAMGS_START_ADDRESS     0x0000F000
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


#define  CCARD_GPIO_LED1          34          // NC: Set up based board
#define  CCARD_GPIO_LED2          31          // NC: Set up based board

#define  CCARD_LED1_WAIT_TIME     4000
#define  CCARD_LED2_WAIT_TIME     1000

//
// define deadband delay cout for rising edge
//
#define EPWM_DB_DELAY_RISE      200         // 200*10ns=2.0us, EPWMCLK=100MHz

//
// define deadband delay cout for falling edge
//
#define EPWM_DB_DELAY_FALL      200         // 200*10ns=2.0us, EPWMCLK=100MHz



//! \brief    Defines the hardware abstraction layer (HAL) data
//! \details  The HAL object contains all handles to peripherals. When
//!           accessing a peripheral on a processor, use a HAL function along
//!           with the HAL handle for that processor to access its peripherals.
//!
typedef struct _HAL_Obj_
{
    uint32_t  claHandle;          //!< the CLA handle

    uint32_t  timerHandle[3];     //<! the timer handles

    uint32_t  sciHandle[2];       //!< the SCI handle

    uint32_t  dacHandle[3];       //!< the DAC handle

    uint32_t  pwmHandle[3];       //<! the PWM handles

    uint32_t  fsiTxHandle;        //!< the FSI TX handle
    uint32_t  fsiRxHandle;        //!< the FSI RX handle

    uint32_t  dmaHandle;          //!< the DMA handle
    uint32_t  dmaChHandle[4];     //!< the DMA Channel handle
} HAL_Obj;

//! \brief    Defines the HAL handle
//! \details  The HAL handle is a pointer to a HAL object.  In all HAL functions
//!           the HAL handle is passed so that the function knows what
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

    EPWM_clearEventTriggerInterruptFlag(obj->pwmHandle[M_CISR_NUM]);

    //
    // ACK PIE for CLA INT GROUP
    // FCL is not clearing the ACK bit for CLA group
    // because the example might have other CLA Tasks
    // ACK the PWM, ADC and CLA interrupts
    //
//    Interrupt_clearACKGroup( INTERRUPT_ACK_GROUP3 |
//                             INTERRUPT_ACK_GROUP4 |
//                             INTERRUPT_ACK_GROUP11 );

    Interrupt_clearACKGroup( INTERRUPT_ACK_GROUP3);

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

//! \brief     Sets up the FSI
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSI(HAL_Handle handle);

//! \brief     Sets up the FSI interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupFSIInterrupts(HAL_Handle handle);


//! \brief     Sets up the GPIO (General Purpose I/O) pins
//!
extern void HAL_setupGPIOs(HAL_Handle handle);

//! \brief     Sets up the interrupts
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupInterrupts(HAL_Handle handle);

//! \brief     Sets up the PWMs (Pulse Width Modulators)
//! \param[in] handle  The hardware abstraction layer (HAL) handle
//! \param[in] systemFreq_MHz  The system frequency, MHz
//! \param[in] pwmPeriod_usec  The PWM period, usec
extern void HAL_setupCtrlPWMs(HAL_Handle handle);

//! \brief     Sets up the CLA
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupCLA(HAL_Handle handle);

//! \brief     Sets up the DAC
//! \param[in] handle  The hardware abstraction layer (HAL) handle
extern void HAL_setupDACs(HAL_Handle handle);


//! \brief      Sets the hardware abstraction layer parameters
//! \details    Sets up the microcontroller peripherals.
//! \param[in]  handle    The hardware abstraction layer (HAL) handle
extern void HAL_setParams(HAL_Handle handle);

//! \brief     Sets up fault protection for motor
//! \param[in] handle  The hardware abstraction layer (HAL) handle for motor
extern void  HAL_setupMotorFaultXBAR(HAL_Handle handle);


//
// software prioritized interrupts
//
static inline void setInterruptPriority(void)
{
#ifdef FSI_DMA_ENABLE
    //
    // Set interrupt priority:
    //
    tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER4);

    // set the global and group priority to allow CPU interrupts
    // with higher priority
    IER = INTERRUPT_CPU_INT4;

    HWREGH(PIECTRL_BASE + PIE_O_IER4) = PIE_IER4_INTX13;

    // enable Interrupts
    Interrupt_clearACKGroup(0xFFFFU);

    __asm("  NOP");
    EINT;
#else
    //
    // Set interrupt priority:
    //
    tempPIEIER = HWREGH(PIECTRL_BASE + PIE_O_IER4);

    // set the global and group priority to allow CPU interrupts
    // with higher priority
    IER = INTERRUPT_CPU_INT4;

    HWREGH(PIECTRL_BASE + PIE_O_IER4) = PIE_IER4_INTX9 | PIE_IER4_INTX13;

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

    HWREGH(PIECTRL_BASE + PIE_O_IER4) = tempPIEIER;
}

//! \brief      clear the memory
//! \param[in]  pMemory   A pointer to the start address memory
//! \param[in]  lengthMemory  the length of memory
extern void HAL_clearDataRAM(void *pMemory, uint16_t lengthMemory);


//
// Close the Doxygen group.
//! @} //defgroup MASTER_CTRL
//

#endif  // end of MULTI_AXIS_LEAD_HAL_CPU1_H definition
