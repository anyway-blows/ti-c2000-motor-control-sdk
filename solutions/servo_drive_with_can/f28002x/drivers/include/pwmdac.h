//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
//     Copyright (C) 2020 Texas Instruments Incorporated -
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef _PWMDAC_H_
#define _PWMDAC_H_

//! \file   solutions/drv8312_c2_kit/f28004x/drivers/pwmdac.h
//! \brief  Contains public interface to various functions related
//!         to the pulse width modulation digital-to-analog 
//!         converter (PWMDAC) object
//!


// **************************************************************************
// the includes

#include "epwm.h"


//!
//!
//! \defgroup PWMDAC PWMDAC
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//!  \brief  Defines the pulse width modulation digital-to-analog (PWMDAC) handle
//!
#define   PWMDAC_Handle                             PWM_Handle

//!  \brief  Links the PWMDAC_setPeriod() function to the PWM_setPeriod() function
//!
#define   PWMDAC_getPeriod                          EPWM_getTimeBasePeriod


//!  \brief  Links the PWMDAC_init() function to the PWM_init() function
//!
#define   PWMDAC_init                               PWM_init

// **************************************************************************
// the typedefs

//! \brief Enumeration to define the pulse width modulation digital-to-analog (PWM) numbers
//!
typedef enum
{
  PWMDAC_NUMBER_1=0,
  PWMDAC_NUMBER_2,
  PWMDAC_NUMBER_3,
  PWMDAC_NUMBER_4,
  PWMDAC_NUMBER_5,
  PWMDAC_NUMBER_6,
  PWMDAC_NUMBER_7
} PWMDAC_Number_e;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif  // end of _PWMDAC_H_ definition

