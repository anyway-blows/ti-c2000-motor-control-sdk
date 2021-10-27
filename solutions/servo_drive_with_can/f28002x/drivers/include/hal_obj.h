//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:28 CDT 2021 $
// $Copyright:
//     Copyright (C) 2020 Texas Instruments Incorporated -
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

#ifndef _HAL_OBJ_H_
#define _HAL_OBJ_H_

//! \file  solutions/drv8312_c2_kit/f28004x/drivers/hal_obj.h
//! \brief Defines the structures for the HAL object 
//!

// drivers
#include "device.h"
#include "driverlib.h"
#include "pwmdac.h"

//#include "device.h"
//
//#include "asysctl.h"
//#include "adc.h"
//#include "cla.h"
//#include "cmpss.h"
//#include "cputimer.h"
//#include "dac.h"
//#include "dcsm.h"
//#include "dma.h"
//#include "epwm.h"
//#include "flash.h"
//#include "gpio.h"
//#include "interrupt.h"
//#include "memcfg.h"
//#include "pin_map.h"
//#include "pga.h"
//#include "sci.h"
//#include "pwmdac.h"
//#include "spi.h"
//#include "sysctl.h"
//#include "version.h"
//#include "xbar.h"

// modules
#include "hal_data.h"

#include "user.h"

#ifdef DRV8301_SPI
#include "drv8301.h"
#endif

#ifdef DRV8305_SPI
#include "drv8305.h"
#endif

#ifdef DRV8320_SPI
#include "drv8320.h"
#endif

#ifdef DRV8323_SPI
#include "drv8323.h"
#endif

// platforms


//!
//!
//! \defgroup HAL_OBJ HAL_OBJ
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

//! \brief      Defines the hardware abstraction layer (HAL) data
//! \details    The HAL object contains all handles to peripherals.  When accessing a
//!             peripheral on a processor, use a HAL function along with the HAL handle
//!             for that processor to access its peripherals.
//!

typedef struct _HAL_Obj_
{
  uint32_t      adcHandle[2];       //!< the ADC handles

  uint32_t      adcResult[2];       //!< the ADC results

  uint32_t      pwmHandle[3];       //<! the PWM handles

  uint32_t      timerHandle[3];     //<! the timer handles

  uint32_t      sciHandle;          //!< the SCI handle
  uint32_t      linHandle;          //!< the LIN handle

  uint32_t      canHandle;       //!< the CAN handle
  uint32_t      spiHandle[2];       //!< the SPI handle

  uint32_t      cmpssHandle[3];     //!< the CMPSS handle

#ifdef DATALOG_ENABLE
  uint32_t      dmaHandle;          //!< the DMA handle
  uint32_t      dmaChHandle[4];     //!< the DMA Channel handle
#endif  // DATALOG_ENABLE

  float_t       current_sf;         //!< the current scale factor, amps/cnt

  float_t       voltage_sf;         //!< the voltage scale factor, volts/cnt

  uint_least8_t numCurrentSensors;  //!< the number of current sensors
  uint_least8_t numVoltageSensors;  //!< the number of voltage sensors

#ifdef PWMDAC_ENABLE
  uint32_t      pwmDACHandle[4];    //<! the PWMDAC handles
#endif  // PWMDAC_ENABLE

  float_t       dcbus_voltage_sf;   //!< the voltage scale factor, volts/cnt

  bool          flagEnablePWM;     //<! the pwm enable flag


  DRV8323_Handle drv8323Handle;   //!< the drv8320 interface handle
  DRV8323_Obj    drv8323;         //!< the drv8320 interface object

  uint32_t      qepHandle[2];      //!< the QEP handle

} HAL_Obj;

//! \brief      Defines the HAL handle
//! \details    The HAL handle is a pointer to a HAL object.  In all HAL functions
//!             the HAL handle is passed so that the function knows what peripherals
//!             are to be accessed.
//!
typedef struct _HAL_Obj_ *HAL_Handle;


//! \brief      Defines the HAL object
//!
extern HAL_Obj hal;


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _HAL_OBJ_H_ definition

