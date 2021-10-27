//#############################################################################
//
// FILE:   pto_abs2qep.h
//
// TITLE:  Prototypes and Definitions for the PTO Abs2Qep library
//
//
//#############################################################################
// $TI Release: MotorControl SDK v3.03.00.00 $
// $Release Date: Tue Sep 21 16:33:27 CDT 2021 $
// $Copyright:
// Copyright (C) 2017-2021 Texas Instruments Incorporated
//
//     http://www.ti.com/ ALL RIGHTS RESERVED
// $
//#############################################################################

//
// Library functions
//
#include <stdint.h>
#include "math.h"

//
// This is the GPIO used for Test Pin 1.
// This GPIO toggles in the system ISR and
// stays low if the direction is reverse
// stays high if the direction is forward
// It is to aid in visualization of waveforms
// only
//
#if defined(_F2837x) || defined(_F2838x)
#define PTO_ABSQEP_TEST_PIN_1 32
#else
#define PTO_ABSQEP_TEST_PIN_1 13
#endif

//
// Configurable defines
//
// CLB clock in seconds
// Position sampling period in seconds
// Maximum motor revolutions per minute
//
#define PTO_ABS2QEP_CLB_CLK            10.0e-9f
#define PTO_ABS2QEP_ABS_SAMPLE_PERIOD  100.0e-6f
#define PTO_ABS2QEP_MAX_RPM            30000.0f

//
// Absolute encoder resolution
//
#if defined(_TFORMAT_TS5700N8501)
#define PTO_ABS2QEP_ABS_ENCODER_RESOLUTION 17u        // Q17
#else
#define PTO_ABS2QEP_ABS_ENCODER_RESOLUTION 20u        // Q20
#endif

//
// Incremental encoder resolution
//
#define PTO_ABS2QEP_LINES_PER_REV   1024u    // Lines per revolution
#define PTO_ABS2QEP_QCLKS_PER_LINE  2.0f     // QCLKs (QEP edges) per line

//
// Calculated Defines
//
// Used in translating the absolute encoder position change
// an equivalent incremental position change.
//
// PTO_ABS2QEP_SAMPLE_ADJUST takes into account time required to
// load the next PTO sample.  If this adjustment is not used then
// the PTO will be slightly longer than the sampling period.  This
// time will accumulate over samples if no adjustment is made.
//
#define PTO_ABS2QEP_ABS_MAX_POSITION                                           \
            (1UL << PTO_ABS2QEP_ABS_ENCODER_RESOLUTION)

#define PTO_ABS2QEP_QCLK_PER_REV                                               \
            (PTO_ABS2QEP_LINES_PER_REV * PTO_ABS2QEP_QCLKS_PER_LINE)

#define PTO_ABS2QEP_ABS_TO_INCR                                                \
            ((float32_t)(PTO_ABS2QEP_QCLK_PER_REV)                             \
           / (float32_t)(PTO_ABS2QEP_ABS_MAX_POSITION))

#define PTO_ABS2QEP_SAMPLE_ADJUST        1.0e-6

#define PTO_ABS2QEP_SAMPLING_RATIO                                             \
             ((PTO_ABS2QEP_ABS_SAMPLE_PERIOD - PTO_ABS2QEP_SAMPLE_ADJUST)      \
            / PTO_ABS2QEP_CLB_CLK)

//
// Revolution / sample is the maximum distance the motor can move within
// one sampling period. For example: .05 of a revolution
// This value is used to detect a zero crossing as described in the
// user's guide.
//
#define PTO_ABS2QEP_MAX_REV_PER_SECOND    (PTO_ABS2QEP_MAX_RPM / 60.0f)
#define PTO_ABS2QEP_MAX_REV_PER_SAMPLE    (PTO_ABS2QEP_MAX_REV_PER_SECOND      \
                                         * PTO_ABS2QEP_ABS_SAMPLE_PERIOD)

#define PTO_ABS2QEP_ABS_MAX_DELTA_PER_SAMPLE (PTO_ABS2QEP_ABS_MAX_POSITION     \
                                        * PTO_ABS2QEP_MAX_REV_PER_SAMPLE)

//
// GP Register Bit Definitions
//
#define PTO_ABS2QEP_GPREG_LOAD_PTO_S     0x0u  // in0
#define PTO_ABS2QEP_GPREG_DIRECTION_S    0x1u  // in1

#define PTO_ABS2QEP_SET_LOAD             (0x1u << PTO_ABS2QEP_GPREG_LOAD_PTO_S)
#define PTO_ABS2QEP_SET_CLOCKWISE        (0x1u << PTO_ABS2QEP_GPREG_DIRECTION_S)
#define PTO_ABS2QEP_SET_COUNTERCLOCKWISE (0x0u << PTO_ABS2QEP_GPREG_DIRECTION_S)

//
// Status values
//
#define PTO_ABS2QEP_CLOCKWISE_PTO         0x1u
#define PTO_ABS2QEP_COUNTERCLOCKWISE_PTO  0x2u

#define PTO_ABS2QEP_PTO_WAIT_TAG          0x0u  // HLC no tag set
#define PTO_ABS2QEP_PTO_LOAD_TAG          0x1u  // HLC INTR tag
#define PTO_ABS2QEP_PTO_DONE_TAG          0x2u  // HLC INTR tag

//
// Globals
//
extern volatile uint16_t pto_direction;

//
// Function Prototypes
//
extern void pto_abs2qep_setupPeriph(void);
extern void pto_abs2qep_initCLBXBAR(void);
extern void pto_abs2qep_runPulseGen(uint16_t direction);
extern uint16_t pto_abs2qep_translatePosition(uint32_t current_position);

//
// End of File
//
