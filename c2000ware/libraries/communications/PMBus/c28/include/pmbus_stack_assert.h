//###########################################################################
//
// FILE:   pmbus_stack_assert.h
//
// TITLE:  Defines the PMBUS_STACK_ASSERT function macro
//
// Work in this file is based on the article:
// http://www.embedded.com/electronics-blogs/other/4023329/Assert-Yourself
//
//###########################################################################
// $TI Release: C28x PMBus Communications Stack Library v1.03.00.00 $
// $Release Date: Fri Feb 12 19:16:58 IST 2021 $
// $Copyright: Copyright (C) 2015-2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#ifndef PMBUS_STACK_ASSERT_H
#define PMBUS_STACK_ASSERT_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//!
//! \defgroup PMBUS_STACK_ASSERT Code Development with Assertion
//
//!
//! \ingroup PMBUS_STACK_ASSERT
// @{
//
//*****************************************************************************

//
// Includes
//
#include <stdint.h>
#include <stdbool.h>

//
// Defines
//

//! Assign a "unique" number to each file, compiler error
//! on duplicates
#define PMBUS_STACK_FILENUM(number)   \
    enum{FILE_NUM = number};    \
    void _nullFunction##number(void){}

//! The assert() for the PMBus communications stack
#define PMBUS_STACK_ASSERT(expr) \
    if (expr)              \
    {}                     \
    else                   \
    PMBusStack_assertionFailed(FILE_NUM, __LINE__)

//*****************************************************************************
//
//! Error Handler Function Pointer
//!
//! In the \e Release Mode, the user must define an error handler, and assign
//! it to this function pointer which gets called when PMBUS_STACK_ASSERT
//! fails in the state machine.
//!
//! \note If the library was built in debug mode, i.e. the macro \b _DEBUG
//! defined then it is unnecessary for the user to define this function in
//! their project. It is only required when using the release version of the
//! library; failure to define this will result in a linker error
//!
//! \return none
//
//*****************************************************************************
extern void (*PMBusStack_errorHandler)(void);

//*****************************************************************************
//
//! Handles failed assertions
//!
//! \param file is the file number where the assertion failed
//! \param line is the line number where the assertion failed
//!
//! This function handles any failed assertions within the stack library.
//!
//! \return None.
//
//*****************************************************************************
static inline void
PMBusStack_assertionFailed(int16_t file, int16_t line)
{
#if defined(_DEBUG)
    __asm("    ESTOP0");
#else
    PMBusStack_errorHandler();
#endif //defined(_DEBUG)
}

//*****************************************************************************
//
// Close the Doxygen group.
// @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif // extern "C"

#endif  // PMBUS_STACK_ASSERT_H
