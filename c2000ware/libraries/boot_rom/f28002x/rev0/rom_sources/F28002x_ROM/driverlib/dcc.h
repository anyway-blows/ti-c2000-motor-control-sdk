//###########################################################################
//
// FILE:   dcc.h
//
// TITLE:  C28x DCC driver.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef DCC_H
#define DCC_H

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
//! \addtogroup dcc_api DCC
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_dcc.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "cpu.h"
#include "debug.h"

//
// The reset value required to start or enable specific DCC operations
//
#define DCC_ENABLE_VALUE     (0xAU)

//
// The reset value required to stop or disable specific DCC operations
//
#define DCC_DISABLE_VALUE    (0x5U)

//
// A 16-bit register mask
//
#define DCC_REG_WORD_MASK    (0xFFFFU)

//
// A 7-bit register mask
//
#define DCC_REG_BYTE_MASK    (0x7FU)

//
// A mask for the DCC counter seed registers
//
#define DCC_SEED_REG_MASK    (0xFFF0U)

//
// A mask for the DCC counter seed value
//
#define DCC_SEED_CNT_MASK    (0xF0000U)

//
// Mask to specify all the DCC empty and full FIFO status. It is the logical
// OR of all the individual FIFO status.
//
#define DCC_STATUS2_FIFO_EMPTY DCC_STATUS2_COUNT0_FIFO_EMPTY | \
                               DCC_STATUS2_COUNT1_FIFO_EMPTY | \
                               DCC_STATUS2_VALID0_FIFO_EMPTY

#define DCC_STATUS2_FIFO_FULL  DCC_STATUS2_COUNT1_FIFO_FULL | \
                               DCC_STATUS2_COUNT0_FIFO_FULL | \
                               DCC_STATUS2_VALID0_FIFO_FULL

//*****************************************************************************
//
//! The following are defines for the mode parameter of the
//! DCC_enableSingleShotMode() function.
//
//*****************************************************************************
typedef enum
{
    //! Use to stop counting when counter0 and valid0 both reach zero
    DCC_MODE_COUNTER_ZERO = 0xA00U,

    //! Use to stop counting when counter1 reaches zero
    DCC_MODE_COUNTER_ONE  = 0xB00U
} DCC_SingleShotMode;

//*****************************************************************************
//
//! The following are defines for the mode parameter of the
//! DCC_getFIFOStatus() function.
//
//*****************************************************************************
typedef enum
{
    //! Use to indicate whether Count0 FIFO is Full/Empty.
    DCC_FIFO_COUNT0,
    //! Use to indicate whether Count1 FIFO is Full/Empty.
    DCC_FIFO_COUNT1,
    //! Use to indicate whether Valid0 FIFO is Full/Empty.
    DCC_FIFO_VALID0,
    //! Use to indicate whether Any FIFO is Full/Empty.
    DCC_FIFO_ALL

}DCC_Fifo;
//*****************************************************************************
//
//! The following are defines for the identifier parameter of the
//! DCC_getRevisionNumber() function.
//
//*****************************************************************************
typedef enum
{
    DCC_REVISION_MINOR       = 0x0U,  //!< The module minor revision number
    DCC_REVISION_CUSTOM      = 0x1U,  //!< The custom module revision number
    DCC_REVISION_MAJOR       = 0x2U,  //!< The module major revision number
    DCC_REVISION_DESIGN      = 0x3U,  //!< The module design release number
    DCC_REVISION_FUNCTIONAL  = 0x4U,  //!< The module functional release number
    DCC_REVISION_SCHEME      = 0x5U   //!< The scheme of the module
} DCC_RevisionNumber;

//*****************************************************************************
//
//! The following are defines for the source parameter of the
//! DCC_setCounter1ClkSource() function.
//
//*****************************************************************************
typedef enum
{
    DCC_COUNT1SRC_PLL           = 0x0U, //!< PLL021SSP Clock Out Source
    DCC_COUNT1SRC_AUXPLL        = 0x1U, //!< AUXPLL Clock Out Source
    DCC_COUNT1SRC_INTOSC1       = 0x2U, //!< Internal Oscillator1 Clock Source
    DCC_COUNT1SRC_INTOSC2       = 0x3U, //!< Internal Oscillator2 Clock Source
    DCC_COUNT1SRC_PUMOSC        = 0x4U, //!< PUMOSC Clock Source
    DCC_COUNT1SRC_CMCLK         = 0x5U, //!< CM Clock Source
    DCC_COUNT1SRC_CPU1SYSCLK    = 0x6U, //!< CPU1 System Clock Source
    DCC_COUNT1SRC_ENETRXCLK     = 0x7U, //!< etherNET RXCLK Clock Source
    DCC_COUNT1SRC_CPU2SYSCLK    = 0x8U, //!< CPU2 System Clock Source
    DCC_COUNT1SRC_CROSSBAR      = 0x9U, //!< Input Crossbar Clock Source
    DCC_COUNT1SRC_AUXCLKIN      = 0xAU, //!< AUX Clock input Source
    DCC_COUNT1SRC_ETPWM         = 0xBU, //!< ETPWM Clock Source
    DCC_COUNT1SRC_LSPCLK        = 0xCU, //!< LSP Clock Source
    DCC_COUNT1SRC_ECATMII0RXCLK = 0xDU, //!< MII0 RXCLK(etherCAT)Clock Source
    DCC_COUNT1SRC_WDCLK         = 0xEU, //!< Watch Dog Clock Source
    DCC_COUNT1SRC_CANX          = 0xFU, //!< CANxBIT Clock Source
    DCC_COUNT1SRC_SYSAPLL_CLKOUT  = 0x10U, //!< System APLL Clock Source
    DCC_COUNT1SRC_SYSAPLL_VCO     = 0x11U, //!< System APLL Voltage control
    DCC_COUNT1SRC_SYSAPLL_VCO_2   = 0x12U, //!< System APLL Voltage control/2
    DCC_COUNT1SRC_SYSAPLL_FMDIV   = 0x13U, //!< System APLL FM DIV
    DCC_COUNT1SRC_SYSAPLL_FNDIV   = 0x14U, //!< System APLL FN DIV
    DCC_COUNT1SRC_SYSAPLL_INTOSC  = 0x15U, //!< System APLL Internal Osc Clk
    DCC_COUNT1SRC_SYSAPLL_CLK_AUX = 0x16U, //!< System APLL Aux Clk
    DCC_COUNT1SRC_ECATMII1RXCLK   = 0x17U, //!< MII1 RXCLK(etherCAT) Clk Src
    DCC_COUNT1SRC_AUXAPLL_CLKOUT  = 0x18U, //!< Aux APLL Clock Source
    DCC_COUNT1SRC_AUXAPLL_VCO     = 0x19U, //!< Aux APLL Voltage control Src
    DCC_COUNT1SRC_AUXAPLL_VCO_2   = 0x1AU, //!< Aux APLL Voltage control Src/2
    DCC_COUNT1SRC_AUXAPLL_FMDIV   = 0x1BU, //!< Aux APLL FM DIV Source
    DCC_COUNT1SRC_AUXAPLL_FNDIV   = 0x1CU, //!< Aux APLL FN DIV Source
    DCC_COUNT1SRC_AUXAPLL_INTOSC  = 0x1DU, //!< Aux APLL Internal Osc Clk
    DCC_COUNT1SRC_AUXAPLL_CLK_AUX = 0x1EU, //!< Aux APLL Aux Clk Source
} DCC_Count1ClockSource;

//*****************************************************************************
//
//! The following are defines for the source parameter of the
//! DCC_setCounter0ClkSource() function.
//
//*****************************************************************************
typedef enum
{
    DCC_COUNT0SRC_XTAL       = 0x0U,    //!< Accurate Clock Source
    DCC_COUNT0SRC_INTOSC1    = 0x1U,    //!< Internal Oscillator1 Clock Source
    DCC_COUNT0SRC_INTOSC2    = 0x2U,    //!< Internal Oscillator2 Clock Source
    DCC_COUNT0SRC_CPU1SYSCLK = 0x5U,    //!< CPU1 System Clock Source
    DCC_COUNT0SRC_CPU2SYSCLK = 0x6U,    //!< CPU2 System Clock Source
    DCC_COUNT0SRC_XBAR       = 0xCU,    //!< Input XBAR Clock Source
    DCC_COUNT0SRC_SYSPLLREFCLK = 0xEU,  //!< SYSPLL reference clock input Src
    DCC_COUNT0SRC_AUXPLLREFCLK = 0xFU   //!< AUXPLL reference clock input Src
} DCC_Count0ClockSource;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
//*****************************************************************************
//
//! \internal
//! Checks DCC base address.
//!
//! \param base specifies the DCC module base address.
//!
//! This function determines if an DCC module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static bool
DCC_isBaseValid(uint32_t base)
{
    return((base == DCC0_BASE) || (base == DCC1_BASE));
}
#endif

//*****************************************************************************
//
//! Enables the DCC module.
//!
//! \param base is the DCC module base address
//!
//! This function starts the DCC counter operation.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Set DCC enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DCCENA_M)) | DCC_ENABLE_VALUE;

    EDIS;
}

//*****************************************************************************
//
//! Disable the DCC module.
//!
//! \param base is the DCC module base address
//!
//! This function stops the DCC counter operation.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableModule(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Reset DCC enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DCCENA_M)) | DCC_DISABLE_VALUE;

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Error Signal
//!
//! \param base is the DCC module base address
//!
//! This function enables the error signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableErrorSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable the error signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_ERRENA_M)) |
                                  (DCC_ENABLE_VALUE << 4U);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Done Signal
//!
//! \param base is the DCC module base address
//!
//! This function enables the done signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableDoneSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable the done interrupt signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DONEENA_M)) |
                                  (DCC_ENABLE_VALUE << 12U);

    EDIS;
}

//*****************************************************************************
//
//! Disable DCC Error Signal
//!
//! \param base is the DCC module base address
//!
//! This function disables the error signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableErrorSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the error signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_ERRENA_M)) |
                                 (DCC_DISABLE_VALUE << 4U);

    EDIS;
}

//*****************************************************************************
//
//! Disable DCC Done Signal
//!
//! \param base is the DCC module base address
//!
//! This function disables the done signal interrupt.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableDoneSignal(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the done interrupt signal
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_DONEENA_M)) |
                                 (DCC_DISABLE_VALUE << DCC_GCTRL_DONEENA_S);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC Single-Shot Mode
//!
//! \param base is the DCC module base address
//! \param mode is the selected Single-Shot operation mode
//!
//! This function enables the single-shot mode and sets the operation mode.
//!
//! The \e mode parameter can have one of two values:
//! - \b DCC_MODE_COUNTER_ZERO - Stops counting when counter0 and valid0 both
//!   reach zero
//! - \b DCC_MODE_COUNTER_ONE  - Stops counting when counter1 reaches zero
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableSingleShotMode(uint32_t base, DCC_SingleShotMode mode)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Set Single-Shot mode value to the appropriate register
    //
    if(mode == DCC_MODE_COUNTER_ZERO)
    {
        EALLOW;

        HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                      ~(DCC_GCTRL_SINGLESHOT_M)) |
                                     (uint16_t)DCC_MODE_COUNTER_ZERO;

        EDIS;
    }
    else
    {
        EALLOW;

        HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                      ~(DCC_GCTRL_SINGLESHOT_M)) |
                                     (uint16_t)DCC_MODE_COUNTER_ONE;

        EDIS;
    }
}

//*****************************************************************************
//
//! Disable DCC Single-Shot Mode
//!
//! \param base is the DCC module base address
//!
//! This function disables the DCC Single-Shot operation mode
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableSingleShotMode(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Reset Single-Shot enable bit field.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL) = (HWREGH(base + DCC_O_GCTRL) &
                                  ~(DCC_GCTRL_SINGLESHOT_M)) |
                                 (DCC_DISABLE_VALUE << DCC_GCTRL_SINGLESHOT_S);

    EDIS;
}


//*****************************************************************************
//
//! Continue DCC counting on Error
//!
//! \param base is the DCC module base address
//!
//! This function allows the DCC to continue counting to the next window of
//! comparison when an error condition occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_continueOnError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable continue counting on Error .
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_CONT_ON_ERR_M)) |
   (DCC_ENABLE_VALUE << DCC_GCTRL2_CONT_ON_ERR_S);

    EDIS;
}

//*****************************************************************************
//
//! Stop DCC counting on Error
//!
//! \param base is the DCC module base address
//!
//! This function stops the DCC from counting to the next window of
//! comparison when an error condition occurs.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_stopOnError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Stop counting on Error .
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_CONT_ON_ERR_M)) |
   (DCC_DISABLE_VALUE << DCC_GCTRL2_CONT_ON_ERR_S);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC FIFO output read
//!
//! \param base is the DCC module base address
//!
//! This function enables the counter read registers to reflect FIFO
//! output instead of live counter value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableFIFORead(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Enable the FIFO Read.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_FIFO_READ_M)) |
   (DCC_ENABLE_VALUE << DCC_GCTRL2_FIFO_READ_S);

     EDIS;
}

//*****************************************************************************
//
//! Disable DCC FIFO output read
//!
//! \param base is the DCC module base address
//!
//! This function disables the counter read registers to reflect FIFO
//! output instead of live counter value.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableFIFORead(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the FIFO Read.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_FIFO_READ_M)) |
   (DCC_DISABLE_VALUE << DCC_GCTRL2_FIFO_READ_S);

    EDIS;
}

//*****************************************************************************
//
//! Enable DCC FIFO update regardless of any Error
//!
//! \param base is the DCC module base address
//!
//! This function enables FIFO writes regardless of an error event
//! on completion of comparison window.
//!
//! \note This is applicable only in Continuous mode ,in single shot
//! mode FIFO captures counts only on Error.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_enableFIFOUpdateOnNonError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));


    //
    // Enable the FIFO update.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_FIFO_NONERR_M)) |
   (DCC_ENABLE_VALUE << DCC_GCTRL2_FIFO_NONERR_S);

     EDIS;
}

//*****************************************************************************
//
//! Disable DCC FIFO update only on Error
//!
//! \param base is the DCC module base address
//!
//! This function disables FIFO writes done regardless of an error event and
//! only allows writes on an error event.
//!
//! \note This is applicable only in Continuous mode ,in single shot
//! mode FIFO captures counts only on Error.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_disableFIFOUpdateOnNonError(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Disable the FIFO update.
    //
    EALLOW;

    HWREGH(base + DCC_O_GCTRL2) =
   (HWREGH(base + DCC_O_GCTRL2) & ~(DCC_GCTRL2_FIFO_NONERR_M)) |
   (DCC_DISABLE_VALUE << DCC_GCTRL2_FIFO_NONERR_S);

    EDIS;
}

//*****************************************************************************
//
//! Gets the status of the DCC FIFOs
//!
//! \param base is the DCC module base address
//! \param type is the FIFO whose status is needed.
//!
//! The function indicates whether the FIFOs are Full/Empty.
//!
//! The \e type parameter can have one of three values:
//! - \b DCC_FIFO_COUNT0 - Indicate whether Count0 FIFO is Full/Empty.
//! - \b DCC_FIFO_COUNT1  - Indicate whether Count1 FIFO is Full/Empty.
//! - \b DCC_FIFO_VALID0  - Indicate whether Valid0 FIFO is Full/Empty.
//! - \b DCC_FIFO_ALL - Indicate whether any FIFO is Full/Empty.
//!
//! \return status of the specific DCC FIFO.
//
//*****************************************************************************

static inline uint16_t
DCC_getFIFOStatus(uint32_t base, DCC_Fifo type)
{
   uint16_t status;

   //
   // Check the arguments.
   //
   ASSERT(DCC_isBaseValid(base));

   //
   // Get specified DCC FIFOs status
   //
   if(type == DCC_FIFO_COUNT0)
    {
     status = (HWREGH(base + DCC_O_STATUS2) & (DCC_STATUS2_COUNT0_FIFO_EMPTY |
                                               DCC_STATUS2_COUNT0_FIFO_FULL));
    }
   else if(type == DCC_FIFO_COUNT1)
    {
     status = (HWREGH(base + DCC_O_STATUS2) & (DCC_STATUS2_COUNT1_FIFO_EMPTY |
                                               DCC_STATUS2_COUNT1_FIFO_FULL));
    }
   else if(type == DCC_FIFO_VALID0)
    {
     status = (HWREGH(base + DCC_O_STATUS2) & (DCC_STATUS2_VALID0_FIFO_EMPTY |
                                               DCC_STATUS2_VALID0_FIFO_FULL));
    }
   else
    {
     status = (HWREGH(base + DCC_O_STATUS2) & (DCC_STATUS2_FIFO_EMPTY |
                                               DCC_STATUS2_FIFO_FULL));
    }

   return(status);
}

//*****************************************************************************
//
//! Get number of errors until DCC terminal count.
//!
//! \param base is the DCC module base address
//!
//! This function counts the number of errors either since the last
//! write/clear to this register or till terminal count is reached.
//!
//! \note Once terminal count is reached,the count freezes.
//!  User needs to clear it.
//!
//! \return Returns the error count until DCC terminal count is reached
//
//*****************************************************************************
static inline uint16_t
DCC_getErrorCount(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the number of errors until DCC terminal count
    //
    return(HWREGH(base + DCC_O_ERRCNT));
}

//*****************************************************************************
//
//! Clear Error Count
//!
//! \param base is the DCC module base address
//!
//! This function clears the error counts.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_clearErrorCount(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));


    EALLOW;

    HWREGH(base + DCC_O_ERRCNT) =
   (HWREGH(base + DCC_O_ERRCNT) & ~(DCC_ERRCNT_ERRCNT_M)) |
   (0U << DCC_ERRCNT_ERRCNT_S);

    EDIS;
}

//*****************************************************************************
//
//! Get Error Flag Status
//!
//! \param base is the DCC module base address
//!
//! This function gets the error flag status.
//!
//! \return Returns \b true if an error has occurred, \b false if no errors
//! have occurred.
//
//*****************************************************************************
static inline bool
DCC_getErrorStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the error flag
    //
    return((bool)((HWREGH(base + DCC_O_STATUS) & DCC_STATUS_ERR) ==
                  DCC_STATUS_ERR));
}

//*****************************************************************************
//
//! Get Single-Shot Done Flag Status
//!
//! \param base is the DCC module base address
//!
//! This function gets the single-shot done flag status.
//!
//! \return Returns \b true if single-shot mode has completed, \b false if
//! single-shot mode has not completed.
//
//*****************************************************************************
static inline bool
DCC_getSingleShotStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Read the done flag
    //
    return((bool)((HWREGH(base + DCC_O_STATUS) & DCC_STATUS_DONE) ==
                  DCC_STATUS_DONE));
}

//*****************************************************************************
//
//! Clear Error Status Flag
//!
//! \param base is the DCC module base address
//!
//! This function clears the DCC error status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_clearErrorFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Clear error status flag
    //
    EALLOW;

    HWREGH(base + DCC_O_STATUS) |= DCC_STATUS_ERR;

    EDIS;
}

//*****************************************************************************
//
//! Clear Single-Shot Done Status Flag
//!
//! \param base is the DCC module base address
//!
//! This function clears the DCC single-shot done status flag.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_clearDoneFlag(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Clear done status flag
    //
    EALLOW;

    HWREGH(base + DCC_O_STATUS) |= DCC_STATUS_DONE;

    EDIS;
}

//*****************************************************************************
//
//! Get Current Value of Counter 0
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of counter 0.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of counter 0.
//
//*****************************************************************************
static inline uint32_t
DCC_getCounter0Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current counter 0 value
    //
    return(HWREG(base + DCC_O_CNT0));
}

//*****************************************************************************
//
//! Get Current Value of the Valid Duration Counter for Counter 0
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of the valid duration counter for
//! counter 0.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of the valid duration counter.
//
//*****************************************************************************
static inline uint16_t
DCC_getValidCounter0Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current valid duration counter value
    //
    return(HWREGH(base + DCC_O_VALID0));
}

//*****************************************************************************
//
//! Get Current Value of Counter 1
//!
//! \param base is the DCC module base address
//!
//! This function gets current value of counter 1.
//!
//! \note Reads of the counter value may not be exact since the read operation
//! is synchronized to the vbus clock.
//!
//! \return Returns the current value of counter 1.
//
//*****************************************************************************
static inline uint32_t
DCC_getCounter1Value(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Get the current counter 1 value
    //
    return(HWREG(base + DCC_O_CNT1));
}

//*****************************************************************************
//
//! Set Counter 1 Clock Source
//!
//! \param base is the DCC module base address
//! \param source is the selected clock source for counter 1
//!
//! This function sets the counter 1 clock source.
//!
//! The \e source parameter can have one of thirty-one values:
//! - \b DCC_COUNT1SRC_PLL        - PLL021SSP Clock Out Source
//! - \b DCC_COUNT1SRC_AUXPLL     - AUXPLL Clock Out Source
//! - \b DCC_COUNT1SRC_INTOSC1    - Internal Oscillator1 Clock Source
//! - \b DCC_COUNT1SRC_INTOSC2    - Internal Oscillator2 Clock Source
//! - \b DCC_COUNT1SRC_PUMOSC     - PUMOSC Clock Source
//! - \b DCC_COUNT1SRC_CMCLK      - CM Clock Source
//! - \b DCC_COUNT1SRC_CPU1SYSCLK - CPU1 System Clock Source
//! - \b DCC_COUNT1SRC_ENETRXCLK  - etherNET RXCLK Clock Source
//! - \b DCC_COUNT1SRC_CPU2SYSCLK - CPU2 System Clock Source
//! - \b DCC_COUNT1SRC_CROSSBAR   - Input Crossbar Clock Source
//! - \b DCC_COUNT1SRC_AUXCLK     - AUX Clock Source
//! - \b DCC_COUNT1SRC_ETPWM      - ETPWM Clock Source
//! - \b DCC_COUNT1SRC_LSPCLK     - LSP Clock Source
//! - \b DCC_COUNT1SRC_ECATMII0RXCLK - MII0 RXCLK(etherCAT) Clock Source
//! - \b DCC_COUNT1SRC_WDCLK      - Watch Dog Clock Source
//! - \b DCC_COUNT1SRC_CANX       - CANxBIT Clock Source
//! - \b DCC_COUNT1SRC_SYSAPLL_CLKOUT  - System APLL Clock Source
//! - \b DCC_COUNT1SRC_SYSAPLL_VCO     - System APLL Voltage control
//! - \b DCC_COUNT1SRC_SYSAPLL_VCO_2   - System APLL Voltage control/2
//! - \b DCC_COUNT1SRC_SYSAPLL_FMDIV   - System APLL FM DIV
//! - \b DCC_COUNT1SRC_SYSAPLL_FNDIV   - System APLL FN DIV
//! - \b DCC_COUNT1SRC_SYSAPLL_INTOSC  - System APLL Internal Osc Clk
//! - \b DCC_COUNT1SRC_SYSAPLL_CLK_AUX - System APLL Aux Clk
//! - \b DCC_COUNT1SRC_ECATMII1RXCLK    - MII1 RXCLK(etherCAT) Clk Src
//! - \b DCC_COUNT1SRC_AUXAPLL_CLKOUT  - Aux APLL Clock Source
//! - \b DCC_COUNT1SRC_AUXAPLL_VCO     - Aux APLL Voltage control Src
//! - \b DCC_COUNT1SRC_AUXAPLL_VCO_2   - Aux APLL Voltage control Src/2
//! - \b DCC_COUNT1SRC_AUXAPLL_FMDIV   - Aux APLL FM DIV Source
//! - \b DCC_COUNT1SRC_AUXAPLL_FNDIV   - Aux APLL FN DIV Source
//! - \b DCC_COUNT1SRC_AUXAPLL_INTOSC  - Aux APLL Internal Osc Clk
//! - \b DCC_COUNT1SRC_AUXAPLL_CLK_AUX - Aux APLL Aux Clk Source
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_setCounter1ClkSource(uint32_t base, DCC_Count1ClockSource source)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Set the specified clock source
    //
    EALLOW;

    //
    //  DCC Clk source is of 5bits DCCCLKSRC1[4:0]
    //
    HWREGH(base + DCC_O_CLKSRC1) = (HWREGH(base + DCC_O_CLKSRC1) &
                                   (DCC_REG_BYTE_MASK << 5U)) |
                                   ((DCC_ENABLE_VALUE << 12U) |
                                   (uint16_t)source);

    EDIS;
}

//*****************************************************************************
//
//! Set Counter 0 Clock Source
//!
//! \param base is the DCC module base address
//! \param source is the selected clock source for counter 0
//!
//! This function sets the counter 0 clock source.
//!
//! The \e source parameter can have one of six values:
//! - \b DCC_COUNT0SRC_XTAL       - Accurate Clock Source
//! - \b DCC_COUNT0SRC_INTOSC1    - Internal Oscillator1 Clock Source
//! - \b DCC_COUNT0SRC_INTOSC2    - Internal Oscillator2 Clock Source
//! - \b DCC_COUNT0SRC_CPU1SYSCLK - CPU1 System Clock Source
//! - \b DCC_COUNT0SRC_CPU2SYSCLK - CPU2 System Clock Source
//! - \b DCC_COUNT0SRC_XBAR       - Input XBAR Clock Source
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_setCounter0ClkSource(uint32_t base, DCC_Count0ClockSource source)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Set the specified clock source
    //
    EALLOW;

    //
    //  DCC Clk source is of 5bits DCCCLKSRC0[4:0]
    //
    HWREGH(base + DCC_O_CLKSRC0) = (HWREGH(base + DCC_O_CLKSRC0) &
                                   (DCC_REG_BYTE_MASK << 5U)) |
                                   ((DCC_ENABLE_VALUE << 12U) |
                                   (uint16_t)source);

    EDIS;
}

//*****************************************************************************
//
//! Get Counter 1 Clock Source
//!
//! \param base is the DCC module base address
//!
//! This function gets the counter 1 clock source.
//!
//! \return Returns one of the following enumerated source values:
//! - \b DCC_COUNT1SRC_PLL        - PLL021SSP Clock Out Source
//! - \b DCC_COUNT1SRC_AUXPLL     - AUXPLL Clock Out Source
//! - \b DCC_COUNT1SRC_INTOSC1    - Internal Oscillator1 Clock Source
//! - \b DCC_COUNT1SRC_INTOSC2    - Internal Oscillator2 Clock Source
//! - \b DCC_COUNT1SRC_PUMOSC     - PUMOSC Clock Source
//! - \b DCC_COUNT1SRC_CMCLK      - CM Clock Source
//! - \b DCC_COUNT1SRC_CPU1SYSCLK - CPU1 System Clock Source
//! - \b DCC_COUNT1SRC_ENETRXCLK  - etherNET RXCLK Clock Source
//! - \b DCC_COUNT1SRC_CPU2SYSCLK - CPU2 System Clock Source
//! - \b DCC_COUNT1SRC_CROSSBAR   - Input Crossbar Clock Source
//! - \b DCC_COUNT1SRC_AUXCLK     - AUX Clock Source
//! - \b DCC_COUNT1SRC_ETPWM      - ETPWM Clock Source
//! - \b DCC_COUNT1SRC_LSPCLK     - LSP Clock Source
//! - \b DCC_COUNT1SRC_ECATMII0RXCLK - MII0 RXCLK(etherCAT) Clock Source
//! - \b DCC_COUNT1SRC_WDCLK      - Watch Dog Clock Source
//! - \b DCC_COUNT1SRC_CANX       - CANxBIT Clock Source
//! - \b DCC_COUNT1SRC_SYSAPLL_CLKOUT  - System APLL Clock Source
//! - \b DCC_COUNT1SRC_SYSAPLL_VCO     - System APLL Voltage control
//! - \b DCC_COUNT1SRC_SYSAPLL_VCO_2   - System APLL Voltage control/2
//! - \b DCC_COUNT1SRC_SYSAPLL_FMDIV   - System APLL FM DIV
//! - \b DCC_COUNT1SRC_SYSAPLL_FNDIV   - System APLL FN DIV
//! - \b DCC_COUNT1SRC_SYSAPLL_INTOSC  - System APLL Internal Osc Clk
//! - \b DCC_COUNT1SRC_SYSAPLL_CLK_AUX - System APLL Aux Clk
//! - \b DCC_COUNT1SRC_ECATMII1RXCLK    - MII1 RXCLK(etherCAT) Clk Src
//! - \b DCC_COUNT1SRC_AUXAPLL_CLKOUT  - Aux APLL Clock Source
//! - \b DCC_COUNT1SRC_AUXAPLL_VCO     - Aux APLL Voltage control Src
//! - \b DCC_COUNT1SRC_AUXAPLL_VCO_2   - Aux APLL Voltage control Src/2
//! - \b DCC_COUNT1SRC_AUXAPLL_FMDIV   - Aux APLL FM DIV Source
//! - \b DCC_COUNT1SRC_AUXAPLL_FNDIV   - Aux APLL FN DIV Source
//! - \b DCC_COUNT1SRC_AUXAPLL_INTOSC  - Aux APLL Internal Osc Clk
//! - \b DCC_COUNT1SRC_AUXAPLL_CLK_AUX - Aux APLL Aux Clk Source

//
//*****************************************************************************
static inline uint16_t
DCC_getCounter1ClkSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Get the specified clock source
    //
    return(HWREGH(base + DCC_O_CLKSRC1) & DCC_CLKSRC1_CLKSRC1_M);
}

//*****************************************************************************
//
//! Get Counter 0 Clock Source
//!
//! \param base is the DCC module base address
//!
//! This function gets the counter 0 clock source.
//!
//! \return Returns one of the following enumerated source values:
//! - \b DCC_COUNT0SRC_XTAL       - Accurate Clock Source
//! - \b DCC_COUNT0SRC_INTOSC1    - Internal Oscillator1 Clock Source
//! - \b DCC_COUNT0SRC_INTOSC2    - Internal Oscillator2 Clock Source
//! - \b DCC_COUNT0SRC_CPU1SYSCLK - CPU1 System Clock Source
//! - \b DCC_COUNT0SRC_CPU2SYSCLK - CPU2 System Clock Source
//! - \b DCC_COUNT0SRC_XBAR       - Input XBAR Clock Source
//
//*****************************************************************************
static inline uint16_t
DCC_getCounter0ClkSource(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    //  Get the specified clock source
    //
    return(HWREGH(base + DCC_O_CLKSRC0) & DCC_CLKSRC0_CLKSRC0_M);
}

//*****************************************************************************
//
//! Set the seed values
//!
//! \param base is the DCC module base address
//! \param counter0 sets the seed value that gets loaded into Counter 0
//! \param validCounter0 sets the seed value that gets loaded into the valid
//!  duration counter for Counter 0
//! \param counter1 sets the seed value that gets loaded into Counter 1
//!
//! This function sets the seed values for Counter 0, Valid Duration Counter 0,
//! and Counter 1.
//!
//! \note
//! -# Operating DCC with '0' set as the seed value for Counter 0, Valid
//! Duration Counter 0, and/or Counter 1 will result in undefined operation.
//! -# The Valid Duration Counter 0 is designed to be at least four cycles
//! wide and shouldn't be programmed with a value less than '4'.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCC_setCounterSeeds(uint32_t base, uint32_t counter0, uint32_t validCounter0,
                    uint32_t counter1)
{
    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));
    ASSERT(validCounter0 >= 4U);
    ASSERT(counter0 > 0U);
    ASSERT(counter1 > 0U);

    EALLOW;

    //
    // Set Counter 0 Seed
    //
    HWREGH(base + DCC_O_CNTSEED0) = counter0 & DCC_REG_WORD_MASK;
    HWREGH(base + DCC_O_CNTSEED0 + 2U) = (HWREGH(base + DCC_O_CNTSEED0 + 2U) &
                                          DCC_SEED_REG_MASK) |
                                         ((uint32_t)(counter0 &
                                          DCC_SEED_CNT_MASK) >> 16U);

    //
    // Set Valid Duration Counter 0 Seed
    //
    HWREGH(base + DCC_O_VALIDSEED0) = validCounter0;

    //
    // Set Counter 1 Seed
    //
    HWREGH(base + DCC_O_CNTSEED1) = counter1 & DCC_REG_WORD_MASK;
    HWREGH(base + DCC_O_CNTSEED1 + 2U) = (HWREGH(base + DCC_O_CNTSEED1 + 2U) &
                                          DCC_SEED_REG_MASK) |
                                         ((uint32_t)(counter1 &
                                          DCC_SEED_CNT_MASK) >> 16U);

    EDIS;
}

//*****************************************************************************
//
//! Get DCC Version Number
//!
//! \param base is the DCC module base address
//! \param identifier is the selected revision number identifier
//!
//! This function gets the specific version number.
//!
//! The \e identifier parameter can have one of six values:
//! - \b DCC_REVISION_MINOR      - The minor revision number
//! - \b DCC_REVISION_CUSTOM     - The custom module number
//! - \b DCC_REVISION_MAJOR      - The major revision number
//! - \b DCC_REVISION_DESIGN     - The design release number
//! - \b DCC_REVISION_FUNCTIONAL - The functional release number
//! - \b DCC_REVISION_SCHEME     - The scheme of the module
//!
//! \return Specified revision number
//
//*****************************************************************************
extern uint16_t
DCC_getRevisionNumber(uint32_t base, DCC_RevisionNumber identifier);

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // DCC_H
