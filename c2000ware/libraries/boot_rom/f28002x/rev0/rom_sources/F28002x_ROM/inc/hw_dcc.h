//###########################################################################
//
// FILE:    hw_dcc.h
//
// TITLE:   Definitions for the DCC registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_DCC_H
#define HW_DCC_H

//*****************************************************************************
//
// The following are defines for the DCC register offsets
//
//*****************************************************************************
#define DCC_O_GCTRL               0x0U         // Starts / stops the counters.
                                               // Clears the error signal.
#define DCC_O_REV                 0x4U         // Specifies the module version.
#define DCC_O_CNTSEED0            0x8U         // Seed value for the counter
                                               // attached to Clock Source 0.
#define DCC_O_VALIDSEED0          0xCU         // Seed value for the timeout
                                               // counter attached to Clock Source
                                               // 0.
#define DCC_O_CNTSEED1            0x10U        // Seed value for the counter
                                               // attached to Clock Source 1.
#define DCC_O_STATUS              0x14U        // Specifies the status of the
                                               // DCC Module.
#define DCC_O_CNT0                0x18U        // Value of the counter attached
                                               // to Clock Source 0.
#define DCC_O_VALID0              0x1CU        // Value of the valid counter
                                               // attached to Clock Source 0.
#define DCC_O_CNT1                0x20U        // Value of the counter attached
                                               // to Clock Source 1.
#define DCC_O_CLKSRC1             0x24U        // Selects the clock source for
                                               // Counter 1.
#define DCC_O_CLKSRC0             0x28U        // Selects the clock source for
                                               // Counter 0.
#define DCC_O_GCTRL2              0x2CU        // Allows configuring different
                                               // modes of operation for DCC
#define DCC_O_STATUS2             0x30U        // Specifies the status of the
                                               // DCC FIFOs
#define DCC_O_ERRCNT              0x34U        // Counts number of errors since
                                               // last clear

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCGCTRL register
//
//*****************************************************************************
#define DCC_GCTRL_DCCENA_S        0U
#define DCC_GCTRL_DCCENA_M        0xFU         // DCC Enable
#define DCC_GCTRL_ERRENA_S        4U
#define DCC_GCTRL_ERRENA_M        0xF0U        // Error Enable
#define DCC_GCTRL_SINGLESHOT_S    8U
#define DCC_GCTRL_SINGLESHOT_M    0xF00U       // Single-Shot Enable
#define DCC_GCTRL_DONEENA_S       12U
#define DCC_GCTRL_DONEENA_M       0xF000U      // DONE Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCREV register
//
//*****************************************************************************
#define DCC_REV_MINOR_S           0U
#define DCC_REV_MINOR_M           0x3FU        // Minor Revision Number
#define DCC_REV_CUSTOM_S          6U
#define DCC_REV_CUSTOM_M          0xC0U        // Custom Module Number
#define DCC_REV_MAJOR_S           8U
#define DCC_REV_MAJOR_M           0x700U       // Major Revision Number
#define DCC_REV_RTL_S             11U
#define DCC_REV_RTL_M             0xF800U      // Design Release Number
#define DCC_REV_FUNC_S            16U
#define DCC_REV_FUNC_M            0xFFF0000U   // Functional Release Number
#define DCC_REV_SCHEME_S          30U
#define DCC_REV_SCHEME_M          0xC0000000U  // Defines Scheme for Module

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCNTSEED0 register
//
//*****************************************************************************
#define DCC_CNTSEED0_COUNTSEED0_S  0U
#define DCC_CNTSEED0_COUNTSEED0_M  0xFFFFFU     // Seed Value for Counter 0

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCVALIDSEED0 register
//
//*****************************************************************************
#define DCC_VALIDSEED0_VALIDSEED_S  0U
#define DCC_VALIDSEED0_VALIDSEED_M  0xFFFFU      // Seed Value for Valid Duration
                                               // Counter 0

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCNTSEED1 register
//
//*****************************************************************************
#define DCC_CNTSEED1_COUNTSEED1_S  0U
#define DCC_CNTSEED1_COUNTSEED1_M  0xFFFFFU     // Seed Value for Counter 1

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCSTATUS register
//
//*****************************************************************************
#define DCC_STATUS_ERR            0x1U         // Error Flag
#define DCC_STATUS_DONE           0x2U         // Single-Shot Done Flag

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCNT0 register
//
//*****************************************************************************
#define DCC_CNT0_COUNT0_S         0U
#define DCC_CNT0_COUNT0_M         0xFFFFFU     // Current Value of Counter 0

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCVALID0 register
//
//*****************************************************************************
#define DCC_VALID0_VALID0_S       0U
#define DCC_VALID0_VALID0_M       0xFFFFU      // Current Value of Valid 0

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCNT1 register
//
//*****************************************************************************
#define DCC_CNT1_COUNT1_S         0U
#define DCC_CNT1_COUNT1_M         0xFFFFFU     // Current Value of Counter 1

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCLKSRC1 register
//
//*****************************************************************************
#define DCC_CLKSRC1_CLKSRC1_S     0U
#define DCC_CLKSRC1_CLKSRC1_M     0x1FU        // Clock Source Select for
                                               // Counter 1
#define DCC_CLKSRC1_KEY_S         12U
#define DCC_CLKSRC1_KEY_M         0xF000U      // Enables or Disables Clock
                                               // Source Selection for COUNT1

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCCLKSRC0 register
//
//*****************************************************************************
#define DCC_CLKSRC0_CLKSRC0_S     0U
#define DCC_CLKSRC0_CLKSRC0_M     0xFU         // Clock Source Select for
                                               // Counter 0
#define DCC_CLKSRC0_KEY_S         12U
#define DCC_CLKSRC0_KEY_M         0xF000U      // Enables or Disables Clock
                                               // Source Selection for COUNT0

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCGCTRL2 register
//
//*****************************************************************************
#define DCC_GCTRL2_CONT_ON_ERR_S  0U
#define DCC_GCTRL2_CONT_ON_ERR_M  0xFU         // Continue on Error enable
#define DCC_GCTRL2_FIFO_READ_S    4U
#define DCC_GCTRL2_FIFO_READ_M    0xF0U        // FIFO read enable
#define DCC_GCTRL2_FIFO_NONERR_S  8U
#define DCC_GCTRL2_FIFO_NONERR_M  0xF00U       // FIFO update on non-Error

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCSTATUS2 register
//
//*****************************************************************************
#define DCC_STATUS2_COUNT0_FIFO_EMPTY  0x1U         // Count0 FIFO Empty
#define DCC_STATUS2_VALID0_FIFO_EMPTY  0x2U         // Valid0 FIFO Empty
#define DCC_STATUS2_COUNT1_FIFO_EMPTY  0x4U         // Count1 FIFO Empty
#define DCC_STATUS2_COUNT0_FIFO_FULL  0x8U         // Count0 FIFO Full
#define DCC_STATUS2_VALID0_FIFO_FULL  0x10U        // Valid0 FIFO Full
#define DCC_STATUS2_COUNT1_FIFO_FULL  0x20U        // Count1 FIFO Full

//*****************************************************************************
//
// The following are defines for the bit fields in the DCCERRCNT register
//
//*****************************************************************************
#define DCC_ERRCNT_ERRCNT_S       0U
#define DCC_ERRCNT_ERRCNT_M       0x3FFU       // Error Count of number of
                                               // errors since last write or reset.
#endif
