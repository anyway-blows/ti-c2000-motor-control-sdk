//###########################################################################
//
// FILE:    hw_clb.h
//
// TITLE:   Definitions for the CLB registers.
//
//###########################################################################
// $TI Release:  $
// $Release Date:  $
// $Copyright:  $
//###########################################################################

#ifndef HW_CLB_H
#define HW_CLB_H

//*****************************************************************************
//
// The following are defines for the CLB register offsets
//
//*****************************************************************************
#define CLB_O_COUNT_RESET         0x2U         // Counter Block RESET
#define CLB_O_COUNT_MODE_1        0x4U         // Counter Block MODE_1
#define CLB_O_COUNT_MODE_0        0x6U         // Counter Block MODE_0
#define CLB_O_COUNT_EVENT         0x8U         // Counter Block EVENT
#define CLB_O_FSM_EXTRA_IN0       0xAU         // FSM Extra EXT_IN0
#define CLB_O_FSM_EXT_IN0         0xCU         // FSM EXT_IN0
#define CLB_O_FSM_EXT_IN1         0xEU         // FSM_EXT_IN1
#define CLB_O_FSM_EXTRA_IN1       0x10U        // FSM Extra_EXT_IN1
#define CLB_O_LUT4_IN0            0x12U        // LUT4_0/1/2 IN0 input source
#define CLB_O_LUT4_IN1            0x14U        // LUT4_0/1/2 IN1 input source
#define CLB_O_LUT4_IN2            0x16U        // LUT4_0/1/2 IN2 input source
#define CLB_O_LUT4_IN3            0x18U        // LUT4_0/1/2 IN3 input source
#define CLB_O_FSM_LUT_FN10        0x1CU        // LUT function for FSM Unit 1
                                               // and Unit 0
#define CLB_O_FSM_LUT_FN2         0x1EU        // LUT function for FSM Unit 2
#define CLB_O_LUT4_FN10           0x20U        // LUT function for LUT4 block
                                               // of Unit 1 and 0
#define CLB_O_LUT4_FN2            0x22U        // LUT function for LUT4 block
                                               // of Unit 2
#define CLB_O_FSM_NEXT_STATE_0    0x24U        // FSM Next state equations for
                                               // Unit 0
#define CLB_O_FSM_NEXT_STATE_1    0x26U        // FSM Next state equations for
                                               // Unit 1
#define CLB_O_FSM_NEXT_STATE_2    0x28U        // FSM Next state equations for
                                               // Unit 2
#define CLB_O_MISC_CONTROL        0x2AU        // Static controls for Ctr,FSM
#define CLB_O_OUTPUT_LUT_0        0x2CU        // Inp Sel, LUT fns for Out0
#define CLB_O_OUTPUT_LUT_1        0x2EU        // Inp Sel, LUT fns for Out1
#define CLB_O_OUTPUT_LUT_2        0x30U        // Inp Sel, LUT fns for Out2
#define CLB_O_OUTPUT_LUT_3        0x32U        // Inp Sel, LUT fns for Out3
#define CLB_O_OUTPUT_LUT_4        0x34U        // Inp Sel, LUT fns for Out4
#define CLB_O_OUTPUT_LUT_5        0x36U        // Inp Sel, LUT fns for Out5
#define CLB_O_OUTPUT_LUT_6        0x38U        // Inp Sel, LUT fns for Out6
#define CLB_O_OUTPUT_LUT_7        0x3AU        // Inp Sel, LUT fns for Out7
#define CLB_O_HLC_EVENT_SEL       0x3CU        // Event Selector register for
                                               // the High Level controller
#define CLB_O_COUNT_MATCH_TAP_SEL  0x3EU        // Counter tap values for match1
                                               // and match2 outputs
#define CLB_O_OUTPUT_COND_CTRL_0  0x40U        // Output conditioning control
                                               // for output 0
#define CLB_O_OUTPUT_COND_CTRL_1  0x42U        // Output conditioning control
                                               // for output 1
#define CLB_O_OUTPUT_COND_CTRL_2  0x44U        // Output conditioning control
                                               // for output 2
#define CLB_O_OUTPUT_COND_CTRL_3  0x46U        // Output conditioning control
                                               // for output 3
#define CLB_O_OUTPUT_COND_CTRL_4  0x48U        // Output conditioning control
                                               // for output 4
#define CLB_O_OUTPUT_COND_CTRL_5  0x4AU        // Output conditioning control
                                               // for output 5
#define CLB_O_OUTPUT_COND_CTRL_6  0x4CU        // Output conditioning control
                                               // for output 6
#define CLB_O_OUTPUT_COND_CTRL_7  0x4EU        // Output conditioning control
                                               // for output 7
#define CLB_O_MISC_ACCESS_CTRL    0x50U        // Miscellaneous Access and
                                               // enable control
#define CLB_O_SPI_DATA_CTRL_HI    0x51U        // CLB to SPI buffer control
                                               // High
#define CLB_O_LOAD_EN             0x0U         // Global enable & indirect load
                                               // enable control
#define CLB_O_LOAD_ADDR           0x2U         // Indirect address
#define CLB_O_LOAD_DATA           0x4U         // Data for indirect loads
#define CLB_O_INPUT_FILTER        0x6U         // Input filter selection for
                                               // both edge detection and
                                               // synchronizers
#define CLB_O_IN_SEL_0            0x8U         // Input selection to decide
                                               // between Signals and GP register
#define CLB_O_LCL_MUX_SEL_1       0xAU         // Input Mux selection for local
                                               // mux
#define CLB_O_LCL_MUX_SEL_2       0xCU         // Input Mux selection for local
                                               // mux
#define CLB_O_BUF_PTR             0xEU         // PUSH and PULL pointers
#define CLB_O_GP_REG              0x10U        // General purpose register for
                                               // CELL inputs
#define CLB_O_GLBL_MUX_SEL_1      0x14U        // Global Mux select for CELL
                                               // inputs
#define CLB_O_GLBL_MUX_SEL_2      0x16U        // Global Mux select for CELL
                                               // inputs
#define CLB_O_PRESCALE_CTRL       0x18U        // Prescaler register control
#define CLB_O_LOCK                0x22U        // Lock control register
#define CLB_O_HLC_INSTR_READ_PTR  0x24U        // HLC instruction read pointer
#define CLB_O_HLC_INSTR_VALUE     0x26U        // HLC instruction read value
#define CLB_O_DBG_OUT_2           0x2EU        // Visibility for CLB inputs and
                                               // final  asynchronous outputs
#define CLB_O_DBG_R0              0x30U        // R0 of High level Controller
#define CLB_O_DBG_R1              0x32U        // R1 of High level Controller
#define CLB_O_DBG_R2              0x34U        // R2 of High level Controller
#define CLB_O_DBG_R3              0x36U        // R3 of High level Controller
#define CLB_O_DBG_C0              0x38U        // Count of Unit 0
#define CLB_O_DBG_C1              0x3AU        // Count of Unit 1
#define CLB_O_DBG_C2              0x3CU        // Count of Unit 2
#define CLB_O_DBG_OUT             0x3EU        // Outputs of various units in
                                               // the Cell
#define CLB_O_PUSH(i)             (0x0U + ((i) * 0x2U))   // (0 <= i < 4) CLB_PUSH FIFO Registers (from HLC)
#define CLB_O_PULL(i)             (0x40U + ((i) * 0x2U))   // (0 <= i < 4) CLB_PULL FIFO Registers (TO HLC)

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_MISC_CONTROL register
//
//*****************************************************************************
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_0  0x1U         // Add/Shift for counter 0
#define CLB_MISC_CONTROL_COUNT_DIR_0  0x2U         // Direction for counter 0
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_0  0x4U         // Event control for counter 0
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_1  0x8U         // Add/Shift for counter 1
#define CLB_MISC_CONTROL_COUNT_DIR_1  0x10U        // Direction for counter 1
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_1  0x20U        // Event control for counter 1
#define CLB_MISC_CONTROL_COUNT_ADD_SHIFT_2  0x40U        // Add/Shift for counter 2
#define CLB_MISC_CONTROL_COUNT_DIR_2  0x80U        // Direction for counter 2
#define CLB_MISC_CONTROL_COUNT_EVENT_CTRL_2  0x100U       // Event control for counter 2
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_0  0x200U       // Serializer enable 0
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_1  0x400U       // Serializer enable 1
#define CLB_MISC_CONTROL_COUNT_SERIALIZER_2  0x800U       // Serializer enable 2
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_0  0x1000U      // FSM extra_sel0 for 0
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_0  0x2000U      // FSM extra_sel1 for 0
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_1  0x4000U      // FSM extra_sel0 for 1
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_1  0x8000U      // FSM extra_sel1 for 1
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL0_2  0x10000U     // FSM extra_sel0 for 2
#define CLB_MISC_CONTROL_FSM_EXTRA_SEL1_2  0x20000U     // FSM extra_sel1 for 2
#define CLB_MISC_CONTROL_COUNT0_MATCH1_TAP_EN  0x40000U     // Match1 Tap Enable for Counter
                                               // 0
#define CLB_MISC_CONTROL_COUNT1_MATCH1_TAP_EN  0x80000U     // Match1 Tap Enable for Counter
                                               // 1
#define CLB_MISC_CONTROL_COUNT2_MATCH1_TAP_EN  0x100000U    // Match1 Tap Enable for Counter
                                               // 2
#define CLB_MISC_CONTROL_COUNT0_MATCH2_TAP_EN  0x200000U    // Match2 Tap Enable for Counter
                                               // 0
#define CLB_MISC_CONTROL_COUNT1_MATCH2_TAP_EN  0x400000U    // Match2 Tap Enable for Counter
                                               // 1
#define CLB_MISC_CONTROL_COUNT2_MATCH2_TAP_EN  0x800000U    // Match2 Tap Enable for Counter
                                               // 2
#define CLB_MISC_CONTROL_COUNT0_LFSR_EN  0x1000000U   // Enable LFSR mode for Counter
                                               // 0
#define CLB_MISC_CONTROL_COUNT1_LFSR_EN  0x2000000U   // Enable LFSR mode for Counter
                                               // 1
#define CLB_MISC_CONTROL_COUNT2_LFSR_EN  0x4000000U   // Enable LFSR mode for Counter
                                               // 2

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_EVENT_SEL register
//
//*****************************************************************************
#define CLB_HLC_EVENT_SEL_HLC_EVENT0_SEL_S  0U
#define CLB_HLC_EVENT_SEL_HLC_EVENT0_SEL_M  0x1FU        // Event Select 0
#define CLB_HLC_EVENT_SEL_HLC_EVENT1_SEL_S  5U
#define CLB_HLC_EVENT_SEL_HLC_EVENT1_SEL_M  0x3E0U       // Event Select 1
#define CLB_HLC_EVENT_SEL_HLC_EVENT2_SEL_S  10U
#define CLB_HLC_EVENT_SEL_HLC_EVENT2_SEL_M  0x7C00U      // Event Select 2
#define CLB_HLC_EVENT_SEL_HLC_EVENT3_SEL_S  15U
#define CLB_HLC_EVENT_SEL_HLC_EVENT3_SEL_M  0xF8000U     // Event Select 3
#define CLB_HLC_EVENT_SEL_HLC_ALT_EVENT0_SEL  0x100000U    // Event Select 3
#define CLB_HLC_EVENT_SEL_HLC_ALT_EVENT1_SEL  0x200000U    // Event Select 3
#define CLB_HLC_EVENT_SEL_HLC_ALT_EVENT2_SEL  0x400000U    // Event Select 3
#define CLB_HLC_EVENT_SEL_HLC_ALT_EVENT3_SEL  0x800000U    // Event Select 3

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_COUNT_MATCH_TAP_SEL register
//
//*****************************************************************************
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH1_TAP_SEL_S  0U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH1_TAP_SEL_M  0x1FU        // Match1 tap select for Counter
                                               // 0
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH1_TAP_SEL_S  5U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH1_TAP_SEL_M  0x3E0U       // Match1 tap select for Counter
                                               // 1
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH1_TAP_SEL_S  10U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH1_TAP_SEL_M  0x7C00U      // Match1 tap select for Counter
                                               // 2
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH2_TAP_SEL_S  16U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT0_MATCH2_TAP_SEL_M  0x1F0000U    // Match2 tap select for Counter
                                               // 0
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH2_TAP_SEL_S  21U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT1_MATCH2_TAP_SEL_M  0x3E00000U   // Match2 tap select for Counter
                                               // 1
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH2_TAP_SEL_S  26U
#define CLB_COUNT_MATCH_TAP_SEL_COUNT2_MATCH2_TAP_SEL_M  0x7C000000U  // Match2 tap select for Counter
                                               // 2

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_0 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_0_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_0_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_0_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_0_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_0_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_0_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_0_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_0_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_0_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_1 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_1_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_1_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_1_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_1_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_1_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_1_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_1_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_1_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_1_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_2 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_2_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_2_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_2_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_2_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_2_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_2_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_2_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_2_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_2_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_3 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_3_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_3_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_3_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_3_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_3_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_3_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_3_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_3_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_3_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_4 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_4_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_4_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_4_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_4_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_4_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_4_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_4_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_4_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_4_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_5 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_5_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_5_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_5_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_5_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_5_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_5_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_5_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_5_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_5_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_6 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_6_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_6_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_6_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_6_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_6_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_6_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_6_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_6_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_6_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_OUTPUT_COND_CTRL_7 register
//
//*****************************************************************************
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_1_SEL  0x1U         // Level 1 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_2_SEL_S  1U
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_2_SEL_M  0x6U         // Level 2 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_3_SEL_S  3U
#define CLB_OUTPUT_COND_CTRL_7_LEVEL_3_SEL_M  0x18U        // Level 3 Mux Select
#define CLB_OUTPUT_COND_CTRL_7_SEL_GATING_CTRL_S  5U
#define CLB_OUTPUT_COND_CTRL_7_SEL_GATING_CTRL_M  0xE0U        // Gating control mux select
#define CLB_OUTPUT_COND_CTRL_7_SEL_RELEASE_CTRL_S  8U
#define CLB_OUTPUT_COND_CTRL_7_SEL_RELEASE_CTRL_M  0x700U       // Releast control mux select
#define CLB_OUTPUT_COND_CTRL_7_HW_GATING_CTRL_SEL  0x800U       // Select HW for gating control
#define CLB_OUTPUT_COND_CTRL_7_HW_RLS_CTRL_SEL  0x1000U      // Select HW for release control
#define CLB_OUTPUT_COND_CTRL_7_SEL_RAW_IN  0x2000U      // Select Input for
#define CLB_OUTPUT_COND_CTRL_7_ASYNC_COND_EN  0x4000U      // Enable for conditioning

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_MISC_ACCESS_CTRL register
//
//*****************************************************************************
#define CLB_MISC_ACCESS_CTRL_CLB_SPI_BUFFER_EN  0x1U         // Enable CLB SPI Buffer feature
#define CLB_MISC_ACCESS_CTRL_BLOCK_REG_WRITE_0  0x2U         // Block Register write

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_SPI_DATA_CTRL_HI register
//
//*****************************************************************************
#define CLB_SPI_DATA_CTRL_HI_SPI_STROBE_SEL_S  0U
#define CLB_SPI_DATA_CTRL_HI_SPI_STROBE_SEL_M  0x1FU        // Select value for strobe
#define CLB_SPI_DATA_CTRL_HI_DATA_SHIFT_VALUE_S  8U
#define CLB_SPI_DATA_CTRL_HI_DATA_SHIFT_VALUE_M  0x1F00U      // Shift value select

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_LOAD_EN register
//
//*****************************************************************************
#define CLB_LOAD_EN_LOAD_EN       0x1U         // Load Enable
#define CLB_LOAD_EN_GLOBAL_EN     0x2U         // Global Enable
#define CLB_LOAD_EN_STOP          0x4U         // Debug stop control
#define CLB_LOAD_EN_NMI_EN        0x8U         // NMI output enable
#define CLB_LOAD_EN_PIPELINE_EN   0x10U        // Enable input pipelining

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_LOAD_ADDR register
//
//*****************************************************************************
#define CLB_LOAD_ADDR_ADDRESS_S   0U
#define CLB_LOAD_ADDR_ADDRESS_M   0x3FU        // Indirect Address

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_INPUT_FILTER register
//
//*****************************************************************************
#define CLB_INPUT_FILTER_FILTIN0_S  0U
#define CLB_INPUT_FILTER_FILTIN0_M  0x3U         // Input filter control 0
#define CLB_INPUT_FILTER_FILTIN1_S  2U
#define CLB_INPUT_FILTER_FILTIN1_M  0xCU         // Input filter control 1
#define CLB_INPUT_FILTER_FILTIN2_S  4U
#define CLB_INPUT_FILTER_FILTIN2_M  0x30U        // Input filter control 2
#define CLB_INPUT_FILTER_FILTIN3_S  6U
#define CLB_INPUT_FILTER_FILTIN3_M  0xC0U        // Input filter control 3
#define CLB_INPUT_FILTER_FILTIN4_S  8U
#define CLB_INPUT_FILTER_FILTIN4_M  0x300U       // Input filter control 4
#define CLB_INPUT_FILTER_FILTIN5_S  10U
#define CLB_INPUT_FILTER_FILTIN5_M  0xC00U       // Input filter control 5
#define CLB_INPUT_FILTER_FILTIN6_S  12U
#define CLB_INPUT_FILTER_FILTIN6_M  0x3000U      // Input filter control 6
#define CLB_INPUT_FILTER_FILTIN7_S  14U
#define CLB_INPUT_FILTER_FILTIN7_M  0xC000U      // Input filter control 7
#define CLB_INPUT_FILTER_SYNCSEL0  0x10000U     // Synchronizer control 0
#define CLB_INPUT_FILTER_SYNCSEL1  0x20000U     // Synchronizer control 1
#define CLB_INPUT_FILTER_SYNCSEL2  0x40000U     // Synchronizer control 2
#define CLB_INPUT_FILTER_SYNCSEL3  0x80000U     // Synchronizer control 3
#define CLB_INPUT_FILTER_SYNCSEL4  0x100000U    // Synchronizer control 4
#define CLB_INPUT_FILTER_SYNCSEL5  0x200000U    // Synchronizer control 5
#define CLB_INPUT_FILTER_SYNCSEL6  0x400000U    // Synchronizer control 6
#define CLB_INPUT_FILTER_SYNCSEL7  0x800000U    // Synchronizer control 7
#define CLB_INPUT_FILTER_PIPESEL0  0x1000000U   // Enable pipeline 0
#define CLB_INPUT_FILTER_PIPESEL1  0x2000000U   // Enable pipeline 1
#define CLB_INPUT_FILTER_PIPESEL2  0x4000000U   // Enable pipeline 2
#define CLB_INPUT_FILTER_PIPESEL3  0x8000000U   // Enable pipeline 3
#define CLB_INPUT_FILTER_PIPESEL4  0x10000000U  // Enable pipeline 4
#define CLB_INPUT_FILTER_PIPESEL5  0x20000000U  // Enable pipeline 5
#define CLB_INPUT_FILTER_PIPESEL6  0x40000000U  // Enable pipeline 6
#define CLB_INPUT_FILTER_PIPESEL7  0x80000000U  // Enable pipeline 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_IN_SEL_0 register
//
//*****************************************************************************
#define CLB_IN_SEL_0_SEL_GP_IN_0  0x1U         // Select GP register 0
#define CLB_IN_SEL_0_SEL_GP_IN_1  0x2U         // Select GP register 1
#define CLB_IN_SEL_0_SEL_GP_IN_2  0x4U         // Select GP register 2
#define CLB_IN_SEL_0_SEL_GP_IN_3  0x8U         // Select GP register 3
#define CLB_IN_SEL_0_SEL_GP_IN_4  0x10U        // Select GP register 4
#define CLB_IN_SEL_0_SEL_GP_IN_5  0x20U        // Select GP register 5
#define CLB_IN_SEL_0_SEL_GP_IN_6  0x40U        // Select GP register 6
#define CLB_IN_SEL_0_SEL_GP_IN_7  0x80U        // Select GP register 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_LCL_MUX_SEL_1 register
//
//*****************************************************************************
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_S  0U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_0_M  0x1FU        // Local Mux select 0
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_S  5U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_1_M  0x3E0U       // Local Mux select 1
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_2_S  10U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_2_M  0x7C00U      // Local Mux select 2
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_3_S  15U
#define CLB_LCL_MUX_SEL_1_LCL_MUX_SEL_IN_3_M  0xF8000U     // Local Mux select 3
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_0  0x10000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_1  0x20000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_2  0x40000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_1_MISC_INPUT_SEL_3  0x80000000U  // Select MISC_INPUT

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_LCL_MUX_SEL_2 register
//
//*****************************************************************************
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_S  0U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_4_M  0x1FU        // Local Mux select 4
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_S  5U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_5_M  0x3E0U       // Local Mux select 5
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_6_S  10U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_6_M  0x7C00U      // Local Mux select 6
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_7_S  15U
#define CLB_LCL_MUX_SEL_2_LCL_MUX_SEL_IN_7_M  0xF8000U     // Local Mux select 7
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_4  0x10000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_5  0x20000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_6  0x40000000U  // Select MISC_INPUT
#define CLB_LCL_MUX_SEL_2_MISC_INPUT_SEL_7  0x80000000U  // Select MISC_INPUT

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_BUF_PTR register
//
//*****************************************************************************
#define CLB_BUF_PTR_PULL_DATA_PTR_S  0U
#define CLB_BUF_PTR_PULL_DATA_PTR_M  0xFFU        // Data pointer for pull
#define CLB_BUF_PTR_PUSH_DATA_PTR_S  16U
#define CLB_BUF_PTR_PUSH_DATA_PTR_M  0xFF0000U    // Data pointer for pull

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_GP_REG register
//
//*****************************************************************************
#define CLB_GP_REG_GP_REG_0       0x1U         // General Purpose bit 0
#define CLB_GP_REG_GP_REG_1       0x2U         // General Purpose bit 1
#define CLB_GP_REG_GP_REG_2       0x4U         // General Purpose bit 2
#define CLB_GP_REG_GP_REG_3       0x8U         // General Purpose bit 3
#define CLB_GP_REG_GP_REG_4       0x10U        // General Purpose bit 4
#define CLB_GP_REG_GP_REG_5       0x20U        // General Purpose bit 5
#define CLB_GP_REG_GP_REG_6       0x40U        // General Purpose bit 6
#define CLB_GP_REG_GP_REG_7       0x80U        // General Purpose bit 7
#define CLB_GP_REG_OC_SW_GATING_0  0x10000U     // Software gating value for
                                               // output 0
#define CLB_GP_REG_OC_SW_GATING_1  0x20000U     // Software gating value for
                                               // output 1
#define CLB_GP_REG_OC_SW_GATING_2  0x40000U     // Software gating value for
                                               // output 2
#define CLB_GP_REG_OC_SW_GATING_3  0x80000U     // Software gating value for
                                               // output 3
#define CLB_GP_REG_OC_SW_GATING_4  0x100000U    // Software gating value for
                                               // output 4
#define CLB_GP_REG_OC_SW_GATING_5  0x200000U    // Software gating value for
                                               // output 5
#define CLB_GP_REG_OC_SW_GATING_6  0x400000U    // Software gating value for
                                               // output 6
#define CLB_GP_REG_OC_SW_GATING_7  0x800000U    // Software gating value for
                                               // output 7
#define CLB_GP_REG_OC_SW_RLS_0    0x1000000U   // Software release control for
                                               // output 0
#define CLB_GP_REG_OC_SW_RLS_1    0x2000000U   // Software release control for
                                               // output 1
#define CLB_GP_REG_OC_SW_RLS_2    0x4000000U   // Software release control for
                                               // output 2
#define CLB_GP_REG_OC_SW_RLS_3    0x8000000U   // Software release control for
                                               // output 3
#define CLB_GP_REG_OC_SW_RLS_4    0x10000000U  // Software release control for
                                               // output 4
#define CLB_GP_REG_OC_SW_RLS_5    0x20000000U  // Software release control for
                                               // output 5
#define CLB_GP_REG_OC_SW_RLS_6    0x40000000U  // Software release control for
                                               // output 6
#define CLB_GP_REG_OC_SW_RLS_7    0x80000000U  // Software release control for
                                               // output 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_GLBL_MUX_SEL_1 register
//
//*****************************************************************************
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_S  0U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_0_M  0x7FU        // Global Mux select 0
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_S  7U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_1_M  0x3F80U      // Global Mux select 1
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_2_S  14U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_2_M  0x1FC000U    // Global Mux select 2
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_3_S  21U
#define CLB_GLBL_MUX_SEL_1_GLBL_MUX_SEL_IN_3_M  0xFE00000U   // Global Mux select 3

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_GLBL_MUX_SEL_2 register
//
//*****************************************************************************
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_4_S  0U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_4_M  0x7FU        // Global Mux select 4
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_5_S  7U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_5_M  0x3F80U      // Global Mux select 5
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_6_S  14U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_6_M  0x1FC000U    // Global Mux select 6
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_7_S  21U
#define CLB_GLBL_MUX_SEL_2_GLBL_MUX_SEL_IN_7_M  0xFE00000U   // Global Mux select 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_PRESCALE_CTRL register
//
//*****************************************************************************
#define CLB_PRESCALE_CTRL_PRESCALE_ENABLE  0x1U         // Enable the prescale clock
                                               // generator
#define CLB_PRESCALE_CTRL_STROBE_MODE  0x2U         // Enable the Strobe mode of
                                               // operation
#define CLB_PRESCALE_CTRL_TAP_SELECT_VALUE_S  2U
#define CLB_PRESCALE_CTRL_TAP_SELECT_VALUE_M  0x3CU        // TAP Select value
#define CLB_PRESCALE_CTRL_PRESCALE_VALUE_S  16U
#define CLB_PRESCALE_CTRL_PRESCALE_VALUE_M  0xFFFF0000U  // Value of prescale register

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_LOCK register
//
//*****************************************************************************
#define CLB_LOCK_LOCK             0x1U         // LOCK enable
#define CLB_LOCK_WRITE_ENABLE_KEY_S  16U
#define CLB_LOCK_WRITE_ENABLE_KEY_M  0xFFFF0000U  // Key for enabling write

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_INSTR_READ_PTR register
//
//*****************************************************************************
#define CLB_HLC_INSTR_READ_PTR_HLC_READ_PTR_S  0U
#define CLB_HLC_INSTR_READ_PTR_HLC_READ_PTR_M  0x1FU        // HLC instruction read pointer

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_HLC_INSTR_VALUE register
//
//*****************************************************************************
#define CLB_HLC_INSTR_VALUE_HLC_INSTR_VALUE_S  0U
#define CLB_HLC_INSTR_VALUE_HLC_INSTR_VALUE_M  0xFFFU       // HLC instruction value

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_DBG_OUT_2 register
//
//*****************************************************************************
#define CLB_DBG_OUT_2_FINAL_OUT_0  0x1U         // Output of CLB Async block bit
                                               // 0
#define CLB_DBG_OUT_2_FINAL_OUT_1  0x2U         // Output of CLB Async block bit
                                               // 1
#define CLB_DBG_OUT_2_FINAL_OUT_2  0x4U         // Output of CLB Async block bit
                                               // 2
#define CLB_DBG_OUT_2_FINAL_OUT_3  0x8U         // Output of CLB Async block bit
                                               // 3
#define CLB_DBG_OUT_2_FINAL_OUT_4  0x10U        // Output of CLB Async block bit
                                               // 4
#define CLB_DBG_OUT_2_FINAL_OUT_5  0x20U        // Output of CLB Async block bit
                                               // 5
#define CLB_DBG_OUT_2_FINAL_OUT_6  0x40U        // Output of CLB Async block bit
                                               // 6
#define CLB_DBG_OUT_2_FINAL_OUT_7  0x80U        // Output of CLB Async block bit
                                               // 7
#define CLB_DBG_OUT_2_CLB_IN_0    0x100U       // CLB CELL Input 0
#define CLB_DBG_OUT_2_CLB_IN_1    0x200U       // CLB CELL Input 1
#define CLB_DBG_OUT_2_CLB_IN_2    0x400U       // CLB CELL Input 2
#define CLB_DBG_OUT_2_CLB_IN_3    0x800U       // CLB CELL Input 3
#define CLB_DBG_OUT_2_CLB_IN_4    0x1000U      // CLB CELL Input 4
#define CLB_DBG_OUT_2_CLB_IN_5    0x2000U      // CLB CELL Input 5
#define CLB_DBG_OUT_2_CLB_IN_6    0x4000U      // CLB CELL Input 6
#define CLB_DBG_OUT_2_CLB_IN_7    0x8000U      // CLB CELL Input 7

//*****************************************************************************
//
// The following are defines for the bit fields in the CLB_DBG_OUT register
//
//*****************************************************************************
#define CLB_DBG_OUT_COUNT0_MATCH2  0x2U         // COUNT_MATCH2 UNIT 0
#define CLB_DBG_OUT_COUNT0_ZERO   0x4U         // COUNT_ZERO UNIT 0
#define CLB_DBG_OUT_COUNT0_MATCH1  0x8U         // COUNT_MATCH1 UNIT 0
#define CLB_DBG_OUT_FSM0_S0       0x10U        // FSM_S0 UNIT 0
#define CLB_DBG_OUT_FSM0_S1       0x20U        // FSM_S1 UNIT 0
#define CLB_DBG_OUT_FSM0_LUTOUT   0x40U        // FSM_LUT_OUT UNIT 0
#define CLB_DBG_OUT_LUT40_OUT     0x80U        // LUT4_OUT UNIT 0
#define CLB_DBG_OUT_COUNT1_MATCH2  0x200U       // COUNT_MATCH2 UNIT 1
#define CLB_DBG_OUT_COUNT1_ZERO   0x400U       // COUNT_ZERO UNIT 1
#define CLB_DBG_OUT_COUNT1_MATCH1  0x800U       // COUNT_MATCH1 UNIT 1
#define CLB_DBG_OUT_FSM1_S0       0x1000U      // FSM_S0 UNIT 1
#define CLB_DBG_OUT_FSM1_S1       0x2000U      // FSM_S1 UNIT 1
#define CLB_DBG_OUT_FSM1_LUTOUT   0x4000U      // FSM_LUT_OUT UNIT 1
#define CLB_DBG_OUT_LUT41_OUT     0x8000U      // LUT4_OUT UNIT 1
#define CLB_DBG_OUT_COUNT2_MATCH2  0x20000U     // COUNT_MATCH2 UNIT 2
#define CLB_DBG_OUT_COUNT2_ZERO   0x40000U     // COUNT_ZERO UNIT 2
#define CLB_DBG_OUT_COUNT2_MATCH1  0x80000U     // COUNT_MATCH1 UNIT 2
#define CLB_DBG_OUT_FSM2_S0       0x100000U    // FSM_S0 UNIT 2
#define CLB_DBG_OUT_FSM2_S1       0x200000U    // FSM_S1 UNIT 2
#define CLB_DBG_OUT_FSM2_LUTOUT   0x400000U    // FSM_LUT_OUT UNIT 2
#define CLB_DBG_OUT_LUT42_OUT     0x800000U    // LUT4_OUT UNIT 2
#define CLB_DBG_OUT_OUT0          0x1000000U   // CELL Output 0
#define CLB_DBG_OUT_OUT1          0x2000000U   // CELL Output 1
#define CLB_DBG_OUT_OUT2          0x4000000U   // CELL Output 2
#define CLB_DBG_OUT_OUT3          0x8000000U   // CELL Output 3
#define CLB_DBG_OUT_OUT4          0x10000000U  // CELL Output 4
#define CLB_DBG_OUT_OUT5          0x20000000U  // CELL Output 5
#define CLB_DBG_OUT_OUT6          0x40000000U  // CELL Output 6
#define CLB_DBG_OUT_OUT7          0x80000000U  // CELL Output 7
#endif
