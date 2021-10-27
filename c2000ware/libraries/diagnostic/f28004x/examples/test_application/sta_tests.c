//#############################################################################
//
// FILE:  sta_tests.c
//
// TITLE: Self Test Application Tests source
//
//#############################################################################
// $TI Release: C2000 Diagnostic Library v2.01.00 $
// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
// Included Files
//
#include "driverlib.h"
#include "device.h"

#include "sta_tests.h"
#include "sta_user.h"
#include "sta_util.h"
#include "sta_comm.h"
#include "sta_timer.h"
#include "stl_util.h"

#include <stdio.h>

//
// Defines
//
#define STA_TESTS_REPORT_SIZE       29
#define STA_TESTS_REPORT_TOTAL      15
#define STA_TESTS_REPORT_PASS       5

//
// Globals
//
const STA_TestsTypes STA_Tests_testArray[STA_TESTS_NUMBERS] =
{
    STA_TEST_START,
    STA_TEST_PIE_RAM,
    STA_TEST_PIE_HANDLER,
    STA_CPU_REG,
    STA_FPU_REG,
    STA_VCU_REG,
    STA_MARCH,
    STA_MARCH_COPY,
    STA_MARCH_CAN,
    STA_MARCH_CAN_COPY,
    STA_CAN_RAM_PARITY,
    STA_FLASH_CRC,
    STA_OSC_CT,
    STA_OSC_HR,
    STA_TEST_END
};

#if STA_UTIL_PROFILE
uint32_t STA_Tests_cycleCounts[STA_TESTS_NUMBERS] = {0};
#endif

static uint16_t STA_Tests_passCount = 0U;
static unsigned char STA_Tests_report[STA_TESTS_REPORT_SIZE] =
    {
        '\r','\n','\n','\n',' ','X',
        'X',' ','o','u','t',' ','o',
        'f',' ','X','X',' ','t','e',
        's','t','s',' ','P','A','S',
        'S','\0'
    };

static bool STA_Tests_injectError;

//*****************************************************************************
//
// STA_Tests_testDevice(STA_TestsTypes testItem)
//
//*****************************************************************************
unsigned char* STA_Tests_testDevice(STA_TestsTypes testItem)
{

    unsigned char *testReport;

    switch(testItem)
    {
        case STA_TEST_START:
        {
            testReport = "\r\n\n\n Starting Test Loop\0";

            STA_Tests_passCount = 0U;

            break;
        }

        case STA_TEST_PIE_RAM:
        {
            uint16_t entry = STA_USER_PIE_RAM_ENTRY;

            if(STA_Tests_injectError)
            {
                STL_PIE_RAM_injectFault(entry);
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif
            //
            // Compare the PIE RAM against the redundant PIE RAM.
            //
            uint16_t returnValue = STL_PIE_RAM_testRAM();

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_TEST_PIE_RAM] = cycleCount;
#endif

            if(STL_PIE_RAM_PASS == returnValue)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: PIE RAM Test!\0";
            }
            else if(returnValue == entry + PIEVECTTABLE_BASE)
            {
                //
                // This is expected when a fault is injected.
                //
                testReport = "\r\n\n\n TEST FAIL: PIE RAM Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n Unexpected Behavior: PIE RAM Test!\0";
            }

            //
            // Restore single vector table entry where error injection was used
            //
            if(STA_Tests_injectError)
            {
                STL_PIE_RAM_restoreVector(entry);
            }

            break;
        }

        case STA_TEST_PIE_HANDLER:
        {
            uint16_t intGroup =
                ((uint16_t)(STA_USER_PIE_TEST_INT & STL_PIE_RAM_TABLE_COL_M) >>
                 STL_PIE_RAM_TABLE_COL_S) - 1U;
            uint16_t intMask = 1U << ((uint16_t)(STA_USER_PIE_TEST_INT &
                                                 STL_PIE_RAM_TABLE_ROW_M) - 1U);

            //
            // This test itself is injecting a fault and verifying that the
            // safety mechanism is working properly, but it can be forced to
            // fail by disabling interrupts and preventing the vector from being
            // fetched.
            //
            if(STA_Tests_injectError)
            {
                DINT;
            }

            //
            // Inject and error in the PIE RAM redundant table and generate
            // an interrupt for that entry. This will test the PIEVERRADDR
            // and PIE RAM mismatch functionality.
            //
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal = STL_PIE_RAM_testHandler(STA_USER_PIE_TEST_INT);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_TEST_PIE_HANDLER] = cycleCount;
#endif
            if(STL_PIE_RAM_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: PIE RAM Handler Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: PIE RAM Handler Test!\0";
            }

            if(STA_Tests_injectError)
            {
                //
                // Unforce the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) &= ~intMask;

                //
                // Disable the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) &= ~intMask;
                Interrupt_clearACKGroup(STA_USER_PIE_TEST_INT_GROUP_M);

                //
                // Unforce the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IFR1 + (intGroup * 2U)) &= ~intMask;

                //
                // Disable the PIE interrupt.
                //
                HWREGH(PIECTRL_BASE + PIE_O_IER1 + (intGroup * 2U)) &= ~intMask;

                STL_Util_delayUS(5);

                IFR &= ~STA_USER_PIE_TEST_INT_GROUP_M;

                STL_Util_delayUS(5);
            }

            EINT;

            break;
        }

        case STA_CPU_REG:
        {
            //
            // It's recommended to disable interrupts during this test.
            //
            DINT;
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal =
                    STL_CPU_REG_checkCPURegisters(STA_Tests_injectError);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_CPU_REG] = cycleCount;
#endif
            EINT;

            if(STL_CPU_REG_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: CPU Register Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: CPU Register Test!\0";
            }

            break;
        }

        case STA_FPU_REG:
        {
            //
            // It's recommended to disable interrupts during this test.
            //
            DINT;
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal =
                    STL_CPU_REG_checkFPURegisters(STA_Tests_injectError);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_FPU_REG] = cycleCount;
#endif
            EINT;

            if(STL_CPU_REG_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: FPU Register Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: FPU Register Test!\0";
            }

            break;
        }

        case STA_VCU_REG:
        {
            //
            // It's recommended to disable interrupts during this test.
            //
            DINT;
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal =
                    STL_CPU_REG_checkVCURegisters(STA_Tests_injectError);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_VCU_REG] = cycleCount;
#endif
            EINT;

            if(STL_CPU_REG_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: VCU Register Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: VCU Register Test!\0";
            }

            break;
        }

        case STA_MARCH:
        {
            //
            // Clear RAM error status flags.
            //
                MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD |
                                              MEMCFG_UCERR_DMAREAD |
                                              MEMCFG_UCERR_CLA1READ);
                MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD |
                                            MEMCFG_CERR_DMAREAD |
                                            MEMCFG_CERR_CLA1READ);

            //
            // Initialize the inject error handle for stl_march.
            //
            STA_User_initMarch();

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                //
                // This test should always PASS. Cannot inject error in middle
                // of March13N test since the test starts with a write that
                // triggers an update of the ECC.
                //
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            STL_March_testRAM(STL_MARCH_PATTERN_TWO,
                              (uint32_t)STA_User_marchTestData,
                              (STA_USER_MARCH_DATA_SIZE / 2U) - 1U);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_MARCH] = cycleCount;
#endif

            uint16_t returnVal = STL_March_checkErrorStatus();

            //
            // Clear RAM error status flags.
            //
                MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD |
                                              MEMCFG_UCERR_DMAREAD |
                                              MEMCFG_UCERR_CLA1READ);
                MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD |
                                            MEMCFG_CERR_DMAREAD |
                                            MEMCFG_CERR_CLA1READ);

            if(STL_MARCH_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: March13N No Copy Test!\0";
            }
            else
            {
                //
                // Since there is no way to inject an error for this test,
                // under normal circumstances, this code will not be reached.
                //
                testReport = "\r\n\n\n TEST FAIL: March13N No Copy Test!\0";
            }
            break;
        }

        case STA_MARCH_COPY:
        {
            uint32_t originalValue;
            uint16_t index;

            //
            // Filling STA_User_marchTestData to make the copy easier to
            // observe.
            //
            for(index = 0; index < STA_USER_MARCH_DATA_SIZE; index++)
            {
                STA_User_marchTestData[index] = index;
            }

            //
            // Clear RAM error status flags.
            //
                MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD |
                                              MEMCFG_UCERR_DMAREAD |
                                              MEMCFG_UCERR_CLA1READ);
                MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD |
                                            MEMCFG_CERR_DMAREAD |
                                            MEMCFG_CERR_CLA1READ);

            //
            // Initialize the inject error handle for stl_march.
            //
            STA_User_initMarch();

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                //
                // Cause a single bit error in M0 (ECC, correctable)
                //
                originalValue = HWREG(STA_User_marchErrorObj.address);
                STA_User_marchErrorObj.testMode = MEMCFG_TEST_WRITE_DATA;
                STA_User_marchErrorObj.xorMask = 0x00000001U;
                STL_March_injectError(STA_User_marchErrorHandle);
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            STL_March_testRAMCopy(STL_MARCH_PATTERN_ONE,
                                  (uint32_t)STA_User_marchTestData,
                                  (STA_USER_MARCH_DATA_SIZE / 2U) - 1U,
                                  (uint32_t)STA_User_marchTestDataCopy);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_MARCH_COPY] = cycleCount;
#endif

            uint16_t returnVal = STL_March_checkErrorStatus();

            //
            // Restore value and clear status flags if error was injected.
            //
            if(STA_Tests_injectError)
            {
                HWREG(STA_User_marchErrorObj.address) = originalValue;
                MemCfg_clearUncorrErrorStatus(MEMCFG_UCERR_CPUREAD |
                                              MEMCFG_UCERR_DMAREAD |
                                              MEMCFG_UCERR_CLA1READ);
                MemCfg_clearCorrErrorStatus(MEMCFG_CERR_CPUREAD |
                                            MEMCFG_CERR_DMAREAD |
                                            MEMCFG_CERR_CLA1READ);
            }

            if(STL_MARCH_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: March13N Copy Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: March13N Copy Test!\0";
            }
            break;
        }

        case STA_MARCH_CAN:
        {
            //
            // Initialize the message RAM.
            //
            CAN_initRAM(CANA_BASE);

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                //
                // This test should always PASS. Cannot inject error in middle
                // of March13N test since the test starts with a write that
                // triggers an update of the parity.
                //
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif
            //
            // Run March test on 5 message objects starting with object 32
            // (which is also the "0th" spot in the message RAM).
            //
            STL_CAN_RAM_testRAM(CANA_BASE, STL_MARCH_PATTERN_FOUR,
                                STA_USER_MARCH_CANA_OBJ32_START_ADDR,
                                STA_USER_MARCH_CANA_OBJ4_END_ADDR,
                                STL_CAN_RAM_NO_COPY);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_MARCH_CAN] = cycleCount;
#endif

            uint16_t returnVal = STL_CAN_RAM_checkErrorStatus(CANA_BASE);

            if(STL_CAN_RAM_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: March13N CAN Message RAM Test!\0";
            }
            else
            {
                //
                // Since there is no way to inject an error for this test,
                // under normal circumstances, this code will not be reached.
                //
                testReport = "\r\n\n\n TEST FAIL: March13N CAN Message RAM Test!\0";
            }
            break;
        }

        case STA_MARCH_CAN_COPY:
        {
            uint16_t index, returnVal;

            //
            // Filling message objects to make the copy easier to observe.
            //
            CAN_enableTestMode(CANA_BASE, 0);
            CAN_enableMemoryAccessMode(CANA_BASE);

            for(index = 0; index < STA_USER_MARCH_CAN_COPY_SIZE; index += 2)
            {
                HWREGH(STA_USER_MARCH_CANA_OBJ6_START_ADDR + index) = index;
            }

            CAN_disableMemoryAccessMode(CANA_BASE);
            CAN_disableTestMode(CANA_BASE);

            //
            // Enable parity error to trigger an interrupt.
            //
            CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
            CAN_enableInterrupt(CANA_BASE, CAN_INT_ERROR | CAN_INT_IE0);
            CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

            STA_User_canInterruptFlag = false;
            STA_User_parityErrorCode = 0;

            Interrupt_register(INT_CANA0, &STA_User_canParityErrorISR);
            Interrupt_enable(INT_CANA0);

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                //
                // Cause a single bit error in message object 6.
                //
                STL_CAN_RAM_injectError(CANA_BASE,
                                        STA_USER_MARCH_CANA_OBJ6_START_ADDR,
                                        0x00000001U);

                //
                // When injecting an error expect this variable to get set to
                // fail in the ISR, so initialize to PASS.
                //
                STA_User_canErrorReturnVal = STL_CAN_RAM_PASS;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif
            //
            // Run March test on 10 message objects starting with object 6.
            //
            STL_CAN_RAM_testRAM(CANA_BASE, STL_MARCH_PATTERN_THREE,
                                STA_USER_MARCH_CANA_OBJ6_START_ADDR,
                                STA_USER_MARCH_CANA_OBJ15_END_ADDR,
                                (uint32_t)STA_User_marchTestDataCANCopy);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_MARCH_CAN_COPY] = cycleCount;
#endif

            if(!STA_Tests_injectError)
            {
                returnVal = STL_CAN_RAM_checkErrorStatus(CANA_BASE);
            }
            else
            {
                //
                // Interrupt should have had time to fire and set
                // STA_User_canErrorReturnVal by now.
                //
                returnVal = STA_User_canErrorReturnVal;
            }

            if(STL_CAN_RAM_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: March13N CAN Message RAM Copy Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: March13N CAN Message RAM Copy Test!\0";
            }
            break;
        }

        case STA_CAN_RAM_PARITY:
        {
            uint16_t returnVal;

            //
            // This example performs a diagnostic of the parity logic by simply
            // injecting a parity error and detecting it by reading the location
            // of the error.

            //
            // Enable parity error to trigger an interrupt.
            //
            CAN_clearGlobalInterruptStatus(CANA_BASE, CAN_GLOBAL_INT_CANINT0);
            CAN_enableInterrupt(CANA_BASE, CAN_INT_ERROR | CAN_INT_IE0);
            CAN_enableGlobalInterrupt(CANA_BASE, CAN_GLOBAL_INT_CANINT0);

            STA_User_canInterruptFlag = false;
            STA_User_parityErrorCode = 0;

            Interrupt_register(INT_CANA0, &STA_User_canParityErrorISR);
            Interrupt_enable(INT_CANA0);

            //
            // Make sure parity is enabled.
            //
            HWREGH(CANA_BASE + CAN_O_CTL) &= ~CAN_CTL_PMD_M;

            //
            // Inject an error in the memory.
            //
            if(STA_Tests_injectError)
            {
                //
                // Cause a single bit error in message object 6.
                //
                STL_CAN_RAM_injectError(CANA_BASE,
                                        STA_USER_MARCH_CANA_OBJ6_START_ADDR,
                                        0x00000001U);

                //
                // When injecting an error expect this variable to get set to
                // fail in the ISR, so initialize to PASS.
                //
                STA_User_canErrorReturnVal = STL_CAN_RAM_PASS;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif
            //
            // Transfer message from message object 6 to interface 1. This
            // will detect the parity error if one exists.
            //
            CAN_transferMessage(CANA_BASE, 1U, 6U, false, false);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_CAN_RAM_PARITY] = cycleCount;
#endif

            if(!STA_Tests_injectError)
            {
                returnVal = STL_CAN_RAM_checkErrorStatus(CANA_BASE);
            }
            else
            {
                //
                // Interrupt should have had time to fire and set
                // STA_User_canErrorReturnVal by now.
                //
                returnVal = STA_User_canErrorReturnVal;
            }

            if(STL_CAN_RAM_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: CAN Message RAM Parity Logic Test!\0";            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: CAN Message RAM Parity Logic Test!\0";
            }
            break;
        }

        case STA_FLASH_CRC:
        {
            //
            // Set range to calculate CRC for and the golden CRC value. For this
            // example, the first several words of the DCSM OTP are used.
            //
            uint32_t startAddress = DCSMBANK0_Z1OTP_BASE;
            uint32_t endAddress = DCSMBANK0_Z1OTP_BASE + 16;
            uint32_t goldenCRC = STA_USER_DCSM_OTP_GOLDEN_CRC;

            if(STA_Tests_injectError)
            {
                goldenCRC = goldenCRC ^ 0x0001U;
            }
#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal = STL_CRC_checkCRC(startAddress,
                                                  endAddress,
                                                  goldenCRC);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_FLASH_CRC] = cycleCount;
#endif
            if(STL_CRC_PASS == returnVal)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: Flash CRC Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: Flash CRC Test!\0";
            }
            break;
        }

        case STA_OSC_CT:
        {
            //
            // Initialize the Oscillator Timer2 test object.
            //
            STA_User_initOSCTimer2Test();

            if(STA_Tests_injectError)
            {
                //
                // Allow for zero range.
                //
                STA_User_oscTimer2Obj.minCount = STA_USER_OSC_DELAY_US;
                STA_User_oscTimer2Obj.maxCount = STA_USER_OSC_DELAY_US;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            STL_OSC_CT_startTest(STA_User_oscTimer2Handle);

#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_OSC_CT] = cycleCount;
#endif

            STL_Util_delayUS(STA_USER_OSC_DELAY_US);

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif

            uint16_t returnVal =
                STL_OSC_CT_stopTest(STA_User_oscTimer2Handle);

#if STA_UTIL_PROFILE
            cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_OSC_CT] += cycleCount;
#endif
            //
            // Run the oscillator test using Timer2.
            //
            if(STL_OSC_CT_FAIL == returnVal)
            {
                testReport = "\r\n\n\n TEST FAIL: OSC TIMER 2 Test!\0";
            }
            else
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: OSC TIMER 2 Test!\0";
            }

            break;
        }

        case STA_OSC_HR:
        {
            uint16_t testStatus;
            STA_User_initOSCHRTest();

            //
            // Shorten the MEP bounds by changing the max value to inject an
            // error.
            //
            // Calculating the expected MEP scale factor based on step size
            // listed in the Technical Reference Manual if EPWMCLK is 100 MHz:
            // (1/EPWMCLK)/Step size = (1/100 MHz)/150 ps is about 66.
            // mepMin and mepMax should be set to provide a range of a minimum
            // count of 3 greater and lower than the calculated value.
            // Setting mepMax lower than 66-3 = 63 will inject an error in
            // this particular test.
            //
            if(STA_Tests_injectError)
            {
                STA_User_oscHRObj.mepMin = STA_USER_OSC_HR_MEP_MAX;
            }

#if STA_UTIL_PROFILE
            STA_Util_startProfiler(CPUTIMER1_BASE);
#endif
            testStatus = STL_OSC_HR_testSFO(STA_User_oscHRHandle);
#if STA_UTIL_PROFILE
            uint32_t cycleCount = STA_Util_stopProfiler(CPUTIMER1_BASE);
            STA_Tests_cycleCounts[STA_OSC_HR] = cycleCount;
#endif

            if(STL_OSC_HR_PASS == testStatus)
            {
                STA_Tests_passCount++;
                testReport = "\r\n\n\n TEST PASS: OSC HRPWM Test!\0";
            }
            else
            {
                testReport = "\r\n\n\n TEST FAIL: OSC HRPWM Test!\0";
            }
            break;
        }

        case STA_TEST_END:
        {
            //
            // Convert test counts to characters and insert into the report.
            //
            STA_Tests_report[STA_TESTS_REPORT_PASS] =
                                        (STA_Tests_passCount / 10) + 48U;
            STA_Tests_report[STA_TESTS_REPORT_PASS + 1] =
                                        (STA_Tests_passCount % 10) + 48U;
            STA_Tests_report[STA_TESTS_REPORT_TOTAL] =
                                        ((STA_TESTS_NUMBERS - 2U) / 10) + 48U;
            STA_Tests_report[STA_TESTS_REPORT_TOTAL + 1] =
                                        ((STA_TESTS_NUMBERS - 2U) % 10) + 48U;

            testReport = STA_Tests_report;

            break;
        }

        default:
        {
            testReport = "\r\n\n\n No Test Specified\0";

            break;
        }
    }

    return(testReport);
}

//*****************************************************************************
//
// STA_Tests_injectErrorEnable(void)
//
//*****************************************************************************
void STA_Tests_injectErrorEnable(void)
{
    STA_Tests_injectError = true;
}

//*****************************************************************************
//
// STA_Tests_injectErrorDisable(void)
//
//*****************************************************************************
void STA_Tests_injectErrorDisable(void)
{
    STA_Tests_injectError = false;
}

//
// End of File
//
