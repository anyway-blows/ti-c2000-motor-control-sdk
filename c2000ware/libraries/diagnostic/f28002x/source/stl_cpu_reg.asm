;;//###########################################################################
;;//
;;// FILE:  stl_cpu_reg.asm
;;//
;;// TITLE: Diagnostic Library CPU register test
;;//
;;//###########################################################################
;;// $TI Release: C2000 Diagnostic Library v2.01.00 $
;;// $Release Date: Fri Feb 12 19:23:23 IST 2021 $
;;// $Copyright:
;// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
;//
;// Redistribution and use in source and binary forms, with or without 
;// modification, are permitted provided that the following conditions 
;// are met:
;// 
;//   Redistributions of source code must retain the above copyright 
;//   notice, this list of conditions and the following disclaimer.
;// 
;//   Redistributions in binary form must reproduce the above copyright
;//   notice, this list of conditions and the following disclaimer in the 
;//   documentation and/or other materials provided with the   
;//   distribution.
;// 
;//   Neither the name of Texas Instruments Incorporated nor the names of
;//   its contributors may be used to endorse or promote products derived
;//   from this software without specific prior written permission.
;// 
;// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;// $
;;//###########################################################################

        .global STL_CPU_REG_testCPURegisters
        .global STL_CPU_REG_testFPURegisters
        .global STL_CPU_REG_testVCRCRegisters

;;*****************************************************************************
; Constants
;;*****************************************************************************
TEST_ITERATIONS         .set    0x1
RPC_TEST_MASK_MSB       .set    0x003F
ST1_CLR_TEST_MASK       .set    0xFF5B
ST1_SET_TEST_MASK       .set    0x0A00
STF_TEST_MASK_MSB       .set    0x0000
STF_TEST_MASK_LSB       .set    0x027C
VSTATUS_TEST_MASK_MSB   .set    0x8000
VSTATUS_TEST_MASK_LSB   .set    0x0000
VCRCSIZE_TEST_MASK_MSB  .set    0x001F
VCRCSIZE_TEST_MASK_LSB  .set    0x0007
REG_TEST_PATTERN        .set    0xAAAA
N_REG_TEST_PATTERN      .set    0x5555
REG_ERROR_PATTERN       .set    0x555D
TEST_PASSED             .set    0x0
TEST_FAILED             .set    0x1

    .text
;;*****************************************************************************
;;
;; uint16_t STL_CPU_TEST_testCPURegisters(bool injectError)
;;                                             AL
;;
;;*****************************************************************************
STL_CPU_REG_testCPURegisters:
    ;
    ; Push save-on-entry registers onto the stack
    ;
        PUSH    XAR1
        PUSH    XAR2
        PUSH    XAR3
    ;
    ; First test ACC register using opcode only
    ; ACC is used to test other registers
    ;
        MOV     AH, #REG_TEST_PATTERN

    ;
    ; See if the injectError parameter is set
    ; Skip error injection if not
    ;
        CMP     AL, #0x1                ; AL contains injectError parameter
        B       skipCPUErrorTest, NEQ
        MOV     AH, #REG_ERROR_PATTERN ; Write the error pattern to AH

skipCPUErrorTest:
        CMP     @AH, #REG_TEST_PATTERN
        B       failCPURegTest, NEQ      ; Test AH for pattern
        MOV     AL, #REG_TEST_PATTERN
        CMP     @AL, #REG_TEST_PATTERN
        B       failCPURegTest, NEQ      ; Test AL for pattern
        MOV     AH, #N_REG_TEST_PATTERN
        CMP     @AH, #N_REG_TEST_PATTERN
        B       failCPURegTest, NEQ      ; Test AH for pattern inverse
        MOV     AL, #N_REG_TEST_PATTERN
        CMP     @AL, #N_REG_TEST_PATTERN
        B       failCPURegTest, NEQ      ; Test AH for pattern inverse

    ;
    ; Second Test P register using ACC,
    ; P register is used as a loop counter
    ; P is used as a loop for other register tests
    ;
        MOVL    P, @ACC
        CMPL    ACC, @P
        B       failCPURegTest, NEQ      ; Test P for pattern inverse
        NOT     ACC
        MOVL    P, @ACC
        CMPL    ACC, @P
        B       failCPURegTest, NEQ      ; Test P for pattern

    ;
    ; Use PL as a loop counter
    ; Test pattern first then pattern inverse
    ;
        MOV     PL, #TEST_ITERATIONS

loopCPURegTest:
    ;
    ; Test Auxiliary Registers
    ;
        NOT     ACC
        MOVL    XAR0, @ACC
        MOVL    XAR1, @ACC
        MOVL    XAR2, @ACC
        MOVL    XAR3, @ACC
        MOVL    XAR4, @ACC
        MOVL    XAR5, @ACC
        MOVL    XAR6, @ACC
        MOVL    XAR7, @ACC

        CMPL    ACC, @XAR0
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR1
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR2
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR3
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR4
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR5
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR6
        B       failCPURegTest, NEQ
        CMPL    ACC, @XAR7
        B       failCPURegTest, NEQ

    ;
    ; Test Multiplicand register
    ;
        MOVL    XT, @ACC
        CMPL    ACC, @XT
        B       failCPURegTest, NEQ

        DEC     PL
        B       loopCPURegTest, GEQ

    ;
    ; Test stack pointer
    ;
        MOVZ    AR4, @SP                ; save SP contents in AR4
        MOV     @SP, AL
        CMP     AL, @SP
        B       failSPTest, NEQ
        NOT     ACC
        MOV     @SP, AL
        CMP     AL, @SP
        B       failSPTest, NEQ
        MOV     @SP, AR4
        B       rpcRegTest, UNC

failSPTest:
        MOV     @SP, AR4                ; try restoring stack pointer
        B       failCPURegTest, UNC

    ;
    ; Test return program counter
    ;
rpcRegTest:
        MOVL    XAR6, @ACC              ; save ACC contents

        AND     AH, #RPC_TEST_MASK_MSB  ; RPC is 22 bits

        PUSH    RPC
        POP     P                       ; save RPC in P

        PUSH    ACC
        POP     RPC                     ; write to RPC
        PUSH    RPC
        POP     XAR4
        CMPL    ACC, XAR4
        B       failRPCTest, NEQ

        NOT     ACC
        AND     AH, #RPC_TEST_MASK_MSB
        PUSH    ACC
        POP     RPC                     ; write to RPC
        PUSH    RPC
        POP     XAR4
        CMPL    ACC, XAR4
        B       failRPCTest, NEQ
        B       restoreRPC, UNC

failRPCTest:
        PUSH    P
        POP     RPC                     ; restore RPC
        B       failCPURegTest, UNC

restoreRPC:
        PUSH    P
        POP     RPC                     ; restore RPC
        MOVL    ACC, @XAR6              ; restore ACC contents

cpu16BitRegTest:
    ;
    ; Test ST0, ST1, DP, IER, IFR and DBGIER register
    ;
        PUSH    ST0
        POP     @PL                     ; save ST0 in PL
        PUSH    ST1
        POP     @AR3                    ; save ST1 in AR3
        PUSH    DP
        POP     @PH                     ; save DP in PH
        MOV     @AR0, IER               ; save IER in AR0
        PUSH    IFR
        POP     @TH                     ; save IFR in TH
        PUSH    DBGIER
        POP     @AR5                    ; save DBGIER in AR5

    ;
    ; Test ST0, IER, IFR, DBGIER and DP
    ;
        MOV     AR6, #TEST_ITERATIONS   ; use AR6 as a loop counter

loopCPUReg2Test:
    ;
    ; Note that these checks have a different fail branch
    ; Need to restore registers before the returning
    ;
        PUSH    @AL
        POP     DP                      ; write to DP
        PUSH    DP
        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        PUSH    @AL
        POP     ST0                     ; write to ST0
        PUSH    ST0

        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        PUSH    @AL
        POP     IFR                     ; write to IFR
        PUSH    IFR
        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        PUSH    @AL
        POP     DBGIER                  ; write to DBGIER
        PUSH    DBGIER
        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        MOV     IER, @AL                ; write to IER
        MOV     @AR4, IER
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        NOT     ACC
        DEC     AR6                     ; decrement loop counter
        B       loopCPUReg2Test, GEQ;

    ;
    ; Test ST1. Treatment of each bit is as follows:
    ;
    ; ARP       15:13   Tested
    ; XF        12      Tested
    ; M0M1MAP   11      Always 1 for C28x object mode
    ; rsvd      10      Tested
    ; OBJMODE   9       Always 1 for C28x object mode
    ; AMODE     8       Tested
    ; IDLESTAT  7       NOT tested
    ; EALLOW    6       Tested
    ; LOOP      5       NOT tested
    ; SPA       4       Tested
    ; VMAP      3       Tested
    ; PAGE0     2       Always 1 for suggested C28x operating mode
    ; DBGM      1       Tested
    ; INTM      0       Tested
    ;
st1RegTest:
        MOVL    XAR6, @ACC              ; save ACC contents
        AND     IER, #0                 ; clear IER so we can test INTM

        AND     AL, #ST1_CLR_TEST_MASK  ; clear the bits that need to stay 0
        OR      AL, #ST1_SET_TEST_MASK  ; set the bits that need to stay 1

        PUSH    @AL
        POP     ST1                     ; write to ST1
        PUSH    ST1
        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        NOT     ACC
        AND     AL, #ST1_CLR_TEST_MASK  ; clear the bits that need to stay 0
        OR      AL, #ST1_SET_TEST_MASK  ; set the bits that need to stay 1

        PUSH    @AL
        POP     ST1                     ; write to ST1
        PUSH    ST1
        POP     AR4
        CMP     AL, @AR4
        B       fail16BitRegTest, NEQ

        MOVL    ACC, @XAR6              ; restore ACC contents

passCPURegTest:
        MOV     AL, #TEST_PASSED        ; return a success code
        B       restore16BitReg, UNC

fail16BitRegTest:
        MOV     AL, #TEST_FAILED        ; return a fail code

restore16BitReg:
        PUSH    @PL
        POP     ST0                     ; restore ST0
        PUSH    @AR3
        POP     ST1                     ; restore ST1
        PUSH    @PH
        POP     DP                      ; restore DP
        MOV     IER, @AR0               ; restore IER
        PUSH    @TH
        POP     IFR                     ; restore IFR
        PUSH    @AR5
        POP     DBGIER                  ; restore DBGIER
        B       endCPURegTest, UNC

failCPURegTest:
        MOV     AL, #TEST_FAILED        ; return a fail code

endCPURegTest:
        POP     XAR3
        POP     XAR2
        POP     XAR1
        LRETR

;;*****************************************************************************
;;
;; uint16_t STL_CPU_TEST_testFPURegisters(bool injectError)
;;                                             AL
;;
;;*****************************************************************************
STL_CPU_REG_testFPURegisters:
    ;
    ; Push save-on-entry registers onto the stack
    ;
        MOV32   *SP++, R4H
        MOV32   *SP++, R5H
        MOV32   *SP++, R6H
        MOV32   *SP++, R7H

    ;
    ; Save injectError parameter to AR5
    ;
        MOV     AR5, AL

    ;
    ; Push status register onto the stack
    ;
        MOV32   *SP++, STF

    ;
    ; ACC = Test pattern
    ;
        MOV     AH, #REG_TEST_PATTERN
        MOV     AL, #REG_TEST_PATTERN

    ;
    ; Use PL as a loop counter
    ; Test pattern first then pattern inverse
    ;
        MOV     PL, #TEST_ITERATIONS    ; loop TEST_ITERATIONS + 1 times
loopFPURegTest:
        NOT     ACC                     ; write pattern to FPU registers
        MOV32   R0H, @ACC
        MOV32   R1H, @ACC
        MOV32   R2H, @ACC
        MOV32   R3H, @ACC
        MOV32   R4H, @ACC
        MOV32   R5H, @ACC
        MOV32   R6H, @ACC
        MOV32   R7H, @ACC

    ;
    ; See if the injectError parameter is set
    ; Skip error injection if not
    ;
        CMP     AR5, #0x1                ; AR5 contains injectError parameter
        B       skipFPUErrorTest, NEQ
        MOVXI   R5H, #REG_ERROR_PATTERN  ; Write the error pattern to R5H

skipFPUErrorTest:
    ;
    ; SAVE/RESTORE will write/read the shadow registers
    ;
        SAVE
        RESTORE
        MOV32   @XAR0, R0H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R1H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R2H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R3H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R4H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R5H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R6H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ
        MOV32   @XAR0, R7H
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ

        MOVL    XAR6, @ACC              ; save ACC contents

        AND     AH, #STF_TEST_MASK_MSB
        AND     AL, #STF_TEST_MASK_LSB  ; Use mask to test STF register
        PUSH    ACC
        MOV32   STF, *--SP              ; STF = ACC
        SAVE                            ; Write/read the shadow register
        RESTORE
        MOV32   *SP++, STF
        POP     XAR0
        CMPL    ACC, @XAR0
        B       failFPURegTest, NEQ

        MOVL    ACC, @XAR6              ; restore ACC contents

        DEC     PL                      ; decrement loop counter
        B       loopFPURegTest, GEQ

passFPURegTest:
        MOV     AL, #TEST_PASSED        ; return a success code
        B       endFPURegTest, UNC

failFPURegTest:
        MOV     AL, #TEST_FAILED

endFPURegTest:
        MOV32   STF, *--SP              ; restore FPU registers
        MOV32   R7H, *--SP
        MOV32   R6H, *--SP
        MOV32   R5H, *--SP
        MOV32   R4H, *--SP
        LRETR

;;*****************************************************************************
;;
;; uint16_t STL_CPU_REG_testVCRCRegisters(uint32_t *scratchRAM,bool injectError)
;;                                                 XAR4            AL
;;
;;*****************************************************************************
STL_CPU_REG_testVCRCRegisters:
    ;
    ; Save contents of VCRCSIZE and VCRCPOLY to stack, VSTATUS to XAR7
    ;
		VMOV32  *XAR4, VCRCPOLY
        MOVL    XAR7, *XAR4
        PUSH	XAR7
		VMOV32  *XAR4, VCRCSIZE
        MOVL    XAR7, *XAR4
        PUSH	XAR7
        VMOV32  *XAR4, VSTATUS
        MOVL    XAR7, *XAR4

    ;
    ; Save injectError parameter to AR5
    ;
        MOV     AR5, AL

    ;
    ; ACC = Test pattern
    ;
        MOV     AH, #REG_TEST_PATTERN
        MOV     AL, #REG_TEST_PATTERN

    ;
    ; Use PL as a loop counter
    ; Test pattern first then pattern inverse
    ;
        MOV     PL, #TEST_ITERATIONS
loopVCRCRegTest:
        NOT     ACC
        MOVL    *XAR4, ACC              ; write pattern to VCRC registers
        VMOV32  VCRC, *XAR4
		VMOV32  VCRCPOLY, *XAR4

    ;
    ; See if the injectError parameter is set
    ; Skip error injection if not
    ;
        CMP     AR5, #0x1                   ; AR5 contains injectError parameter
        B       skipVCRCErrorTest, NEQ
        VMOVZI VCRCPOLY, #REG_ERROR_PATTERN ; Write the error pattern to lower word

skipVCRCErrorTest:
        VMOV32  *XAR4, VCRC
        CMPL    ACC, *XAR4
        B       failVCRCRegTest, NEQ
        VMOV32  *XAR4, VCRCPOLY
        CMPL    ACC, *XAR4
        B       failVCRCRegTest, NEQ

        MOVL    XAR6, @ACC                  ; save ACC contents
        AND     AH, #VSTATUS_TEST_MASK_MSB  ; use a mask to test VSTATUS reg
        AND     AL, #VSTATUS_TEST_MASK_LSB
        MOVL    *XAR4, ACC
        VMOV32  VSTATUS, *XAR4
        VMOV32  *XAR4, VSTATUS
        CMPL    ACC, *XAR4
        B       failVCRCRegTest, NEQ
        MOVL    ACC, @XAR6                  ; restore ACC contents
        AND     AH, #VCRCSIZE_TEST_MASK_MSB ; use a mask to test VCRCSIZE reg
        AND     AL, #VCRCSIZE_TEST_MASK_LSB
        MOVL    *XAR4, ACC
        VMOV32  VCRCSIZE, *XAR4
        VMOV32  *XAR4, VCRCSIZE
        CMPL    ACC, *XAR4
        B       failVCRCRegTest, NEQ
        MOVL    ACC, @XAR6              ; restore ACC contents

        DEC     PL                      ; decrement loop counter
        B       loopVCRCRegTest, GEQ

passVCRCRegTest:
        MOV     AL, #TEST_PASSED
        B       endVCRCRegTest, UNC      ; return success code if pass

failVCRCRegTest:
        MOV     AL, #TEST_FAILED

endVCRCRegTest:
        MOVL    *XAR4, XAR7
        VMOV32  VSTATUS, *XAR4
        POP 	XAR7
        MOVL    *XAR4, XAR7
        VMOV32  VCRCSIZE, *XAR4
        POP 	XAR7
        MOVL    *XAR4, XAR7
        VMOV32  VCRCPOLY, *XAR4
        LRETR
