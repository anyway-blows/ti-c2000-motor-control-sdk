;;//###########################################################################
;;//
;;// FILE:  stl_hwbist_s.asm
;;//
;;// TITLE: Diagnostic Library HWBIST software module assembly
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

        .cdecls C, LIST, "inc/hw_memmap.h"
        .cdecls C, LIST, "inc/hw_sysctl.h"

;;
;; C-Callable function defs
;;
        .def    STL_HWBIST_runMicroTest
        .def    STL_HWBIST_handleReset

;;
;; RAM section globals
;;
        .global biststack ; HWBIST Stack (for BIST context save/rest)

;;
;; C-Callable function globals
;;
        .global STL_HWBIST_runMicroTest
        .global STL_HWBIST_handleReset

;;
;; RAM section (stack addresses filled in by the linker)
;; Reserve 66 (0x42) words for stack. Use blocking flag and alignment flag to
;; allocate 66 words on a long word boundary.
;;
biststack:    .usect  "hwbiststack", 66, 1, 2

;;
;; HWBIST Macros
;;
HWBRegs  .set   0x1780  ; DP to HWB resisters (64 * 0x1780 = 0x5E000)
CSTCCRD  .set   0x0038  ; Register in DP
CSTCGCR2 .set   0x0008  ; Register in DP
CSTCGCR3 .set   0x000C  ; Register in DP
CSTCGCR4 .set   0x0010  ; Register in DP

;;
;; CPU Timer Macros
;;
TmrRegs  .set   0x0030  ; DP to HWB resisters (64 * 0x30 = 0xC00)
TMR1TCR  .set   0x000C  ; Register in DP
TMR2TCR  .set   0x0014  ; Register in DP

;;
;; Code section
;;
        .sect   ".TI.ramfunc"

;;*****************************************************************************
;;
;; BIST reset handler. This is the entry point that is called by BootROM after
;; a BIST reset. This function is NOT C-Callable and does NOT return anywhere.
;; After this function is called, the processor context is restored (the PC is
;; set to retpcloc: in STL_HWBIST_runMicroTest).
;;
;;*****************************************************************************
STL_HWBIST_handleReset:

        MOVW    DP, #0        ; Set DP so we can access @SP

    ;
    ; 14 NOPs
    ;
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

    ;
    ; Begin Context Restore
    ;
        MOV     SP, #(biststack + 0x3C) ; Move SP last BIST stack str loc
        POP     RPC                     ; Start restoring in reverse order
        POP     DP:ST1
        POP     T:ST0
        POP     XAR7
        POP     XAR6
        POP     XAR5
        POP     XAR4
        POP     XAR3
        POP     XAR2
        POP     XAR1
        POP     XAR0
        POP     P
        POP     ACC          ; Remember: AH = ST1, AL = ST0 from user context
        POP     XT
        NASP                 ; Un-align stack pointer
        POP     IER
        POP     RB           ; Restore FPU Registers

        MOV32   STF, *--SP
        MOV32   R7H, *--SP
        MOV32   R6H, *--SP
        MOV32   R5H, *--SP
        MOV32   R4H, *--SP
        MOV32   R3H, *--SP
        MOV32   R2H, *--SP
        MOV32   R1H, *--SP
        MOV32   R0H, *--SP

        VMOV32  VCRCSIZE, *--SP         ; Restore VCRC Registers
        VMOV32  VCRCPOLY, *--SP
        VMOV32  VSTATUS, *--SP
        VMOV32  VCRC, *--SP

        MOV     SP, #(biststack + 0x3E) ; Move SP to end of BIST Stack to
                                        ; Restore XAR7
        POP     XAR7                    ; Get return PC into XAR7
        MOV     @SP, *(0:biststack)     ; Restore user SP

    ;
    ; End ContextRestore
    ;
        EALLOW

    ;
    ; Reset HW BIST reset source bits
    ;
        MOVW    DP, #((CPUSYS_BASE + SYSCTL_O_RESC) >> 6)
        MOV     @(SYSCTL_O_RESC & 0x40), #0x00000000

    ;
    ; Context restore done, re-issue logged interrupts
    ;
        MOVW    DP, #HWBRegs
        MOVW    @CSTCCRD, #0x0A

    ;
    ; 14 NOPs
    ;
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

    ;
    ; Restore the CPU Timer 1 & 2 interrupt enable status. If the TCR.TIF bit
    ; is set and interrupts were enabled force the corresponding bit in IFR.
    ;
        MOVW    DP, #TmrRegs

        TBIT    AL, #14             ; Branch if TIE wasn't set
        SB      noEnableTIE1, NTC

        TBIT    @TMR1TCR, #15       ; Branch if TIE was set, but TIF isn't
        SB      enableTIE1, NTC
        OR      IFR, #(1 << 12)     ; Set the IFR flag for INT13

enableTIE1:
        MOV     AL, @TMR1TCR        ; Reenable CPU Timer 1 interrupts
        AND     AL, #~(1 << 15)     ; TIF is W1C, so take care not to clear it
        OR      AL, #(1 << 14)
        MOV     @TMR1TCR, AL

noEnableTIE1:
        TBIT    AH, #14             ; Check if TIE was set
        SB      noEnableTIE2, NTC

        TBIT    @TMR2TCR, #15       ; Branch if TIE was set, but TIF isn't
        SB      enableTIE2, NTC
        OR      IFR, #(1 << 13)     ; Set the IFR flag for INT14

enableTIE2:
        MOV     AL, @TMR2TCR        ; Reenable CPU Timer 2 interrupts
        AND     AL, #~(1 << 15)     ; TIF is W1C, so take care not to clear it
        OR      AL, #(1 << 14)
        MOV     @TMR2TCR, AL

noEnableTIE2:

    ;
    ; Restore PC (this will branch back to retpcloc: in STL_HWBIST_runMicroTest)
    ;
        LB      *XAR7

;;*****************************************************************************
;;
;; void STL_HWBIST_runMicroTest(void)
;; C-callable function that starts the self test run. This function will
;; return to user code after the BIST reset cycle. It may also return if
;; BISTDISABLE = 0xA.
;;
;;*****************************************************************************
STL_HWBIST_runMicroTest:
    ;
    ; Explicitly save modified regs on user stack because they are modified in
    ; this function. While this is saved by the calling function in the normal
    ; convention, we are explicitly saving it just in case someone wants to call
    ; this from ASM.
    ;
        PUSH    ACC
        PUSH    XAR4
        PUSH    XAR7

        PUSH    ST1         ; Save the user's ST1 register
                            ; This preserves EALLOW, INTM, and status bits
        POP     AH

        PUSH    ST0         ; Save the user's ST0 register
        POP     AL

        MOVL    XAR4, ACC   ; Back up ST0/ST1 settings to XAR4

    ;
    ; This is a workaround for a corner case where generation of a spurious
    ; CPU Timer 1 & 2 interrupt is possible. CPU Timer interrupts will be
    ; saved and disabled before HWBIST interrupt logging is started and
    ; restored after it is ended.
    ;
        MOVW    DP, #TmrRegs
        MOV     AL, @TMR1TCR            ; Save TCR registers in the ACC
        MOV     AH, @TMR2TCR

        AND     @TMR1TCR, #~(3 << 14)   ; Clear TIE bit without clearing TIF
        AND     @TMR2TCR, #~(3 << 14)   ; Clear TIE bit without clearing TIF

        EALLOW

        MOVW    DP, #HWBRegs
        MOVW    @CSTCGCR3, #0x0A        ; Start logging interrupts

    ;
    ; Pipeline flush (9 NOPs).
    ; These NOPs MUST be located here in order to guarantee that pending
    ; interrupts in the IFR register are serviced before moving onto the
    ; context save (which cannot be interrupted).
    ;
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

        EDIS

        MOVW    DP, #0                  ; For @SP accesses
    ;
    ; Begin Context Save Routine
    ;
        MOV     *(0:biststack), @SP     ; (0) Save user SP to top of stack
        MOV     SP, #(biststack + 2)    ; Load SP with next BIST stack loc

        VMOV32  *SP++, VCRC             ; Save VCRC Registers
        VMOV32  *SP++, VSTATUS
        VMOV32  *SP++, VCRCPOLY
        VMOV32  *SP++, VCRCSIZE

        MOV32   *SP++, R0H              ; Save FPU Registers
        MOV32   *SP++, R1H
        MOV32   *SP++, R2H
        MOV32   *SP++, R3H
        MOV32   *SP++, R4H
        MOV32   *SP++, R5H
        MOV32   *SP++, R6H
        MOV32   *SP++, R7H
        MOV32   *SP++, STF
        PUSH    RB
        PUSH    IER                     ; (0)
        ASP                             ; Align stack pointer (SP+1)
        PUSH    XT                      ; (2:3)
        PUSH    ACC                     ; (4:5) AH = ST1 AL = ST0 user context
        PUSH    P                       ; (6:7)
        PUSH    XAR0                    ; (8:9)
        PUSH    XAR1                    ; (A:B)
        PUSH    XAR2                    ; (C:D)
        PUSH    XAR3                    ; (E:F)
        PUSH    XAR4                    ; (10:11)
        PUSH    XAR5                    ; (12:13)
        PUSH    XAR6                    ; (14:15)
        PUSH    XAR7                    ; (16:17)
        PUSH    T:ST0                   ; (18:19)
        PUSH    DP:ST1                  ; (1A:1B)
        PUSH    RPC                     ; (1C:1D)

curpcloc:
        MOV     XAR7, PC                    ; Add PC return offset
        ADD     AR7, #(retpcloc - curpcloc) ; instruction after BIST_GO
        PUSH    XAR7                        ; (1E:1F) Push PC
    ;
    ; End Context Save Routine
    ;
        EALLOW

        NOP                             ; Pipeline flush (17 NOPs)
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

    ;
    ; Start BIST Self Test
    ;
        MOVW    DP, #HWBRegs
        MOVW    @CSTCGCR4, #0x0A    ; (BIST_GO)

        RPT     #20 || NOP

        NOP                         ; Pipeline flush (9 NOPs)
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP

retpcloc:
    ;
    ; This is the return point from the BIST ResetHandler routine. XAR4
    ; contains the ST0/ST1 values from before the BIST_GO was triggered. ST1
    ; must be restored now to return the EALLOW, INTM, DBGM state back to
    ; original status.
    ;
        MOVL    ACC, XAR4           ; ACC = backed up ST0/ST1 registers

        PUSH    AL                  ; Restore user's ST0
        POP     ST0                 ; ST0 = AL

        PUSH    AH                  ; Restore user's ST1
        POP     ST1                 ; ST1 = AH

    ;
    ; At this point, INTM and DBGM will be restored to original state
    ;
        POP     XAR7                ; Restore regs from user stack
        POP     XAR4
        POP     ACC
        LRETR

;;
;; End of File
;;
