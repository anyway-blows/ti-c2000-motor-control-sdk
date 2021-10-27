;;//###########################################################################
;;//
;;// FILE:  stl_crc_s.asm
;;//
;;// TITLE: Diagnostic Library CLA software module assembly
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

;;
;;
;; Macros
;;
;; Context save. PUSH XAR1
;;
CRC_CONTEXT_SAVE    .macro
        PUSH      XAR1
        .endm
;;
;; Context restore. POP XAR1
;;
CRC_CONTEXT_RESTORE    .macro
        POP       XAR1
        .endm
;;
;; Globals
;;
  ;
  ; Module definition for external reference
  ;
        .global STL_CRC_calculate
        .global STL_CRC_calculateLowBytes
        .global STL_CRC_reset

;;
;; Argument structure defines
;;
ARG_SEEDVAL         .set    0
ARG_NBYTES          .set    2
ARG_PARITY          .set    3
ARG_CRCRESULT       .set    4
ARG_MSGBUFFER       .set    6

    .text

;;*****************************************************************************
;; Subroutine STL_CRC_calculateLowBytes
;;*****************************************************************************
STL_CRC_calculateLowBytes:
        CRC_CONTEXT_SAVE

  ;
  ; Clear out the CRC result register
  ; Load number of message bytes into AR0
  ; Load seed value into the CRC result register
  ; XAR5 points to the message buffer
  ;
        VCRCCLR
        MOVL    XAR0, *+XAR4[ARG_NBYTES]
        VMOV32  VCRC, *+XAR4[ARG_SEEDVAL]
        MOVL    XAR5, *+XAR4[ARG_MSGBUFFER]

STL_CRC_run32BitPoly1_LowBytes_Loop:
  ;
  ; Check to see if length greater than 8 bytes
  ; if true, handle the <8 bytes in a loop
  ; AL is now a multiple of 8
  ;
        MOV     AL, AR0
        MOV     AH, AR0
        AND     AL, #0xFFF8

  ;
  ; loop in 8 bytes at a time
  ; move count into AR1
  ; subtract 1, accounts for the RPTB instruction i.e. it loops
  ; N + 1 times
  ;
        SBF     STL_CRC_run32BitPoly1_LowBytes_LT8BytesLeft, EQ
        LSR     AL, #3
        MOV     AR1, AL
        SUB     AR1, #1

  ;
  ; align at 32-bit boundary to remove penalty
  ;
        .align  2

  ;
  ; loop through the message 8 bytes at a time
  ;
        RPTB         STL_CRC_run32BitPoly1_LowBytes_RepeatBlock, AR1
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++
        VCRC32L_1  *XAR5++

STL_CRC_run32BitPoly1_LowBytes_RepeatBlock:

  ;
  ; multiply by 8 to get the pre RPTB count
  ; AH holds the number of remaining bytes(<8)
  ; if multiple of 8, AH is 0, done processing
  ;
        LSL     AL, #3
        SUB     AH, AL
        SBF     STL_CRC_run32BitPoly1_LowBytes_End, EQ
        MOV     AR0, AH
STL_CRC_run32BitPoly1_LowBytes_LT8BytesLeft:
        VCRC32L_1  *XAR5++
        DEC        AR0

        SBF      STL_CRC_run32BitPoly1_LowBytes_LT8BytesLeft, NEQ
STL_CRC_run32BitPoly1_LowBytes_End:
  ;
  ; Save the result to the structure
  ;
        VMOV32  *+XAR4[ARG_CRCRESULT], VCRC

        CRC_CONTEXT_RESTORE
        LRETR

;;*****************************************************************************
;; Subroutine STL_CRC_calculate
;;*****************************************************************************
STL_CRC_calculate:
        CRC_CONTEXT_SAVE

  ;
  ; Clear out the CRC result register
  ; Load number of message bytes into AR0
  ; Load seed value into the CRC result register
  ; XAR5 points to the message buffer
  ; Check the parity
  ; If Parity = LOW_BYTE, skip to loop
  ; Parity = HIGH_BYTE, calculate high byte of the first word,
  ; ignore the low byte and proceed to next word
  ;
        VCRCCLR
        MOVL         XAR0, *+XAR4[ARG_NBYTES]
        VMOV32       VCRC, *+XAR4[ARG_SEEDVAL]
        MOVL         XAR5, *+XAR4[ARG_MSGBUFFER]
        MOV           AL,  *+XAR4[ARG_PARITY]
        SBF          STL_CRC_run32BitPoly1_Loop, EQ
        VCRC32H_1    *XAR5++

        DEC          AR0

  ;
  ; Jump to end if no more bytes
  ;
        SBF          STL_CRC_run32BitPoly1_End, EQ

STL_CRC_run32BitPoly1_Loop:
  ;
  ; Check to see if length greater than 8 bytes
  ; if true, handle the <8 bytes in a loop
  ; AL is now a multiple of 8
  ;
        MOV     AL, AR0
        MOV     AH, AR0
        AND     AL, #0xFFF8

  ;
  ; loop in 8 bytes at a time
  ; move count into AR1
  ; subtract 1, accounts for the RPTB instruction i.e. it loops
  ; N + 1 times
  ;
        SBF     STL_CRC_run32BitPoly1_LT8BytesLeft, EQ
        LSR     AL, #3
        MOV     AR1, AL
        SUB     AR1, #1


  ;
  ; align at 32-bit boundary to remove penalty
  ;
        .align       2

  ;
  ; loop through the message 8 bytes at a time
  ;
        RPTB         STL_CRC_run32BitPoly1_RepeatBlock, AR1
        VCRC32L_1    *XAR5
        VCRC32H_1    *XAR5++
        VCRC32L_1    *XAR5
        VCRC32H_1    *XAR5++
        VCRC32L_1    *XAR5
        VCRC32H_1    *XAR5++
        VCRC32L_1    *XAR5
        VCRC32H_1    *XAR5++
STL_CRC_run32BitPoly1_RepeatBlock:
  ;
  ; multiply by 8 to get the pre RPTB count
  ; AH holds the number of remaining bytes(<8)
  ; if multiple of 8, AH is 0, done processing
  ;
        LSL          AL, #3
        SUB          AH, AL
        SBF          STL_CRC_run32BitPoly1_End, EQ
        MOV          AR0, AH
STL_CRC_run32BitPoly1_LT8BytesLeft:
        VCRC32L_1    *XAR5
        DEC          AR0
        SBF          STL_CRC_run32BitPoly1_End, EQ
        VCRC32H_1    *XAR5++
        DEC          AR0
        SBF          STL_CRC_run32BitPoly1_LT8BytesLeft, NEQ
STL_CRC_run32BitPoly1_End:
  ;
  ; Save the result to the structure
  ;
        VMOV32       *+XAR4[ARG_CRCRESULT], VCRC

        CRC_CONTEXT_RESTORE
        LRETR

;;*****************************************************************************
;; Subroutine STL_CRC_reset
;;*****************************************************************************
STL_CRC_reset:
  ;
  ; resets the VCRC and clears the VCRC register
  ;
        MOVB      XAR7, #0
        VCRC8L_1  *XAR7
        VCRCCLR
        LRETR

;;
;;  End of File
;;
