;;//###########################################################################
;;//
;;// FILE:  stl_march_s.asm
;;//
;;// TITLE: Diagnostic Library March13N software module assembly
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

        .global STL_March_testRAMCopy
        .global STL_March_testRAM

        .asg    "*-SP[4]", STARTADDR
        .asg    "*-SP[6]", LENGTH
        .asg    "*-SP[8]", COPYADDR

;;*****************************************************************************
;; Subroutine STL_March_testRAMCopy
;;     ACC contains the test pattern
;;     SP-2 contains previous RPC
;;     SP-4 contains the start address
;;     SP-6 contains the length
;;     SP-8 contains the copy address
;;*****************************************************************************
STL_March_testRAMCopy:

        MOVL    XAR7, LENGTH         ; XAR7 = length (permanent)
        MOVL    XT, STARTADDR        ; XT = start address

        MOV     AR5,  #0001          ; AR5 = 1
        MOVL    XAR6, XT             ; XAR6 = start address
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; ACC  = pattern

        MOVL    XAR4, COPYADDR       ; XAR4 = copy address

        DINT                         ; Disable interrupts

  ;
  ; Save from start to end
  ;
aSave:
        MOVL    P, *XAR6++           ; Get content
        MOVL    *XAR4++, P           ; Save content to stack
        BANZ    aSave, AR0--         ; Continue until done
                                     ; XAR4 = copy end address + 1

        MOVL    XAR6, XT             ; XAR6 = start address
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; ACC  = pattern

aFill:
        MOVL    *XAR6++, ACC         ; Write(1) fill with pattern (ACC)
        BANZ    aFill, AR0--         ; Continue for length words


aAscend:
  ;
  ; Get ready to ascend
  ;
        NOT     ACC                  ; Invert the pattern in ACC
        MOVL    XAR6, XT             ; XAR6 = start address
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; ACC = pattern
aAscendInner:
        MOVL    P, *XAR6             ; Read(2) pattern / Read(5) inverse
        MOVL    *XAR6, ACC           ; Write(3) inverse / Write(6) pattern
        MOVL    P, *XAR6++           ; Read(4) inverse / Read(7) pattern
        BANZ    aAscendInner, AR0--  ; Continue for length words

        BANZ    aAscend, AR5--       ; Continue for 2x ascending
                                     ; 1st with inverse pattern
                                     ; 2nd with pattern

        MOV     AR5,  #0001          ; AR5 = 1
                                     ; ACC = pattern
                                     ; XAR6 = end address + 1
        MOVL    XT, XAR6             ; XT = end address + 1
                                     ; XT no longer needed to hold start addr

aDescend:
  ;
  ; Get ready to descend
  ;
        NOT     ACC                  ; Invert the pattern in ACC
        MOVL    XAR0, XAR7           ; XAR0 = length

aDescendInner:
        MOVL    P, *--XAR6           ; Read(8) pattern / Read(11) inverse
        MOVL    *XAR6, ACC           ; Write(9) inverse / Write(12) pattern
        MOVL    P, *XAR6             ; Read(10) inverse / Read(13) pattern
        BANZ    aDescendInner, AR0-- ; Continue for length words

        MOVL    XAR6, XT             ; XAR6 = end address + 1 (stored in XT)

        BANZ    aDescend, AR5--      ; Continue for 2x descending
                                     ; 1st with inverse pattern
                                     ; 2nd with pattern
  ;
  ; Get ready to restore
  ;
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; XAR4 = copy address + 1
                                     ; XAR6 = end address + 1
  ;
  ; Restore from end to start
  ;
aRestore:
        MOVL    ACC,*--XAR4          ; Get content from copy address
        MOVL    *--XAR6,ACC          ; Restore content to original address
        BANZ    aRestore, AR0--      ; Continue for length

        EINT                         ; Enable interrupts

        LRETR                        ; Return
;;
;; End of STL_March_testRAMCopy
;;

;;*****************************************************************************
;; Subroutine STL_March_testRAM
;;     ACC contains the test pattern
;;     SP-2 contains previous RPC
;;     SP-4 contains the start address
;;     SP-6 contains the length
;;*****************************************************************************
STL_March_testRAM:

        MOVL    XAR7, LENGTH         ; XAR7 = length (permanent)
        MOVL    XT, STARTADDR        ; XT = start address (permanent)

        MOV     AR5,  #0001          ; AR5 = 1
        MOVL    XAR6, XT             ; XAR6 = start address
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; ACC  = pattern

        DINT                         ; Disable interrupts

bFill:
        MOVL    *XAR6++, ACC         ; Write(1) fill with pattern (ACC)
        BANZ    bFill, AR0--         ; Continue for length words

bAscend:
  ;
  ; Get ready to ascend
  ;
        NOT     ACC                  ; Invert the pattern in ACC
        MOVL    XAR6, XT             ; XAR6 = start address
        MOVL    XAR0, XAR7           ; XAR0 = length
                                     ; ACC = pattern
bAscendInner:
        MOVL    P, *XAR6             ; Read(2) pattern / Read(5) inverse
        MOVL    *XAR6, ACC           ; Write(3) inverse / Write(6) pattern
        MOVL    P, *XAR6++           ; Read(4) inverse / Read(7) pattern
        BANZ    bAscendInner, AR0--  ; Continue for length words

        BANZ    bAscend, AR5--       ; Continue for 2x ascending
                                     ; 1st with inverse pattern
                                     ; 2nd with pattern

        MOV     AR5,  #0001          ; AR5 = 1
                                     ; ACC = pattern
                                     ; XAR6 = end address + 1
        MOVL    XT, XAR6             ; XT = end address + 1
                                     ; XT no longer needed to hold start addr

bDescend:
  ;
  ; Get ready to descend
  ;
        NOT     ACC                  ; Invert the pattern in ACC
        MOVL    XAR0, XAR7           ; XAR0 = length

bDescendInner:
        MOVL    P, *--XAR6           ; Read(8) pattern / Read(11) inverse
        MOVL    *XAR6, ACC           ; Write(9) inverse / Write(12) pattern
        MOVL    P, *XAR6             ; Read(10) inverse / Read(13) pattern
        BANZ    bDescendInner, AR0-- ; Continue for length words

        MOVL    XAR6, XT             ; XAR6 = end address + 1 (stored in XT)

        BANZ    bDescend, AR5--      ; Continue for 2x descending
                                     ; 1st with inverse pattern
                                     ; 2nd with pattern

        EINT                         ; Enable interrupts

        LRETR                        ; Return
;;
;; End of STL_March_testRAM
;;

        .unasg  STARTADDR
        .unasg  LENGTH
        .unasg  COPYADDR

;;
;; End of File
;;

