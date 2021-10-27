;;#############################################################################
;;! \file source/vector/mpy_SP_RMxRM.asm
;;!
;;! \brief  C-Callable multiplication of two real matrices
;;! \author David M. Alter
;;! \date   09/17/15
; FILE:        
;;
;; HISTORY:
;;   09/17/15 - original (D. Alter)
;;
;; DESCRIPTION: C-Callable multiplication of two real matrices y[] = w[] * x[]
;;                where   w[] is of dimension m x n
;;                        x[] is of dimension n x p
;;                        y[] is of dimension m x p
;;
;; Note that in C, a two-dimensional array has elements stored sequentially 
;; down the column. For example, this array:
;;                             | 1 2 |
;;                             | 3 4 |
;;
;; will be stored in memory as [1, 3, 2, 4].  It could be declared in C as 
;; x[4] or x[2][2].
;;
;; FUNCTION: 
;;   extern void mpy_SP_RMxRM(float *y, const float *w, const float *x, 
;;                       const uint16_t m, const uint16_t n, const uint16_t p)
;;
;; USAGE:       mpy_SP_RMxRM(y, w, x, m, n, p);
;;
;; PARAMETERS:  float* y = pointer to result matrix
;;              float* w = pointer to 1st src matrix
;;              float* x = pointer to 2nd src matrix
;;              m = dimension m of matrices
;;              n = dimension n of matrices
;;              p = dimension p of matrices
;;
;; RETURNS:     none
;;
;; BENCHMARK: 5*m*n*p + overhead  (cycles)
;;   Note that the overhead is significant for small dimensions.
;;   For example, 
;;           m=2 n=8, p=2 takes ~274 cycles (versus 5*m*n*p = 160).
;;           m=8, n=8, p=8 takes ~3694 cycles (versus 5*m*n*p = 2560).
;;           m=64, n=64, p=64 takes ~718030 cycles (versus 5*m*n*p = 1310720).
;;   This benchmark equation is best used to guage relative performance against 
;;   other matrix multiply functions rather than for trying to determine exact 
;;   execution cycles.
;;
;; NOTES:
;; 1) There are no restrictions on the values for n, m, and p with this 
;;    function.
;; 2) If n is even and at least 4, you can use mpy_SP_RMxRM_2() for better 
;;    performance if desired.
;;
;; ***PASSED PARAMETERS***
;;  XAR4 = &y[0,0]
;;  XAR5 = &w[0,0]
;;    AH = n
;;    AL = m
;;  *-SP[4] = &x[0,0]
;;  *-SP[5] = p
;;
;; ***REGISTER USAGE***
;;   AR0 = row index adder (2*m)
;;   AR1 = inner loop count seed (n-1)
;;  XAR2 = &w (working)
;;  XAR3 = &x (working)
;;  XAR4 = &y (working)
;;  XAR5 = &w[0,0]
;;   AR6 = middle loop counter (p iterations)
;;   AR7 = outer loop counter (m interations)
;;     T = middle loop count seed (p-1)
;;
;; ***STACK USAGE***
;; -SP[13] = p
;; -SP[12] = &x[0,0]
;; -SP[10] = return PC
;; -SP[8]  = saved XAR1
;; -SP[6]  = saved XAR2
;; -SP[4]  = saved XAR3
;; -SP[2]  = &y[0,0]
;; -SP[0]  = next free location
;;
;;
;;  Group:            C2000
;;  Target Family:    C28x+FPU32
;;
;;#############################################################################
;; $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
;; $Release Date: Feb 12, 2021 $
;; $Copyright: Copyright (C) 2021 Texas Instruments Incorporated -
;;             http://www.ti.com/ ALL RIGHTS RESERVED $
;;#############################################################################
        .if __TI_EABI__
        .asg mpy_SP_RMxRM, _mpy_SP_RMxRM
        .endif
    
        .global _mpy_SP_RMxRM
        .text

_mpy_SP_RMxRM:
        MOVL        *SP++, XAR1     ;Save XAR1 (save-on-entry register)
        MOVL        *SP++, XAR2     ;Save XAR2 (save-on-entry register)
        MOVL        *SP++, XAR3     ;Save XAR3 (save-on-entry register)
                                    
        MOVL        *SP++, XAR4     ;Store &y[0,0] on stack
                                    
        ADDB        AH, #-1         ;Subtract 1 from n since RPTB is 'N-1'
        MOV         AR1, AH         ;AR1 = inner loop count seed (n-1)
                                    
        MOV         AH, AL          ;AH = AL = m
        ADDB        AL, #-1         ;Subtract 1 from m since BANZ is 'N-1'
        MOV         AR7, AL         ;AR7 = outer loop counter (m-1)
                                    
        MOV         AL, *-SP[13]    ;AL = p
        ADDB        AL, #-1         ;Subtract 1 from m since BANZ is 'N-1'
        MOV         T, AL           ;T = middle loop count seed (p-1)
                                    
        LSL         AH, #1          ;AH = 2*m (because floating pt is 2 word 
                                    ;          addresses)
        MOV         AR0, AH         ;AR0 = results row index adder (2*m)

;---------------------------------------------------------------
;--- Outer loop
;---------------------------------------------------------------
loop1:

; Middle loop preparation
        MOVL        XAR3, *-SP[12]  ;XAR3 = &x
        MOVL        XAR4, *-SP[2]   ;XAR4 = &y
        MOV         AR6, T          ;Load middle loop counter with seed
                                    
        SETC        AMODE           ;C2xLP addressing mode
        .lp_amode                  

;-----------------------------------------------------
;--- Middle loop
;
; Results are computed on a row by row basis.  In other words,
; Row 1 of results are computed first, then row 2, etc.
;-----------------------------------------------------
loop2:

; Inner loop preparation
        MOVL        XAR2, XAR5      ;XAR2 = &w
        ZERO        R3H             ;Zero out the result accumulation
        NOP         *, ARP2         ;ARP = XAR2

;-------------------------------------------
;--- Inner loop
;-------------------------------------------
        RPTB        end_loop3, AR1
        MOV32       R0H, *0++, ARP3 ;R0H=w, XAR2+=AR0, ARP=XAR3
        MOV32       R1H, *++, ARP2  ;R1H=x, ARP=XAR2
        MPYF32      R2H, R0H, R1H   
        NOP                         ;Delay slot for MPYF32
        ADDF32      R3H, R3H, R2H   
end_loop3:                          
                                    
;-------------------------------------------
;--- End of inner loop
;-------------------------------------------

        NOP         *, ARP4
        MOV32       *0++, R3H       ;Store the result
        BANZ        loop2, AR6--    ;Middle loop branch
;-----------------------------------------------------
;--- End of middle loop
;-----------------------------------------------------

        CLRC        AMODE           ;C28 addressing mode
        .c28_amode  
    
        MOVB        ACC, #2         ;ACC = 2 (2 words per floating pt value)
        ADDL        XAR5, ACC       ;Advance &x to the next row
        ADDL        *-SP[2], ACC    ;Advance &y to the next row
    
        BANZ        loop1, AR7--    ;Outer loop branch
;---------------------------------------------------------------
;--- End of outer loop
;---------------------------------------------------------------

;Finish up
        SUBB        SP, #2          ;De-allocate local frame
        MOVL        XAR3, *--SP     ;Restore XAR3 (save-on-entry register)
        MOVL        XAR2, *--SP     ;Restore XAR2 (save-on-entry register)
        MOVL        XAR1, *--SP     ;Restore XAR1 (save-on-entry register)
        LRETR                       ;return

;end of function mpy_SP_RMxRM()
;**********************************************************************

        .end

;;#############################################################################
;;  End of File
;;#############################################################################
