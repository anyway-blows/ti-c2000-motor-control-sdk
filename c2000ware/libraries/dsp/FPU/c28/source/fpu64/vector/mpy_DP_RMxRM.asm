;;#############################################################################
;;! \file source/fpu64/vector/mpy_DP_RMxRM.asm
;;!
;;! \brief  C-Callable multiplication of two double precision real matrices
;;! \author Vishal Coelho (adapted from SP version by David Alter)
;;! \date   04/11/2017
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
;; FUNCTIONS:
;;   extern void mpy_DP_RMxRM(long double *y, long double *w, long double *x,
;;                       const uint16_t m, const uint16_t n, const uint16_t p)
;;  
;; PARAMETERS:  long double* y = pointer to result matrix
;;              long double* w = pointer to 1st src matrix
;;              long double* x = pointer to 2nd src matrix
;;              m = dimension m of matrices
;;              n = dimension n of matrices
;;              p = dimension p of matrices
;;
;; RETURNS:    none
;;
;; BENCHMARK:  8*m*n*p + overhead  (cycles)
;;   Note that the overhead is significant for small dimensions.
;;   For example, 
;;           m=2 n=8, p=2 takes ~TODO cycles (versus 8*m*n*p = 256).
;;           m=8, n=8, p=8 takes ~TODO cycles (versus 8*m*n*p = 4096).
;;           m=64, n=64, p=64 takes ~TODO cycles (versus 8*m*n*p = 2097152).
;;   This benchmark equation is best used to guage relative performance against 
;;   other matrix multiply functions rather than for trying to determine exact 
;;   execution cycles.
;;
;; NOTES:
;; 1) There are no restrictions on the values for n, m, and pwith this 
;;    function.
;; 2) If n is even and at least 4, you can use mpy_DP_RMxRM_2() for better 
;;    performance if desired.
;;            
;;  Group:            C2000
;;  Target Family:    C28x+FPU64
;;
;;#############################################################################
;; $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
;; $Release Date: Feb 12, 2021 $
;; $Copyright: Copyright (C) 2021 Texas Instruments Incorporated -
;;             http://www.ti.com/ ALL RIGHTS RESERVED $
;;#############################################################################

;; the includes

;; external references

;;*****************************************************************************
;; void mpy_DP_RMxRM(long double *y, long double *w, long double *x,
;;                   const uint16_t m, const uint16_t n, const uint16_t p)
;;
;; C-Callable multiplication of two real matrices,
;;                y[] = w[] * x[]
;;
;; Register Usage:
;;     AH  :  m, dimension m of matrices   (argument + working)
;;     AL  :  n, dimension n of matrices   (argument + working)
;;   XAR2  : *w, pointer to 1st src matrix (working)
;;   XAR3  : *x, pointer to 2nd src matrix (working)
;;   XAR4  : *y, pointer to result matrix  (argument + working)
;;   XAR5  : *w, pointer to 1st src matrix (argument)
;;    AR6  : middle loop counter (p iterations)
;;    AR7  : outer loop counter (m interations)
;;      T  : middle loop count seed (p-1)
;;
;;   
;; Stack Usage:
;;   |_______|<- Stack Pointer                    (SP)
;;   |_______|<- XAR4 &y[0,0]                     (SP-2)--> Local Frame
;;   |_______|<- XAR3                             (SP-4)
;;   |_______|<- XAR2                             (SP-6)
;;   |_______|<- XAR1                             (SP-8)
;;   |_______|<- rpc calling function             (SP-10)
;;   |_______|<- &x[0,0]                          (SP-12)
;;   |_______|<- p                                (SP-13)

;;*****************************************************************************
    .if __TI_EABI__
    .asg mpy_DP_RMxRM, _mpy_DP_RMxRM
    .endif
    
    .page
    .global _mpy_DP_RMxRM
    .sect   ".text"
_mpy_DP_RMxRM:
    .asmfunc
    .asg    *-SP[2] , STK_PTR_Y
    .asg    *-SP[12], STK_PTR_X
    .asg    *-SP[13], STK_P
    MOVL    *SP++, XAR1     ; Save XAR1 (save-on-entry register)
    MOVL    *SP++, XAR2     ; Save XAR2 (save-on-entry register)
    MOVL    *SP++, XAR3     ; Save XAR3 (save-on-entry register)
                              
    MOVL    *SP++, XAR4     ; Store &y[0,0] on stack
                              
    ADDB    AH, #-1         ; Subtract 1 from n since RPTB is 'N-1'
    MOV     AR1, AH         ; AR1 = inner loop count seed (n-1)
                              
    MOV     AH, AL          ; AH = AL = m
    ADDB    AL, #-1         ; Subtract 1 from m since BANZ is 'N-1'
    MOV     AR7, AL         ; AR7 = outer loop counter (m-1)
                              
    MOV     AL, STK_P       ; AL = p
    ADDB    AL, #-1         ; Subtract 1 from m since BANZ is 'N-1'
    MOV     T, AL           ; T = middle loop count seed (p-1)
                            
    LSL     AH, #2          ; AH = 4*m-2(because double is 4 word
    SUBB	AH, #2          ;         addresses and we increment
                            ;         after reading low 32-bits)
    MOV     AR0, AH         ; AR0 = results row index adder (4*m-2)

;---------------------------------------------------------------
;--- Outer loop
;---------------------------------------------------------------
_mpy_DP_RMxRM_loop1:

; Middle loop preparation
    MOVL    XAR3, STK_PTR_X ; XAR3 = &x
    MOVL    XAR4, STK_PTR_Y ; XAR4 = &y
    MOV     AR6, T          ; Load middle loop counter with seed
                              
    SETC    AMODE           ; C2xLP addressing mode
    .lp_amode                  

;-----------------------------------------------------
;--- Middle loop
;
; Results are computed on a row by row basis.  In other words,
; Row 1 of results are computed first, then row 2, etc.
;-----------------------------------------------------
_mpy_DP_RMxRM_loop2:

; Inner loop preparation
    MOVL    XAR2, XAR5      ; XAR2 = &w
    ZERO    R3              ; Zero out the result accumulation
    NOP     *, ARP2         ; ARP = XAR2

;-------------------------------------------
;--- Inner loop
;-------------------------------------------
    RPTB    _mpy_DP_RMxRM_end_loop3, AR1
    MOV32   R0L, *++        ; 1| R0=w,
    MOV32   R0H, *0++, ARP3 ; 2|       XAR2+=AR0, ARP=XAR3
    MOV32   R1L, *++        ;  | R1=x
    MOV32   R1H, *++, ARP2  ;  |                , ARP=XAR2
    MPYF64  R2, R0, R1      ; *|     
    NOP                     ; 1|
    NOP                     ; 2|
    ADDF64  R3, R3, R2      ; *|   
_mpy_DP_RMxRM_end_loop3:                          
                                    
;-------------------------------------------
;--- End of inner loop
;-------------------------------------------
    NOP     *, ARP4         ; 1|                 , ARP=XAR4
    NOP                     ; 2|
    MOV32   *++, R3L        ; Store the result (XAR4+=AR0)
    MOV32   *0++, R3H
                            ; Middle loop branch
    BANZ    _mpy_DP_RMxRM_loop2, AR6--    
;-----------------------------------------------------
;--- End of middle loop
;-----------------------------------------------------
    
    CLRC    AMODE           ; C28 addressing mode
    .c28_amode  
    
    MOVB    ACC, #4         ; ACC = 4 (4 words per double value)
    ADDL    XAR5, ACC       ; Advance &x to the next row
    ADDL    STK_PTR_Y, ACC  ; Advance &y to the next row
                            ; Outer loop branch  
    BANZ    _mpy_DP_RMxRM_loop1, AR7--    
;---------------------------------------------------------------
;--- End of outer loop
;---------------------------------------------------------------

;Finish up
    SUBB    SP, #2          ; De-allocate local frame
    MOVL    XAR3, *--SP     ; Restore XAR3 (save-on-entry register)
    MOVL    XAR2, *--SP     ; Restore XAR2 (save-on-entry register)
    MOVL    XAR1, *--SP     ; Restore XAR1 (save-on-entry register)
    LRETR
    .endasmfunc
    .unasg  STK_PTR_Y
    .unasg  STK_PTR_X
    .unasg  STK_P
  
;; End of File
