;;#############################################################################
;;! \file source/fpu64/vector/abs_DP_CV.asm
;;!
;;! \brief  C-Callable absolute value of a complex vector.
;;! \author Vishal Coelho (adapted from SP version by David Alter)
;;! \date   07/06/2016
;;
;; DESCRIPTION: C-Callable absolute value of a complex vector
;;          y[i] = sqrt(x_re[i]^2 + x_im[i]^2)
;;
;; FUNCTIONS:
;;   void abs_DP_CV(long double *y, const complexf64_t *x, 
;;                  const uint16_t N)
;;  
;; PARAMETERS: long double *y     = output array
;;             complexf64_t *x    = input array
;;             uint16_t N         = length of x and y arrays
;;
;; RETURNS:    none
;;
;; BENCHMARK:  73*N + 8 cycles (including the call and return)
;;
;; NOTES:
;;   1) The type 'complexf64_t' is defined as
;;
;;      typedef struct {
;;         long double real;
;;         long double imag;
;;      } complexf64_t;
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
;;  void abs_DP_CV(long double *y, const complexf64_t *x, 
;;                  const uint16_t N)
;;
;;  C-Callable absolute value of a complex vector
;;          y[i] = sqrt(x_re[i]^2 + x_im[i]^2)
;;
;; Register Usage:
;;     AL  : N, the number of complex data points
;;    AR0  : N, the number of complex data points
;;   XAR4  : *y, pointer to the output array
;;   XAR5  : *x, pointer to the complex input array
;;
;;*****************************************************************************
    .if __TI_EABI__
    .asg    abs_DP_CV, _abs_DP_CV
    .endif
    
    .page
    .global _abs_DP_CV
    .sect   ".text"
_abs_DP_CV:
    .asmfunc
    SUBB    AL, #1              ; AL  = N - 1
    MOVZ    AR0, AL             ; AR0 = N - 1
    
_abs_DP_CV_loop:
    ;; Computing the magnitude squared
    MOV32       R0L, *XAR5++    ;    R0 = x_re[i]
    MOV32       R0H, *XAR5++    ;
    MPYF64      R0, R0, R0      ; *| R0 = x_re[i]^2
    MOV32       R1L, *XAR5++    ; 1| R1 = x_im[i]
    MOV32       R1H, *XAR5++    ; 2|
    MPYF64      R1, R1, R1      ; *| R1 = x_im[i]^2
    NOP                         ; 1|
    NOP                         ; 2|
    ADDF64      R0, R0, R1      ; *| R0 = x_re[i]^2 + x_im[i]^2
    NOP                         ; 1|
    NOP                         ; 2|
    
    ;; Compute the square root
    EISQRTF64   R1, R0          ; *| R1 = Ye = Estimate(1/sqrt(X))
    NOP                         ; 1|
    MPYF64      R2, R0, #0.5    ; *| R2 = X*0.5
    ;; run loop 3 times
    .align  2
    RPTB        #_abs_DP_CV_isqrt_loop, #2 
    MPYF64      R3, R1, R1      ; 1|*| R3 = Ye*Ye
    NOP                         ; 2|1|
    NOP                         ;  |2|
    MPYF64      R3, R3, R2      ; *| | R3 = Ye*Ye*X*0.5
    NOP                         ; 1| | 
    NOP                         ; 2| | 
    SUBF64      R3, #1.5, R3    ; *| | R3 = 1.5 - Ye*Ye*X*0.5
    NOP                         ; 1| | 
    NOP                         ; 2| | 
    MPYF64      R1, R1, R3      ; *| | R1 = Ye = Ye*(1.5 - Ye*Ye*X*0.5)
    NOP                         ; 1| |
    NOP                         ; 2| |
_abs_DP_CV_isqrt_loop:
    MPYF64      R3, R1, R2      ; *| R3 = Ye'''*X*0.5
    NOP                         ; 1|
    NOP                         ; 2|
    MPYF64      R3, R1, R3      ; *| R3 = Ye'''*Ye'''*X*0.5
    NOP                         ; 1|
    NOP                         ; 2|
    SUBF64      R3, #1.5, R3    ; *| R3 = 1.5 - Ye'''*Ye'''*X*0.5
    CMPF64      R0, #0.0        ; 1| if X == 0, set ZF, NF
    NOP                         ; 2|
    MPYF64      R1, R1, R3      ; *| R1 = Ye'''*(1.5 - Ye'''*Ye'''*X*0.5)
    NOP                         ; 1|
    NOP                         ; 2|
    MOV64       R1, R0, EQ      ;  | If X is zero, set Ye'''' to 0
    MPYF64      R0, R0, R1      ; *| R0 = Y = X * Ye''''
    NOP                         ; 1|
    NOP                         ; 2|
    MOV32       *XAR4++, R0L    ;
    MOV32       *XAR4++, R0H    ;
    BANZ        _abs_DP_CV_loop, AR0--
    LRETR
    .endasmfunc

  
;; End of File
