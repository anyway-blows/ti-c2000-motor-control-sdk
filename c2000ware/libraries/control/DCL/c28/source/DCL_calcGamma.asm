; DCL_calcGamma.asm - compultes the linear region gain on TMU type 1
;
; Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
; ALL RIGHTS RESERVED

   	  .if $defined(__TI_EABI__)
		.if __TI_EABI__
		.asg	DCL_calcGamma, _DCL_calcGamma
		.endif
      .endif

		.global _DCL_calcGamma

		.sect	"dclfuncs"

; C prototype: float32_t DCL_calcGamma(float32_t a, float32_t d)
; argument 1 = a : [R0H] alpha - normalized [0.0f, +2.0f]
; argument 2 = d : [R1H] delta - normalized (0.0f, +1.0f]
; return = v : [R0H] calcGamma(d,a)

		.align	2

_DCL_calcGamma:
		.asmfunc
		.if $defined(.TMS320C2800_TMU1)
;		.if (.TMS320C2800_TMU1 = 1)

		LOG2F32		R2H, R1H			; R2H = log2(d)
		NOP
		NOP
		NOP
		MPYF32		R3H, R0H, R2H		; R3H = a*log2(d)
		NOP
		NEGF32		R2H, R3H			; R2H = -a*log2(d)
		IEXP2F32	R3H, R2H			; R3H = d^a
		NOP
		NOP
		NOP
		NOP
		DIVF32		R0H, R3H, R1H		; R0H = d^(a-1)
		NOP								; first delay slot for DIVF32, others in LRETR pipeline
		.endif
		LRETR							; return

		.endasmfunc
		.end

; end of file
