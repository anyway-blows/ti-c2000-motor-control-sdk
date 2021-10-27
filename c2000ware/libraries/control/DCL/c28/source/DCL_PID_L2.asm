; DCL_PID_L2.asm - Parallel PID controller
;
; Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
; ALL RIGHTS RESERVED 

   	  .if $defined(__TI_EABI__)
		.if __TI_EABI__
		.asg	DCL_runPID_L2, _DCL_runPID_L2
		.endif
      .endif

   		.global	_DCL_runPID_L2
		.def	__cla_DCL_runPID_L2_sp

SIZEOF_LFRAME	.set	10
LFRAME_MR3		.set	0
LFRAME_V5		.set	2
LFRAME_V6		.set	4
LFRAME_V7		.set	6
LFRAME_LK		.set	8

__cla_DCL_runPID_L2_sp	.usect ".scratchpad:Cla1Prog:_DCL_runPID_L2", SIZEOF_LFRAME, 0, 1
		.asg	 __cla_DCL_runPID_L2_sp, LFRAME

		.sect 	"Cla1Prog:_DCL_runPID_L2"

		.align 	2

; C prototype:
; float DCL_runPID_L2(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk)
; argument 1 = *p : PID structure address [MAR0]
; argument 2 = rk : control loop reference [MR0]
; argument 3 = yk : control loop feedback [MR1]
; argument 4 = lk : controller saturation input [MR2]
; return = uk : control effort [MR0]

_DCL_runPID_L2:
;		MDEBUGSTOP
		MSETFLG 	RNDF32=1					; round to nearest even
		MMOV32		@LFRAME + LFRAME_MR3, MR3 	; save MR3
		MMOV32		@LFRAME + LFRAME_LK, MR2 	; save lk

;*** servo error ***
		MSUBF32		MR0, MR0, MR1				; MR0 = v5
||		MMOV32		MR1, *MAR0[4]++				; MR1 = Kpa

;*** derivative path ***
		MMOV32		MR1, *MAR0[4]++				; MR1 = Kda
		MMPYF32		MR3, MR0, MR1				; MR3 = Kda * v5
||		MMOV32		MR1, *MAR0[4]++				; MR1 = c1a
		MMPYF32		MR3, MR1, MR3				; MR3 = v1
||		MMOV32		MR1, *MAR0					; MR1 = d2
		MSUBF32		MR1, MR3, MR1				; MR1 = v1 - d2
||		MMOV32		*MAR0[2]++, MR3				; save d2
		MMOV32		MR3, *MAR0[-4]++			; MR3 = d3
		MSUBF32		MR1, MR1, MR3				; MR1 = v4
||		MMOV32		MR3, *MAR0[-10]++			; MR3 = c2a
		MMPYF32		MR3, MR1, MR3				; MR3 = d3
||		MMOV32		MR2, *MAR0[14]++			; MR2 = Kpa

;*** proportional path ***
		MMPYF32		MR2, MR0, MR2				; MR2 = v6
||		MMOV32		*MAR0[-12]++, MR3 			; save d3
		MADDF32		MR1, MR1, MR2				; MR1 = v4 + v6
||		MMOV32		MR3, *MAR0[16]++			; MR3 = Kia

;*** integral path ***
		MMPYF32		MR0, MR0, MR3				; MR0 = v7
||		MMOV32		MR3, *MAR0[-2]++			; MR3 = i14
		MMPYF32		MR0, MR0, MR3				; MR0 = v7 * i14
||		MMOV32		MR3, *MAR0					; MR3 = i10
		MADDF32		MR0, MR0, MR3				; MR0 = v8
		MMOV32		*MAR0[4]++, MR0				; save i10

;*** PID sum ***
		MADDF32		MR0, MR0, MR1				; MR0 = v9
||		MMOV32		MR1, *MAR0[2]++				; MR1 = Umaxa

;*** saturation ***
		MMOVF32		MR2, #0.0f					; MR2 = 0.0f
		MMOVF32		MR3, #1.0f					; MR3 = 1.0f
		MMINF32		MR0, MR1					; MR0 = sat+
		MMOV32		MR3, MR2, GT				; MR3 = v12
		MMOV32		MR1, *MAR0[-4]++			; MR1 = Umina
		MMAXF32		MR0, MR1					; MR0 = sat-
		MMOV32		MR3, MR2, LT				; MR3 = v12
		MRCNDD		UNC							; return call
		MMOV32		MR1, @LFRAME + LFRAME_LK	; MR1 = lk
		MMPYF32		MR2, MR1, MR3				; MR2 = lk * v12
||		MMOV32		MR3, @LFRAME + LFRAME_MR3	; restore MR3
		MMOV32		*MAR0, MR2					; save i14

		.unasg	LFRAME

; end of file
