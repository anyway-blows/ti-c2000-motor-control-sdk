;;//###########################################################################
;;//
;;// FILE:  sdl_ex_flash_prefetch_test_func.asm
;;//
;;// TITLE: Function for testing testing flash logic configurations.
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
;; Function needs to operate on some data in flash to allow us to observe the
;; impact of the data cache disable/enable.
;;
        .sect   ".const:prefetchTestTable"
        .def    prefetchTestTable
        .align  8

prefetchTestTable:
        .long   0x00001111
        .long   0x22223333
        .long   0x44445555
        .long   0x66667777
        .long   0x88889999
        .long   0xAAAABBBB
        .long   0xCCCCDDDD

;;
;; This is nonsense code for the sake of testing. It has some branches and some
;; accesses to const data which will allow us to observe differences in the
;; execution time as we disable prefetch and cache and increase wait-states.
;;
;; Written in assembly to keep cycle counts consistent across compiler versions
;; and optimization levels.
;;
        .sect   ".text:runPrefetchTestCode"
        .global runPrefetchTestCode
        .align  8

runPrefetchTestCode:
        ADDB         SP, #8

        MOVB         ACC, #61
        MOVL         *-SP[6], ACC

        MOVB         ACC, #0
        MOVL         *-SP[8], ACC

        CLRC         SXM
        MOVL         ACC, *-SP[6]
        SFR          ACC, 5
        LSL          ACC, 5
        LSL          ACC, 1
        ADD          ACC, #0x1f << 10
        MOVL         *-SP[2], ACC

        MOVB         ACC, #1
        MOVL         @XAR6, ACC
        MOV          AL, *-SP[6]
        ANDB         AL, #0x1f
        MOV          T, @AL
        MOVL         ACC, @XAR6
        LSLL         ACC, T
        MOVL         *-SP[4], ACC

        TBIT         *-SP[8], #0x2
        SB           testLabel1, NTC

        OR           *-SP[2], AL
        OR           *-SP[1], AH
        SB           testLabel2, UNC

testLabel1:
        NOT          ACC
        AND          *-SP[2], AL
        AND          *-SP[1], AH

testLabel2:
        TBIT         *-SP[8], #0x0
        SB           testLabel3, NTC
        NOP
        NOP          ; NOPs added to make sure branch above is a prefetch miss
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
        MOVL         ACC, *-SP[4]
        NOT          ACC
        AND          *-SP[2], AL
        AND          *-SP[1], AH
        SB           testLabel4, UNC

testLabel3:
        MOVL         ACC, *-SP[4]
        OR           *-SP[2], AL
        OR           *-SP[1], AH

testLabel4:
        TBIT         *-SP[8], #0x1
        SB           testLabel5, NTC

        MOVL         ACC, *-SP[4]
        OR           *-SP[2], AL
        OR           *-SP[1], AH
        SB           testLabel6, UNC

testLabel5:
        MOVL         ACC, *-SP[4]
        NOT          ACC
        AND          *-SP[2], AL
        AND          *-SP[1], AH

testLabel6:
        MOVL         XAR6, #prefetchTestTable
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6++
        MOVL         ACC, *XAR6

        SUBB         SP, #8
        LRETR
