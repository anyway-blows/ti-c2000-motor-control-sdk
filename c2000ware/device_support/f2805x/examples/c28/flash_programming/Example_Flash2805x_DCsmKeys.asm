;//###########################################################################
;//
;// FILE:  Example_Flash2805x_DCsmKeys.asm
;//
;// TITLE: F2805x Dual Code Security Module Keys for
;//        the Flash API example
;//
;// NOTE:
;//###########################################################################
;// $TI Release: F2805x Support Library v2.02.00.00 $
;// $Release Date: Fri Feb 12 19:14:47 IST 2021 $
;// $Copyright:
;// Copyright (C) 2012-2021 Texas Instruments Incorporated - http://www.ti.com/
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
;//###########################################################################

; Use this file to pass key values to the Example Flash program
; to unlock the Code Security module before attempting to erase,
; program or verify the part.


          .global _Z1_PRG_key0
          .global _Z1_PRG_key1
          .global _Z1_PRG_key2
          .global _Z1_PRG_key3

          .global _Z2_PRG_key0
          .global _Z2_PRG_key1
          .global _Z2_PRG_key2
          .global _Z2_PRG_key3


; For erased flash the password locations will all be 0xFFFF
           .text
_Z1_PRG_key0  .long 0xFFFFFFFF  ; PSWD bits 31-0 Zone1
_Z1_PRG_key1  .long 0xFFFFFFFF  ; PSWD bits 63-32
_Z1_PRG_key2  .long 0xFFFFFFFF  ; PSWD bits 95-64
_Z1_PRG_key3  .long 0xFFFFFFFF  ; PSWD bits 127-96

_Z2_PRG_key0  .long 0xFFFFFFFF  ; PSWD bits 31-0 Zone2
_Z2_PRG_key1  .long 0xFFFFFFFF  ; PSWD bits 63-32
_Z2_PRG_key2  .long 0xFFFFFFFF  ; PSWD bits 95-64
_Z2_PRG_key3  .long 0xFFFFFFFF  ; PSWD bits 127-96




