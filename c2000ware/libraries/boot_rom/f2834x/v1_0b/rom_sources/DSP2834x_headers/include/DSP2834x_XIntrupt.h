// TI File $Revision: /main/1 $
// Checkin $Date: May 30, 2008   14:11:41 $
//###########################################################################
//
// FILE:  DSP2834x_XIntrupt.h
//
// TITLE: DSP2834x Device External Interrupt Register Definitions.
//
//###########################################################################
// $TI Release: 2834x Boot ROM V1b $
// $Release Date: June 3, 2008 $
//###########################################################################

#ifndef DSP2834x_XINTRUPT_H
#define DSP2834x_XINTRUPT_H


#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------

struct XINTCR_BITS {
    Uint16   ENABLE:1;    // 0      enable/disable
    Uint16   rsvd1:1;     // 1      reserved
    Uint16   POLARITY:2;  // 3:2    pos/neg, both triggered
    Uint16   rsvd2:12;    //15:4    reserved
};

union XINTCR_REG {
   Uint16               all;
   struct XINTCR_BITS   bit;
};  

struct XNMICR_BITS {
    Uint16   ENABLE:1;    // 0      enable/disable
    Uint16   SELECT:1;    // 1      Timer 1 or XNMI connected to int13
    Uint16   POLARITY:2;  // 3:2    pos/neg, or both triggered
    Uint16   rsvd2:12;    // 15:4   reserved
};

union XNMICR_REG {
   Uint16               all;
   struct XNMICR_BITS   bit;
};  




//---------------------------------------------------------------------------
// External Interrupt Register File:
//
struct XINTRUPT_REGS {
   union XINTCR_REG XINT1CR;
   union XINTCR_REG XINT2CR;
   union XINTCR_REG XINT3CR;
   union XINTCR_REG XINT4CR;
   union XINTCR_REG XINT5CR;
   union XINTCR_REG XINT6CR;
   union XINTCR_REG XINT7CR;
   union XNMICR_REG XNMICR;
   Uint16           XINT1CTR;
   Uint16           XINT2CTR;
   Uint16           rsvd[5];
   Uint16           XNMICTR;
};

//---------------------------------------------------------------------------
// External Interrupt References & Function Declarations:
//
extern volatile struct XINTRUPT_REGS XIntruptRegs;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of DSP2834x_XINTF_H definition

//===========================================================================
// End of file.
//===========================================================================

