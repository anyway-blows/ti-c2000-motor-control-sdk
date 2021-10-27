
MEMORY
{
   ADCA_RESULT   : origin = 0x000B00, length = 0x000020
   ADCB_RESULT   : origin = 0x000B20, length = 0x000020
   ADCC_RESULT   : origin = 0x000B40, length = 0x000020
   ADCD_RESULT   : origin = 0x000B60, length = 0x000020

   ADCA          : origin = 0x007400, length = 0x000080
   ADCB          : origin = 0x007480, length = 0x000080
   ADCC          : origin = 0x007500, length = 0x000080
   ADCD          : origin = 0x007580, length = 0x000080



   BGCRC         : origin = 0x00006340, length = 0x000080
   CANA          : origin = 0x048000, length = 0x000800
   CANB          : origin = 0x04A000, length = 0x000800

   CLA1          : origin = 0x001400, length = 0x000048     /* CLA registers */

   CMPSS1       : origin = 0x005C80, length = 0x000020
   CMPSS2       : origin = 0x005CA0, length = 0x000020
   CMPSS3       : origin = 0x005CC0, length = 0x000020
   CMPSS4       : origin = 0x005CE0, length = 0x000020
   CMPSS5       : origin = 0x005D00, length = 0x000020
   CMPSS6       : origin = 0x005D20, length = 0x000020
   CMPSS7       : origin = 0x005D40, length = 0x000020
   CMPSS8       : origin = 0x005D60, length = 0x000020

   CPU_TIMER0   : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
   CPU_TIMER1   : origin = 0x000C08, length = 0x000008     /* CPU Timer1 registers */
   CPU_TIMER2   : origin = 0x000C10, length = 0x000008     /* CPU Timer2 registers */

   DACA          : origin = 0x005C00, length = 0x000010
   DACB          : origin = 0x005C10, length = 0x000010
   DACC          : origin = 0x005C20, length = 0x000010

   DCC0          : origin = 0x05E700, length = 0x000040
   DCC1          : origin = 0x05E740, length = 0x000040
   DCC2          : origin = 0x05E780, length = 0x000040

   DCSM_Z1      : origin = 0x05F000, length = 0x000040     /* Zone 1 Dual code security module registers */
   DCSM_Z2      : origin = 0x05F080, length = 0x000040     /* Zone 2 Dual code security module registers */
   DCSM_COMMON  : origin = 0x05F0C0, length = 0x000040     /* Common Dual code security module registers */
   DCSM_Z1_OTP  : origin = 0x078000, length = 0x000020     /* Part of Z1 OTP.  LinkPointer/JTAG lock/ Boot Mode */
   DCSM_Z2_OTP  : origin = 0x078200, length = 0x000020     /* Part of Z2 OTP.  LinkPointer/JTAG lock */

   DMA          : origin = 0x001000, length = 0x0000E0

   ECAP1        : origin = 0x005200, length = 0x000020     /* Enhanced Capture 1 registers */
   ECAP2        : origin = 0x005240, length = 0x000020     /* Enhanced Capture 2 registers */
   ECAP3        : origin = 0x005280, length = 0x000020     /* Enhanced Capture 3 registers */
   ECAP4        : origin = 0x0052C0, length = 0x000020     /* Enhanced Capture 4 registers */
   ECAP5        : origin = 0x005300, length = 0x000020     /* Enhanced Capture 5 registers */
   ECAP6        : origin = 0x005340, length = 0x000020     /* Enhanced Capture 6 registers */
   ECAP7        : origin = 0x005380, length = 0x000020     /* Enhanced Capture 7 registers */

   EMIF1        : origin = 0x047000, length = 0x000800
   EMIF2        : origin = 0x047800, length = 0x000800

   EQEP1        : origin = 0x005100, length = 0x000040     /* Enhanced QEP 1 registers */
   EQEP2        : origin = 0x005140, length = 0x000040     /* Enhanced QEP 2 registers */
   EQEP3        : origin = 0x005180, length = 0x000040     /* Enhanced QEP 3 registers */

   EPWM1        : origin = 0x004000, length = 0x000100     /* Enhanced PWM 1 registers */
   EPWM2        : origin = 0x004100, length = 0x000100     /* Enhanced PWM 2 registers */
   EPWM3        : origin = 0x004200, length = 0x000100     /* Enhanced PWM 3 registers */
   EPWM4        : origin = 0x004300, length = 0x000100     /* Enhanced PWM 4 registers */
   EPWM5        : origin = 0x004400, length = 0x000100     /* Enhanced PWM 5 registers */
   EPWM6        : origin = 0x004500, length = 0x000100     /* Enhanced PWM 6 registers */
   EPWM7        : origin = 0x004600, length = 0x000100     /* Enhanced PWM 7 registers */
   EPWM8        : origin = 0x004700, length = 0x000100     /* Enhanced PWM 8 registers */
   EPWM9        : origin = 0x004800, length = 0x000100     /* Enhanced PWM 9 registers */
   EPWM10       : origin = 0x004900, length = 0x000100     /* Enhanced PWM 10 registers */
   EPWM11       : origin = 0x004A00, length = 0x000100     /* Enhanced PWM 11 registers */
   EPWM12       : origin = 0x004B00, length = 0x000100     /* Enhanced PWM 12 registers */
   EPWM13       : origin = 0x004C00, length = 0x000100     /* Enhanced PWM 13 registers */
   EPWM14       : origin = 0x004D00, length = 0x000100     /* Enhanced PWM 14 registers */
   EPWM15       : origin = 0x004E00, length = 0x000100     /* Enhanced PWM 15 registers */
   EPWM16       : origin = 0x004F00, length = 0x000100     /* Enhanced PWM 16 registers */

   ERAD_GLOBAL_REGISTERS : origin = 0x0005E800, length = 0x00000100
   ERAD_HWBP_REGISTERS   : origin = 0x0005E900, length = 0x00000080
   ERAD_COUNTER_REGISTERS : origin = 0x0005E980, length = 0x00000080
   ERAD_CRC_REGISTERS    : origin = 0x0005EA00, length = 0x00000100
   FLASH0_CTRL  : origin = 0x05F800, length = 0x000300
   FLASH0_ECC   : origin = 0x05FB00, length = 0x000040

   FSITXA       : origin = 0x006600, length = 0x000080
   FSI_TXB      : origin = 0x006700, length = 0x000080
   FSIRXA       : origin = 0x006680, length = 0x000080
   FSI_RXB      : origin = 0x006780, length = 0x000080
   FSI_RXC      : origin = 0x006880, length = 0x000080
   FSI_RXD      : origin = 0x006980, length = 0x000080
   FSI_RXE      : origin = 0x006A80, length = 0x000080
   FSI_RXF      : origin = 0x006B80, length = 0x000080
   FSI_RXG      : origin = 0x006C80, length = 0x000080
   FSI_RXH      : origin = 0x006D80, length = 0x000080
   GPIODATA     : origin = 0x007F00, length = 0x000030     /* GPIO data registers */
   HRCAP6       : origin = 0x00005360, length = 0x00000020
   HRCAP7       : origin = 0x000053A0, length = 0x00000020


   HWBIST       : origin = 0x0005E000, length = 0X000000D0
   I2CA         : origin = 0x007300, length = 0x000040     /* I2C-A registers */
   I2CB         : origin = 0x007340, length = 0x000040     /* I2C-B registers */

   
   CPU2TOCPU1_IPC_CPU2VIEW : origin = 0x0005CE00, length = 0x00000040
   CPU2TOCM_IPC_CPU2VIEW   : origin = 0x0005CE40, length = 0x00000040


   MEMCFG       : origin = 0x05F400, length = 0x0000C0     /* Mem Config registers */
   EMIF1CONFIG  : origin = 0x05F4C0, length = 0x000020     /* Emif-1 Config registers */
   ACCESSPROTECTION  : origin = 0x05F500, length = 0x000040     /* Access Protection registers */
   MEMORYERROR  : origin = 0x05F540, length = 0x000040     /* Access Protection registers */

   TEST_ERROR : origin = 0x05F590, length = 0x00000F  

   MCBSPA       : origin = 0x006000, length = 0x000040     /* McBSP-A registers */
   MCBSPB       : origin = 0x006040, length = 0x000040     /* McBSP-A registers */

   NMIINTRUPT   : origin = 0x007060, length = 0x000010     /* NMI Watchdog Interrupt Registers */

   PIE_CTRL     : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
   PIE_VECT_TABLE : origin = 0x000D00, length = 0x000200     /* PIE Vector Table */
   PMBUSA       : origin = 0x006400, length = 0x000020

   SCIA         : origin = 0x007200, length = 0x000010     /* SCI-A registers */
   SCIB         : origin = 0x007210, length = 0x000010     /* SCI-B registers */
   SCIC         : origin = 0x007220, length = 0x000010     /* SCI-C registers */
   SCID         : origin = 0x007230, length = 0x000010     /* SCI-D registers */

   SDFM1        : origin = 0x005E00, length = 0x000080     /* Sigma delta 1 registers */
   SDFM2        : origin = 0x005E80, length = 0x000080     /* Sigma delta 2 registers */

   SPIA         : origin = 0x006100, length = 0x000010
   SPIB         : origin = 0x006110, length = 0x000010
   SPIC         : origin = 0x006120, length = 0x000010
   SPID         : origin = 0x006130, length = 0x000010

   DMA_CLA_SRC_SEL : origin = 0x007980, length = 0x000040
   CLK_CFG     : origin = 0x05D200, length = 0x000100
   CPU_SYS     : origin = 0x05D300, length = 0x000100
   PERIPH_AC    : origin = 0x05D500, length = 0x000200
   SYS_STATUS   : origin = 0x05D400, length = 0x000100

   WD           : origin = 0x007000, length = 0x000040

   XINT         : origin = 0x007070, length = 0x000010

}


SECTIONS
{
/*** PIE Vect Table and Boot ROM Variables Structures ***/
  UNION run = PIE_VECT_TABLE
   {
      PieVectTableFile
      GROUP
      {
         EmuKeyVar
         EmuBModeVar
         EmuBootPinsVar
         FlashCallbackVar
         FlashScalingVar
      }
   }

   AdcaResultFile        : > ADCA_RESULT, type=NOINIT
   AdcbResultFile        : > ADCB_RESULT, type=NOINIT
   AdccResultFile        : > ADCC_RESULT, type=NOINIT
   AdcdResultFile        : > ADCD_RESULT, type=NOINIT

   AdcaRegsFile          : > ADCA, type=NOINIT
   AdcbRegsFile          : > ADCB, type=NOINIT
   AdccRegsFile          : > ADCC, type=NOINIT
   AdcdRegsFile          : > ADCD, type=NOINIT

   BgcrcRegsFile         : > BGCRC, type=NOINIT
   CanaRegsFile          : > CANA, type=NOINIT
   CanbRegsFile          : > CANB, type=NOINIT

   Cla1RegsFile          : > CLA1, type=NOINIT

   Cmpss1RegsFile        : > CMPSS1, type=NOINIT
   Cmpss2RegsFile        : > CMPSS2, type=NOINIT
   Cmpss3RegsFile        : > CMPSS3, type=NOINIT
   Cmpss4RegsFile        : > CMPSS4, type=NOINIT
   Cmpss5RegsFile        : > CMPSS5, type=NOINIT
   Cmpss6RegsFile        : > CMPSS6, type=NOINIT
   Cmpss7RegsFile        : > CMPSS7, type=NOINIT
   Cmpss8RegsFile        : > CMPSS8, type=NOINIT

   CpuTimer0RegsFile     : > CPU_TIMER0, type=NOINIT
   CpuTimer1RegsFile     : > CPU_TIMER1, type=NOINIT
   CpuTimer2RegsFile     : > CPU_TIMER2, type=NOINIT

   DacaRegsFile          : > DACA, type=NOINIT
   DacbRegsFile          : > DACB, type=NOINIT
   DaccRegsFile          : > DACC, type=NOINIT

   Dcc0RegsFile               : > DCC0, type=NOINIT
   Dcc1RegsFile               : > DCC1, type=NOINIT
   Dcc2RegsFile               : > DCC2, type=NOINIT
   DcsmZ1RegsFile        : > DCSM_Z1, type=NOINIT
   DcsmZ2RegsFile        : > DCSM_Z2, type=NOINIT
   DcsmCommonRegsFile    : > DCSM_COMMON, type=NOINIT

   /*** Warning:  Only remove "Type = NOLOAD" to program OTP Locations ***/
   DcsmZ1OtpFile         : > DCSM_Z1_OTP, type = NOLOAD, type=NOINIT
   DcsmZ2OtpFile         : > DCSM_Z2_OTP, type = NOLOAD, type=NOINIT

   DmaRegsFile           : > DMA, type=NOINIT

   ECap1RegsFile         : > ECAP1, type=NOINIT
   ECap2RegsFile         : > ECAP2, type=NOINIT
   ECap3RegsFile         : > ECAP3, type=NOINIT
   ECap4RegsFile         : > ECAP4, type=NOINIT
   ECap5RegsFile         : > ECAP5, type=NOINIT
   ECap6RegsFile         : > ECAP6, type=NOINIT
   ECap7RegsFile         : > ECAP7, type=NOINIT

   Emif1RegsFile         : > EMIF1, type=NOINIT
   Emif2RegsFile         : > EMIF2, type=NOINIT

   EPwm1RegsFile         : > EPWM1, type=NOINIT
   EPwm2RegsFile         : > EPWM2, type=NOINIT
   EPwm3RegsFile         : > EPWM3, type=NOINIT
   EPwm4RegsFile         : > EPWM4, type=NOINIT
   EPwm5RegsFile         : > EPWM5, type=NOINIT
   EPwm6RegsFile         : > EPWM6, type=NOINIT
   EPwm7RegsFile         : > EPWM7, type=NOINIT
   EPwm8RegsFile         : > EPWM8, type=NOINIT
   EPwm9RegsFile         : > EPWM9, type=NOINIT
   EPwm10RegsFile        : > EPWM10, type=NOINIT
   EPwm11RegsFile        : > EPWM11, type=NOINIT
   EPwm12RegsFile        : > EPWM12, type=NOINIT
   EPwm13RegsFile        : > EPWM13, type=NOINIT
   EPwm14RegsFile        : > EPWM14, type=NOINIT
   EPwm15RegsFile        : > EPWM15, type=NOINIT
   EPwm16RegsFile        : > EPWM16, type=NOINIT

   EQep1RegsFile         : > EQEP1, type=NOINIT
   EQep2RegsFile         : > EQEP2, type=NOINIT
   EQep3RegsFile         : > EQEP3, type=NOINIT

   EradGlobalRegsFile         : > ERAD_GLOBAL_REGISTERS, type=NOINIT
   EradHwbpRegsFile           : > ERAD_HWBP_REGISTERS, type=NOINIT
   EradCounterRegsFile        : > ERAD_COUNTER_REGISTERS, type=NOINIT
   EradCrcRegsFile            : > ERAD_CRC_REGISTERS, type=NOINIT
   Flash0CtrlRegsFile         : > FLASH0_CTRL, type=NOINIT
   Flash0EccRegsFile          : > FLASH0_ECC, type=NOINIT

   FsiTxaRegsFile             : > FSITXA, type=NOINIT
   FsiTxbRegsFile             : > FSI_TXB, type=NOINIT
   FsiRxaRegsFile             : > FSIRXA, type=NOINIT
   FsiRxbRegsFile             : > FSI_RXB, type=NOINIT
   FsiRxcRegsFile             : > FSI_RXC, type=NOINIT
   FsiRxdRegsFile             : > FSI_RXD, type=NOINIT
   FsiRxeRegsFile             : > FSI_RXE, type=NOINIT
   FsiRxfRegsFile             : > FSI_RXF, type=NOINIT
   FsiRxgRegsFile             : > FSI_RXG, type=NOINIT
   FsiRxhRegsFile             : > FSI_RXH, type=NOINIT

   GpioDataRegsFile           : > GPIODATA, type=NOINIT

   HRCap6RegsFile             : > HRCAP6, type=NOINIT
   HRCap7RegsFile             : > HRCAP7, type=NOINIT


   HwbistRegsFile             : > HWBIST, type=NOINIT
   I2caRegsFile          : > I2CA, type=NOINIT
   I2cbRegsFile          : > I2CB, type=NOINIT

   IpcRegsCPUFile           : > CPU2TOCPU1_IPC_CPU2VIEW, type=NOINIT
   IpcRegsCMFile            : > CPU2TOCM_IPC_CPU2VIEW, type=NOINIT

   MemCfgRegsFile            : > MEMCFG, type=NOINIT
   Emif1ConfigRegsFile       : > EMIF1CONFIG, type=NOINIT
   AccessProtectionRegsFile  : > ACCESSPROTECTION, type=NOINIT
   MemoryErrorRegsFile       : > MEMORYERROR, type=NOINIT

   McbspaRegsFile        : > MCBSPA, type=NOINIT
   McbspbRegsFile        : > MCBSPB, type=NOINIT

   NmiIntruptRegsFile    : > NMIINTRUPT, type=NOINIT

   PieCtrlRegsFile       : > PIE_CTRL, type=NOINIT

   PieVectTableFile      : > PIE_VECT_TABLE, type=NOINIT
   PmbusaRegsFile        : > PMBUSA, type=NOINIT

   SciaRegsFile          : > SCIA, type=NOINIT
   ScibRegsFile          : > SCIB, type=NOINIT
   ScicRegsFile          : > SCIC, type=NOINIT
   ScidRegsFile          : > SCID, type=NOINIT

   Sdfm1RegsFile         : > SDFM1, type=NOINIT
   Sdfm2RegsFile         : > SDFM2, type=NOINIT

   SpiaRegsFile          : > SPIA, type=NOINIT
   SpibRegsFile          : > SPIB, type=NOINIT
   SpicRegsFile          : > SPIC, type=NOINIT
   SpidRegsFile          : > SPID, type=NOINIT

   DmaClaSrcSelRegsFile  : > DMA_CLA_SRC_SEL, type=NOINIT
   ClkCfgRegsFile        : > CLK_CFG, type=NOINIT
   CpuSysRegsFile        : > CPU_SYS, type=NOINIT
   SysPeriphAcRegsFile   : > PERIPH_AC, type=NOINIT
   SysStatusRegsFile     : > SYS_STATUS, type=NOINIT

   WdRegsFile            : > WD, type=NOINIT

   XintRegsFile          : > XINT, type=NOINIT

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
