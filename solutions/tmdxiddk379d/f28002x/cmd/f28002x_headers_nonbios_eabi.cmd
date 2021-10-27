MEMORY
{
   ADCA_RESULT   : origin = 0x000B00, length = 0x000020
   ADCB_RESULT   : origin = 0x000B20, length = 0x000020
   ADCC_RESULT   : origin = 0x000B40, length = 0x000020
   ADCA          : origin = 0x007400, length = 0x000080
   ADCB          : origin = 0x007480, length = 0x000080
   ADCC          : origin = 0x007500, length = 0x000080

   ANALOG_SUBSYS : origin = 0x05D700, length = 0x000100

   CANA          : origin = 0x048000, length = 0x000800
   CANB          : origin = 0x04A000, length = 0x000800

   CLA1          : origin = 0x001400, length = 0x000080     /* CLA registers */

   CLAPROMCRC    : origin = 0x0061C0, length = 0x000020

   CLB_XBAR      : origin = 0x007A40, length = 0x000040

   CMPSS1        : origin = 0x005C80, length = 0x000020
   CMPSS2        : origin = 0x005CA0, length = 0x000020
   CMPSS3        : origin = 0x005CC0, length = 0x000020
   CMPSS4        : origin = 0x005CE0, length = 0x000020
   CMPSS5        : origin = 0x005D00, length = 0x000020
   CMPSS6        : origin = 0x005D20, length = 0x000020
   CMPSS7        : origin = 0x005D40, length = 0x000020

   CPU_TIMER0    : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
   CPU_TIMER1    : origin = 0x000C08, length = 0x000008     /* CPU Timer1 registers */
   CPU_TIMER2    : origin = 0x000C10, length = 0x000008     /* CPU Timer2 registers */

   DACA          : origin = 0x005C00, length = 0x000010
   DACB          : origin = 0x005C10, length = 0x000010

   DCC0          : origin = 0x05E700, length = 0x000040

   DCSM_BANK0_Z1    : origin = 0x05F000, length = 0x000030
   DCSM_BANK0_Z2    : origin = 0x05F040, length = 0x000030
   DCSM_BANK1_Z1    : origin = 0x05F100, length = 0x000030
   DCSM_BANK1_Z2    : origin = 0x05F140, length = 0x000030
   DCSM_COMMON      : origin = 0x05F070, length = 0x000010     /* Common Dual code security module registers */

   DMA          : origin = 0x001000, length = 0x000200

   ECAP1        : origin = 0x005200, length = 0x000040     /* Enhanced Capture 1 registers */
   ECAP2        : origin = 0x005240, length = 0x000040     /* Enhanced Capture 2 registers */
   ECAP3        : origin = 0x005280, length = 0x000040     /* Enhanced Capture 3 registers */
   ECAP4        : origin = 0x0052C0, length = 0x000040     /* Enhanced Capture 4 registers */
   ECAP5        : origin = 0x005300, length = 0x000040     /* Enhanced Capture 5 registers */
   ECAP6        : origin = 0x005340, length = 0x000040     /* Enhanced Capture 6 registers */
   ECAP7        : origin = 0x005380, length = 0x000040     /* Enhanced Capture 7 registers */

   PGA1         : origin = 0x005B00, length = 0x000010
   PGA2         : origin = 0x005B10, length = 0x000010
   PGA3         : origin = 0x005B20, length = 0x000010
   PGA4         : origin = 0x005B30, length = 0x000010
   PGA5         : origin = 0x005B40, length = 0x000010
   PGA6         : origin = 0x005B50, length = 0x000010
   PGA7         : origin = 0x005B60, length = 0x000010

   EPWM1        : origin = 0x004000, length = 0x000100     /* Enhanced PWM 1 registers */
   EPWM2        : origin = 0x004100, length = 0x000100     /* Enhanced PWM 2 registers */
   EPWM3        : origin = 0x004200, length = 0x000100     /* Enhanced PWM 3 registers */
   EPWM4        : origin = 0x004300, length = 0x000100     /* Enhanced PWM 4 registers */
   EPWM5        : origin = 0x004400, length = 0x000100     /* Enhanced PWM 5 registers */
   EPWM6        : origin = 0x004500, length = 0x000100     /* Enhanced PWM 6 registers */
   EPWM7        : origin = 0x004600, length = 0x000100     /* Enhanced PWM 7 registers */
   EPWM8        : origin = 0x004700, length = 0x000100     /* Enhanced PWM 8 registers */

   EPWM_XBAR    : origin = 0x007A00, length = 0x000040

   EQEP1        : origin = 0x005100, length = 0x000040     /* Enhanced QEP 1 registers */
   EQEP2        : origin = 0x005140, length = 0x000040     /* Enhanced QEP 2 registers */

   FLASH0_CTRL  : origin = 0x05F800, length = 0x000300
   FLASH0_ECC   : origin = 0x05FB00, length = 0x000040

   FSITXA       : origin = 0x006600, length = 0x000080
   FSIRXA       : origin = 0x006680, length = 0x000080

   GPIOCTRL     : origin = 0x007C00, length = 0x000200     /* GPIO control registers */
   GPIODAT      : origin = 0x007F00, length = 0x000040     /* GPIO data registers */

   I2CA         : origin = 0x007300, length = 0x000040     /* I2C-A registers */

   INPUT_XBAR   : origin = 0x007900, length = 0x000020

   LINA         : origin = 0x006A00, length = 0x000100
   LINB         : origin = 0x006B00, length = 0x000100

   MEMCFG       : origin = 0x05F400, length = 0x000080     /* Mem Config registers */
   ACCESSPROTECTION  : origin = 0x05F4C0, length = 0x000040     /* Access Protection registers */
   MEMORYERROR  : origin = 0x05F500, length = 0x000040     /* Access Protection registers */

   NMIINTRUPT   : origin = 0x007060, length = 0x000010     /* NMI Watchdog Interrupt Registers */

   OUTPUT_XBAR  : origin = 0x007A80, length = 0x000040

   PIE_CTRL     : origin = 0x000CE0, length = 0x000020     /* PIE control registers */

   PIE_VECT     : origin = 0x000D00, length = 0x000200     /* PIE Vector Table */

   PMBUSA       : origin = 0x006400, length = 0x000020

   SCIA         : origin = 0x007200, length = 0x000010     /* SCI-A registers */
   SCIB         : origin = 0x007210, length = 0x000010     /* SCI-B registers */

   SDFM1        : origin = 0x005E00, length = 0x000080     /* Sigma delta 1 registers */

   SPIA         : origin = 0x006100, length = 0x000010
   SPIB         : origin = 0x006110, length = 0x000010

   WD           : origin = 0x007000, length = 0x000040
   DMACLASRCSEL : origin = 0x007980, length = 0x000040
   DEV_CFG      : origin = 0x05D000, length = 0x000180
   CLK_CFG      : origin = 0x05D200, length = 0x000100
   CPU_SYS      : origin = 0x05D300, length = 0x000100
   PERIPH_AC    : origin = 0x05D500, length = 0x000200

   ERAD_GLOBAL  : origin = 0x05E800, length = 0x000013
   ERAD_HWBP1   : origin = 0x05E900, length = 0x000008
   ERAD_HWBP2   : origin = 0x05E908, length = 0x000008
   ERAD_HWBP3   : origin = 0x05E910, length = 0x000008
   ERAD_HWBP4   : origin = 0x05E918, length = 0x000008
   ERAD_HWBP5   : origin = 0x05E920, length = 0x000008
   ERAD_HWBP6   : origin = 0x05E928, length = 0x000008
   ERAD_HWBP7   : origin = 0x05E930, length = 0x000008
   ERAD_HWBP8   : origin = 0x05E938, length = 0x000008
   ERAD_CTR1    : origin = 0x05E980, length = 0x000010
   ERAD_CTR2    : origin = 0x05E990, length = 0x000010
   ERAD_CTR3    : origin = 0x05E9A0, length = 0x000010
   ERAD_CTR4    : origin = 0x05E9B0, length = 0x000010

   XBAR         : origin = 0x007920, length = 0x000020
   SYNC_SOC     : origin = 0x007940, length = 0x000010

   XINT         : origin = 0x007070, length = 0x000010
}

SECTIONS
{
/*** PIE Vect Table and Boot ROM Variables Structures ***/
  UNION run = PIE_VECT
   {
      PieVectTableFile
      GROUP
      {
         EmuKeyVar
         EmuBModeVar
         FlashCallbackVar
         FlashScalingVar
      }
   }

   AdcaResultFile        : > ADCA_RESULT,   type=NOINIT
   AdcbResultFile        : > ADCB_RESULT,   type=NOINIT
   AdccResultFile        : > ADCC_RESULT,   type=NOINIT

   AdcaRegsFile          : > ADCA,          type=NOINIT
   AdcbRegsFile          : > ADCB,          type=NOINIT
   AdccRegsFile          : > ADCC,          type=NOINIT

   AnalogSubsysRegsFile  : > ANALOG_SUBSYS, type=NOINIT

   CanaRegsFile          : > CANA,          type=NOINIT
   CanbRegsFile          : > CANB,          type=NOINIT

   Cla1RegsFile          : > CLA1,          type=NOINIT
   Cla1SoftIntRegsFile   : > PIE_CTRL,      type=NOINIT, type=DSECT

   ClaPromCrc0RegsFile   : > CLAPROMCRC,    type=NOINIT

   ClbXbarRegsFile       : > CLB_XBAR,      type=NOINIT

   Cmpss1RegsFile        : > CMPSS1,        type=NOINIT
   Cmpss2RegsFile        : > CMPSS2,        type=NOINIT
   Cmpss3RegsFile        : > CMPSS3,        type=NOINIT
   Cmpss4RegsFile        : > CMPSS4,        type=NOINIT
   Cmpss5RegsFile        : > CMPSS5,        type=NOINIT
   Cmpss6RegsFile        : > CMPSS6,        type=NOINIT
   Cmpss7RegsFile        : > CMPSS7,        type=NOINIT

   CpuTimer0RegsFile     : > CPU_TIMER0,    type=NOINIT
   CpuTimer1RegsFile     : > CPU_TIMER1,    type=NOINIT
   CpuTimer2RegsFile     : > CPU_TIMER2,    type=NOINIT

   DacaRegsFile          : > DACA           type=NOINIT
   DacbRegsFile          : > DACB           type=NOINIT

   Dcc0RegsFile          : > DCC0           type=NOINIT

   DcsmBank0Z1RegsFile        : > DCSM_BANK0_Z1,         type=NOINIT
   DcsmBank0Z2RegsFile        : > DCSM_BANK0_Z2,         type=NOINIT
   DcsmBank1Z1RegsFile        : > DCSM_BANK1_Z1,         type=NOINIT
   DcsmBank1Z2RegsFile        : > DCSM_BANK1_Z2,         type=NOINIT
   DcsmCommonRegsFile         : > DCSM_COMMON,           type=NOINIT

   DmaRegsFile                : > DMA,                   type=NOINIT

   ECap1RegsFile         : > ECAP1,        type=NOINIT
   ECap2RegsFile         : > ECAP2,        type=NOINIT
   ECap3RegsFile         : > ECAP3,        type=NOINIT
   ECap4RegsFile         : > ECAP4,        type=NOINIT
   ECap5RegsFile         : > ECAP5,        type=NOINIT
   ECap6RegsFile         : > ECAP6,        type=NOINIT
   ECap7RegsFile         : > ECAP7,        type=NOINIT

   Pga1RegsFile          : > PGA1,         type=NOINIT
   Pga2RegsFile          : > PGA2,         type=NOINIT
   Pga3RegsFile          : > PGA3,         type=NOINIT
   Pga4RegsFile          : > PGA4,         type=NOINIT
   Pga5RegsFile          : > PGA5,         type=NOINIT
   Pga6RegsFile          : > PGA6,         type=NOINIT
   Pga7RegsFile          : > PGA7,         type=NOINIT

   EPwm1RegsFile         : > EPWM1,        type=NOINIT
   EPwm2RegsFile         : > EPWM2,        type=NOINIT
   EPwm3RegsFile         : > EPWM3,        type=NOINIT
   EPwm4RegsFile         : > EPWM4,        type=NOINIT
   EPwm5RegsFile         : > EPWM5,        type=NOINIT
   EPwm6RegsFile         : > EPWM6,        type=NOINIT
   EPwm7RegsFile         : > EPWM7,        type=NOINIT
   EPwm8RegsFile         : > EPWM8,        type=NOINIT

   EPwmXbarRegsFile      : > EPWM_XBAR     type=NOINIT

   EQep1RegsFile         : > EQEP1,        type=NOINIT
   EQep2RegsFile         : > EQEP2,        type=NOINIT

   EradGlobalRegsFile    : > ERAD_GLOBAL, type=NOINIT
   EradHWBP1RegsFile     : > ERAD_HWBP1,  type=NOINIT
   EradHWBP2RegsFile     : > ERAD_HWBP2,  type=NOINIT
   EradHWBP3RegsFile     : > ERAD_HWBP3,  type=NOINIT
   EradHWBP4RegsFile     : > ERAD_HWBP4,  type=NOINIT
   EradHWBP5RegsFile     : > ERAD_HWBP5,  type=NOINIT
   EradHWBP6RegsFile     : > ERAD_HWBP6,  type=NOINIT
   EradHWBP7RegsFile     : > ERAD_HWBP7,  type=NOINIT
   EradHWBP8RegsFile     : > ERAD_HWBP8,  type=NOINIT
   EradCounter1RegsFile  : > ERAD_CTR1,   type=NOINIT
   EradCounter2RegsFile  : > ERAD_CTR2,   type=NOINIT
   EradCounter3RegsFile  : > ERAD_CTR3,   type=NOINIT
   EradCounter4RegsFile  : > ERAD_CTR4,   type=NOINIT

   Flash0CtrlRegsFile    : > FLASH0_CTRL    type=NOINIT
   Flash0EccRegsFile     : > FLASH0_ECC     type=NOINIT

   FsiTxaRegsFile        : > FSITXA         type=NOINIT
   FsiRxaRegsFile        : > FSIRXA         type=NOINIT

   GpioCtrlRegsFile      : > GPIOCTRL,      type=NOINIT
   GpioDataRegsFile      : > GPIODAT,       type=NOINIT

   I2caRegsFile          : > I2CA,          type=NOINIT

   InputXbarRegsFile     : > INPUT_XBAR     type=NOINIT
   XbarRegsFile          : > XBAR           type=NOINIT

   LinaRegsFile          : > LINA,          type=NOINIT
   LinbRegsFile          : > LINB,          type=NOINIT

   MemCfgRegsFile            : > MEMCFG,            type=NOINIT
   AccessProtectionRegsFile  : > ACCESSPROTECTION,  type=NOINIT
   MemoryErrorRegsFile       : > MEMORYERROR,       type=NOINIT

   NmiIntruptRegsFile       : > NMIINTRUPT,     type=NOINIT

   OutputXbarRegsFile       : > OUTPUT_XBAR,    type=NOINIT

   PieCtrlRegsFile          : > PIE_CTRL,       type=NOINIT

   PmbusaRegsFile           : > PMBUSA,         type=NOINIT

   SciaRegsFile             : > SCIA,           type=NOINIT
   ScibRegsFile             : > SCIB,           type=NOINIT

   Sdfm1RegsFile            : > SDFM1,          type=NOINIT

   SpiaRegsFile             : > SPIA,           type=NOINIT
   SpibRegsFile             : > SPIB,           type=NOINIT

   WdRegsFile               : > WD,             type=NOINIT
   DmaClaSrcSelRegsFile     : > DMACLASRCSEL    type=NOINIT
   DevCfgRegsFile           : > DEV_CFG,        type=NOINIT
   ClkCfgRegsFile           : > CLK_CFG,        type=NOINIT
   CpuSysRegsFile           : > CPU_SYS,        type=NOINIT
   SysPeriphAcRegsFile      : > PERIPH_AC,      type=NOINIT

   SyncSocRegsFile          : > SYNC_SOC,       type=NOINIT

   XintRegsFile             : > XINT,           type=NOINIT

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
