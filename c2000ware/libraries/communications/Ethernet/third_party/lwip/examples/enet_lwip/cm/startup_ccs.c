//###########################################################################
//
// FILE:   startup_ccs.c
//
// TITLE:  startup file for f2838x device.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//###########################################################################

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// External declaration for the reset handler that is to be called when the
// processor is started
//
//*****************************************************************************
extern void _c_int00(void);

//*****************************************************************************
//
// Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern unsigned long __STACK_END;

//*****************************************************************************
//
// External declarations for the interrupt handlers used by the application.
//
//*****************************************************************************
extern void lwIPEthernetIntHandler(void);
extern void SysTickIntHandler(void);
extern void Ethernet_genericISRCustom(void);

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000 or at the start of
// the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_ALIGN(vectorTableFlash, 1024)
#pragma DATA_SECTION(vectorTableFlash, ".vftable")

void (* const vectorTableFlash[])(void) =
{
 (void (*)(void))((unsigned long)&__STACK_END),
                                             /* The initial stack pointer */
     ResetISR,                               /* The reset handler         */
     NmiSR,                                  /* The NMI handler           */
     FaultISR,                               /* The hard fault handler    */
     IntDefaultHandler,                      /* The MPU fault handler     */
     IntDefaultHandler,                      /* The bus fault handler     */
     IntDefaultHandler,                      /* The usage fault handler   */
     0,                                      /* Reserved                  */
     0,                                      /* Reserved                  */
     0,                                      /* Reserved                  */
     0,                                      /* Reserved                  */
     IntDefaultHandler,                      /* SVCall handler            */
     IntDefaultHandler,                      /* Debug monitor handler     */
     0,                                      /* Reserved                  */
     IntDefaultHandler,                      /* The PendSV handler        */
     SysTickIntHandler,                      /* The SysTick handler       */
     IntDefaultHandler,                      /* MCANSS_0 handler          */
     IntDefaultHandler,                      /* MCANSS_1 handler          */
     IntDefaultHandler,                      /* MCANSS_WAKE_AND_TS_PLS ISR*/
     IntDefaultHandler,                      /* MCANSS_ECC_CORR_PLS ISR   */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* ECAT ISR                  */
     IntDefaultHandler,                      /* ECAT_SYNC0 ISR            */
     IntDefaultHandler,                      /* ECAT_SYNC1 ISR            */
     IntDefaultHandler,                      /* ECAT_RST ISR  ISR         */
     IntDefaultHandler,                      /* CANA0 ISR                 */
     IntDefaultHandler,                      /* CANA1 ISR                 */
     IntDefaultHandler,                      /* CANB0 ISR                 */
     IntDefaultHandler,                      /* CANB1 ISR                 */
     Ethernet_genericISRCustom,              /* EMAC ISR                  */
     lwIPEthernetIntHandler,                 /* EMAC_TX0 ISR              */
     IntDefaultHandler,                      /* EMAC_TX1 ISR              */
     lwIPEthernetIntHandler,                 /* EMAC_RX0 ISR              */
     IntDefaultHandler,                      /* EMAC_RX1 ISR              */
     IntDefaultHandler,                      /* UART0 ISR                 */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* SSI0 ISR                  */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* I2C0 ISR                  */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* USB ISR                   */
     IntDefaultHandler,                      /* UDMA_SW ISR               */
     IntDefaultHandler,                      /* UDMA_ERR ISR              */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* Reserved                  */
     IntDefaultHandler,                      /* CPU1TOCMIPC0 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC1 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC2 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC3 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC4 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC5 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC6 ISR          */
     IntDefaultHandler,                      /* CPU1TOCMIPC7 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC0 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC1 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC2 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC3 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC4 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC5 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC6 ISR          */
     IntDefaultHandler,                      /* CPU2TOCMIPC7 ISR          */
     IntDefaultHandler,                      /* FMC ISR                   */
     IntDefaultHandler,                      /* FMC_CORR_ERR ISR          */
     IntDefaultHandler,                      /* AES Interrupt ISR         */
     IntDefaultHandler,                      /* TIMER0 ISR                */
     IntDefaultHandler,                      /* TIMER1 ISR                */
     IntDefaultHandler,                      /* TIMER2 ISR                */
     IntDefaultHandler,                      /* CMRAM_TESTERROR_LOG ISR   */
     IntDefaultHandler,                      /* Reserved 52               */
     IntDefaultHandler,                      /* Reserved 53               */
     IntDefaultHandler,                      /* Reserved 54               */
     IntDefaultHandler,                      /* Reserved 55               */
     IntDefaultHandler,                      /* Reserved 56               */
     IntDefaultHandler,                      /* Reserved 57               */
     IntDefaultHandler,                      /* Reserved 58               */
     IntDefaultHandler,                      /* Reserved 59               */
     IntDefaultHandler,                      /* Reserved 60               */
     IntDefaultHandler,                      /* Reserved 61               */
     IntDefaultHandler,                      /* Reserved 62               */
     IntDefaultHandler                       /* Reserved 63               */
};

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
#pragma CODE_SECTION(ResetISR, ".resetisr")
void
ResetISR(void)
{
    // Jump to the CCS C Initialization Routine.
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}


//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

