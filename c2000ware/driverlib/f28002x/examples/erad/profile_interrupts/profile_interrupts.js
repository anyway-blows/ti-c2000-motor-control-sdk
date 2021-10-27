//#############################################################################
//
// profile_interrupts.js
//
// This JavaScript file is used to configure ERAD registers for use with the CCS
// example "erad_ex1_profileinterrupts".
//
// To properly use this script, the following variables must be set in the
// scripting environment prior to launching the ERAD script:
//
// - var PROJ_NAME = "erad_ex1_profileinterrupts"
// - var PROJ_WKSPC_LOC = "<proj_workspace_path>"
// - var PROJ_CONFIG = "<name of active configuration [CPU1_FLASH|CPU1_RAM]>"
//
// To run this ERAD script, use the following command in the scripting console:
//
// - loadJSFile("<proj_workspace_path>\\erad_ex1_profileinterrupts\\profile_interrupts.js", 0);
//
// This JavaScript file uses Debug Server Scripting (DSS) features. For
// information on using the DSS, please visit:
// http://software-dl.ti.com/ccs/esd/documents/users_guide/sdto_dss_handbook.html
//
//#############################################################################

//
// Import the DSS packages
//
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

//
// Load pre-defined macros for register addresses and useful functions
//
load(PROJ_WKSPC_LOC + "\\" + PROJ_NAME + "\\erad.js");
load(PROJ_WKSPC_LOC + "\\" + PROJ_NAME + "\\erad_util.js");

//
// Create our scripting environment object. This is the main entry point into
// any script and the factory for creating other scriptable servers and sessions
//
var script = ScriptingEnvironment.instance();

script.traceSetConsoleLevel(TraceLevel.ALL)

//
// Create a debug server
//
var ds = script.getServer( "DebugServer.1" );

//
// Open a debug session
//
debugSession = ds.openSession( "*", "C28xx_CPU1" );

//
// Variables for data read/write sizes
//
var DATA_SIZE_16 = 16;
var DATA_SIZE_32 = 32;

//*****************************************************************************
// Example: Profile Interrupts
//*****************************************************************************

//
// Set owner as debugger
//
debugSession.memory.writeData(Memory.Page.DATA,
                              EradGlobalRegs + GLBL_OWNER,
                              OWNER_DEBUGGER,
                              DATA_SIZE_16);

//
// Ensure that counters 1-4 are in idle mode (bits 15:12 are set to 0)
//
var counter1Status =
    debugSession.memory.readData(Memory.Page.DATA,
                                 EradCounter1Regs + CTM_STATUS,
                                 DATA_SIZE_16);

var counter2Status =
    debugSession.memory.readData(Memory.Page.DATA,
                                 EradCounter2Regs + CTM_STATUS,
                                 DATA_SIZE_16);

var counter3Status =
    debugSession.memory.readData(Memory.Page.DATA,
                                 EradCounter3Regs + CTM_STATUS,
                                 DATA_SIZE_16);

var counter4Status =
    debugSession.memory.readData(Memory.Page.DATA,
                                 EradCounter4Regs + CTM_STATUS,
                                 DATA_SIZE_16);

var CTM_STATUS_M = 0xF000;

if((0 == (counter1Status & CTM_STATUS_M)) &&
   (0 == (counter2Status & CTM_STATUS_M)) &&
   (0 == (counter3Status & CTM_STATUS_M)) &&
   (0 == (counter4Status & CTM_STATUS_M)))
{
    //
    // Reset HWBP_1 and HWBP_2
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP1Regs + HWBP_CLEAR,
                                  0x1,
                                  DATA_SIZE_16);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP2Regs + HWBP_CLEAR,
                                  0x1,
                                  DATA_SIZE_16);

    //
    // Get the start and end addresses of the Timer 2 ISR
    //
    var startTimer2ISR = debugSession.symbol.getAddress("cpuTimer2ISR");
    var endTimer2ISR = getFuncEndAddress("cpuTimer2ISR");

    //
    // Set two hardware breakpoints at the start and end of cpuTimer2ISR
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP1Regs + HWBP_REF,
                                  startTimer2ISR,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP2Regs + HWBP_REF,
                                  endTimer2ISR,
                                  DATA_SIZE_32);

    //
    // Set HWBP_1 and HWBP_2 to generate an event on PC match
    // BUS_SEL  -> VPC for program counter match
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP1Regs + HWBP_CNTL,
                                  0x0002,
                                  DATA_SIZE_16);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP2Regs + HWBP_CNTL,
                                  0x0002,
                                  DATA_SIZE_16);

    //
    // Reset Counters 1-4 globally
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradGlobalRegs + GLBL_CTM_RESET,
                                  0xF,
                                  DATA_SIZE_16);

    //
    // Clear CTM_COUNT
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    //
    // Clear CTM_MAX_COUNT
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    //
    // Clear EVENT_FIRED and OVERFLOW sticky bits
    // EVENT_CLEAR    -> Clear EVENT_FIRED sticky bit in CTM_STATUS
    // OVERFLOW_CLEAR -> Clear OVERFLOW sticky bit in CTM_STATUS
    // RESERVED       -> RESERVED
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_16);

    //
    // Clear CTM_REF
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_REF,
                                  0x0,
                                  DATA_SIZE_32);

    //
    // Configure CTM_1 to count cpuTimer2ISR execution cycles
    // CTM_INPUT_SEL
    // STA_INP_SEL -> Use HWBP_1 as start event for CTM_1
    // CTM_INPUT_SEL_MSB
    // STO_INP_SEL -> Use HWBP_2 as stop event for CTM_1
    //

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_INPUT_SEL,
                                  (0x0),
                                  DATA_SIZE_16);
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_INPUT_SEL_MSB,
                                  (0x1),
                                  DATA_SIZE_16);

    //
    // Configure CTM_1 to be in start-stop mode
    // START_STOP_MODE -> Enable start-stop mode
    //

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_CNTL,
                                  0x004,
                                  DATA_SIZE_16);

    //
    // Clear CTM_COUNT, CTM_MAX_COUNT, CTM_STATUS sticky bits
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_16);

    //
    // Configure CTM_2 to count number of times system event TIMER2_TINT2 occurs
    // CTM_INP_SEL    = 34   -> Count system event TIMER2_TINT2
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_INPUT_SEL,
                                  34,
                                  DATA_SIZE_16);
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_INPUT_SEL_MSB,
                                  0x0000,
                                  DATA_SIZE_16);

    //
    // Configure CTM_2 to be in rising-edge count mode
    // EVENT_MODE     -> Count only on rising-edge of count input
    // CNT_INP_SEL_EN -> Count only on rising-edge of count input
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_CNTL,
                                  0x0808,
                                  DATA_SIZE_16);

    //
    // Clear CTM_COUNT, CTM_MAX_COUNT, CTM_STATUS sticky bits
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_16);

    //
    // Configure CTM_3 to count number of times cputTimer2ISR executes
    // CNT_INP_SEL    -> Use HWBP_1 as count event
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_INPUT_SEL,
                                  0x0000,
                                  DATA_SIZE_16);

    //
    // Configure CTM_3 to be in rising-edge count mode
    // EVENT_MODE     -> Count only on rising-edge of count input    
    // CTM_INP_SEL_EN -> Enable CNT_INP_SEL
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_CNTL,
                                  0x0808,
                                  DATA_SIZE_16);

    //
    // Clear CTM_COUNT, CTM_MAX_COUNT, CTM_STATUS sticky bits
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_16);

    //
    // Configure CTM_4 to grab latency from TIMER2_TINT2 to cpuTimer2ISR
    // execution
    // STA_INP_SEL    = 34 -> Use TIMER2_TINT2 as start event for CTM_4
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_INPUT_SEL,
                                  0x2200,
                                  DATA_SIZE_16);

    // STO_INP_SEL    -> Use cpuTimer2ISR as stop event for CTM_4
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_INPUT_SEL_MSB,
                                  0x0,
                                  DATA_SIZE_16);

    //
    // Configure CTM_4 to be in start-stop mode
    // START_STOP_MODE = (1 <<  2)  -> Enable start-stop mode
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_CNTL,
                                  0x0004,
                                  DATA_SIZE_16);

    //
    // Enable HWBP_1-2 and CTM_1-4
    // HWBP_GLOBAL_ENABLE    = (1 << 0) | (1 << 1)
    // COUNTER_GLOBAL_ENABLE = (1 << 8) | (1 << 9) | (1 << 10) | (1 << 11)
    //
    var enableCounter =
        debugSession.memory.readData(Memory.Page.DATA,
                                     EradGlobalRegs + GLBL_ENABLE,
                                     DATA_SIZE_16);
    var enableBits = 0x0100 | 0x0200 | 0x0400 | 0x0800 | 0x0001 | 0x0002;
    enableCounter = enableCounter | enableBits;
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradGlobalRegs + GLBL_ENABLE,
                                  enableCounter,
                                  DATA_SIZE_16);

    print("\nCounter Started\n");

    var coreRan = 0;

    while(1)
    {
        var cycleCount =
            debugSession.memory.readData(Memory.Page.DATA,
                                         EradCounter1Regs + CTM_COUNT,
                                         DATA_SIZE_32);

        var cycleMax =
            debugSession.memory.readData(Memory.Page.DATA,
                                         EradCounter1Regs +
                                         CTM_MAX_COUNT,
                                         DATA_SIZE_32);

        var intEventCount =
            debugSession.memory.readData(Memory.Page.DATA,
                                         EradCounter2Regs + CTM_COUNT,
                                         DATA_SIZE_32);

        var isrExecCount =
            debugSession.memory.readData(Memory.Page.DATA,
                                         EradCounter3Regs + CTM_COUNT,
                                         DATA_SIZE_32);

        var isrDelayCycleMax =
            debugSession.memory.readData(Memory.Page.DATA,
                                         EradCounter4Regs +
                                         CTM_MAX_COUNT,
                                         DATA_SIZE_32);

        print("Current ISR Cycle Count = " + cycleCount.toString() +
              "\t Max ISR Cycle Count = " + cycleMax.toString() +
              "\t Int Occurence Count = " + intEventCount.toString() +
              "\t ISR Execution Count = " + isrExecCount.toString() +
              "\t ISR Delay Cycle Count = " + isrDelayCycleMax.toString());

        //
        // Halt script if CPU is halted
        //
        var halted = debugSession.target.isHalted();

        if(!halted)
        {
            coreRan = 1;
        }

        if(coreRan && halted)
        {
            break;
        }
    }

    //
    // Disable  HWBP_1-2 and CTM_1-4
    //
    enableCounter =
        debugSession.memory.readData(Memory.Page.DATA,
                                     EradGlobalRegs + GLBL_ENABLE,
                                     DATA_SIZE_16);
    enableCounter = enableCounter & (~enableBits);
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradGlobalRegs + GLBL_ENABLE,
                                  enableCounter,
                                  DATA_SIZE_16);

    //
    // Reset CTM_1-4 values
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter1Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter2Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter3Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_MAX_COUNT,
                                  0x0,
                                  DATA_SIZE_32);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradCounter4Regs + CTM_CLEAR,
                                  0x3,
                                  DATA_SIZE_32);

    //
    // Reset HWBP_1-2
    //
    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP1Regs + HWBP_CLEAR,
                                  0x1,
                                  DATA_SIZE_16);

    debugSession.memory.writeData(Memory.Page.DATA,
                                  EradHWBP2Regs + HWBP_CLEAR,
                                  0x1,
                                  DATA_SIZE_16);

    print("\nCounter Stopped\n");
}
else
{
    print("Failed\n");
}

//
// End of File
//