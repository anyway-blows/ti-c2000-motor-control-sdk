// Import the DSS packages into our namespace to save on typing
importPackage(Packages.com.ti.debug.engine.scripting)
importPackage(Packages.com.ti.ccstudio.scripting.environment)
importPackage(Packages.java.lang)

// ERAD counter register addresses
var GLBL_OWNER = 0x0005E80A;
var GLBL_ENABLE = 0x0005E804;
var GLBL_CTM_RESET = 0x0005E806;

var CTM_1 = 0x0005E980;
var CTM_2 = 0x0005E990;
var CTM_3 = 0x0005E9A0;
var CTM_4 = 0x0005E9B0;

// ERAD counter register offsets
var CTM_STATUS = 0x1;
var CTM_INPUT_SEL = 0x8;
var CTM_CNTL = 0x0;
var CTM_COUNT = 0x4;
var CTM_REF = 0x2;
var CTM_MAX = 0x6;
var CTM_CLEAR = 0x9;

var OWNER_DEBUGGER = 0x2;

// ERAD comparator register addresses
var HWBP_1 = 0x0005E900;
var HWBP_2 = 0x0005E908;
var HWBP_3 = 0x0005E910;

// ERAD comparator register offsets
var HWBP_REF = 0x2;
var HWBP_CLEAR = 0x4;
var HWBP_CNTL = 0x6;



function print_stats(debugSession)
{
    var cycle_count = debugSession.memory.readData(Memory.Page.DATA, CTM_1 + CTM_COUNT, 32);
    var cycle_max = debugSession.memory.readData(Memory.Page.DATA, CTM_1 + CTM_MAX, 32);
    var int_event_count = debugSession.memory.readData(Memory.Page.DATA, CTM_2 + CTM_COUNT, 32);
    var isr_delay_cycle_max = debugSession.memory.readData(Memory.Page.DATA, CTM_3 + CTM_MAX, 32);
    
    print("INT occurrence count = " + int_event_count.toString() +
          " \t INT to ISR cycle count (max) = " + isr_delay_cycle_max +
          " \t ISR cycle count(max) = " + cycle_max.toString());
}

function release_resources(debugSession)
{
  // Disable counter and HWBP0 and HWBP1
  var enable_counter = debugSession.memory.readData(Memory.Page.DATA, GLBL_ENABLE, 16);
  enable_counter = enable_counter & (~enable_bits); // GLBL_ENABLE disable counter
  debugSession.memory.writeData(Memory.Page.DATA, GLBL_ENABLE, enable_counter, 16);
  
  // Reset count
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_CLEAR, 0x3, 32);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_CLEAR, 0x3, 32);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_CLEAR, 0x3, 32);

  debugSession.memory.writeData(Memory.Page.DATA, HWBP_1+HWBP_CLEAR, 0x1, 16);
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_2+HWBP_CLEAR, 0x1, 16);
}


// START OF MAIN SCRIPT

// Create our scripting environment object - which is the main entry point into any script and
// the factory for creating other Scriptable ervers and Sessions
var script = ScriptingEnvironment.instance();

script.traceSetConsoleLevel(TraceLevel.ALL)

// Create a debug server
var ds = script.getServer( "DebugServer.1" );

// Open a debug session
debugSession = ds.openSession( "*","C28xx_CPU1" );


// Set owner as debugger
debugSession.memory.writeData(Memory.Page.DATA, GLBL_OWNER, OWNER_DEBUGGER, 16);

// Set counter
var ctr_stat1 = debugSession.memory.readData(Memory.Page.DATA, CTM_1 + CTM_STATUS, 16);
var ctr_stat2 = debugSession.memory.readData(Memory.Page.DATA, CTM_2 + CTM_STATUS, 16);
var ctr_stat3 = debugSession.memory.readData(Memory.Page.DATA, CTM_3 + CTM_STATUS, 16);

if (0 == (ctr_stat1 & 0xF000) && 0 == (ctr_stat2 & 0xF000) && 0 == (ctr_stat3 & 0xF000)) // 15:12 == 0 in idle mode
{
  // Reset Comparator 0 and 1 
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_1+HWBP_CLEAR, 0x1, 16);
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_2+HWBP_CLEAR, 0x1, 16);

  // Note: The ISR addresses map to "SignalChain_RAM_TMU" build configuration.
  // Update the addresses if running application from a different build configuration
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_1+HWBP_REF, 0xcd05, 32); // ISR entry address (first line in ISR function)
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_2+HWBP_REF, 0xd531, 32); // ISR exit address (last IRET instruction in ISR function)

  debugSession.memory.writeData(Memory.Page.DATA, HWBP_1+HWBP_CNTL, 0x0004, 16); // VPC match, generate watchpoint
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_2+HWBP_CNTL, 0x0004, 16); // VPC match, generate watchpoint
  
  // Reset counter 0, 1 and 2
  debugSession.memory.writeData(Memory.Page.DATA, GLBL_CTM_RESET, 0x7, 16);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_CLEAR, 0x3, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_REF, 0x0, 32);

  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_INPUT_SEL, 0x0800, 16); // CTM_INPUT_SEL1 set start to Comparator 1(ISR entry) and stop to Comparator 2(ISR exit).
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_CNTL, 0x0004, 16); // CTM_CNTL1 set control to START-STOP mode

  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_CLEAR, 0x3, 32);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_INPUT_SEL, 0x19, 16); // CTM_INPUT_SEL2 set count event 12(ADCAINT1 - Int1).
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_CNTL, 0x0008, 16); // CTM_CNTL2 in EVENT MODE rising edge
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_CLEAR, 0x3, 32);

  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_INPUT_SEL, 0x0300, 16); // CTM_INPUT_SEL4 set start to ADCAINT1 and stop to Comparator 1
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_CNTL, 0x0004, 16); // CTM_CNTL1 set control to START-STOP mode

  var enable_counter = debugSession.memory.readData(Memory.Page.DATA, GLBL_ENABLE, 16);
  var enable_bits = 0x0100 | 0x0200 | 0x0400 | 0x0800 | 0x0001 | 0x0002 | 0x0004;
  enable_counter = enable_counter | enable_bits; // GLBL_ENABLE enable counter
  debugSession.memory.writeData(Memory.Page.DATA, GLBL_ENABLE, enable_counter, 16);

  var core_ran = 0;
  
  print("\nStats at start of test:");
  print_stats(debugSession);
  
  // Wait for application to be run and subsequently for core to be halted
  var halted;
  while(1)
  {
    halted = debugSession.target.isHalted();
    if (!halted)
    {
      core_ran = 1;
    }
    if (core_ran && halted)
    {
        break;
    }
  }
  
  print("\nStats at end of test:");
  print_stats(debugSession);
  

  // Disable counter and HWBP0 and HWBP1
  enable_counter = debugSession.memory.readData(Memory.Page.DATA, GLBL_ENABLE, 16);
  enable_counter = enable_counter & (~enable_bits); // GLBL_ENABLE disable counter
  debugSession.memory.writeData(Memory.Page.DATA, GLBL_ENABLE, enable_counter, 16);
  
  // Reset count
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_1 + CTM_CLEAR, 0x3, 32);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_2 + CTM_CLEAR, 0x3, 32);
  
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_COUNT, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_MAX, 0x0, 32);
  debugSession.memory.writeData(Memory.Page.DATA, CTM_3 + CTM_CLEAR, 0x3, 32);

  debugSession.memory.writeData(Memory.Page.DATA, HWBP_1+HWBP_CLEAR, 0x1, 16);
  debugSession.memory.writeData(Memory.Page.DATA, HWBP_2+HWBP_CLEAR, 0x1, 16);
  
  release_resources(debugSession);
}
else
{
  print("Resources not in idle state. Rerun script.");
  release_resources(debugSession);
}


// Terminate session
//ds.stop();
//debugSession.terminate();