let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_sci.js");

function onChangeUseFIFO(inst, ui)
{
    onChangeUseInterrupts(inst,ui);
}

function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts) 
    {
        ui.enabledErrorInterrupts.hidden = false;

        if (inst.useFifo) 
        {
            ui.enabledFIFOInterrupts.hidden = false;
            ui.enabledInterrupts.hidden = true;

            ui.txFifo.hidden = false;
            ui.rxFifo.hidden = false;
        }
        else 
        {
            ui.enabledFIFOInterrupts.hidden = true;
            ui.enabledInterrupts.hidden = false;

            ui.txFifo.hidden = true;
            ui.rxFifo.hidden = true;
        }
    }
    else
    {
        ui.enabledInterrupts.hidden = true;
        ui.enabledFIFOInterrupts.hidden = true;
        ui.enabledErrorInterrupts.hidden = true;

        if (inst.useFifo) 
        {
            ui.txFifo.hidden = false;
            ui.rxFifo.hidden = false;
        }
        else 
        {
            ui.txFifo.hidden = true;
            ui.rxFifo.hidden = true;
        }
    }

    
}

/* Intro splash on GUI */
let longDescription = "An SCI peripheral is used to translate data between " +
    "the chip and a serial port. The SCI driver simplifies reading " +
    "and writing to any of the SCI(UART) peripherals on the board, with " +
    "multiple modes of operation and performance. These include " +
    "blocking, non-blocking, and polling, as well as text/binary " +
    "mode, echo and return characters.";

/* Array of SCI configurables that are common across device families */
let config = [
    {
        name        : "wordLenght",
        displayName : "Word Length",
        description : 'Length of word.',
        hidden      : false,
        default     : '8',
        options     : [
            { name: "1" },
            { name: "2" },
            { name: "3" },
            { name: "4" },
            { name: "5" },
            { name: "6" },
            { name: "7" },
            { name: "8" }
        ]
    },

    {
        name        : "stop",
        displayName : "Stop Mode",
        description : 'Number of stop bits.',
        hidden      : false,
        default     : 'ONE',
        options     : [
            { name: "ONE", displayName: "1" },
            { name: "TWO", displayName: "2" }
        ]
    },

    {
        name        : "parity",
        displayName : "Parity Mode",
        description : 'Parity mode.',
        hidden      : false,
        default     : device_driverlib_peripheral.SCI_ParityType[0].name,
        options     : device_driverlib_peripheral.SCI_ParityType
    },

    {
        name        : "useInterrupts",
        displayName : "Use Interrupt",
        description : 'Whether or not to use Interrupt mode.',
        hidden      : false,
        onChange    : onChangeUseInterrupts,
        default     : true
        
    },

    {
        name        : "enabledInterrupts",
        displayName : "Enabled Interrupts",
        description : 'Which interrupts to enabled.',
        hidden      : true,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "SCI_INT_RXRDY_BRKDT", displayName: "Receive Interrupt"},
            {name: "SCI_INT_TXRDY", displayName: "Transmit Interrupt"},
        ],
        
    },

    {
        name        : "enabledErrorInterrupts",
        displayName : "Enabled Error Interrupts",
        description : 'Which error interrupts to enabled.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "SCI_INT_RXERR", displayName: "Receive Error Interrupt"},
            {name: "SCI_INT_PE", displayName: "Parity Error Interrupt"},
            {name: "SCI_INT_FE", displayName: "Frame Error Interrupt"},
            {name: "SCI_INT_OE", displayName: "Overrun Error Interrupt"}
        ],
        
    },

    {
        name        : "enabledFIFOInterrupts",
        displayName : "Enabled FIFO Interrupts",
        description : 'Which FIFO interrupts to enabled.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "SCI_INT_RXFF", displayName: "Receive Interrupt"},
            {name: "SCI_INT_TXFF", displayName: "Transmit Interrupt"},
        ],
        
    },

    {
        name        : "useFifo",
        displayName : "Use FIFO",
        description : 'Whether or not to use FIFO mode.',
        hidden      : false,
        onChange    : onChangeUseFIFO,
        default     : true
        
    },

    {
        name        : "txFifo",
        displayName : "Transmit FIFO Interrupt Level",
        description : 'Transmit FIFO interrupt level used.',
        hidden      : false,
        default     : device_driverlib_peripheral.SCI_TxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.SCI_TxFIFOLevel
    },

    {
        name        : "rxFifo",
        displayName : "Receive FIFO Interrupt Level",
        description : 'Receive FIFO interrupt level used.',
        hidden      : false,
        default     : device_driverlib_peripheral.SCI_RxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.SCI_RxFIFOLevel
    },

    {
        name        : "loopback",
        displayName : "Use loopback Mode",
        description : 'Whether or not to use loopback mode.',
        hidden      : false,
        default     : false
        
    },
    {
        name        : "baudRates",
        displayName : "Baud Rate",
        description : 'The set of baud rates that are used by this instance. '
                      + 'On some devices, these rates are used to pre-compute '
                      + 'a table of clock dividers necessary for each rate.',
        hidden      : false,
        default     : 115200,
        options     : [
            { name:   1200 },
            { name:   2400 },
            { name:   4800 },
            { name:   9600 },
            { name:  19200 },
            { name:  38400 },
            { name:  57600 },
            { name: 115200 },
            { name: 230400 },
            { name: 460800 },
            { name: 921600 }
        ]
    },
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("SCI"),
        onChange    : Pinmux.useCaseChanged,
    },
];


/*
 *  ======== filterHardware ========
 *  Control RX, TX Pin usage by the user specified dataDirection.
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    return (Common.typeMatches(component.type, ["SCI"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var sciModule = {
    peripheralName: "SCI",
    displayName: "SCI",
    maxInstances: Common.peripheralCount("SCI"),
    defaultInstanceName: "mySCI",
    description: "Serial Communication Interface Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/sci/sci.board.c.xdt",
        boardh : "/driverlib/sci/sci.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.sciPinmuxRequirements,
};

if (sciModule.maxInstances <= 0)
{
    delete sciModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(sciModule)
}


exports = sciModule;