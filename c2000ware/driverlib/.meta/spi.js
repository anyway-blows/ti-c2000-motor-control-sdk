let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_spi.js");


/* Intro splash on GUI */
let longDescription = "The Serial Peripheral Interface (SPI) "
    + "driver is a generic, full-duplex driver that transmits "
    + "and receives data on a SPI bus. SPI is sometimes called SSI "
    + "(Synchronous Serial Interface). The SPI protocol defines "
    + "the format of a data transfer over the SPI bus, but it "
    + "leaves flow control, data formatting, and handshaking "
    + "mechanisms to higher-level software layers."


function onChangeUseFIFO(inst, ui)
{
    onChangeUseInterrupts(inst, ui);
}


function onValidate(inst, validation) 
{   
    var bitRateError = false;
    try{
        var bitRateInt = parseInt(inst.bitRate);
        if (bitRateInt < 1 || bitRateInt > 50000000)
        {
            bitRateError = true;
        }
    }
    catch (ex){
        bitRateError = true;
    }
    if(bitRateError)
    {
        validation.logError(
            "Enter an integer for bit rates between 1 and LSPCLK/4!", 
            inst, "bitRate");
    }
}


function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts)
    {
        if (inst.useFifo)
        {
            ui.enabledFIFOInterrupts.hidden = false;
            ui.enabledInterrupts.hidden = false;

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

        ui.enabledFIFOInterrupts.hidden = true;
        ui.enabledInterrupts.hidden = true;
        
        ui.txFifo.hidden = true;
        ui.rxFifo.hidden = true;
    }

}

/* Array of SPI configurables that are common across device families */
let config = [
    
    {
        name        : "transferProtocol",
        displayName : "Transfer Protocol",
        description : 'Mode for transfer protocol',
        hidden      : false,
        default     : device_driverlib_peripheral.SPI_TransferProtocol[0].name,
        options     : device_driverlib_peripheral.SPI_TransferProtocol
    },

    {
        name        : "mode",
        displayName : "Mode",
        description : 'Mode for device',
        hidden      : false,
        default     : device_driverlib_peripheral.SPI_Mode[0].name,
        options     : device_driverlib_peripheral.SPI_Mode
    },

    {
        name        : "stePolarity",
        displayName : "STE Polarity",
        description : 'Polarity of the STE Pin',
        hidden      : false,
        default     : device_driverlib_peripheral.SPI_STEPolarity[0].name,
        options     : device_driverlib_peripheral.SPI_STEPolarity
    },

    {
        name        : "emulationMode",
        displayName : "Emulation Mode",
        description : 'Emulation Mode',
        hidden      : false,
        default     : device_driverlib_peripheral.SPI_EmulationMode[0].name,
        options     : device_driverlib_peripheral.SPI_EmulationMode
    },

    {
        name        : "bitRate",
        displayName : "Bit Rate (Hz)",
        description : 'Bit rate for device in (Hz). Cannot exceed LSPCLK/4',
        hidden      : false,
        default     : 25000
    },

    {
        name        : "dataWidth",
        displayName : "Data Width",
        description : 'Data width used for transmission',
        hidden      : false,
        default     : '16',
        options     : [
            {name: "1"},
            {name: "2"},
            {name: "3"},
            {name: "4"},
            {name: "5"},
            {name: "6"},
            {name: "7"},
            {name: "8"},
            {name: "9"},
            {name: "10"},
            {name: "11"},
            {name: "12"},
            {name: "13"},
            {name: "14"},
            {name: "15"},
            {name: "16"}
        ]
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
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "SPI_INT_RX_DATA_TX_EMPTY", displayName: "Receive Interrupt"},
            {name: "SPI_INT_RX_OVERRUN", displayName: "Receive Overrun Interrupt"},
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
            {name: "SPI_INT_RXFF", displayName: "Receive Interrupt"},
            {name: "SPI_INT_RXFF_OVERFLOW", displayName: "Receive Overflow Interrupt"},
            {name: "SPI_INT_TXFF", displayName: "Transmit Interrupt"},
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
        default     : device_driverlib_peripheral.SPI_TxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.SPI_TxFIFOLevel
    },

    {
        name        : "rxFifo",
        displayName : "Receive FIFO Interrupt Level",
        description : 'Receive FIFO interrupt level used.',
        hidden      : false,
        default     : device_driverlib_peripheral.SPI_RxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.SPI_RxFIFOLevel
    },

    {
        name        : "loopback",
        displayName : "Use loopback Mode",
        description : 'Whether or not to use loopback mode.',
        hidden      : false,
        default     : false
        
    },
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("SPI"),
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
    return (Common.typeMatches(component.type, ["SPI"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var spiModule = {
    peripheralName: "SPI",
    displayName: "SPI",
    maxInstances: Common.peripheralCount("SPI"),
    defaultInstanceName: "mySPI",
    description: "Serial Peripheral Interface Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/spi/spi.board.c.xdt",
        boardh : "/driverlib/spi/spi.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.spiPinmuxRequirements,
    validate    : onValidate
};


if (spiModule.maxInstances <= 0)
{
    delete spiModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(spiModule)
}


exports = spiModule;