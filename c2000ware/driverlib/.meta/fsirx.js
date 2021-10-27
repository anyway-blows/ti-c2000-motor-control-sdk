let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");



let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_fsi.js");

/* Intro splash on GUI */
let longDescription = "The Fast Serial Interface (FSI) module is a serial " +
                    "communication peripheral capable of reliable high-speed " + 
                    "communication across isolation devices.";

function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts.indexOf(device_driverlib_peripheral.FSI_InterruptNum[0].name) != -1) 
    {
        ui.enabledINT1Interrupts.hidden = false;
    }
    else
    {
        ui.enabledINT1Interrupts.hidden = true;
    }

    if (inst.useInterrupts.indexOf(device_driverlib_peripheral.FSI_InterruptNum[1].name) != -1) 
    {
        ui.enabledINT2Interrupts.hidden = false;
    }
    else
    {
        ui.enabledINT2Interrupts.hidden = true;
    }
}
/* Array of CAN configurables that are common across device families */
let config = [
    

    {
        name        : "loopback",
        displayName : "Loopback Mode",
        description : 'Whether or not to use internal loopback mode',
        hidden      : false,
        default     : false
    },

    {
        name        : "dataWidth",
        displayName : "Data Width",
        description : 'Number of lanes for communication',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_DataWidth[0].name,
        options     : device_driverlib_peripheral.FSI_DataWidth,
    },

    {
        name        : "frameType",
        displayName : "Type of Frame",
        description : 'The type of frame for data transmission',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_FrameType[0].name,
        options     : device_driverlib_peripheral.FSI_FrameType
    },

    {
        name        : "softwareFrameSize",
        displayName : "Software Frame Size",
        description : 'When selecting N-WORD frame type, this determines number of words (N)',
        hidden      : false,
        default     : '8',
        options     : [
            { name: "1" , displayName : "1-Word"},
            { name: "2" , displayName : "2-Word"},
            { name: "3" , displayName : "3-Word"},
            { name: "4" , displayName : "4-Word"},
            { name: "5" , displayName : "5-Word"},
            { name: "6" , displayName : "6-Word"},
            { name: "7" , displayName : "7-Word"},
            { name: "8" , displayName : "8-Word"},
            { name: "9" , displayName : "9-Word"},
            { name: "10" , displayName : "10-Word"},
            { name: "11" , displayName : "11-Word"},
            { name: "12" , displayName : "12-Word"},
            { name: "13" , displayName : "13-Word"},
            { name: "14" , displayName : "14-Word"},
            { name: "15" , displayName : "15-Word"},
            { name: "16" , displayName : "16-Word" }
        ]
    },

    {
        name        : "pingTimeout",
        displayName : "PING Timeout Mode",
        description : 'The type of timeout mode for PING frames',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_PingTimeoutMode[0].name,
        options     : device_driverlib_peripheral.FSI_PingTimeoutMode
    },


    {
        name        : "useInterrupts",
        displayName : "Use Interrupt",
        description : 'Whether or not to use Interrupt mode.',
        onChange    : onChangeUseInterrupts,
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : device_driverlib_peripheral.FSI_InterruptNum
        
    },

    {
        name        : "enabledINT1Interrupts",
        displayName : "Enabled INT1 Interrupts",
        description : 'Which interrupts to be enabled for INT1.',
        hidden      : true,
        default     : [],
        minSelections: 0,
        options     : [
            {name:"FSI_RX_EVT_PING_WD_TIMEOUT", displayName: "Ping watchdog times out"},
            {name:"FSI_RX_EVT_FRAME_WD_TIMEOUT", displayName: "Frame watchdog times out"},
            {name:"FSI_RX_EVT_CRC_ERR", displayName: "Mismatch hardware computed CRC and received CRC"},
            {name:"FSI_RX_EVT_TYPE_ERR", displayName: "Invalid Frame type detected"},
            {name:"FSI_RX_EVT_EOF_ERR", displayName: "Invalid EndofFrame bit-pattern"},
            {name:"FSI_RX_EVT_OVERRUN", displayName: "Buffer Overrun in Rx buffer"},
            {name:"FSI_RX_EVT_FRAME_DONE", displayName: "Received frame without errors"},
            {name:"FSI_RX_EVT_UNDERRUN", displayName: "Software reads empty Rx buffer"},
            {name:"FSI_RX_EVT_ERR_FRAME", displayName: "Received error frame"},
            {name:"FSI_RX_EVT_PING_FRAME", displayName: "Received ping frame"},
            {name:"FSI_RX_EVT_FRAME_OVERRUN", displayName: "Software didn't clear FRAME_DONE after receiving new frame"},
            {name:"FSI_RX_EVT_DATA_FRAME", displayName: "Received data frame"},

        ],
        
    },

        {
        name        : "enabledINT2Interrupts",
        displayName : "Enabled INT2 Interrupts",
        description : 'Which interrupts to be enabled for INT2.',
        hidden      : true,
        default     : [],
        minSelections: 0,
        options     : [
            {name:"FSI_RX_EVT_PING_WD_TIMEOUT", displayName: "Ping watchdog times out"},
            {name:"FSI_RX_EVT_FRAME_WD_TIMEOUT", displayName: "Frame watchdog times out"},
            {name:"FSI_RX_EVT_CRC_ERR", displayName: "Mismatch hardware computed CRC and received CRC"},
            {name:"FSI_RX_EVT_TYPE_ERR", displayName: "Invalid Frame type detected"},
            {name:"FSI_RX_EVT_EOF_ERR", displayName: "Invalid EndofFrame bit-pattern"},
            {name:"FSI_RX_EVT_OVERRUN", displayName: "Buffer Overrun in Rx buffer"},
            {name:"FSI_RX_EVT_FRAME_DONE", displayName: "Received frame without errors"},
            {name:"FSI_RX_EVT_UNDERRUN", displayName: "Software reads empty Rx buffer"},
            {name:"FSI_RX_EVT_ERR_FRAME", displayName: "Received error frame"},
            {name:"FSI_RX_EVT_PING_FRAME", displayName: "Received ping frame"},
            {name:"FSI_RX_EVT_FRAME_OVERRUN", displayName: "Software didn't clear FRAME_DONE after receiving new frame"},
            {name:"FSI_RX_EVT_DATA_FRAME", displayName: "Received data frame"},

        ],
        
    },

    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("FSIRX"),
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
    return (Common.typeMatches(component.type, ["FSIRX"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var fsirxModule = {
    peripheralName: "FSIRX",
    displayName: "FSIRX",
    maxInstances: Common.peripheralCount("FSIRX"),
    defaultInstanceName: "myFSIRX",
    description: "Fast Serial Interface Receiver Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/fsirx/fsirx.board.c.xdt",
        boardh : "/driverlib/fsirx/fsirx.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.fsirxPinmuxRequirements
};



if (fsirxModule.maxInstances <= 0)
{
    delete fsirxModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(fsirxModule)
}

exports = fsirxModule;