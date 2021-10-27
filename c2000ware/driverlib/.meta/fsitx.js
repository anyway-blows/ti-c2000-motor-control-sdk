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
        name        : "clkPres",
        displayName : "Clock Prescalar",
        description : 'Prescaler for the TX CLK, 50 MHz max.',
        hidden      : false,
        default     : '4'
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
        name        : "startOfTransmissionMode",
        displayName : "Start of Transmision Mode",
        description : 'Mode for the start of transmission',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_TxStartMode[0].name,
        options     : device_driverlib_peripheral.FSI_TxStartMode,
    },


    {
        name        : "extTriggerSource",
        displayName : "External Start of Transmision Trigger Source",
        description : 'External Start of Transmision Trigger Source',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_ExtFrameTriggerSrc[0].name,
        options     : device_driverlib_peripheral.FSI_ExtFrameTriggerSrc,
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
        name        : "userCRC",
        displayName : "User calculates CRC",
        description : 'Wether the user calculates the CRC or the calculation is done by the CRC hardware',
        hidden      : false,
        default     : false
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
        name        : "eccCompWidth",
        displayName : "ECC Compute Width Mode",
        description : 'The ECC Compute Width Mode',
        hidden      : false,
        default     : device_driverlib_peripheral.FSI_ECCComputeWidth[0].name,
        options     : device_driverlib_peripheral.FSI_ECCComputeWidth
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
            {name: "FSI_TX_EVT_FRAME_DONE", displayName: "Frame Transmission Done Interrupt"},
            {name: "FSI_TX_EVT_BUF_UNDERRUN", displayName: "Transmit Buffer is Underrun Interrupt"},
            {name: "FSI_TX_EVT_BUF_OVERRUN", displayName: "Transmit Buffer is Overrun Interrupt"},
            {name: "FSI_TX_EVT_PING_HW_TRIG", displayName: "Ping Counter Timeout Interrupt"}
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
            {name: "FSI_TX_EVT_FRAME_DONE", displayName: "Frame Transmission Done Interrupt"},
            {name: "FSI_TX_EVT_BUF_UNDERRUN", displayName: "Transmit Buffer is Underrun Interrupt"},
            {name: "FSI_TX_EVT_BUF_OVERRUN", displayName: "Transmit Buffer is Overrun Interrupt"},
            {name: "FSI_TX_EVT_PING_HW_TRIG", displayName: "Ping Counter Timeout Interrupt"}
        ],
        
    },

    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("FSITX"),
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
    return (Common.typeMatches(component.type, ["FSITX"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var fsitxModule = {
    peripheralName: "FSITX",
    displayName: "FSITX",
    maxInstances: Common.peripheralCount("FSITX"),
    defaultInstanceName: "myFSITX",
    description: "Fast Serial Interface Transmitter Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/fsitx/fsitx.board.c.xdt",
        boardh : "/driverlib/fsitx/fsitx.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.fsitxPinmuxRequirements
};


if (fsitxModule.maxInstances <= 0)
{
    delete fsitxModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(fsitxModule)
}

exports = fsitxModule;