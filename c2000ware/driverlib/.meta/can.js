let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "CAN is a communication peripheral which is used " +
    "when there is high noise in the signal bus. It uses a differential " +
    "wire topology to reduce the common noise. This applicatoin provides " +
    "a simplified interface to configure the different parameters of the " +
    "peripheral such as bit rate, time quanta and test mode etc. ";

function onChangeUseLoopback(inst, ui)
{
    if (inst.enableLoopback) {
        ui.loopbackMode.hidden = false;
    }
    else {
        ui.loopbackMode.hidden = true;
    }
}
function onChangeEnableInterrupt(inst, ui)
{
    if (inst.enableInterrupt) {
        ui.interruptFlags.hidden = false;
        ui.interruptLine.hidden = false;
    }
    else {
        ui.interruptFlags.hidden = true;
        ui.interruptLine.hidden = true;
    }
}

let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("CAN"),
        onChange    : Pinmux.useCaseChanged,
    },
    {
        name        : "bitRate",
        displayName : "Bit Rate",
        description : 'CAN bus bit rate',
        hidden      : false,
        default     : 500000,
    },
    {
        name        : "bitTime",
        displayName : "Bit Time",
        description : 'Number of time quanta per bit',
        hidden      : false,
        default     : 20,
    },
    {
        name        : "enableLoopback",
        displayName : "Enable Loopback",
        description : 'Whether loopback mode to be enabled.',
        hidden      : false,
        onChange    : onChangeUseLoopback,
        default     : false,
    },
    {
        name        : "loopbackMode",
        displayName : "Loopback Mode",
        description : 'Which loopback mode to use if enabled.',
        hidden      : true,
        default     : "CAN_TEST_SILENT",
        options     : [
            {name: "CAN_TEST_SILENT", displayName : "Silent Mode"},
            {name: "CAN_TEST_LBACK", displayName : "Internal Loopback Mode"},
            {name: "CAN_TEST_EXL", displayName : "External Loopback Mode"},
        ],
        
    },
    {
        name        : "enableInterrupt",
        displayName : "Enable Interrupt",
        description : 'To Enable CAN Interrupts.',
        hidden      : false,
        onChange    : onChangeEnableInterrupt,
        default     : false,
    },
    {
        name        : "interruptFlags",
        displayName : "Interrupt Flags",
        description : 'Which Interrupts to enable.',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : [
            {name: "CAN_INT_ERROR", displayName : "Controller error condition "},
            {name: "CAN_INT_STATUS", displayName : "Message transfer complete or bus error"},
            {name: "CAN_INT_IE0", displayName : "Enable CAN line 0 Interrupts"},
            {name: "CAN_INT_IE1", displayName : "Enable CAN line 1 Interrupts"},
        ],
        
    },
    {
        name        : "interruptLine",
        displayName : "Interrupt Line",
        description : 'Which Interrupts line to use.',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : [
            {name: "CAN_GLOBAL_INT_CANINT0", displayName : "Enable CAN line 0"},
            {name: "CAN_GLOBAL_INT_CANINT1", displayName : "Enable CAN line 1"},
        ],
        
    },
    {
        name        : "msgObjs",
        displayName : "Number of Message Objects used (1-32)",
        description : 'Number of message objects need to be configured',
        hidden      : false,
        default     : 0,
    },
];

function moduleInstances(inst,ui)
{
    let components = [];
    for(let i=1; i<=inst.msgObjs; i++)
    {
        components = components.concat([
        {
            moduleName : "/driverlib/canMsgObj.js",
            name : "msgObj"+i,
            collapsed : true,
        },
        ]);  
    }
    return components;
}

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
    return (Common.typeMatches(component.type, ["CAN"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var canModule = {
    peripheralName: "CAN",
    displayName: "CAN",
    maxInstances: Common.peripheralCount("CAN"),
    defaultInstanceName: "myCAN",
    description: "Controller Area Network Peripheral",
    filterHardware : filterHardware,
    moduleInstances : moduleInstances,
    config: config,
    templates: {
        boardc : "/driverlib/can/can.board.c.xdt",
        boardh : "/driverlib/can/can.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.canPinmuxRequirements
};


if (canModule.maxInstances <= 0)
{
    delete canModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(canModule)
}

exports = canModule;