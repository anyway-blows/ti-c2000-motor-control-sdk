let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

function onChangeuseDeadband(inst, ui)
{
    for(var uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
    {
        var configName = config[uiConfigIndex].name;
        ui[configName].hidden = !inst.useDeadband;
    }
}

function onChangeDeadbandMode(inst, ui)
{
    if (inst.deadbandMode == "CUSTOM")
    {

    }
    else if (inst.deadbandMode == "AH")
    {
        inst.inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.enableRED = true;
        inst.enableFED = true;

        inst.outputSwapOutA = false;
        inst.outputSwapOutB = false;
    }
    else if (inst.deadbandMode == "AL")
    {        
        inst.inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.polarityRED = "EPWM_DB_POLARITY_ACTIVE_LOW";
        inst.polarityFED = "EPWM_DB_POLARITY_ACTIVE_LOW";

        inst.enableRED = true;
        inst.enableFED = true;

        inst.outputSwapOutA = false;
        inst.outputSwapOutB = false;

    }
    else if (inst.deadbandMode == "AHC")
    {
        inst.inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.polarityFED = "EPWM_DB_POLARITY_ACTIVE_LOW";

        inst.enableRED = true;
        inst.enableFED = true;

        inst.outputSwapOutA = false;
        inst.outputSwapOutB = false;
    }
    else if (inst.deadbandMode == "ALC")
    {
        inst.inputRED = "EPWM_DB_INPUT_EPWMA";
        inst.inputFED = "EPWM_DB_INPUT_EPWMA";

        inst.polarityRED = "EPWM_DB_POLARITY_ACTIVE_LOW";
        inst.polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.enableRED = true;
        inst.enableFED = true;

        inst.outputSwapOutA = false;
        inst.outputSwapOutB = false;
    }
    else if (inst.deadbandMode == "DUAL")
    {
        inst.inputRED = "EPWM_DB_INPUT_EPWMB";
        inst.inputFED = "EPWM_DB_INPUT_DB_RED";

        inst.polarityRED = "EPWM_DB_POLARITY_ACTIVE_HIGH";
        inst.polarityFED = "EPWM_DB_POLARITY_ACTIVE_HIGH";

        inst.enableRED = false;
        inst.enableFED = true;

        inst.outputSwapOutA = false;
        inst.outputSwapOutB = false;
    }
}

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "useDeadband",
        displayName : "Use Dead-Band",
        description : 'Check to configure the Dead-Band Submodule',
        hidden      : false,
        default     : false,
        onChange    : onChangeuseDeadband
    },
    {
        name: "deadbandMode",
        displayName : "Dead-Band Mode",
        description : 'Mode for the Dead-Band Submodule',
        hidden      : true,
        default     : "CUSTOM",
        options     :[
            {name: "CUSTOM", displayName: "User defined custom setting"},
            {name: "AH", displayName: "Active High"},
            {name: "AL", displayName: "Active Low"},
            {name: "AHC", displayName: "Active High Complementary"},
            {name: "ALC", displayName: "Active Low Complementary"},
            {name: "DUAL", displayName: "Dual Edge Delay Mode"},
        ],
        onChange    : onChangeDeadbandMode,

    },
    {
        name: "inputRED",
        displayName : "Rising Edge Delay Input",
        description : 'Select the source for DBRED (Rising Edge Delay)',
        hidden      : true,
        default     : "EPWM_DB_INPUT_EPWMA",
        options     : [
            {name: "EPWM_DB_INPUT_EPWMA", displayName : "ePWMA is the input signal"},
            {name: "EPWM_DB_INPUT_EPWMB", displayName : "ePWMB is the input signal"},
        ]
    },
    {
        name: "inputFED",
        displayName : "Falling Edge Delay Input",
        description : 'Select the source for DBFED (Falling Edge Delay)',
        hidden      : true,
        default     : "EPWM_DB_INPUT_EPWMA",
        options     : [
            {name: "EPWM_DB_INPUT_EPWMA", displayName : "ePWMA is the input signal"},
            {name: "EPWM_DB_INPUT_EPWMB", displayName : "ePWMB is the input signal"},
            {name: "EPWM_DB_INPUT_DB_RED", displayName : "RED output is the input signal (Both edge delay use case)"},
        ]
    },

    {
        name: "polarityRED",
        displayName : "Rising Edge Delay Polarity",
        description : 'Whether or not to invert RED (Rising Edge Delay)',
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_DeadBandPolarity[0].name,
        options     : device_driverlib_peripheral.EPWM_DeadBandPolarity
    },

    {
        name: "polarityFED",
        displayName : "Falling Edge Delay Polarity",
        description : 'Whether or not to invert FED (Falling Edge Delay)',
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_DeadBandPolarity[0].name,
        options     : device_driverlib_peripheral.EPWM_DeadBandPolarity
    },

    {
        name: "enableRED",
        displayName : "Enable Rising Edge Delay",
        description : 'Enable DBRED (Rising Edge Delay) by choosing it as the source for the A path',
        hidden      : true,
        default     : false,
    },

    {
        name: "enableFED",
        displayName : "Enable Falling Edge Delay",
        description : 'Enable DBFED (Falling Edge Delay) by choosing it as the source for the B path',
        hidden      : true,
        default     : false,
    },

    {
        name: "outputSwapOutA",
        displayName : "Swap Output for EPWMxA",
        description : 'Check to enable output swap. Channel A path to Out B, B path to Out A.',
        hidden      : true,
        default     : false,
    },
    {
        name: "outputSwapOutB",
        displayName : "Swap Output for EPWMxB",
        description : 'Check to enable output swap. Channel B path to Out A.',
        hidden      : true,
        default     : false,
    },

    {
        name: "delayRED",
        displayName : "Rising Edge Delay Value",
        description : 'DBRED (Rising Edge Delay) delay value',
        hidden      : true,
        default     : 0,
    },

    {
        name: "delayFED",
        displayName : "Falling Edge Delay Value",
        description : 'DBFED (Falling Edge Delay) delay value',
        hidden      : true,
        default     : 0,
    },

];



var epwmDeadbandSubmodule = {
    displayName: "EPWM Dead-Band",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_DB",
    description: "Enhanced Pulse Width Modulator Dead-Band",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmDeadbandSubmodule;