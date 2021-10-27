let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

function onChangeuseChopper(inst, ui)
{
    for(var uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
    {
        var configName = config[uiConfigIndex].name;
        ui[configName].hidden = !inst.useChopper;
    }
}

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "useChopper",
        displayName : "Use Chopper",
        description : 'Check to configure the Chopper Submodule',
        hidden      : false,
        default     : false,
        onChange    : onChangeuseChopper
    },
    {
        name: "chopperDuty",
        displayName : "Chopper Duty Cycle",
        description : 'The chopper submodule duty cycle.',
        hidden      : true,
        default     : "0",
        options     : [
            {name:"0", displayName: "1/8 DUTY"},
            {name:"1", displayName: "2/8 DUTY"},
            {name:"2", displayName: "3/8 DUTY"},
            {name:"3", displayName: "4/8 DUTY"},
            {name:"4", displayName: "5/8 DUTY"},
            {name:"5", displayName: "6/8 DUTY"},
            {name:"6", displayName: "7/8 DUTY"},
        ]
    },
    {
        name: "chopperFreq",
        displayName : "Chopper Frequency",
        description : 'The chopper submodule frequency.',
        hidden      : true,
        default     : "0",
        options     : [
            {name:"0", displayName: "SYSCLKOUT / 1"},
            {name:"1", displayName: "SYSCLKOUT / 2"},
            {name:"2", displayName: "SYSCLKOUT / 3"},
            {name:"3", displayName: "SYSCLKOUT / 4"},
            {name:"4", displayName: "SYSCLKOUT / 5"},
            {name:"5", displayName: "SYSCLKOUT / 6"},
            {name:"6", displayName: "SYSCLKOUT / 7"},
            {name:"7", displayName: "SYSCLKOUT / 8"},
        ]
    },
    {
        name: "chopperFirstPulseWidth",
        displayName : "Chopper First Pulse Width",
        description : 'The chopper submodule first pulse width.',
        hidden      : true,
        default     : "0",
        options     : [
            {name:"0", displayName: "8/(EPWMCLK)"},
            {name:"1", displayName: "8/(2xEPWMCLK)"},
            {name:"2", displayName: "8/(3xEPWMCLK)"},
            {name:"3", displayName: "8/(4xEPWMCLK)"},
            {name:"4", displayName: "8/(5xEPWMCLK)"},
            {name:"5", displayName: "8/(6xEPWMCLK)"},
            {name:"6", displayName: "8/(7xEPWMCLK)"},
            {name:"7", displayName: "1/(EPWMCLK)"},
            {name:"8", displayName: "8/(9xEPWMCLK)"},
            {name:"9", displayName: "8/(10xEPWMCLK)"},
            {name:"10",displayName: "8/(11xEPWMCLK)"},
            {name:"11",displayName: "8/(12xEPWMCLK)"},
            {name:"12",displayName: "8/(13xEPWMCLK)"},
            {name:"13",displayName: "8/(14xEPWMCLK)"},
            {name:"14",displayName: "8/(15xEPWMCLK)"},
            {name:"15",displayName: "1/(2xEPWMCLK)"},
        ]
    },

];



var epwmChopperSubmodule = {
    displayName: "EPWM Chopper",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_PC",
    description: "Enhanced Pulse Width Modulator Chopper",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmChopperSubmodule;