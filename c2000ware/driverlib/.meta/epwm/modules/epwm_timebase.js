let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "clockDiv",
        displayName : "Time Base Clock Divider",
        description : 'Clock divider for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_ClockDivider[0].name,
        options     : device_driverlib_peripheral.EPWM_ClockDivider,
    },
    {
        name: "hsClockDiv",
        displayName : "High Speed Clock Divider",
        description : 'Clock divider for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_HSClockDivider[0].name,
        options     : device_driverlib_peripheral.EPWM_HSClockDivider,
    },
    {
        name: "period",
        displayName : "Time Base Period",
        description : 'Period for the Time Base Counter Submodule',
        hidden      : false,
        default     : 0,
    },
    {
        name: "counterValue",
        displayName : "Initial Counter Value",
        description : 'Initial Counter value for the Time Base Counter Submodule',
        hidden      : false,
        default     : 0,
    },
    {
        name: "counterMode",
        displayName : "Counter Mode",
        description : 'Mode of the Counter value for the Time Base Counter Submodule',
        hidden      : false,
        default     : device_driverlib_peripheral.EPWM_TimeBaseCountMode[0].name,
        options     : device_driverlib_peripheral.EPWM_TimeBaseCountMode,
    },
    {
        name: "phaseEnable",
        displayName : "Enable Phase Shift Load",
        description : 'Enable phase shift load for the Time Base Counter Submodule',
        hidden      : false,
        default     : false,
    },
    {
        name: "phaseShift",
        displayName : 'Phase Shift Value',
        description : 'Phase Shift Value for the Time Base Counter Submodule',
        hidden      : false,
        default     : 0,
    },
];



var epwmTimebaseSubmodule = {
    displayName: "EPWM Time Base",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_TB",
    description: "Enhanced Pulse Width Modulator Time Base Counter",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmTimebaseSubmodule;