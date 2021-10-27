let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "cmpA",
        displayName : "Counter Compare A (CMPA)",
        description : 'Counter Compare A (CMPA) value',
        hidden      : false,
        default     : 0,
    },
    {
        name: "cmpB",
        displayName : "Counter Compare B (CMPB)",
        description : 'Counter Compare B (CMPB) value',
        hidden      : false,
        default     : 0,
    },
    {
        name: "cmpC",
        displayName : "Counter Compare C (CMPC)",
        description : 'Counter Compare C (CMPC) value',
        hidden      : false,
        default     : 0,
    },
    {
        name: "cmpD",
        displayName : "Counter Compare D (CMPD)",
        description : 'Counter Compare D (CMPD) value',
        hidden      : false,
        default     : 0,
    },
];



var epwmCCSubmodule = {
    displayName: "EPWM Counter Compare",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_CC",
    description: "Enhanced Pulse Width Modulator Counter Compare",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmCCSubmodule;