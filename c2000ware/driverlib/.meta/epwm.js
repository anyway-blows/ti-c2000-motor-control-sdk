let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "An EPWM peripheral is used to translate data between " +
    "the chip and a serial port. The SCI driver simplifies reading " +
    "and writing to any of the UART peripherals on the board, with " +
    "multiple modes of operation and performance. These include " +
    "blocking, non-blocking, and polling, as well as text/binary " +
    "mode, echo and return characters.";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("EPWM"),
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
    return (Common.typeMatches(component.type, ["EPWM"]));
}


function moduleInstances(inst)
{
    let components = [
        {
            moduleName: "/driverlib/epwm/modules/epwm_timebase.js",
            name: "epwmTimebase",
            displayName:"EPWM Time Base",
            description:"Time Base Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_counterCompare.js",
            name: "epwmCounterCompare",
            displayName:"EPWM Counter Compare",
            description:"Counter Compare Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_actionQualifier.js",
            name: "epwmActionQualifier",
            displayName:"EPWM Action Qualifier",
            description:"Action Qualifier Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_tripZone.js",
            name: "epwmTripZone",
            displayName:"EPWM Trip Zone",
            description:"Trip Zone Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_digitalCompare.js",
            name: "epwmDigitalCompare",
            displayName:"EPWM Digital Compare",
            description:"Digital Compare Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_deadband.js",
            name: "epwmDeadband",
            displayName:"EPWM Dead-Band",
            description:"Dead-Band Submodule",
            collapsed: true
        },
        {
            moduleName: "/driverlib/epwm/modules/epwm_chopper.js",
            name: "epwmChopper",
            displayName:"EPWM Chopper",
            description:"Chopper Submodule",
            collapsed: true
        },
    ];

    return components;
}

//if (Common.onlyPinmux())
//{
    config = [config[config.length - 1]];
    moduleInstances = null
//}
var epwmModule = {
    peripheralName: "EPWM",
    displayName: "EPWM",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "myEPWM",
    description: "Enhanced Pulse Width Modulator Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/epwm/epwm.board.c.xdt",
        boardh : "/driverlib/epwm/epwm.board.h.xdt"
    },
    moduleInstances     : moduleInstances,
    pinmuxRequirements    : Pinmux.epwmPinmuxRequirements
};


if (epwmModule.maxInstances <= 0)
{
    delete epwmModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(epwmModule)
}


exports = epwmModule;