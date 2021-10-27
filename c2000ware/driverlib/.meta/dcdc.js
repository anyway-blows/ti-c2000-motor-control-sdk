let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "DC-DC";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("DC-DC"),
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
    return (Common.typeMatches(component.type, ["DC-DC"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];


var dcdcModule = {
    peripheralName: "DC-DC",
    displayName: "DC-DC",
    maxInstances: Common.peripheralCount("DC-DC"),
    defaultInstanceName: "myDCDC",
    description: "DC-DC Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/dcdc/dcdc.board.c.xdt",
        boardh : "/driverlib/dcdc/dcdc.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.dcdcPinmuxRequirements
};


if (dcdcModule.maxInstances <= 0)
{
    delete dcdcModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(dcdcModule)
}

exports = dcdcModule;