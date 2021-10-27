let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "LIN";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("LIN"),
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
    return (Common.typeMatches(component.type, ["LIN"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var linModule = {
    peripheralName: "LIN",
    displayName: "LIN",
    maxInstances: Common.peripheralCount("LIN"),
    defaultInstanceName: "myLIN",
    description: "LIN Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/lin/lin.board.c.xdt",
        boardh : "/driverlib/lin/lin.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.linPinmuxRequirements
};


if (linModule.maxInstances <= 0)
{
    delete linModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(linModule)
}

exports = linModule;