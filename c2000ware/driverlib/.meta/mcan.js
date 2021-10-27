let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "MCAN";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("MCAN"),
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
    return (Common.typeMatches(component.type, ["MCAN"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var mcanModule = {
    peripheralName: "MCAN",
    displayName: "MCAN",
    maxInstances: Common.peripheralCount("MCAN"),
    defaultInstanceName: "myMCAN",
    description: "MCAN Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/mcan/mcan.board.c.xdt",
        boardh : "/driverlib/mcan/mcan.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.mcanPinmuxRequirements
};


if (mcanModule.maxInstances <= 0)
{
    delete mcanModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(mcanModule)
}

exports = mcanModule;