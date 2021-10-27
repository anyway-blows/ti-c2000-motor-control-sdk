let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "PMBUS";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("PMBUS"),
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
    return (Common.typeMatches(component.type, ["PMBUS"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var pmbusModule = {
    peripheralName: "PMBUS",
    displayName: "PMBUS",
    maxInstances: Common.peripheralCount("PMBUS"),
    defaultInstanceName: "myPMBUS",
    description: "PMBUS Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/pmbus/pmbus.board.c.xdt",
        boardh : "/driverlib/pmbus/pmbus.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.pmbusPinmuxRequirements
};


if (pmbusModule.maxInstances <= 0)
{
    delete pmbusModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(pmbusModule)
}

exports = pmbusModule;