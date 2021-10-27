let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "ECAT";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("ECAT"),
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
    return (Common.typeMatches(component.type, ["ECAT"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var ecatModule = {
    peripheralName: "ECAT",
    displayName: "ECAT",
    maxInstances: Common.peripheralCount("ECAT"),
    defaultInstanceName: "myECAT",
    description: "ECAT Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/ecat/ecat.board.c.xdt",
        boardh : "/driverlib/ecat/ecat.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.ecatPinmuxRequirements
};


if (ecatModule.maxInstances <= 0)
{
    delete ecatModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(ecatModule)
}

exports = ecatModule;