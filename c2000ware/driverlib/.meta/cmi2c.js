let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "CM-I2C";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("CM-I2C"),
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
    return (Common.typeMatches(component.type, ["CM-I2C"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var cmi2cModule = {
    peripheralName: "CM-I2C",
    displayName: "CM-I2C",
    maxInstances: Common.peripheralCount("CM-I2C"),
    defaultInstanceName: "myCMI2C",
    description: "CM-I2C Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/cmi2c/cmi2c.board.c.xdt",
        boardh : "/driverlib/cmi2c/cmi2c.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.cmi2cPinmuxRequirements
};


if (cmi2cModule.maxInstances <= 0)
{
    delete cmi2cModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(cmi2cModule)
}

exports = cmi2cModule;