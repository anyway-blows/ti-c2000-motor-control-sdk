let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "An EQEP peripheral is used to translate data between " +
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
        options     : Pinmux.getPeripheralUseCaseNames("EQEP"),
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
    return (Common.typeMatches(component.type, ["EQEP"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var eqepModule = {
    peripheralName: "EQEP",
    displayName: "EQEP",
    maxInstances: Common.peripheralCount("EQEP"),
    defaultInstanceName: "myEQEP",
    description: "Enhanced Quadrature Encoder Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/eqep/eqep.board.c.xdt",
        boardh : "/driverlib/eqep/eqep.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.eqepPinmuxRequirements
};


if (eqepModule.maxInstances <= 0)
{
    delete eqepModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(eqepModule)
}


exports = eqepModule;