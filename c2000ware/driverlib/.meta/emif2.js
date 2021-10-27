let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "An FSI peripheral is used to translate data between " +
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
        options     : Pinmux.getPeripheralUseCaseNames("EMIF2"),
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
    return (Common.typeMatches(component.type, ["EMIF2"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
config = [config[config.length - 1]];

var emif2Module = {
    peripheralName: "EMIF2",
    displayName: "EMIF2",
    maxInstances: Common.peripheralCount("EMIF2"),
    defaultInstanceName: "myEMIF2",
    description: "External Memory Interface Peripheral 2 Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/emif2/emif2.board.c.xdt",
        boardh : "/driverlib/emif2/emif2.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.emif2PinmuxRequirements
};


if (emif2Module.maxInstances <= 0)
{
    delete emif2Module.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(emif2Module)
}


exports = emif2Module;