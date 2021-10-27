let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "A UPP peripheral is used to translate data between " +
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
        options     : Pinmux.getPeripheralUseCaseNames("UPP"),
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
    return (Common.typeMatches(component.type, ["USB"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var uppModule = {
    peripheralName: "UPP",
    displayName: "UPP",
    maxInstances: Common.peripheralCount("UPP"),
    defaultInstanceName: "myUPP",
    description: "Universal Parallel Port Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/upp/upp.board.c.xdt",
        boardh : "/driverlib/upp/upp.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.uppPinmuxRequirements
};


if (uppModule.maxInstances <= 0)
{
    delete uppModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(uppModule)
}

exports = uppModule;