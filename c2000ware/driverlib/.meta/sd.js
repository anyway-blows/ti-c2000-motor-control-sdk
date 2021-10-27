let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "An SD peripheral is used to translate data between " +
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
        options     : Pinmux.getPeripheralUseCaseNames("SD"),
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
    return (Common.typeMatches(component.type, ["SD"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var sdModule = {
    peripheralName: "SD",
    displayName: "SD",
    maxInstances: Common.peripheralCount("SD"),
    defaultInstanceName: "mySD",
    description: "Sigma Delta Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/sd/sd.board.c.xdt",
        boardh : "/driverlib/sd/sd.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.sdPinmuxRequirements
};

if (sdModule.maxInstances <= 0)
{
    delete sdModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(sdModule)
}



exports = sdModule;