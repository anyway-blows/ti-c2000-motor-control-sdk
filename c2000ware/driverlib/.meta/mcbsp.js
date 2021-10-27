let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "A MCBSP peripheral is used to translate data between " +
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
        options     : Pinmux.getPeripheralUseCaseNames("MCBSP"),
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
    return (Common.typeMatches(component.type, ["MCBSP"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var mcbspModule = {
    peripheralName: "MCBSP",
    displayName: "MCBSP",
    maxInstances: Common.peripheralCount("MCBSP"),
    defaultInstanceName: "myMCBSP",
    description: "Multichannel Buffered Serial Port Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/mcbsp/mcbsp.board.c.xdt",
        boardh : "/driverlib/mcbsp/mcbsp.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.mcbspPinmuxRequirements
};

if (mcbspModule.maxInstances <= 0)
{
    delete mcbspModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(mcbspModule)
}

exports = mcbspModule;