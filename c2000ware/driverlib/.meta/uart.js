let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "UART";

/* Array of CAN configurables that are common across device families */
let config = [
    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("UART"),
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
    return (Common.typeMatches(component.type, ["UART"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}

var uartModule = {
    peripheralName: "UART",
    displayName: "UART",
    maxInstances: Common.peripheralCount("UART"),
    defaultInstanceName: "myUART",
    description: "UART Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/uart/uart.board.c.xdt",
        boardh : "/driverlib/uart/uart.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.uartPinmuxRequirements
};


if (uartModule.maxInstances <= 0)
{
    delete uartModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(uartModule)
}

exports = uartModule;