let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_gpio.js");

function onChangeAnalogMode(inst, ui)
{
    if (inst.analogMode == "GPIO_ANALOG_DISABLED")
    {
        ui.direction.hidden = false;
        ui.padConfig.hidden = false;
        ui.masterCore.hidden = false;
        ui.qualMode.hidden = false;
    }
    else
    {
        ui.direction.hidden = true;
        ui.padConfig.hidden = true;
        ui.masterCore.hidden = true;
        ui.qualMode.hidden = true;
    }

}
/* Intro splash on GUI */
let longDescription = "The GPIO module allows you to manage General Purpose I/O"
        + " resources via simple and portable APIs. GPIO pin behavior is"
        + " usually configured statically, but can also be configured or"
        + " reconfigured at runtime.";

/* Array of SCI configurables that are common across device families */
let config = [
    {
        name        : "analogMode",
        displayName : "Analog Mode",
        description : 'Whether pin is in analog mode or not.',
        readOnly    : true,
        hidden      : false,
        onChange    : onChangeAnalogMode,
        default     : device_driverlib_peripheral.GPIO_AnalogMode[0].name,
        options     : device_driverlib_peripheral.GPIO_AnalogMode
    },

    {
        name        : "direction",
        displayName : "GPIO Direction",
        description : 'Whether pin is input or output.',
        hidden      : false,
        default     : device_driverlib_peripheral.GPIO_Direction[0].name,
        options     : device_driverlib_peripheral.GPIO_Direction
    },

    {
        name        : "padConfig",
        displayName : "Pin Type",
        description : 'Type of pin is driving the gpio.',
        hidden      : false,
        default     : 'STD',
        options     : [
            { name: "STD", displayName : "Push-pull output/floating input" },
            { name: "PULLUP", displayName : "Pull-up enabled in input mode"  },
            { name: "OD", displayName : "Open-drain output pin" },
            { name: "INVERT", displayName : "Inverted polarity on an input"  }
        ]
    },

    {
        name        : "qualMode",
        displayName : "Qualification Mode",
        description : 'The type of qualification done on the pin.',
        hidden      : false,
        default     : device_driverlib_peripheral.GPIO_QualificationMode[0].name,
        options     : device_driverlib_peripheral.GPIO_QualificationMode
    }
];


if (Common.getDeviceName() != "F28002x")
{
    var coreSelectConfig = {
        name        : "masterCore",
        displayName : "Master Core",
        description : 'Who owns the GPIO.',
        hidden      : false,
        default     : device_driverlib_peripheral.GPIO_CoreSelect[0].name,
        options     : device_driverlib_peripheral.GPIO_CoreSelect
    };
    config.push(coreSelectConfig)
}


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
    return (Common.typeMatches(component.type, ["GPIO"]));
}

if (Common.onlyPinmux())
{
    config = [];
}
var gpioModule = {
    peripheralName: "GPIO",
    displayName: "GPIO",
    maxInstances: Common.peripheralCount("GPIO"),
    defaultInstanceName: "myGPIO",
    description: "General Purpose IO Interface Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/gpio/gpio.board.c.xdt",
        boardh : "/driverlib/gpio/gpio.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.gpioPinmuxRequirements
};


if (gpioModule.maxInstances <= 0)
{
    delete gpioModule.pinmuxRequirements;
}


exports = gpioModule;