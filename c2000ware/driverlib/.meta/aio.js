let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

/* Intro splash on GUI */
let longDescription = "A AIO peripheral is used to translate data between " +
    "the chip and a serial port. The SCI driver simplifies reading " +
    "and writing to any of the UART peripherals on the board, with " +
    "multiple modes of operation and performance. These include " +
    "blocking, non-blocking, and polling, as well as text/binary " +
    "mode, echo and return characters.";

/* Array of SCI configurables that are common across device families */
let config = [

    {
        name        : "direction",
        displayName : "AIO Direction",
        description : 'Only supports input.',
        hidden      : false,
        default     : 'IN',
        options     : [
            { name: "IN" }
        ]
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
        name        : "masterCore",
        displayName : "Master Core",
        description : 'Who owns the GPIO.',
        hidden      : false,
        default     : 'CPU1',
        options     : [
            { name: "CPU1", displayName : "CPU1 is the master" },
            { name: "CPU1_CLA1", displayName : "CPU1's CLA is the master"  },
            { name: "CPU2", displayName : "CPU2 is the master" },
            { name: "CPU2_CLA1", displayName : "CPU2's CLA is the master"  }
        ]
    },


    {
        name        : "qualMode",
        displayName : "Qualification Mode",
        description : 'The type of qualification done on the pin.',
        hidden      : false,
        default     : 'SYNC',
        options     : [
            { name: "SYNC", displayName : "Synchronous" },
            { name: "3SAMPLE", displayName : "3 samples per qualification"  },
            { name: "6SAMPLE", displayName : "6 samples per qualification" },
            { name: "ASYNC", displayName : "Asynchronous"  }
        ]
    }
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
    return (Common.typeMatches(component.type, ["AIO"]));
}

if (Common.onlyPinmux())
{
    config = [];
}
var aioModule = {
    peripheralName: "AIO",
    displayName: "AIO",
    maxInstances: Common.peripheralCount("AIO"),
    defaultInstanceName: "myAIO",
    description: "Analog IO Interface Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/aio/aio.board.c.xdt",
        boardh : "/driverlib/aio/aio.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.aioPinmuxRequirements
};

if (aioModule.maxInstances <= 0)
{
    delete aioModule.pinmuxRequirements;
}

exports = aioModule;