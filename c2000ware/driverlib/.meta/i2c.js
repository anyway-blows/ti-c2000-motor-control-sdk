let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_i2c.js");

/* Intro splash on GUI */
let longDescription = "The I2C driver provides a simplified application"
        + " interface to access peripherals on an I2C bus.";


function onChangeMode(inst, ui)
{
    if (inst.mode == "MASTER") {
        ui.loopback.hidden = false;
        ui.bitRate.hidden = false;
        ui.duty.hidden = false;
        ui.slaveAddress.hidden = false;
        ui.ownSlaveAddress.hidden = true;
    }
    else {
        ui.loopback.hidden = true;
        ui.bitRate.hidden = true;
        ui.duty.hidden = true;
        ui.slaveAddress.hidden = true;
        ui.ownSlaveAddress.hidden = false;
    }
}

function onChangeLoopback(inst, ui)
{
	if (inst.loopback) {
        ui.ownSlaveAddress.hidden = false;
    }
    else {
        ui.ownSlaveAddress.hidden = true;
    }
    onChangeMode(inst, ui);
}

function onChangeUseInterrupts(inst, ui)
{
    if (inst.useInterrupts)
    {
        ui.enabledInterrupts.hidden = false;
        ui.enabledFIFOInterrupts.hidden = false;
        
        ui.txFifo.hidden = false;
        ui.rxFifo.hidden = false;
        
        if (!inst.useFifo) {
            ui.enabledFIFOInterrupts.hidden = true;
            ui.txFifo.hidden = true;
            ui.rxFifo.hidden = true;
        }
    }
    else
    {
        ui.enabledInterrupts.hidden = true;
        ui.enabledFIFOInterrupts.hidden = true;
        
        ui.txFifo.hidden = true;
        ui.rxFifo.hidden = true;
    }

}

function onValidate(inst, validation) 
{   
    var bitRateError = false;
    var slaveAddrError = false;
    var ownAddrError = false;
    var maxAddress = (inst.addressMode.includes(10)? 0x3FF: 0x7F);
    var bitRateInt = parseInt(inst.bitRate);
    if (bitRateInt < 1 || bitRateInt > 200000000)
    {
        bitRateError = true;
    }
    var slaveAddressInt = parseInt(inst.slaveAddress);
    var ownAddressInt = parseInt(inst.ownSlaveAddress);

    if (slaveAddressInt < 0 || slaveAddressInt > maxAddress)
    {
        slaveAddrError = true;
    }

    if (ownAddressInt < 0 || ownAddressInt > maxAddress)
    {
        ownAddrError = true;
    }
    if(inst.dataCount > 0xFFFF | inst.dataCount < 0)
    {
        validation.logError(
            "Enter an integer for data count between 0 and 65535!", 
            inst, "dataCount");
    }
    if(bitRateError)
    {
        validation.logError(
            "Enter an integer for bit rates between 1 and SYSCLK!", 
            inst, "bitRate");
    }
    if(slaveAddrError && inst.mode == "MASTER")
    {
        validation.logError(
            "Slave address must be between 0 and " + maxAddress + "!", 
            inst, "slaveAddress");
    }
    if(ownAddrError && inst.mode != "MASTER")
    {
        validation.logError(
            "Own Slave address must be between 0 and " + maxAddress + "!", 
            inst, "ownSlaveAddress");
    }
}


/* Array of I2C configurables that are common across device families */
let config = [

    {
        name        : "mode",
        displayName : "I2C Device Mode",
        description : 'I2C device mode: master/slave',
        hidden      : false,
        onChange    : onChangeMode,
        default     : 'MASTER',
        options     : [
            { name: "MASTER", displayName : "Master Mode" },
            { name: "SLAVE", displayName : "Slave Mode"  },
        ]
    },
    {
        name        : "duty",
        displayName : "Clock Duty Cycle",
        description : 'Clock duty cycle mode',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_DutyCycle[0].name,
        options     : device_driverlib_peripheral.I2C_DutyCycle
    },
    {
        name        : "bitRate",
        displayName : "Bit Rate",
        description : 'Master transmitter bitrate',
        hidden      : false,
        default     : 400000,
    },
    {
        name        : "initialMode",
        displayName : "Initial Mode",
        description : 'Initial mode, transmitter/receiver',
        hidden      : false,
        default     : 'SEND_MODE',
        options     : [
            { name: "SEND_MODE", displayName : "Transmitter Mode" },
            { name: "RECEIVE_MODE", displayName : "Receiver Mode"  },
        ]
    },
    {
        name        : "bitCount",
        displayName : "Bit Count",
        description : 'Bit count, number of bits per transfer',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_BitCount[0].name,
        options     : device_driverlib_peripheral.I2C_BitCount
    },
    {
        name        : "dataCount",
        displayName : "Data Count",
        description : 'Set number of bytes to be to transfer or receive when repeat mode is off',
        hidden      : false,
        default     : 1
    },
    {
        name        : "addressMode",
        displayName : "Address Mode",
        description : 'Address mode, 7-bit/8-bit',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_AddressMode[0].name,
        options     : device_driverlib_peripheral.I2C_AddressMode
    },
    {
        name        : "slaveAddress",
        displayName : "Slave Address",
        description : 'The slave address, 7bit/10bit: 0x00 to 0x7F or 0x3FF',
        hidden      : false,
        displayFormat: "hex",
        default     : 0x00
    },
    {
        name        : "ownSlaveAddress",
        displayName : "Own Slave Address",
        description : 'The device\'s own slave address, 7bit/10bit: 0x00 to 0x7F or 0x3FF',
        hidden      : true,
        displayFormat: "hex",
        default     : 0x00
    },
    {
        name        : "loopback",
        displayName : "loopback Mode",
        description : 'loopback mode',
        hidden      : false,
        onChange    : onChangeLoopback,
        default     : false,
    },

    {
        name        : "useInterrupts",
        displayName : "Use Interrupt",
        description : 'Whether or not to use Interrupt mode.',
        hidden      : false,
        onChange    : onChangeUseInterrupts,
        default     : true
        
    },
    {
        name        : "enabledInterrupts",
        displayName : "Enabled Interrupts",
        description : 'Which interrupts to enabled.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "I2C_INT_ARB_LOST", displayName : "Arbitration-lost interrupt"},
            {name: "I2C_INT_NO_ACK", displayName : "NACK interrupt"},
            {name: "I2C_INT_REG_ACCESS_RDY", displayName : "Register-access-ready interrupt"},
            {name: "I2C_INT_RX_DATA_RDY", displayName : "Receive-data-ready interrupt"},
            {name: "I2C_INT_TX_DATA_RDY", displayName : "Transmit-data-ready interrupt"},
            {name: "I2C_INT_STOP_CONDITION", displayName : "Stop condition detected"},
            {name: "I2C_INT_ADDR_SLAVE", displayName : "Addressed as slave interrupt"},
        ],
        
    },
    {
        name        : "enabledFIFOInterrupts",
        displayName : "Enabled FIFO Interrupts",
        description : 'Which FIFO interrupts to enabled.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "I2C_INT_RXFF", displayName: "Receive Interrupt"},
            {name: "I2C_INT_TXFF", displayName: "Transmit Interrupt"},
        ],
        
    },
    {
        name        : "useFifo",
        displayName : "FIFO Mode",
        description : 'FIFO mode',
        hidden      : false,
        onChange    : onChangeUseInterrupts,
        default     : true,
    },

    {
        name        : "txFifo",
        displayName : "Transmit FIFO Interrupt Level",
        description : 'Transmit FIFO interrupt level used.',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_TxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.I2C_TxFIFOLevel
    },

    {
        name        : "rxFifo",
        displayName : "Receive FIFO Interrupt Level",
        description : 'Receive FIFO interrupt level used.',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_RxFIFOLevel[0].name,
        options     : device_driverlib_peripheral.I2C_RxFIFOLevel
    },

    {
        name        : "emulationMode",
        displayName : "Emulation Mode",
        description : 'Emulation Mode',
        hidden      : false,
        default     : device_driverlib_peripheral.I2C_EmulationMode[0].name,
        options     : device_driverlib_peripheral.I2C_EmulationMode
    },

    {
        name: "useCase",
        displayName : "Use Case",
        description : 'Peripheral use case',
        hidden      : false,
        default     : 'ALL',
        options     : Pinmux.getPeripheralUseCaseNames("I2C"),
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
    return (Common.typeMatches(component.type, ["I2C"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
var i2cModule = {
    peripheralName: "I2C",
    displayName: "I2C",
    maxInstances: Common.peripheralCount("I2C"),
    defaultInstanceName: "myI2C",
    description: "Inter-Integrated Circuit Peripheral",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/i2c/i2c.board.c.xdt",
        boardh : "/driverlib/i2c/i2c.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.i2cPinmuxRequirements,
    validate    : onValidate
};


if (i2cModule.maxInstances <= 0)
{
    delete i2cModule.pinmuxRequirements;
}
else
{
    Pinmux.addCustomPinmuxEnumToConfig(i2cModule)
}


exports = i2cModule;