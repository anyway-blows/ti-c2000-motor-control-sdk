let Common   = system.getScript("/driverlib/Common.js");

var supported_peripheralName_moduleFile = [
     { peripheralName : "AIO", moduleFile : "/driverlib/aio.js"},
     { peripheralName : "CAN", moduleFile : "/driverlib/can.js"},
     { peripheralName : "CM-I2C", moduleFile : "/driverlib/cmi2c.js"},
     { peripheralName : "CLB_OUTPUTXBAR", moduleFile : "/driverlib/clb_outputxbar.js"},
     { peripheralName : "DC-DC", moduleFile : "/driverlib/dcdc.js"},
     { peripheralName : "ECAT", moduleFile : "/driverlib/ecat.js"},
     { peripheralName : "ETHERNET", moduleFile : "/driverlib/ethernet.js"},
     { peripheralName : "EMIF1", moduleFile : "/driverlib/emif1.js"},
     { peripheralName : "EMIF2", moduleFile : "/driverlib/emif2.js"},
     { peripheralName : "EPWM", moduleFile : "/driverlib/epwm.js"},
     { peripheralName : "EQEP", moduleFile : "/driverlib/eqep.js"},
     { peripheralName : "FSITX", moduleFile : "/driverlib/fsitx.js"},
     { peripheralName : "FSIRX", moduleFile : "/driverlib/fsirx.js"},
     { peripheralName : "GPIO", moduleFile : "/driverlib/gpio.js"},
     { peripheralName : "HIC", moduleFile : "/driverlib/hic.js"},
     { peripheralName : "I2C", moduleFile : "/driverlib/i2c.js"},
     { peripheralName : "INPUTXBAR", moduleFile : "/driverlib/inputxbar.js"},
     { peripheralName : "LIN", moduleFile : "/driverlib/lin.js"},
     { peripheralName : "MCAN", moduleFile : "/driverlib/mcan.js"},
     { peripheralName : "MCBSP", moduleFile : "/driverlib/mcbsp.js"},
     { peripheralName : "OUTPUTXBAR", moduleFile : "/driverlib/outputxbar.js"},
     { peripheralName : "OTHER", moduleFile : "/driverlib/other.js"},
     { peripheralName : "PMBUS", moduleFile : "/driverlib/pmbus.js"},
     { peripheralName : "SD", moduleFile : "/driverlib/sd.js"},
     { peripheralName : "SPI", moduleFile : "/driverlib/spi.js"},
     { peripheralName : "SCI", moduleFile : "/driverlib/sci.js"},
     { peripheralName : "SSI", moduleFile : "/driverlib/ssi.js"},
     { peripheralName : "UART", moduleFile : "/driverlib/uart.js"},
     { peripheralName : "UPP", moduleFile : "/driverlib/upp.js"},
     { peripheralName : "USB", moduleFile : "/driverlib/usb.js"},
];

var static_peripheralName_moduleFile = [
     { peripheralName : "CLB", moduleFile : "/driverlib/clb.js"},
     { peripheralName : "CLBXBAR", moduleFile : "/driverlib/clbxbar.js"},
     { peripheralName : "EPWMXBAR", moduleFile : "/driverlib/epwmxbar.js"},
     //{ peripheralName : "ECAP", moduleFile : "/driverlib/ecap.js"},
]

var device_specific_modules = [
    { devices : ["F28004x", "F2838x", "F28002x"], peripheralName : "ERAD", moduleFile : "/driverlib/erad.js"},
    { devices : ["F28002x", "F2838x"], peripheralName : "CLB_INPUTXBAR", moduleFile : "/driverlib/clb_inputxbar.js"},
]

if (Common.onlyPinmux())
{
    static_peripheralName_moduleFile = []
    device_specific_modules = []
}

var supported_modules = [];

for (var periphName_moduleFile_index in supported_peripheralName_moduleFile)
{
    if (Common.peripheralCount(
        supported_peripheralName_moduleFile[periphName_moduleFile_index].peripheralName) > 0)
    {
        supported_modules.push(supported_peripheralName_moduleFile[periphName_moduleFile_index].moduleFile)
    }

}

for (var periphName_moduleFile_index in static_peripheralName_moduleFile)
{   
    supported_modules.push(static_peripheralName_moduleFile[periphName_moduleFile_index].moduleFile)
}


for (var device_specific_modules_index in device_specific_modules)
{   
    if (device_specific_modules[device_specific_modules_index].
        devices.includes(Common.getDeviceName()))
    {
        supported_modules.push(device_specific_modules[device_specific_modules_index].moduleFile)
    }
}

exports = {
    topModules: [
        {
            displayName: "Peripherals",
            description: "Peripherals get configured here",
            modules: supported_modules
        }
        
    ],
}