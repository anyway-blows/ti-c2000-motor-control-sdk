let Common   = system.getScript("/driverlib/Common.js");

function useCaseChanged(inst, ui)
{
    if (inst.useCase == "CUSTOM") {
        ui.useInterfacePins.hidden = false;
    }
    else {
        ui.useInterfacePins.hidden = true;
    }
}

function addCustomPinmuxEnumToConfig(module)
{
    var peripheralName = module.peripheralName;

    var options = []
    var InterfaceNames = getPeripheralInterfaces(peripheralName);
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];
        options.push({name:interfaceName, displayName:interfaceName.replace("#", "").replace("@", "")});
    }

    module.config.push(
        {
            name        : "useInterfacePins",
            displayName : "Pins Used",
            description : 'The pins used by the pinmux section.',
            hidden      : true,
            default     : [],
            minSelections: 0,
            options     : options,
        }
        /*,
        {
            name: "cfgMultiEnum2",
            //getDisabledOptions: generateDisabledOptions("cfgMultiEnum2"),
            displayName: "MultiEnum2",
            options: [
                {
                    name: "opt1",
                    displayName: "Option1"
                },
                {
                    name: "opt2",
                    displayName: "Option2"
                },
                {
                    name: "opt3",
                    displayName: "Option3"
                }
            ],
            default: ["opt1", "opt2"]
        },*/
    )
}


function usbPinmuxRequirements(inst)
{
    var peripheralName = "USB";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        if (pt.displayName.includes("EPEN") || pt.displayName.includes("FLT"))
        {

        }
        else
        {
            resources.push(pt);
            signalTypes[pt.name] = interfaceName;
        }
        i++;
    }

    let usb = {
        name          : "usb",
        displayName   : "USB Peripheral",
        interfaceName : "USB",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [usb];
}


function uppPinmuxRequirements(inst)
{
    var peripheralName = "UPP";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    //console.log(useCaseInterfaces);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    //console.log(InterfaceNames);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let upp = {
        name          : "upp",
        displayName   : "UPP Peripheral",
        interfaceName : "UPP",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [upp];
}

function fsitxPinmuxRequirements(inst)
{
    var peripheralName = "FSITX";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let fsitx = {
        name          : "fsitx",
        displayName   : "FSITX Peripheral",
        interfaceName : "FSITX",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [fsitx];
}
function fsirxPinmuxRequirements(inst)
{
    var peripheralName = "FSIRX";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let fsirx = {
        name          : "fsirx",
        displayName   : "FSIRX Peripheral",
        interfaceName : "FSIRX",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [fsirx];
}
function emif1PinmuxRequirements(inst)
{
    var peripheralName = "EMIF1";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
   var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1 ||
        // Special Case
            interfaceName.includes("A20") ||
            interfaceName.includes("A21"))
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let emif1 = {
        name          : "emif1",
        displayName   : "EMIF1 Peripheral",
        interfaceName : "EMIF1",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [emif1];
}
function emif2PinmuxRequirements(inst)
{
    var peripheralName = "EMIF2";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1||
        // No options available
        system.deviceData.interfaces[peripheralName].interfacePins[interfaceName].pinMappings[0].pinMappings.length <= 0)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let emif2 = {
        name          : "emif2",
        displayName   : "EMIF2 Peripheral",
        interfaceName : "EMIF2",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [emif2];
}

function otherPinmuxRequirements(inst)
{
    var peripheralName = "OTHER";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1 ||
            // No options available
            system.deviceData.interfaces[peripheralName].interfacePins[interfaceName].pinMappings[0].pinMappings.length <= 0 ||
            // No EMUs
            // Special Case
            interfaceName.includes("EMU") 
        )
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let other = {
        name          : "other",
        displayName   : "Other Peripheral",
        interfaceName : "OTHER",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [other];
}


function dcdcPinmuxRequirements(inst)
{
    var peripheralName = "DC-DC";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let dcdc = {
        name          : "dcdc",
        displayName   : "DC-DC Peripheral",
        interfaceName : "DC-DC",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [dcdc];
}

function linPinmuxRequirements(inst)
{
    var peripheralName = "LIN";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let lin = {
        name          : "lin",
        displayName   : "LIN Peripheral",
        interfaceName : "LIN",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [lin];
}


function spiPinmuxRequirements(inst)
{
    var peripheralName = "SPI";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let spi = {
        name          : "spi",
        displayName   : "SPI Peripheral",
        interfaceName : "SPI",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [spi];
}

function getPeripheralUseCaseNames(peripheralName)
{
    var useCaseOptions = [];
    if (system.deviceData.interfaces[peripheralName] == null)
        return [{name: "ALL"}, {name: "CUSTOM"}];
    var useCases = Object.keys(system.deviceData.interfaces[peripheralName].useCases);

    var containsALL = false;
    for (var useCaseNumber in useCases)
    {
        var useCaseName = useCases[useCaseNumber];
        if(useCaseName == "ALL")
        {
            containsALL = true;
        }
        useCaseOptions.push({name: useCaseName});
    }
    if (!containsALL)
    {
        useCaseOptions.unshift({name: "ALL"});
    }
    useCaseOptions.unshift({name: "CUSTOM"});
    return useCaseOptions;
}

function addPeripheralUseCaseCustom()
{
    
}

function getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName)
{
    var useCaseInterfaces = [];
    //console.log("Finding use cases...");
    //console.log(system.deviceData.interfaces[peripheralName].useCases);



    if (useCaseName == "CUSTOM")
    {
        //console.log("Doing custom use cases...");
        return getCustomPeripheralInterfaces(inst, peripheralName);
    }

    if (system.deviceData.interfaces[peripheralName].useCases[useCaseName] == null)
    {
        //console.log("Use case not found, showing all!");
        return getPeripheralInterfaces(peripheralName);
    }

    if (useCaseName == "ALL")
    {
        return getPeripheralInterfaces(peripheralName);
    }


    //console.log("Use case found. Showing only relevant interfaces!");
    var numberOfPinsInUseCase = system.deviceData.interfaces[peripheralName].useCases[useCaseName].useCasePin.length;
    
    for (var useCasePinNumber = 0; useCasePinNumber < numberOfPinsInUseCase; useCasePinNumber++)
    {
        if (!system.deviceData.interfaces[peripheralName].useCases[useCaseName].useCasePin[useCasePinNumber].optional)
        {
            useCaseInterfaces.push(system.deviceData.interfaces[peripheralName].useCases[useCaseName].useCasePin[useCasePinNumber].name);
        }
    }
    return useCaseInterfaces;
}

function getCustomPeripheralInterfaces(inst, peripheralName)
{
    return inst.useInterfacePins;
    //console.log(inst.useInterfacePins)
    var interfaces = [];
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    for (var interfaceNumber in InterfaceNames)
    {
        interfaces.push(InterfaceNames[interfaceNumber]);
    }

    return interfaces;
}

function getPeripheralInterfaces(peripheralName)
{
    var interfaces = [];
    //console.log(peripheralName)
    //console.log(system.deviceData.interfaces)
    //console.log(system.deviceData.interfaces[peripheralName])
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    for (var interfaceNumber in InterfaceNames)
    {
        interfaces.push(InterfaceNames[interfaceNumber]);
    }

    return interfaces;
}

function sciPinmuxRequirements(inst)
{
    var peripheralName = "SCI";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);

    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let sci = {
        name          : "sci",
        displayName   : "SCI Peripheral",
        interfaceName : "SCI",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [sci];
}

function sdPinmuxRequirements(inst)
{
    var peripheralName = "SD";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let sd = {
        name          : "sd",
        displayName   : "Sigma-Delta Peripheral",
        interfaceName : "SD",
        resources     : resources,
        signalTypes   : signalTypes
    };


    return [sd];
}

function mcbspPinmuxRequirements(inst)
{
    var peripheralName = "MCBSP";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let mcbsp = {
        name          : "mcbsp",
        displayName   : "MCBSP Peripheral",
        interfaceName : "MCBSP",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [mcbsp];
}

function eqepPinmuxRequirements(inst)
{
    var peripheralName = "EQEP";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let eqep = {
        name          : "eqep",
        displayName   : "EQEP Peripheral",
        interfaceName : "EQEP",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [eqep];
}

function epwmPinmuxRequirements(inst)
{
    var peripheralName = "EPWM";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }
    let epwm = {
        name          : "epwm",
        displayName   : "EPWM Peripheral",
        interfaceName : "EPWM",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [epwm];
}

function i2cPinmuxRequirements(inst)
{
    var peripheralName = "I2C";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }
    let i2c = {
        name          : "i2c",
        displayName   : "I2C Peripheral",
        interfaceName : "I2C",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [i2c];
}


function canPinmuxRequirements(inst)
{
    var peripheralName = "CAN";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
            continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }
    let can = {
        name          : "can",
        displayName   : "CAN Peripheral",
        interfaceName : "CAN",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [can];
}

function outputxbarPinmuxRequirements(inst)
{

    let resources = [];
    var peripheralName = "OUTPUTXBAR";
    var signalTypes = {};
    //var useCaseName = inst.useCase;
    //var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        //if (useCaseInterfaces.indexOf(interfaceName) == -1)
        //{
        //    continue;
        //}
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let outputxbar = {
        name          : "outputxbar",
        displayName   : "OUTPUTXBAR Peripheral",
        interfaceName : "OUTPUTXBAR",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [outputxbar];
}



function clb_outputxbarPinmuxRequirements(inst)
{

    let resources = [];
    var peripheralName = "CLB_OUTPUTXBAR";
    var signalTypes = {};
    //var useCaseName = inst.useCase;
    //var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        //if (useCaseInterfaces.indexOf(interfaceName) == -1)
        //{
        //    continue;
        //}
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let clb_outputxbar = {
        name          : "clb_outputxbar",
        displayName   : "CLB OUTPUTXBAR Peripheral",
        interfaceName : "CLB_OUTPUTXBAR",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [clb_outputxbar];
}


function inputxbarPinmuxRequirements(inst)
{

    let resources = [];
    var INPUTXBARInterfaceNames = Object.keys(system.deviceData.interfaces.INPUTXBAR.interfacePins);
    var i = 1;
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);

    for (var useCaseNumber in INPUTXBARInterfaceNames)
    {
        var interfaceName = INPUTXBARInterfaceNames[interfaceNumber];
        let ix1 = {
            name              : "inputxbar" + i + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : "INPUTXBAR" + i, /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(ix1);

        signalTypes[ix1.name] = interfaceName;
        i++;
    }


    let inputxbar = {
        name          : "inputxbar",
        displayName   : "INPUTXBAR Peripheral",
        interfaceName : "INPUTXBAR",
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [inputxbar];
}


function gpioPinmuxRequirements(inst)
{

    let resources = [];

    var GPIOInterfaceName = "GPIO#";
    
    let gpio = {
        name          : "gpioPin",
        displayName   : "GPIO",
        interfaceName : "GPIO",
        signalTypes   : [GPIOInterfaceName]
        
    };

    return [gpio];
}

function aioPinmuxRequirements(inst)
{

    let resources = [];


    let aio = {
        name          : "aioPin",
        displayName   : "AIO Peripheral",
        interfaceName : "AIO",        
    };

    return [aio];
}

function pmbusPinmuxRequirements(inst)
{
    var peripheralName = "PMBUS";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let pmbus = {
        name          : "pmbus",
        displayName   : "PMBUS Peripheral",
        interfaceName : "PMBUS",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [pmbus];
}


function cmi2cPinmuxRequirements(inst)
{
    var peripheralName = "CM-I2C";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let cmi2c = {
        name          : "cmi2c",
        displayName   : "CM-I2C Peripheral",
        interfaceName : "CM-I2C",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [cmi2c];
}


function mcanPinmuxRequirements(inst)
{
    var peripheralName = "MCAN";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let mcan = {
        name          : "mcan",
        displayName   : "MCAN Peripheral",
        interfaceName : "MCAN",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [mcan];
}

function uartPinmuxRequirements(inst)
{
    var peripheralName = "UART";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let uart = {
        name          : "uart",
        displayName   : "UART Peripheral",
        interfaceName : "UART",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [uart];
}

function ssiPinmuxRequirements(inst)
{
    var peripheralName = "SSI";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let ssi = {
        name          : "ssi",
        displayName   : "SSI Peripheral",
        interfaceName : "SSI",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [ssi];
}


function ecatPinmuxRequirements(inst)
{
    var peripheralName = "ECAT";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1)
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let ecat = {
        name          : "ecat",
        displayName   : "ECAT Peripheral",
        interfaceName : "ECAT",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [ecat];
}

function ethernetPinmuxRequirements(inst)
{
    var peripheralName = "ETHERNET";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1 ||
            // No options available
            system.deviceData.interfaces[peripheralName].interfacePins[interfaceName].pinMappings[0].pinMappings.length <= 0    
        )
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let ethernet = {
        name          : "ethernet",
        displayName   : "ETHERNET Peripheral",
        interfaceName : "ETHERNET",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [ethernet];
}


function hicPinmuxRequirements(inst)
{
    var peripheralName = "HIC";
    let resources = [];
    var signalTypes = {};
    var useCaseName = inst.useCase;
    var useCaseInterfaces = getPeripheralUseCaseInterfaces(inst, peripheralName, useCaseName);
    var InterfaceNames = Object.keys(system.deviceData.interfaces[peripheralName].interfacePins);
    var i = 1;
    for (var interfaceNumber in InterfaceNames)
    {
        var interfaceName = InterfaceNames[interfaceNumber];

        if (useCaseInterfaces.indexOf(interfaceName) == -1 ||
            // No options available
            system.deviceData.interfaces[peripheralName].interfacePins[interfaceName].pinMappings[0].pinMappings.length <= 0    
        )
        {
           continue;
        }
        let pt = {
            name              : interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Pin",  /* config script name  THE ACTUAL NAME USED to find the pin*/
            displayName       : interfaceName.replace("#", "").replace("@", ""), /* GUI name */
            interfaceNames    : [interfaceName]    /* pinmux tool name */
        };

        resources.push(pt);

        signalTypes[pt.name] = interfaceName;
        i++;
    }

    let hic = {
        name          : "hic",
        displayName   : "HIC Peripheral",
        interfaceName : "HIC",
        
        resources     : resources,
        signalTypes   : signalTypes
    };

    return [hic];
}



exports = {
    useCaseChanged : useCaseChanged,
    addCustomPinmuxEnumToConfig : addCustomPinmuxEnumToConfig,
    getPeripheralUseCaseNames : getPeripheralUseCaseNames,
    spiPinmuxRequirements : spiPinmuxRequirements,
    sciPinmuxRequirements : sciPinmuxRequirements,
    gpioPinmuxRequirements : gpioPinmuxRequirements,
    i2cPinmuxRequirements : i2cPinmuxRequirements,
    inputxbarPinmuxRequirements: inputxbarPinmuxRequirements,
    outputxbarPinmuxRequirements : outputxbarPinmuxRequirements,
    clb_outputxbarPinmuxRequirements : clb_outputxbarPinmuxRequirements,
    aioPinmuxRequirements : aioPinmuxRequirements,
    canPinmuxRequirements : canPinmuxRequirements,
    epwmPinmuxRequirements : epwmPinmuxRequirements,
    eqepPinmuxRequirements : eqepPinmuxRequirements,
    mcbspPinmuxRequirements : mcbspPinmuxRequirements,
    sdPinmuxRequirements : sdPinmuxRequirements,
    uppPinmuxRequirements : uppPinmuxRequirements,
    usbPinmuxRequirements : usbPinmuxRequirements,
    fsitxPinmuxRequirements : fsitxPinmuxRequirements,
    fsirxPinmuxRequirements : fsirxPinmuxRequirements,
    otherPinmuxRequirements : otherPinmuxRequirements,
    dcdcPinmuxRequirements : dcdcPinmuxRequirements,
    emif1PinmuxRequirements : emif1PinmuxRequirements,
    emif2PinmuxRequirements : emif2PinmuxRequirements,
    linPinmuxRequirements : linPinmuxRequirements,
    pmbusPinmuxRequirements : pmbusPinmuxRequirements,
    cmi2cPinmuxRequirements : cmi2cPinmuxRequirements,
    mcanPinmuxRequirements : mcanPinmuxRequirements,
    uartPinmuxRequirements : uartPinmuxRequirements,
    ssiPinmuxRequirements : ssiPinmuxRequirements,
    ecatPinmuxRequirements : ecatPinmuxRequirements,
    ethernetPinmuxRequirements : ethernetPinmuxRequirements,
    hicPinmuxRequirements : hicPinmuxRequirements,
};