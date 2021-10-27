let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let InputXbar_DevSpecific  = system.getScript("/driverlib/inputxbar/inputxbar_deviceSpecific.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_xbar.js");

/* Intro splash on GUI */
let longDescription = "The Input X-BAR is used to route signals from a GPIO to many different" +
                    " IP blocks such as the ADC(s), eCAP(s), ePWM(s), and external interrupts";


function onChangeInputsUsed(inst, ui)
{
    var usedInputs = inst.inputsUsed;
    for (var uiInputXbarNameIndex in uiInputXbarNames)
    {
        var uiName = uiInputXbarNames[uiInputXbarNameIndex];
        //console.log(uiName);
        ui[uiName].hidden = true;

        uiName = uiInputXbarLockNames[uiInputXbarNameIndex];
        //console.log(uiName);
        ui[uiName].hidden = true;
    }
    for (var usedInputIndex in usedInputs)
    {
        var usedInput = usedInputs[usedInputIndex];
        for (uiInputXbarNameIndex in uiInputXbarNames)
        {
            var uiName = uiInputXbarNames[uiInputXbarNameIndex];
            //console.log("ui Name:" + uiName)
            //console.log("used input name:" + usedInput)
            if (usedInput == uiName)
            {
                //console.log("Name match")
                //console.log(uiName);
                ui[uiName].hidden = false;

                uiName = uiInputXbarLockNames[uiInputXbarNameIndex];
                //console.log(uiName);
                ui[uiName].hidden = false;
            }
        }
    }
}

/* Array of Input X-bar configurables that are common across device families */
var config = [
/*
    {
        name        : "bitRate",
        displayName : "Bit Rate",
        description : 'Bit rate for device',
        hidden      : false,
        default     : '400'
    }
*/
];

var gpios = Common.getGPIOs();
var aios = Common.getAIOs();
var inputxbarGPIOOptions = InputXbar_DevSpecific.getExtraInputOptions();
for (var gpioIndex = 0; gpioIndex < gpios.length; gpioIndex++)
{
    inputxbarGPIOOptions.push({name: gpios[gpioIndex]});
}
for (var aioIndex = 0; aioIndex < aios.length; aioIndex++)
{
    inputxbarGPIOOptions.push({name: aios[aioIndex]});
}



var INPUTXBARInterfaceNames = Object.keys(system.deviceData.interfaces.INPUTXBAR.interfacePins);

var interfaceNames = []
var uiInputXbarNames = []
var uiInputXbarLockNames = []
for (var interfaceNumber in INPUTXBARInterfaceNames)
{
    var interfaceName = INPUTXBARInterfaceNames[interfaceNumber];

    var uiInputXbarName = interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Gpio"
    var uiInputXbarLockName = interfaceName.toLowerCase().replace("#", "").replace("@", "") + "Lock"

    uiInputXbarNames.push(uiInputXbarName)
    uiInputXbarLockNames.push(uiInputXbarLockName)


    interfaceNames.push({name: uiInputXbarName, displayName: interfaceName})

    var inputxbarGPIOConfig = {
        name        : uiInputXbarName,
        displayName : interfaceName,
        description : 'GPIO for this Input X-Bar',
        hidden      : true,
        default     : inputxbarGPIOOptions[0].name,
        options     : inputxbarGPIOOptions
    }

    var inputxbarLOCKConfig = {
        name        : uiInputXbarLockName,
        displayName : interfaceName + " Lock",
        description : 'Lock this Input X-Bar',
        hidden      : true,
        default     : false
    }

    config.push(inputxbarGPIOConfig);
    config.push(inputxbarLOCKConfig);
}

config.unshift(
    {
        name        : "inputsUsed",
        displayName : "INPUTs to be used",
        description : 'Which inputs to use from the INPUT X-BAR.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        onChange    : onChangeInputsUsed,
        options     :  interfaceNames
        
    })




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
    return (Common.typeMatches(component.type, ["INPUTXBAR"]));
}

if (Common.onlyPinmux())
{
    //config = [];
}
var inputxbarModule = {
    peripheralName: "INPUTXBAR",
    displayName: "INPUTXBAR",
    maxInstances: Common.peripheralCount("INPUTXBAR"),
    defaultInstanceName: "myINPUTXBAR",
    description: "Input X-bar",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/inputxbar/inputxbar.board.c.xdt",
        boardh : "/driverlib/inputxbar/inputxbar.board.h.xdt"
    },
    //pinmuxRequirements    : Pinmux.inputxbarPinmuxRequirements
};


if (inputxbarModule.maxInstances <= 0)
{
    delete inputxbarModule.pinmuxRequirements;
}


exports = inputxbarModule;