let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

/* Array of CAN configurables that are common across device families */
var UsedAQEventConfigs = [];
var UsedAqOutput_aqEvent_Configs = [];
function onChangeUsedOutputs(inst, ui)
{
    for (var aqOutputIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputModule)
    {
        var aqOutput = device_driverlib_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];
        if (inst.usedOutputs.indexOf(aqOutput.name) != -1)
        {
            ui[UsedAQEventConfigs[aqOutputIndex].name].hidden = false;
            for (var aqEventIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent)
            {
                var aqEvent = device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent[aqEventIndex];
                var configName = aqOutput.name + "_" + aqEvent.name + "_config";
                if (inst[UsedAQEventConfigs[aqOutputIndex].name].indexOf(aqEvent.name) != -1)
                {
                    ui[configName].hidden = false;
                }
                else
                {
                    ui[configName].hidden = true;
                }
            }
        }
        else
        {
            ui[UsedAQEventConfigs[aqOutputIndex].name].hidden = true;
            for (var aqEventIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent)
            {
                var aqEvent = device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent[aqEventIndex];
                var configName = aqOutput.name + "_" + aqEvent.name + "_config";
                ui[configName].hidden = true;
            }
        }

    }
    
}

var config = [
    {
        name: "usedOutputs",
        displayName : "Outputs to Configure",
        description : '',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : device_driverlib_peripheral.EPWM_ActionQualifierOutputModule,
        onChange    : onChangeUsedOutputs,
    },
];

for (var aqOutputIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputModule)
{
    var aqOutput = device_driverlib_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];
    var usedEventsAQConfig = {
        name: "usedEvents" + aqOutput.name,
        displayName : "Events to Configure for " + 
                    aqOutput.displayName,
        description : '',
        hidden      : true,
        default     : [],
        minSelections: 0,
        options     : device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent,
        onChange    : onChangeUsedOutputs,
    };
    UsedAQEventConfigs.push(usedEventsAQConfig);
}

config = config.concat(UsedAQEventConfigs);

for (var aqOutputIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputModule)
{
    var aqOutput = device_driverlib_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];
    var aqOutputconfigs = []
    for (var aqEventIndex in device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent)
    {
        var aqEvent = device_driverlib_peripheral.EPWM_ActionQualifierOutputEvent[aqEventIndex];
        var aqOutput_aqEvent_config = {
            name: aqOutput.name + "_" + aqEvent.name + "_config",
            displayName : aqOutput.displayName.replace("output", "") + " " + aqEvent.displayName.replace("AQ OUTPUT ", ""),
            description : '',
            hidden      : true,
            default     : device_driverlib_peripheral.EPWM_ActionQualifierOutput[0].name,
            options     : device_driverlib_peripheral.EPWM_ActionQualifierOutput,
        };
        aqOutputconfigs.push(aqOutput_aqEvent_config);
        
    }
    config = config.concat(aqOutputconfigs);
    UsedAqOutput_aqEvent_Configs.push(aqOutputconfigs);
}





var epwmAQSubmodule = {
    displayName: "EPWM Action Qualifier",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_AQ",
    description: "Enhanced Pulse Width Modulator Action Qualifier",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmAQSubmodule;