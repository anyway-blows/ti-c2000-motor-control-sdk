let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

var DCCompareTypeConfigNames_CombinationNames = []
var DCAConfigs = [];
var DCBConfigs = [];


function onChangeuseDigitalCompare(inst, ui)
{
    for(var uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
    {
        var configName = config[uiConfigIndex].name;
        ui[configName].hidden = !inst.useDigitalCompare;
    }

    if (inst.useDigitalCompare)
    {
        onChangeuseDigitalCompareXModule(inst, ui);
    }

    onChangeTripInput(inst, ui);

}

function onChangeuseDigitalCompareXModule(inst, ui)
{
    for (var index in DCAConfigs)
    {
        var name = DCAConfigs[index].name;
        ui[name].hidden = !inst.useDigitalCompareA;
    }
    for (var index in DCBConfigs)
    {
        var name = DCBConfigs[index].name;
        ui[name].hidden = !inst.useDigitalCompareB;
    }
    
    onChangeTripInput(inst, ui);

}

function onChangeTripInput(inst, ui)
{
    for (var index in DCCompareTypeConfigNames_CombinationNames)
    {
        var typeName = DCCompareTypeConfigNames_CombinationNames[index].CompareTypeName;
        var combName = DCCompareTypeConfigNames_CombinationNames[index].CombinationName;
        if (inst[typeName].indexOf("COMBINATION") != -1)
        {
            if (typeName.indexOf("DCA") != -1 && inst.useDigitalCompareA)
            {
                ui[combName].hidden = false;
            }
            else if (typeName.indexOf("DCB") != -1 && inst.useDigitalCompareB)
            {
                ui[combName].hidden = false;
            }
        }
        else
        {
            ui[combName].hidden = true;
        }
    }
}

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "useDigitalCompare",
        displayName : "Use Digital Compare",
        description : 'Check to configure the Digital Compare Submodule',
        hidden      : false,
        default     : false,
        onChange    : onChangeuseDigitalCompare
    },
    {
        name: "useDigitalCompareA",
        displayName : "Use Digital Compare A Module",
        description : 'Check to configure the Digital Compare A Module (DCAEVTy)',
        hidden      : true,
        default     : false,
        onChange    : onChangeuseDigitalCompareXModule
    },
    {
        name: "useDigitalCompareB",
        displayName : "Use Digital Compare B Module",
        description : 'Check to configure the Digital Compare B Module (DCBEVTy)',
        hidden      : true,
        default     : false,
        onChange    : onChangeuseDigitalCompareXModule
    },

];


for (var dcCompareTypeIndex in device_driverlib_peripheral.EPWM_DigitalCompareType)
{
    var dcCompareType = device_driverlib_peripheral.EPWM_DigitalCompareType[dcCompareTypeIndex];

    var dcCompareTypeConfig = {
        name: dcCompareType.name + "_config",
        displayName : dcCompareType.displayName,
        description : "Trip input to the " +  dcCompareType.displayName,
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_DigitalCompareTripInput[0].name,
        options     : device_driverlib_peripheral.EPWM_DigitalCompareTripInput,
        onChange    : onChangeTripInput
    }


    var dcCombinationConfig = {
        name: dcCompareType.name + "_combinationInputConfig",
        displayName : "Combination Input Sources (" + dcCompareType.displayName + ")",
        description : "Select the sources to include in the Combination input sources",
        hidden      : true,
        minSelections: 0,
        default     : [],
        options     : [
                { displayName: "Trip 1 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN1" },
                { displayName: "Trip 2 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN2" },
                { displayName: "Trip 3 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN3" },
                { displayName: "Trip 4 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN4" },
                { displayName: "Trip 5 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN5" },
                { displayName: "Trip 6 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN6" },
                { displayName: "Trip 7 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN7" },
                { displayName: "Trip 8 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN8" },
                { displayName: "Trip 9 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN9" },
                { displayName: "Trip 10 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN10"},
                { displayName: "Trip 11 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN11"},
                { displayName: "Trip 12 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN12"},
                { displayName: "Trip 14 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN14"},
                { displayName: "Trip 15 input", name : "EPWM_DC_COMBINATIONAL_TRIPIN15"},
            ]
    };

    DCCompareTypeConfigNames_CombinationNames.push(
        {
            CompareTypeName : dcCompareTypeConfig.name,
            CombinationName : dcCombinationConfig.name
        }
    );

    if (dcCompareType.name.indexOf("DCA") != -1)
    {
        DCAConfigs.push(dcCompareTypeConfig);
        DCAConfigs.push(dcCombinationConfig);
    }
    else
    {
        DCBConfigs.push(dcCompareTypeConfig);
        DCBConfigs.push(dcCombinationConfig);
    }
}

for (var dcCompareOutputIndex in device_driverlib_peripheral.EPWM_TripZoneDigitalCompareOutput)
{
    var dcCompareOutput = device_driverlib_peripheral.EPWM_TripZoneDigitalCompareOutput[dcCompareOutputIndex];

    var dcCompareOutputConfig = {
        name: dcCompareOutput.name + "_config",
        displayName : "Condition For " + dcCompareOutput.displayName,
        description : "Select the Condition for " +  dcCompareOutput.displayName + " for which the EVENT is generated.",
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_TripZoneDigitalCompareOutputEvent[0].name,
        options     : device_driverlib_peripheral.EPWM_TripZoneDigitalCompareOutputEvent,
    }

    if (dcCompareOutput.name.indexOf("OUTPUT_A") != -1)
    {
        DCAConfigs.push(dcCompareOutputConfig);
    }
    else
    {
        DCBConfigs.push(dcCompareOutputConfig);
    }
}

for (var dcCompareModuleIndex in device_driverlib_peripheral.EPWM_DigitalCompareModule)
{
    for (var dcCompareEventIndex in device_driverlib_peripheral.EPWM_DigitalCompareEvent)
    {
        var dcCompareModule = device_driverlib_peripheral.EPWM_DigitalCompareModule[dcCompareModuleIndex];
        var dcCompareEvent = device_driverlib_peripheral.EPWM_DigitalCompareEvent[dcCompareEventIndex];

        var dcxevty = "DC" + dcCompareModule.name.replace("EPWM_DC_MODULE_","") + "EVT" + dcCompareEvent.name.replace("EPWM_DC_EVENT_","")

        var dcCompareEventSyncConfig = {
            name: dcCompareModule.name + "_" + dcCompareEvent.name + "_eventSync_config",
            displayName : "Synch Mode (" + dcxevty + ")",
            description : "Select the Sync Mode for " +  dcCompareModule.displayName + " " + dcCompareEvent.displayName,
            hidden      : true,
            default     : device_driverlib_peripheral.EPWM_DigitalCompareSyncMode[0].name,
            options     : device_driverlib_peripheral.EPWM_DigitalCompareSyncMode,
        }

        var dcCompareEventSourceConfig = {
            name: dcCompareModule.name + "_" + dcCompareEvent.name + "_eventSource_config",
            displayName : "Signal Source (" + dcxevty + ")",
            description : "Select the Signal Source for " +  dcCompareModule.displayName + " " + dcCompareEvent.displayName,
            hidden      : true,
            default     : device_driverlib_peripheral.EPWM_DigitalCompareEventSource[0].name,
            options     : device_driverlib_peripheral.EPWM_DigitalCompareEventSource,
        }

        if (dcCompareEvent.name == "EPWM_DC_EVENT_1")
        {
            var dcCompareGenerateADCTriggerConfig = {
                name: dcCompareModule.name + "_" + dcCompareEvent.name + "_adcTrig_config",
                displayName : "Generate ADC SOC (" + dcxevty + ")",
                description : "Enable/disable generation of ADC SOC Event for " +  dcCompareModule.displayName + " " + dcCompareEvent.displayName,
                hidden      : true,
                default     : false,
            }

            var dcCompareGenerateSyncTriggerConfig = {
                name: dcCompareModule.name + "_" + dcCompareEvent.name + "_syncTrig_config",
                displayName : "Generate SYNCOUT (" + dcxevty + ")",
                description : "Enable/disable generation of SYNCOUTEvent for " +  dcCompareModule.displayName + " " + dcCompareEvent.displayName,
                hidden      : true,
                default     : false,
            }

            if (dcCompareModule.name.indexOf("MODULE_A") != -1)
            {
                DCAConfigs.push(dcCompareGenerateADCTriggerConfig);
                DCAConfigs.push(dcCompareGenerateSyncTriggerConfig);
            }
            else
            {
                DCBConfigs.push(dcCompareGenerateADCTriggerConfig);
                DCBConfigs.push(dcCompareGenerateSyncTriggerConfig);
            }
        }

        if (dcCompareModule.name.indexOf("MODULE_A") != -1)
        {
            DCAConfigs.push(dcCompareEventSyncConfig);
            DCAConfigs.push(dcCompareEventSourceConfig);
        }
        else
        {
            DCBConfigs.push(dcCompareEventSyncConfig);
            DCBConfigs.push(dcCompareEventSourceConfig);
        }
    }
}


config = config.concat(DCAConfigs);
config = config.concat(DCBConfigs);


var epwmDigitalCompareSubmodule = {
    displayName: "EPWM Digital Compare",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_DC",
    description: "Enhanced Pulse Width Modulator Digital Compare",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmDigitalCompareSubmodule;