let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");
let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_epwm.js");

var TzEventConfigNames = [];
var TzAdvEventConfigNames = [];


function onChangeUseTripZone(inst, ui)
{
    for(var uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
    {
        var configName = config[uiConfigIndex].name;
        ui[configName].hidden = !inst.useTripZone;
    }
    if (inst.useTripZone)
    {
        onChangeUseAdvancedEPWMTripZoneActions(inst, ui);
    }
}

function onChangeUseAdvancedEPWMTripZoneActions(inst, ui)
{
    for (var tzEventConfigNameIndex in TzEventConfigNames)
    {
        var tzEventConfigName = TzEventConfigNames[tzEventConfigNameIndex];
        ui[tzEventConfigName].hidden = inst.useAdvancedEPWMTripZoneActions;
    }
    for (var tzAdvEventConfigNameIndex in TzAdvEventConfigNames)
    {
        var tzAdvEventConfigName = TzAdvEventConfigNames[tzAdvEventConfigNameIndex];
        ui[tzAdvEventConfigName].hidden = !inst.useAdvancedEPWMTripZoneActions;
    }
    
}

/* Array of CAN configurables that are common across device families */
var config = [
    {
        name: "useTripZone",
        displayName : "Use Trip Zone",
        description : 'Check to configure the Trip Zone Submodule',
        hidden      : false,
        default     : false,
        onChange    : onChangeUseTripZone
    },
    {
        name: "useAdvancedEPWMTripZoneActions",
        displayName : "Use Advanced EPWM Trip Zone Actions",
        description : 'Check this to use Advanced EPWM trip zone actions. Uncheck to use legacy (TZCTL2[ETZE])',
        hidden      : true,
        default     : false,
        onChange    : onChangeUseAdvancedEPWMTripZoneActions
    },

];

for (var tzEventIndex in device_driverlib_peripheral.EPWM_TripZoneEvent)
{
    var tzEvent = device_driverlib_peripheral.EPWM_TripZoneEvent[tzEventIndex];
    var tzEventName = tzEvent.name.replace("EPWM_TZ_ACTION_EVENT_", "");
    var tzEventConfig = {
        name: tzEvent.name + "_config",
        displayName : tzEventName + " Event",
        description : 'The action to take on ' + tzEvent.name + ". " + "(" + tzEvent.displayName + ")",
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_TripZoneAction[0].name,
        options     : device_driverlib_peripheral.EPWM_TripZoneAction
    }
    config.push(tzEventConfig);
    TzEventConfigNames.push(tzEventConfig.name);
}


for (var tzEventIndex in device_driverlib_peripheral.EPWM_TripZoneAdvancedEvent)
{
    var tzEvent = device_driverlib_peripheral.EPWM_TripZoneAdvancedEvent[tzEventIndex];
    var tzEventName = tzEvent.name.replace("EPWM_TZ_ADV_ACTION_EVENT_", "");
    var tzEventConfig = {
        name: tzEvent.name + "_config",
        displayName : tzEventName + " Event (Adv)",
        description : 'The action to take on ' + tzEvent.name + ". " + "(" + tzEvent.displayName + ")",
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction[0].name,
        options     : device_driverlib_peripheral.EPWM_TripZoneAdvancedAction
    }
    config.push(tzEventConfig);
    TzAdvEventConfigNames.push(tzEventConfig.name);
}

var oneShotConfig = [
    {
        name: "oneShotSource",
        displayName : "One-Shot Source",
        description : 'Check to enable the source to the One-Shot OR gate',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : [
            {name: "EPWM_TZ_SIGNAL_OSHT1", displayName : "One-shot TZ1"},
            {name: "EPWM_TZ_SIGNAL_OSHT2", displayName : "One-shot TZ2"},
            {name: "EPWM_TZ_SIGNAL_OSHT3", displayName : "One-shot TZ3"},
            {name: "EPWM_TZ_SIGNAL_OSHT4", displayName : "One-shot TZ4"},
            {name: "EPWM_TZ_SIGNAL_OSHT5", displayName : "One-shot TZ5"},
            {name: "EPWM_TZ_SIGNAL_OSHT6", displayName : "One-shot TZ6"},
            {name: "EPWM_TZ_SIGNAL_DCAEVT1", displayName : "One-shot DCAEVT1"},
            {name: "EPWM_TZ_SIGNAL_DCBEVT1", displayName : "One-shot DCBEVT1"},
        ]
    },

];

var cbcConfig = [
    {
        name: "cbcSource",
        displayName : "CBC Source",
        description : 'Check to enable the source to the CBC OR gate',
        hidden      : true,
        minSelections : 0,
        default     : [],
        options     : [
            {name: "EPWM_TZ_SIGNAL_CBC1", displayName : "TZ1 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_CBC2", displayName : "TZ2 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_CBC3", displayName : "TZ3 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_CBC4", displayName : "TZ4 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_CBC5", displayName : "TZ5 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_CBC6", displayName : "TZ6 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_DCAEVT2", displayName : "DCAEVT2 Cycle By Cycle"},
            {name: "EPWM_TZ_SIGNAL_DCBEVT2", displayName : "DCBEVT2 Cycle By Cycle"},
        ]
    },
    {
        name: "cbcPulse",
        displayName : "CBC Latch Clear Signal",
        description : 'Select the CBC Trip Zone Latch Clear Signal (TZCLR[CBCPULSE])',
        hidden      : true,
        default     : device_driverlib_peripheral.EPWM_CycleByCycleTripZoneClearMode[0].name,
        options     : device_driverlib_peripheral.EPWM_CycleByCycleTripZoneClearMode
    },

];

config = config.concat(oneShotConfig);
config = config.concat(cbcConfig);


var epwmTripZoneSubmodule = {
    displayName: "EPWM Trip Zone",
    maxInstances: Common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_TZ",
    description: "Enhanced Pulse Width Modulator Trip Zone",
    config: config,
    templates: {
        boardc : "", //"/gpio/gpio.board.c.xdt",
        boardh : ""//"/gpio/gpio.board.h.xdt"
    },
};


exports = epwmTripZoneSubmodule;