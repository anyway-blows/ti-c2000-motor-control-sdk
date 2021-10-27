let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_xbar.js");

/* Intro splash on GUI */
let longDescription = "The EPWM X-BAR takes signals from inside " + 
                        "the device and brings them to the EPWM TRIPINx inputs.";

function findDuplicates(arrayToCheck)
{
    const count = arrayToCheck =>
      arrayToCheck.reduce((a, b) => ({ ...a,
        [b]: (a[b] || 0) + 1
      }), {}) // don't forget to initialize the accumulator

    const duplicates = dict =>
      Object.keys(dict).filter((a) => dict[a] > 1)

    //console.log(count(arrayToCheck)) // { Mike: 1, Matt: 1, Nancy: 2, Adam: 1, Jenny: 1, Carl: 1 }
    //console.log(duplicates(count(arrayToCheck)))

    return {count: count(arrayToCheck), duplicates: duplicates(count(arrayToCheck))};
}

function onChangeTripInput(inst, ui)
{
    //console.log(inst.$module.$instances);
    
}

function onValidate(inst, validation) {

    var usedTrips = [];
    for (var instance_index in inst.$module.$instances)
    {
        var instance_obj = inst.$module.$instances[instance_index];
        usedTrips.push(instance_obj.tripInput);
    }

    var duplicatesResult = findDuplicates(usedTrips)

    if (duplicatesResult.duplicates.length != 0)
    {
        var allDuplicates = "";
        for (var duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + Common.stringOrEmpty(allDuplicates, ", ") 
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The Trip Inputs cannot be same for multiple modules. Duplicates: " + allDuplicates, 
            inst, "tripInput");
    }
}

function onChangeMuxesUsed(inst, ui)
{
    var muxesUsed = inst.muxesUsed;
    for (var muxConfigIndex in muxConfigs)
    {
        var muxConfigName = muxConfigs[muxConfigIndex];
        var muxNumber = parseInt(muxConfigName.replace("mux", "").replace("Config", ""))
        var turnOnMux = false
        for (var muxesUsedIndex in muxesUsed)
        {
            var usedMuxName = muxesUsed[muxesUsedIndex]
            var usedMuxNumber = parseInt(usedMuxName.replace("XBAR_MUX", ""))
            //console.log("used: " + usedMuxNumber)
            //console.log("checking: " + muxNumber)
            var turnOnMux = true;

            if (usedMuxNumber == muxNumber)
            {
                turnOnMux = true;
                break;
            }
            else
            {
                turnOnMux = false;
            }
            
        }
        if (turnOnMux)
        {
            ui[muxConfigName].hidden = false;
        }
        else
        {
            ui[muxConfigName].hidden = true;
        }
    }
}

var config = [];
var latchModeConfig = {
    name        : "latchMode",
    displayName : "Latch Mode",
    description : 'Whether the output is latched or not',
    hidden      : false,
    default     : false
};

var invertConfig = {
    name        : "invertMode",
    displayName : "Invert Mode",
    description : 'Whether the output is inverted or not',
    hidden      : false,
    default     : false
    };

var muxesUsed = {
        name        : "muxesUsed",
        displayName : "MUXes to be used",
        description : 'Which MUXes are be to use from the EPWM X-BAR.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        onChange    : onChangeMuxesUsed,
        options     : device_driverlib_peripheral.XBAR_MUXES
        
}

var tripInput = {
        name        : "tripInput",
        displayName : "Trip Input",
        description : 'Which Trip input is configured to be sourced from the muxes.',
        hidden      : false,
        default     : device_driverlib_peripheral.XBAR_TripNum[0].name,
        onChange    : onChangeTripInput,
        options     : device_driverlib_peripheral.XBAR_TripNum,
        
}


config.push(tripInput);
//config.push(latchModeConfig);
config.push(invertConfig);
config.push(muxesUsed);

var muxConfigs = []

for (var mux_number_index in device_driverlib_peripheral.XBAR_EPWMMuxConfig_All_MUXES)
{

    var muxSpecificOptions = device_driverlib_peripheral.XBAR_EPWMMuxConfig_All_MUXES[mux_number_index]
    if (muxSpecificOptions[0] != undefined)
    {
        var muxOptions = {
            name        : "mux" + mux_number_index + "Config",
            displayName : "MUX  " + mux_number_index + " Config",
            description : 'Mux Configuration for EPWM XBAR\'s MUX number ' + mux_number_index,
            hidden      : true,
            default     : muxSpecificOptions[0].name,
            options     : muxSpecificOptions
        };
        muxConfigs.push(muxOptions.name)

        config.push(muxOptions)
    }
}

var epwmxbarModule = {
    peripheralName: "EPWMXBAR",
    displayName: "EPWMXBAR",
    maxInstances: 8,
    defaultInstanceName: "myEPWMXBAR",
    description: "EPWM X-bar",
    config: config,
    templates: {
        boardc : "/driverlib/epwmxbar/epwmxbar.board.c.xdt",
        boardh : "/driverlib/epwmxbar/epwmxbar.board.h.xdt"
    },
    validate    : onValidate,
};

exports = epwmxbarModule;