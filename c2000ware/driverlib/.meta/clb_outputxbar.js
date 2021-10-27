let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_xbar.js");

/* Intro splash on GUI */
let longDescription = "The CLB Output X-BAR takes signals from " + 
                        "the CLB TILE OUTs and brings them out to a GPIO.";

let outputxbar_enum = device_driverlib_peripheral.XBAR_OutputMuxConfig;

var index_of_clbxbar = outputxbar_enum.length;
var clboutputxbar_only = [];
var start_of_clboutputxbar_only = false;
for (var i = 0; i < outputxbar_enum.length; i++){
    if (outputxbar_enum[i].name == "XBAR_OUT_MUX00_CLB1_OUT0")
    {
        start_of_clboutputxbar_only = true;
    }
    if (start_of_clboutputxbar_only)
    {
        clboutputxbar_only.push(outputxbar_enum[i].name)
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
        description : 'Which MUXes are be to use from the OUTPUT X-BAR.',
        hidden      : false,
        default     : [],
        minSelections: 0,
        onChange    : onChangeMuxesUsed,
        options     : device_driverlib_peripheral.XBAR_MUXES
}


config.push(latchModeConfig);
config.push(invertConfig);
config.push(muxesUsed);

var muxConfigs = []

for (var mux_number_index in device_driverlib_peripheral.XBAR_OutputMuxConfig_All_MUXES)
{
    var muxSpecificOptions = device_driverlib_peripheral.XBAR_OutputMuxConfig_All_MUXES[mux_number_index]
    if (muxSpecificOptions[0] != undefined)
    {
        var onlyclbxbar = muxSpecificOptions.filter(muxoption => {
            //console.log(muxoption.name)
            //console.log(!clboutputxbar_only.includes(muxoption.name))
            return clboutputxbar_only.includes(muxoption.name);
        });
        //console.log(removedclbxbar);
        if (onlyclbxbar.length > 0)
        {
            var muxOptions = {
                name        : "mux" + mux_number_index + "Config",
                displayName : "MUX  " + mux_number_index + " Config",
                description : 'Mux Configuration for CLB OUTPUT XBAR\'s MUX number ' + mux_number_index,
                hidden      : true,
                default     : onlyclbxbar[0].name,
                options     : onlyclbxbar
            };
            muxConfigs.push(muxOptions.name)

            config.push(muxOptions)
        }
    }
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
    return (Common.typeMatches(component.type, ["CLB_OUTPUTXBAR"]));
}

if (Common.onlyPinmux())
{
    config = [config[config.length - 1]];
}
var clb_outputxbarModule = {
    peripheralName: "CLB_OUTPUTXBAR",
    displayName: "CLB OUTPUTXBAR",
    maxInstances: Common.peripheralCount("CLB_OUTPUTXBAR"),
    defaultInstanceName: "myCLB_OUTPUTXBAR",
    description: "CLB Output X-bar",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/clb_outputxbar/clb_outputxbar.board.c.xdt",
        boardh : "/driverlib/clb_outputxbar/clb_outputxbar.board.h.xdt"
    },
    pinmuxRequirements    : Pinmux.clb_outputxbarPinmuxRequirements
};


if (clb_outputxbarModule.maxInstances <= 0)
{
    delete clb_outputxbarModule.pinmuxRequirements;
}

exports = clb_outputxbarModule;