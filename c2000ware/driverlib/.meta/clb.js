let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_clb.js");

/* Intro splash on GUI */
let longDescription = "";
var MAX_OUTPUTS = 16;
if (Common.CLB_isType1_Type2())
{
    MAX_OUTPUTS = 32;
}

var staticSwitchOptions = [
    { name: 0	, displayName: '0'                      },
    { name: 8	, displayName: '1'                      },
    { name: 24	, displayName: 'BOUNDARY.in0'           },
    { name: 25	, displayName: 'BOUNDARY.in1'           },
    { name: 26	, displayName: 'BOUNDARY.in2'           },
    { name: 27	, displayName: 'BOUNDARY.in3'           },
    { name: 28	, displayName: 'BOUNDARY.in4'           },
    { name: 29	, displayName: 'BOUNDARY.in5'           },
    { name: 30	, displayName: 'BOUNDARY.in6'           },
    { name: 31	, displayName: 'BOUNDARY.in7'           },
    { name: 7	, displayName: 'LUT_0.OUT'              },
    { name: 15	, displayName: 'LUT_1.OUT'              },
    { name: 23	, displayName: 'LUT_2.OUT'              },
    { name: 6	, displayName: 'FSM_0.OUT'              },
    { name: 4	, displayName: 'FSM_0.S0'               },
    { name: 5	, displayName: 'FSM_0.S1'               },
    { name: 14	, displayName: 'FSM_1.OUT'              },
    { name: 12	, displayName: 'FSM_1.S0'               },
    { name: 13	, displayName: 'FSM_1.S1'               },
    { name: 22	, displayName: 'FSM_2.OUT'              },
    { name: 20	, displayName: 'FSM_2.S0'               },
    { name: 21	, displayName: 'FSM_2.S1'               },
    { name: 2	, displayName: 'COUNTER_0.count_zero'   },
    { name: 3	, displayName: 'COUNTER_0.count_match1' },
    { name: 1	, displayName: 'COUNTER_0.count_match2' },
    { name: 10	, displayName: 'COUNTER_1.count_zero'   },
    { name: 11	, displayName: 'COUNTER_1.count_match1' },
    { name: 9	, displayName: 'COUNTER_1.count_match2' },
    { name: 18	, displayName: 'COUNTER_2.count_zero'   },
    { name: 19	, displayName: 'COUNTER_2.count_match1' },
    { name: 1	, displayName: 'COUNTER_2.count_match2' },
];

var totatCLBOutputs = [];
for (var i = 0; i < MAX_OUTPUTS; i++)
{
    totatCLBOutputs.push({
        name: "CLB_OUT" + i.toString(), displayName: "Output " + i.toString()
    })
}

var CLB_INSTANCE = [
    { name: "CLB1_BASE", displayName: "CLB1"},
    { name: "CLB2_BASE", displayName: "CLB2"},
    { name: "CLB3_BASE", displayName: "CLB3"},
    { name: "CLB4_BASE", displayName: "CLB4"}
]

var numberOfCLBs = 4;

if (Common.getDeviceName().includes("F28002x"))
{
    CLB_INSTANCE = [
        { name: "CLB1_BASE", displayName: "CLB1"},
        { name: "CLB2_BASE", displayName: "CLB2"}
    ];
    numberOfCLBs = 2;
}
else if (Common.getDeviceName().includes("F2838x"))
{
    CLB_INSTANCE = [
        { name: "CLB1_BASE", displayName: "CLB1"},
        { name: "CLB2_BASE", displayName: "CLB2"},
        { name: "CLB3_BASE", displayName: "CLB3"},
        { name: "CLB4_BASE", displayName: "CLB4"},
        { name: "CLB5_BASE", displayName: "CLB5"},
        { name: "CLB6_BASE", displayName: "CLB6"},
        { name: "CLB7_BASE", displayName: "CLB7"},
        { name: "CLB8_BASE", displayName: "CLB8"}
    ]
    numberOfCLBs = 8;
}

let INPUT_TYPES = [
    {name: "GP", displayName: "Use Memory Mapped GPREG BIT"},
    {name: "GLOBAL", displayName: "Use Global Mux"},
    {name: "LOCAL", displayName: "Use Local Mux"}
]

function onChangeCLBBase(inst, ui){
    if (Common.CLB_isType2())
    {
        if (["CLB1_BASE", "CLB2_BASE", "CLB3_BASE", "CLB4_BASE"].includes(inst.clbBase)){
            ui["spiBufferAccess"].hidden = false;
            ui["spiBufferLoadTriggerSignal"].hidden = false;
            ui["spiBufferR0Shift"].hidden = false;
        }
        else
        {
            ui["spiBufferAccess"].hidden = true;
            ui["spiBufferLoadTriggerSignal"].hidden = true;
            ui["spiBufferR0Shift"].hidden = true;
        }
    }
}

function onChangeInputType(inst, ui)
{
    for (var clb_input_i in device_driverlib_peripheral.CLB_Inputs)
    {
        var clbInputName = device_driverlib_peripheral.CLB_Inputs[clb_input_i].name

        if (inst.inputsUsed.includes(clbInputName))
        {
            if (inst["inputType" + clbInputName] == "GP")
            {
                ui["synchronize" + clbInputName].hidden = true;
                ui["localConfig" + clbInputName].hidden = true;
                ui["globalConfig" + clbInputName].hidden = true;
            }
            else if (inst["inputType" + clbInputName] == "GLOBAL")
            {
                ui["synchronize" + clbInputName].hidden = false;
                ui["localConfig" + clbInputName].hidden = true;
                ui["globalConfig" + clbInputName].hidden = false;
            }
            else if (inst["inputType" + clbInputName] == "LOCAL")
            {
                ui["synchronize" + clbInputName].hidden = false;
                ui["localConfig" + clbInputName].hidden = false;
                ui["globalConfig" + clbInputName].hidden = true;
            }
        }
        else
        {
            ui["synchronize" + clbInputName].hidden = true;
            ui["localConfig" + clbInputName].hidden = true;
            ui["globalConfig" + clbInputName].hidden = true;
        }

        ui["inputType" + clbInputName].hidden = !inst.inputsUsed.includes(clbInputName);
        ui["filterConfig" + clbInputName].hidden = !inst.inputsUsed.includes(clbInputName);
        ui["gpStartValueConfig" + clbInputName].hidden = !inst.inputsUsed.includes(clbInputName);
        if (Common.CLB_isType2()){
            ui["inputPipelineConfig" + clbInputName].hidden = !inst.inputsUsed.includes(clbInputName);
        }
    }
}


/* Array of CAN configurables that are common across device families */
let config = [
    {
        name        : "clbBase",
        displayName : "CLB Instance",
        description : 'Instance of the CLB used.',
        onChange    : onChangeCLBBase,
        hidden      : false,
        default     : CLB_INSTANCE[0].name,
        options     : CLB_INSTANCE
    },
    {
        name: "enableCLB",
        displayName : "Enable CLB",
        description : 'Enable the CLB',
        hidden      : false,
        default     : false
    },
    {
        name: "outputsToOverridePeripheral",
        displayName : "Overriding Outputs",
        description : 'Outputs from the CLB which will intersect the peripheral signals',
        hidden      : false,
        default     : [],
        minSelections : 0,
        options     : totatCLBOutputs
    },
];

if (Common.CLB_isType2())
{
    config = config.concat([
        {
            name: "lockOutputsToOverridePeripheral",
            displayName : "Lock Overriding Outputs Setting",
            description : 'Lock the setting for outputs from the CLB which will intersect the peripheral signals',
            hidden      : false,
            default     : false
        }
    ]);
}

if (Common.CLB_isType1_Type2())
{
    config = config.concat([
        {
            name: "hlcNMI",
            displayName : "HLC Generates NMI",
            description : 'The HLC interrupts are not maskable',
            hidden      : false,
            default     : false
        }
    ]);

    config = config.concat([{
        name: "GROUP_PRESCALAR",
        displayName: "Clock Prescalar",
        description: "CLB input prescalar configuration",
        longDescription: "",
        config: [
            {
                name: "clken",
                displayName : "Enable Prescalar",
                description : 'Enable the clock prescalar',
                hidden      : false,
                default     : false
            },
            {
                name: "strb",
                displayName : "Enable Strobe Mode",
                description : 'Enable the strobe mode. When disabled, a strobe output will be sent out whenever' + 
                              ' the prescale counter value matches the Prescale Value.' +  
                              ' When enabled, the output is the prescale counter register bit position as selected by ' +
                              'TAP Select Bit.',
                hidden      : false,
                default     : false
            },
            {
                name: "tap",
                displayName : "Tap Select Bit",
                description : 'The prescaler Tap Select Bit in Strobe Mode.',
                hidden      : false,
                default     : 0,
                options     : Common.zero_to_15
            },
            {
                name: "prescale",
                displayName : "Prescale Value",
                description : 'The prescale value for the clock.',
                hidden      : false,
                default     : 1
            }
        ]
    }]);
}

if (Common.CLB_isType2())
{
    config = config.concat([{
        name: "GROUP_SPIDATAEXPORT",
        displayName: "Data Exporting Through SPI Buffer",
        description: "",
        longDescription: "Export CLB data through SPI buffer. Data is HLC R0 regiter",
        config: [
            {
                name: "spiBufferAccess",
                displayName : "Enable SPI Buffer Access",
                description : 'Enable CLB data exporting through SPI buffer.',
                hidden      : false,
                default     : false
            },
            {
                name: "spiBufferLoadTriggerSignal",
                displayName : "SPI Buffer Load Trigger Signal",
                description : 'SPI buffer load trigger signal.',
                hidden      : false,
                default     : staticSwitchOptions[0].name,
                options     : staticSwitchOptions
            },
            {
                name: "spiBufferR0Shift",
                displayName : "HLC R0 Register Shift Value",
                description : 'HLC R0 register shift value.',
                hidden      : false,
                default     : Common.zero_to_16[0].name,
                options     : Common.zero_to_16
            }
        ]
    }]);
}

config = config.concat([
    {
        name: "inputsUsed",
        displayName : "Inputs Used",
        description : 'Used inputs for the CLB',
        hidden      : false,
        default     : [],
        minSelections : 0,
        options     : device_driverlib_peripheral.CLB_Inputs,
        onChange    : onChangeInputType
    }
]);

for (var clb_input_i in device_driverlib_peripheral.CLB_Inputs)
{
    var clbInputName = device_driverlib_peripheral.CLB_Inputs[clb_input_i].name
    var clbInputDisplayName = device_driverlib_peripheral.CLB_Inputs[clb_input_i].displayName

    var inputTypeConfig = {
        name: "inputType" + clbInputName,
        displayName : "Input Type " + clbInputDisplayName,
        description : 'Input type for the CLB input',
        hidden      : true,
        default     : INPUT_TYPES[0].name,
        options     : INPUT_TYPES,
        onChange    : onChangeInputType
    }

    var synchronizeConfig = {
        name: "synchronize" + clbInputName,
        displayName : "Enable Sync " + clbInputDisplayName,
        description : 'Enable the CLB input synchronization',
        hidden      : true,
        default     : false
    }

    var localConfig = {
        name: "localConfig" + clbInputName,
        displayName : "Local Mux Input " + clbInputDisplayName,
        description : 'Local Mux input for CLB',
        hidden      : true,
        default     : device_driverlib_peripheral.CLB_LocalInputMux[0].name,
        options     : device_driverlib_peripheral.CLB_LocalInputMux
    }
    
    var globalConfig = {
        name: "globalConfig" + clbInputName,
        displayName : "Global Mux Input " + clbInputDisplayName,
        description : 'Global Mux input for CLB',
        hidden      : true,
        default     : device_driverlib_peripheral.CLB_GlobalInputMux[0].name,
        options     : device_driverlib_peripheral.CLB_GlobalInputMux
    }
    
    var gpStartValueConfig = {
        name: "gpStartValueConfig" + clbInputName,
        displayName : "GPREG Initial Value " + clbInputDisplayName,
        description : 'GPREG Initial Value for ' + clbInputDisplayName,
        hidden      : true,
        default     : 0,
        options     : [
            { name : 0 },
            { name : 1 }
        ]
    }

    var filterConfig = {
        name: "filterConfig" + clbInputName,
        displayName : "Input Filter " + clbInputDisplayName,
        description : 'Input filter for ' + clbInputDisplayName,
        hidden      : true,
        default     : device_driverlib_peripheral.CLB_FilterType[0].name,
        options     : device_driverlib_peripheral.CLB_FilterType
    }

    var inputConfigs = [
        inputTypeConfig,
        localConfig,
        globalConfig,
        synchronizeConfig,
        filterConfig,
        gpStartValueConfig,
    ]

    if (Common.CLB_isType2())
    {
        var inputPipelineModeConfig = {
            name: "inputPipelineConfig" + clbInputName,
            displayName : "Input Pipeline " + clbInputDisplayName,
            description : 'Enable Input Pipeline for ' + clbInputDisplayName,
            hidden      : true,
            default     : false
        }

        inputConfigs.push(inputPipelineModeConfig);
    }

    
    var clbInputGroupConfig = [{
        name: "GROUP_" + clbInputName,
        displayName: "CLB " + clbInputDisplayName,
        description: "CLB Input Configuration " + clbInputDisplayName,
        longDescription: "",
        config: inputConfigs
    }];

    config = config.concat(clbInputGroupConfig);
    
}

function findDuplicates(arrayToCheck)
{
    const count = arrayToCheck =>
      arrayToCheck.reduce((a, b) => ({ ...a,
        [b]: (a[b] || 0) + 1
      }), {})

    const duplicates = dict =>
      Object.keys(dict).filter((a) => dict[a] > 1)

    return {count: count(arrayToCheck), duplicates: duplicates(count(arrayToCheck))};
}

function onValidate(inst, validation) {

    var usedCLBInsts = [];
    for (var instance_index in inst.$module.$instances)
    {
        var instance_obj = inst.$module.$instances[instance_index];
        usedCLBInsts.push(instance_obj.clbBase);
    }

    var duplicatesResult = findDuplicates(usedCLBInsts)

    if (duplicatesResult.duplicates.length != 0)
    {
        var allDuplicates = "";
        for (var duplicateNamesIndex in duplicatesResult.duplicates)
        {
            allDuplicates = allDuplicates + Common.stringOrEmpty(allDuplicates, ", ") 
                            + duplicatesResult.duplicates[duplicateNamesIndex];
        }
        validation.logError(
            "The CLB Instance used. Duplicates: " + allDuplicates, 
            inst, "clbBase");
    }

    if (inst.prescale < 0 || inst.prescale > 0xFFFF)
    {
        validation.logError(
            "The CLB prescale value must be a valid 16-bit number", 
            inst, "prescale");
    }

    var base = inst["clbBase"];
    var clb_1to4 = false;
    var clb_1 = false;
    if (["CLB1_BASE", 
        "CLB2_BASE", 
        "CLB3_BASE", 
        "CLB4_BASE", ].includes(base))
    {
        clb_1to4 = true;
    }
    if ("CLB1_BASE" == base)
    {
        clb_1 = true;
    }

    var F2838x = Common.getDeviceName().includes("F2838x");
    var F28002x = Common.getDeviceName().includes("F28002x");

    // Check global and local sources
    for (var clb_input_i in device_driverlib_peripheral.CLB_Inputs)
    {
        var clbInputName = device_driverlib_peripheral.CLB_Inputs[clb_input_i].name

        if (inst.inputsUsed.includes(clbInputName))
        {
            var global_or_local = false;
            var config_pre = "";
            var enum_name = "";
            if (inst["inputType" + clbInputName] == "GLOBAL")
            {
                global_or_local = true;
                config_pre = "globalConfig";
                enum_name = "CLB_GlobalInputMux"
            }
            else if (inst["inputType" + clbInputName] == "LOCAL")
            {
                global_or_local = true; 
                config_pre = "localConfig";
                enum_name = "CLB_LocalInputMux"
            }

            if (global_or_local)
            {
                var result = device_driverlib_peripheral[enum_name].find(input => {
                    return input.name === inst[config_pre + clbInputName]
                })
                var diplayName = result.displayName.replace(/ /g, "")

                if (F2838x &&
                    diplayName.includes("(CLB5") &&
                    clb_1to4)
                {
                    validation.logError(
                        "This CLB input is only applicable for CLB 5-8", 
                        inst, config_pre + clbInputName);
                }
                if (F2838x &&
                    diplayName.includes("(CLB1") &&
                    !clb_1to4)
                {
                    validation.logError(
                        "This CLB input is only applicable for CLB 1-4", 
                        inst, config_pre + clbInputName);
                }
    
                if (F28002x &&
                    diplayName.includes("(CLB2") &&
                    clb_1)
                {
                    validation.logError(
                        "This CLB input is only applicable for CLB 2", 
                        inst, config_pre + clbInputName);
                }
                if (F28002x &&
                    diplayName.includes("(CLB1") &&
                    !clb_1)
                {
                    validation.logError(
                        "This CLB input is only applicable for CLB 1", 
                        inst, config_pre + clbInputName);
                }
            }
        }
    }
}


var clbModule = {
    peripheralName: "CLB",
    displayName: "CLB",
    maxInstances: numberOfCLBs,
    defaultInstanceName: "myCLB",
    description: "Configurable Logic Block",
    config: config,
    templates: {
        boardc : "/driverlib/clb/clb.board.c.xdt",
        boardh : "/driverlib/clb/clb.board.h.xdt"
    },
    validate    : onValidate,
};

exports = clbModule;