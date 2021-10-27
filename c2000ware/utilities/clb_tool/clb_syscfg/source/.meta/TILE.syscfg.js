"use strict";
/*global exports, system*/

let Common   = system.getScript("/utilities/clb_tool/clb_syscfg/source/Common.js");
let logError = Common.logError;


/* Intro splash on GUI */
let longDescription = "Configurable Logic Block (CLB)";

function moduleInstances(inst)
{
    let components = [
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/BOUNDARY",
            name: "BOUNDARY",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/LUT",
            name: "LUT_0",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/LUT",
            name: "LUT_1",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/LUT",
            name: "LUT_2",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/FSM",
            name: "FSM_0",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/FSM",
            name: "FSM_1",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/FSM",
            name: "FSM_2",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/COUNTER",
            name: "COUNTER_0",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/COUNTER",
            name: "COUNTER_1",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/COUNTER",
            name: "COUNTER_2",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_0",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_1",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_2",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_3",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_4",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_5",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_6",
            collapsed: true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/OUTLUT",
            name: "OUTLUT_7",
            collapsed: true
        },
		{
            moduleName: "/utilities/clb_tool/clb_syscfg/source/HLC",
            name: "HLC",
            collapsed: true
        }
    ];

    if (Common.isType1_Type2())
    {
        components = components.concat([
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_0",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_1",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_2",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_3",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_4",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_5",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_6",
                collapsed: true
            },
            {
                moduleName: "/utilities/clb_tool/clb_syscfg/source/AOC",
                name: "AOC_7",
                collapsed: true
            }
        ]);
    }
    return components;
}

var numberOfTiles = 4

if (system.deviceData.deviceId.includes("F2838x"))
{
	numberOfTiles = 8;
}
else if (system.deviceData.deviceId.includes("F28002x"))
{
	numberOfTiles = 2;
}

var config = [ 
	{ 
		name : 'unused', 
		default : '', 
		hidden: true
	} 
]

if (Common.isType2()){
	config = [ 
		{ 
			name : 'pipeline_mode',
			displayName : 'Pipeline Mode',
			description : 'Pipeline Mode for COUNTER and HLC submodules', 
			default : false, 
		} 
	]
}


// Define the common/portable base Watchdog
exports = {
    displayName         : "TILE",
    description         : "TILE",
    defaultInstanceName : "TILE",
    longDescription     : longDescription,
    maxInstances        : numberOfTiles,
    moduleInstances     : moduleInstances,
    documentation       : "/utilities/clb_tool/clb_syscfg/source/CLB Tool Users Guide (beta1).pdf",
    moduleStatic        : {
        config : [
                    {
                        name: "GROUP_SIMULATION",
                        displayName: "Simulation Options",
                        longDescription: "Options for CLB simulation files",
                        config: [
                            {
                            name        : 'clock_period',
                            description : 'Period of the clock (in Nano Seconds) used for simulation in System C',
                            default     : 20
                            },
                            {
                            name        : 'sim_duration',
                            description : 'Duration of the simulation (in Nano Seconds)',
                            default     : 50000
                            },
                            {
                            name        : 'reset_duration',
                            description : 'Time at which reset signal is deassrted, specified in Nano Seconds',
                            default     : 40
                            }
                        ]
                    }
                ]
    },
    modules: (inst) => {
        if (inst) {
            return [
                {
                    name: "pullInTemplateH",
                    moduleName: "/utilities/clb_tool/clb_syscfg/source/clb_run_dynamic_template_clb_h.js"
                },
                {
                    name: "pullInTemplateC",
                    moduleName: "/utilities/clb_tool/clb_syscfg/source/clb_run_dynamic_template_clb_c.js"
                },
                {
                    name: "pullInTemplateSIM",
                    moduleName: "/utilities/clb_tool/clb_syscfg/source/clb_run_dynamic_template_clb_sim.js"
                },
                {
                    name: "pullInTemplateDOT",
                    moduleName: "/utilities/clb_tool/clb_syscfg/source/clb_run_dynamic_template_clb_dot.js"
                },
            ];
        }
        return [];
    }
    // The following line should not be needed, but is here to woraround a bug
    ,config              : config,
};
