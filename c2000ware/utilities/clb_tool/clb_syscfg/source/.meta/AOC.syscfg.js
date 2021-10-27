"use strict";
/*global exports, system*/

let Common   = system.getScript('/utilities/clb_tool/clb_syscfg/source/Common.js');
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = 'Output conditioning block';

/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name        : 'bypass_mux',
        displayName : "Bypass AOC",
        description : 'Bypass the AOC block using the Bypass Mux',
        default     : true
    },
    {
        name        : 'input_mode',
        displayName : "AOC Input Mode",
        description : 'AOC input mode, choose between using CLB input or CLB output as the input source ' +
                      'for the output conidtioning block',
        default     : "OUTPUT",
        options     : [
            {name:"OUTPUT", displayName:"CLB OUTPUT delayed by 1 cycle"},
            {name:"INPUT", displayName:"CLB INPUT"}
        ]
    },
    {
        name: "GROUP_STAGE1",
        displayName: "Stage 1",
        description: "Stage 1 of the AOC block (inverting)",
        config: [
            {
                name        : 'stage1_mux',
                displayName : "Invert Signal",
                description : 'Choose to invert the signal or bypass this mux',
                default     : false
            }
        ]
    },
    {
        name: "GROUP_STAGE2",
        displayName: "Stage 2",
        description: "Stage 2 of the AOC block (gating)",
        config: [
            {
                name        : 'stage2_mux',
                displayName : "Gate Type",
                description : 'Choose the type of GATE or bypass this mux',
                default     : "BYPASS",
                options     : [
                    {name:"BYPASS"},
                    {name:"AND", displayName: "AND Gate"},
                    {name:"OR", displayName: "OR Gate"},
                    {name:"XOR", displayName: "XOR Gate"}
                ]
            },
            {
                name        : 'stage2_gate_mux',
                displayName : "Gate Signal",
                description : 'Choose the GATE signal',
                default     : "SW",
                options     : [
                    {name:"SW", displayName:"Software Control"},
                    {name:"OUTPUT0", displayName: "CLB OUTPUT0"},
                    {name:"OUTPUT1", displayName: "CLB OUTPUT1"},
                    {name:"OUTPUT2", displayName: "CLB OUTPUT2"},
                    {name:"OUTPUT3", displayName: "CLB OUTPUT3"},
                    {name:"OUTPUT4", displayName: "CLB OUTPUT4"},
                    {name:"OUTPUT5", displayName: "CLB OUTPUT5"},
                    {name:"OUTPUT6", displayName: "CLB OUTPUT6"},
                    {name:"OUTPUT7", displayName: "CLB OUTPUT7"},
                ]
            },
        ]
    },
    {
        name: "GROUP_STAGE3",
        displayName: "Stage 3",
        description: "Stage 3 of the AOC block (release control)",
        config: [
            {
                name        : 'stage3_mux',
                displayName : "Release Type",
                description : 'Choose the type of SET/CLEAR/DELAY or bypass this mux',
                default     : "BYPASS",
                options     : [
                    {name:"BYPASS"},
                    {name:"CLEAR", displayName: "Clear signal"},
                    {name:"SET", displayName: "SET signal"},
                    {name:"DELAY", displayName: "Delay signal"}
                ]
            },
            {
                name        : 'stage3_release_mux',
                displayName : "Release Signal",
                description : 'Choose the RELEASE signal',
                default     : "SW",
                options     : [
                    {name:"SW", displayName:"Software Control"},
                    {name:"OUTPUT0", displayName: "CLB OUTPUT0"},
                    {name:"OUTPUT1", displayName: "CLB OUTPUT1"},
                    {name:"OUTPUT2", displayName: "CLB OUTPUT2"},
                    {name:"OUTPUT3", displayName: "CLB OUTPUT3"},
                    {name:"OUTPUT4", displayName: "CLB OUTPUT4"},
                    {name:"OUTPUT5", displayName: "CLB OUTPUT5"},
                    {name:"OUTPUT6", displayName: "CLB OUTPUT6"},
                    {name:"OUTPUT7", displayName: "CLB OUTPUT7"},
                ]
            },
        ]
    }
    
];

/**
 * Validate this module's configuration
 *
 * @param inst       - Watchdog instance to be validated
 * @param validation - Issue reporting object
 */
function validate(inst, vo)
{
    
}


// Define the common/portable base Watchdog
exports = {
    displayName         : 'AOC',
    description         : 'AOC',
    defaultInstanceName : 'AOC_',
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
    //maxInstances        : 3,
};
