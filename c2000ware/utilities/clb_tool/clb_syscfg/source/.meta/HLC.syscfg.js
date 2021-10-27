"use strict";
/*global exports, system*/

let Common   = system.getScript('/utilities/clb_tool/clb_syscfg/source/Common.js');
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = 'TBD';

var hlcInputs = Common.allInputsAry;
if (Common.isType1_Type2())
{
    hlcInputs = hlcInputs.concat(Common.altInputsAry);
}

/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name        : 'e0',
        description : 'Event 0',
        displayName : 'Event 0 (e0)',
        default     : '0',
        onChange    : onChange,
        options     : hlcInputs
    },
    {
        name        : 'e1',
        description : 'Event 1',
        displayName : 'Event 1 (e1)',
        default     : '0',
        onChange    : onChange,
        options     : hlcInputs
    },
    {
        name        : 'e2',
        description : 'Event 2',
        displayName : 'Event 2 (e2)',
        default     : '0',
        onChange    : onChange,
        options     : hlcInputs
    },
    {
        name        : 'e3',
        description : 'Event 3',
        displayName : 'Event 3 (e3)',
        default     : '0',
        onChange    : onChange,
        options     : hlcInputs
    },
    {
        name       : 'R0_init',
        description: 'Initial value for register R0',
        default    : '0'
    },
    {
        name       : 'R1_init',
        description: 'Initial value for register R1',
        default    : '0'
    },
    {
        name       : 'R2_init',
        description: 'Initial value for register R2',
        default    : '0'
    },
	{
        name       : 'R3_init',
        description: 'Initial value for register R3',
        default    : '0'
    }
];

/**
 * Change notification function
 *
 * @param inst - Watchdog instance to be validated
 * @param ui   - The user interface object
 */
function onChange(inst, ui)
{
    ui.program0.hidden = (inst.e0 == '0');
    ui.program1.hidden = (inst.e1 == '0');
    ui.program2.hidden = (inst.e2 == '0');
    ui.program3.hidden = (inst.e3 == '0');
}

/**
 * Validate this module's configuration
 *
 * @param inst       - Watchdog instance to be validated
 * @param validation - Issue reporting object
 */
function validate(inst, vo)
{
    Common.validateNames(inst, vo);
}

function moduleInstances(inst)
{
    let instrs = [
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/HLC_Program",
            name      : "program0",
            collapsed : false,
            hidden    : true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/HLC_Program",
            name      : "program1",
            collapsed : false,
            hidden    : true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/HLC_Program",
            name      : "program2",
            collapsed : false,
            hidden    : true
        },
        {
            moduleName: "/utilities/clb_tool/clb_syscfg/source/HLC_Program",
            name      : "program3",
            collapsed : false,
            hidden    : true
        },
    ];

    return instrs;
}

// Define the common/portable base Watchdog
exports = {
    displayName         : 'HLC',
    description         : 'High Level Controller and Sequencer',
    defaultInstanceName : 'HLC_',
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
    //maxInstances        : 1,
    moduleInstances     : moduleInstances,
};
