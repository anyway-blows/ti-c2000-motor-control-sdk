"use strict";
/*global exports, system*/

let Common   = system.getScript('/utilities/clb_tool/clb_syscfg/source/Common.js');
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = 'TBD';

/* Array of Watchdog configurables that are common across device families */
var config = [
    {
        name       : 'reset',
        description: 'Reset the counter to 0 on next clock cycle (triggers low to high)',
        default    : '0',
        options    : Common.allInputsAry
    },
    {
        name       : 'event',
        description: 'Trigger actions in the counter.',
        default    : '0',
        options    : Common.allInputsAry
    },
    {
        name       : 'mode0',
        description: 'Enables Counting when high',
        default    : '0',
        options    : Common.allInputsAry
    },
    {
        name       : 'mode1',
        description: 'Controls counting direction. High enables count up. Low enables count down.',
        default    : '0',
        options    : Common.allInputsAry
    },
    {
        name       : 'match1_val',
        description: 'Sets the value of the Match reference 1 register.',
        default    : '0'
    },
    {
        name       : 'match2_val',
        description: 'Sets the value of the Match reference 2 register.',
        default    : '0'
    },
    {
        name       : 'event_load_val',
        description: 'Sets the value of the event load value register.',
        default    : '0'
    },
    {
        name       : 'action',
        displayName: 'What action should be taken on an event trigger?',
        default    : 'None',
        options    :
        [
            { name: "None"},
            { name: "Load"},
            { name: "Add"},
            { name: "Subtract"},
            { name: "Shift Left"},
            { name: "Shift Right"}
        ]
    }
];

if (Common.isType1_Type2())
{
    config = config.concat([
        {
            name       : 'serializer_en',
            displayName : 'Enable serializer mode',
            description: 'Enable serializer mode.',
            default    : false
        },
        {
            name: "GROUP_SERIALIZER",
            displayName: "Serializer Mode",
            longDescription: "When the counter is in serializer mode, it no longer operates as a counter. The " +
                         "counter will operate as a shift register",
            config: [
                {
                    name       : 'lfsr_en',
                    displayName : 'Enable LFSR mode',
                    description: 'Enable linear feedback shift register mode.',
                    default    : false
                },
                {
                    name       : 'match1_tap_en',
                    displayName : 'Enable match_1 tap output',
                    description: 'Enables tapping of a specific bit in the counter value and outputing the result ' +
                                 'as the match_1 signal.',
                    default    : false
                },
                {
                    name       : 'match1_tap_sel',
                    displayName : 'Counter bit position for match_1 tap',
                    description: 'Select the bit position in the counter register to be taped and used as match_1 signal',
                    default    : 0,
                    options    : Common.zero_to_31
                },
                {
                    name       : 'match2_tap_en',
                    displayName : 'Enable match_2 tap output',
                    description: 'Enables tapping of a specific bit in the counter value and outputing the result ' +
                                 'as the match_2 signal.',
                    default    : false
                },
                {
                    name       : 'match2_tap_sel',
                    displayName : 'Counter bit position for match_2 tap',
                    description: 'Select the bit position in the counter register to be taped and used as match_2 signal',
                    default    : 0,
                    options    : Common.zero_to_31
                },
                
            ]
        },
        

    ]);
}

/**
 * Validate this module's configuration
 *
 * @param inst       - Watchdog instance to be validated
 * @param validation - Issue reporting object
 */
function validate(inst, vo)
{
    if (inst.action == 'None' && inst.event != '0')
       Common.logError(vo, inst, ['action', 'event'],
                  'The event input must be constant 0 for action to be None');

    if ((inst.match1_val > 0xFFFFFFFF) || (inst.match1_val < -2147483648))
       Common.logError(vo, inst, ['match1_val'],
                  'Match1 Value should be a 32 bit number');
    
	if ((inst.match2_val > 0xFFFFFFFF) || (inst.match2_val < -2147483648))
       Common.logError(vo, inst, ['match2_val'],
                  'Match2 Value should be a 32 bit number');	
    
	if ((inst.event_load_val > 0xFFFFFFFF) || (inst.event_load_val < -2147483648))
       Common.logError(vo, inst, ['event_load_val'],
                  'Event Load Value should be a 32 bit number');					  
    
	Common.validateNames(inst, vo);
}

// Define the common/portable base Watchdog
exports = {
    displayName         : 'COUNTER',
    description         : 'COUNTER',
    defaultInstanceName : 'COUNTER',
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
    //maxInstances        : 3,
    defaultInstanceName : 'COUNTER_',
};
