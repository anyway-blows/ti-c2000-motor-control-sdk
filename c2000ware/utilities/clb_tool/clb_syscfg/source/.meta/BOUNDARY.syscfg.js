"use strict";
/*global exports, system*/

let Common   = system.getScript("/utilities/clb_tool/clb_syscfg/source/Common.js");
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = "Provide simulation input functions for the 8 inputs to a CLB Tile";

function inputCfg(idx) {
    let config = 
    [
        {
            name    : 'in'+idx,
            default : 'None',

            onChange: function(inst,ui) {
                ui['in_sync'+idx].hidden   = ((inst['in'+idx] != 'Custom') && (inst['in'+idx] != 'squareWave'));
                ui['in_edge'+idx].hidden   = ((inst['in'+idx] != 'Custom') && (inst['in'+idx] != 'squareWave'));
                ui['in_custom'+idx].hidden = (inst['in'+idx] != 'Custom');
                ui['in_period'+idx].hidden = (inst['in'+idx] != 'squareWave');
                ui['in_duty'+idx  ].hidden = (inst['in'+idx] != 'squareWave');
				ui['in_repeat_count'+idx  ].hidden = (inst['in'+idx] != 'squareWave');
        },

        options : 
            [
                { name: 'None'       },
                { name: 'squareWave' },
//                { name: 'Custom'     },
				{ name: '0'     },
				{ name: '1'     },
				{ name: 'TILE1_BOUNDARY.out0' },
                { name: 'TILE1_BOUNDARY.out1' },
                { name: 'TILE1_BOUNDARY.out2' },
                { name: 'TILE1_BOUNDARY.out3' },
                { name: 'TILE1_BOUNDARY.out4' },
                { name: 'TILE1_BOUNDARY.out5' },
                { name: 'TILE1_BOUNDARY.out6' },
                { name: 'TILE1_BOUNDARY.out7' },
                { name: 'TILE2_BOUNDARY.out0' },
                { name: 'TILE2_BOUNDARY.out1' },
                { name: 'TILE2_BOUNDARY.out2' },
                { name: 'TILE2_BOUNDARY.out3' },
                { name: 'TILE2_BOUNDARY.out4' },
                { name: 'TILE2_BOUNDARY.out5' },
                { name: 'TILE2_BOUNDARY.out6' },
                { name: 'TILE2_BOUNDARY.out7' },
                { name: 'TILE3_BOUNDARY.out0' },
                { name: 'TILE3_BOUNDARY.out1' },
                { name: 'TILE3_BOUNDARY.out2' },
                { name: 'TILE3_BOUNDARY.out3' },
                { name: 'TILE3_BOUNDARY.out4' },
                { name: 'TILE3_BOUNDARY.out5' },
                { name: 'TILE3_BOUNDARY.out6' },
                { name: 'TILE3_BOUNDARY.out7' },
                { name: 'TILE4_BOUNDARY.out0' },
                { name: 'TILE4_BOUNDARY.out1' },
                { name: 'TILE4_BOUNDARY.out2' },
                { name: 'TILE4_BOUNDARY.out3' },
                { name: 'TILE4_BOUNDARY.out4' },
                { name: 'TILE4_BOUNDARY.out5' },
                { name: 'TILE4_BOUNDARY.out6' },
                { name: 'TILE4_BOUNDARY.out7' },
            ]
        },

        {
            collapsed      : false,
            config :
            [
                {            
                    name    : 'in_edge'+idx,
                    default : 'none',
                    description: 'Edge detection on the inputs, generates 1 cycle pulse on chosen edges - for simulation purposes only',
                    hidden  : true,
                    options : [
                        { name: 'none' },
                        { name: 'rising edge'  },
                        { name: 'falling edge' },
                        { name: 'both edges'   }
                    ]
                },

                {
                    name    : 'in_sync'+idx,
                    default : false,
                    description: 'Turns the Clock synchronization on the inputs - for simulation purposes only',
                    hidden  : true,
                },

                {
                    name    : 'in_custom'+idx,
                    default : '',
                    hidden  : true
                },
                {
                    name    : 'in_period'+idx,
                    default : 10,
                    description: 'Period of the square wave',
                    hidden  : true
                },
                {
                    name    : 'in_duty'+idx,
                    default : 5,
                    description: 'Duty or ON time of the square wave',
                    hidden  : true
                },
                {
                    name    : 'in_repeat_count'+idx,
                    default : 100,
                    description: 'Number of periods the waveform is repeated',
                    hidden  : true
                }
            ]
        }
    ];

    return config;
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

    for (let i = 0; i < 8; ++i) {
        if (inst['in'+i] == 'squareWave') {
            if (inst['in_period'+i] < 2) {
                Common.logError(vo, inst, ['in_period'+i],
                            'The Square Wave period must be >= 2');
            }
            if (inst['in_duty'+i] < 1 || inst['in_duty'+i]>= inst['in_period'+i]) {
                Common.logError(vo, inst, ['in_duty'+i],
                            'The Square Wave duty cycle must be >= 1 and < period.');
            }
        }
    }
}

// Define the common/portable base Watchdog
exports = {
    config : inputCfg(0).concat(
             inputCfg(1),
             inputCfg(2),
             inputCfg(3),
             inputCfg(4),
             inputCfg(5),
             inputCfg(6),
             inputCfg(7)),
    displayName         : "Inputs Conditioner",
    validate            : validate,
    description         : "Inputs Conditioner",
    defaultInstanceName : "BOUNDARY",
    longDescription     : longDescription,
};
