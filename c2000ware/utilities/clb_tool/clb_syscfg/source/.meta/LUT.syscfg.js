"use strict";
/*global exports, system*/

let Common   = system.getScript('/utilities/clb_tool/clb_syscfg/source/Common.js');
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = 'TBD';

/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name        : 'eqn',
        description : 'Boolean equation on the variables i3,i2,i1,i0 for the LUT output.',
        default     : ''
    },
    {
        name        : 'i0',
        description : 'Input 0.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : 'i1',
        description : 'Input 1.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : 'i2',
        description : 'Input 2.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : 'i3',
        description : 'Input 3.',
        default     : '0',
        options     : Common.allInputsAry
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
    if (!validate_equation(inst.eqn))
        Common.logError(vo, inst, 'eqn', 'Invalid equation');

    let inputs = Common.inputs['LUT'];
    for (let i = 0; i < inputs.length; i++) {
        let input = inputs[i];
        if (inst.eqn.match(new RegExp(input)) &&
            (inst[input] == '0' || inst[input] == '1')) {
            Common.logWarning(vo, inst, ['eqn', input],
                            'Equation uses a constant value input');
        }
		if(Common.which_instance(inst)== '0') {
				if ((inst[input] == 'LUT_0.OUT') || (inst[input] == 'LUT_1.OUT') || (inst[input] == 'LUT_2.OUT') ||
					(inst[input] == 'FSM_0.OUT') || (inst[input] == 'FSM_1.OUT') || (inst[input] == 'FSM_2.OUT'))				
				{
					Common.logWarning(vo, inst, [input],
                    'Invalid connection for this LUT instance');
				}
		}
		if(Common.which_instance(inst)==1) {
				if ((inst[input] == 'LUT_1.OUT') || (inst[input] == 'LUT_2.OUT') ||
					(inst[input] == 'FSM_1.OUT') || (inst[input] == 'FSM_2.OUT'))				
				{
					Common.logWarning(vo, inst, [input],
                    'Invalid connection for this LUT instance');
				}
		}
		if(Common.which_instance(inst)==2) {
				if ((inst[input] == 'LUT_2.OUT') ||
					(inst[input] == 'FSM_2.OUT'))				
				{
					Common.logWarning(vo, inst, [input],
                    'Invalid connection for this LUT instance');
				}
		}
    }

    Common.validateNames(inst, vo);
}

// Determine if the LUT equation is a valid function of i0-i3
function validate_equation(expr)
{
    let i0,i1,i2,i3;
    try        { eval(expr);   }
    catch(err) { return false; }

    return true;
}

// Define the common/portable base Watchdog
exports = {
    displayName         : 'LUT',
    description         : 'LUT',
    defaultInstanceName : 'LUT_',
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
    //maxInstances        : 3,
};
