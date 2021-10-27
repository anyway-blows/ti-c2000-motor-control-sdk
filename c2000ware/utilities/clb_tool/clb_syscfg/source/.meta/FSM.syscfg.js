"use strict";
/*global exports, system*/

let Common   = system.getScript("/utilities/clb_tool/clb_syscfg/source/Common.js");
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = "TBD";

/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name        : "eqn_out",
        description : 'Boolean equation on the variables e1,e0,s1,s0 for the LUT output.',
        default     : ''
    },
    {
        name        : "eqn_s0",
        description : 'Boolean equation on the variables e1,e0,s1,s0 for the S0 output.',
        default     : ''
    },
    {
        name        : "eqn_s1",
        description : 'Boolean equation on the variables e1,e0,s1,s0 for the S1 output.',
        default     : ''
    },
    {
        name        : "e0",
        description : 'External Input 0.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : "e1",
        description : 'External Input 1.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : "xe0",
        description : 'Extra External Input 0.',
        default     : '0',
        options     : Common.allInputsAry
    },
    {
        name        : "xe1",
        description : 'Extra External Input 1.',
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
    if (!validate_out_equation(inst.eqn_out))
        Common.logError(vo, inst, 'eqn_out', 'Invalid equation');
    if (!validate_equation(inst.eqn_s0))
        Common.logError(vo, inst, 'eqn_s0', 'Invalid equation');
    if (!validate_equation(inst.eqn_s1))
        Common.logError(vo, inst, 'eqn_s1', 'Invalid equation');

	if (((inst.eqn_out.includes("xe0")) && (inst.eqn_out.includes("s0")))
		|| ((inst.eqn_out.includes("xe1")) && (inst.eqn_out.includes("s1")))){
		Common.logError(vo, inst, ['eqn_out'], 'Equation uses both (xe0 and s0) or (xe1 and s1) at the same time');
	}
	if ((inst.eqn_s0.includes("xe0")) || (inst.eqn_s0.includes("xe1"))) {
		Common.logError(vo, inst, ['eqn_s0'], 'S0 equation can not use xe0 or xe1 inputs');
	}
	if ((inst.eqn_s1.includes("xe0")) || (inst.eqn_s1.includes("xe1")))	 {
		Common.logError(vo, inst, ['eqn_s1'], 'S1 equation can not use xe0 or xe1 inputs');
	}

    let inputs = Common.inputs['FSM'];
    for (let i = 0; i < inputs.length; i++) {
        let input = inputs[i];
		let expr_val = new RegExp("\\b" + input + "\\b");
        if (inst.eqn_out.match(expr_val) && (inst[input] == '0' || inst[input] == '1')) {
            Common.logWarning(vo, inst, ['eqn_out', input], 'Equation uses a constant value input');
        }
        if (inst.eqn_s0.match(expr_val) && (inst[input] == '0' || inst[input] == '1')) {
            Common.logWarning(vo, inst, ['eqn_s0', input], 'Equation uses a constant value input');
        }
        if (inst.eqn_s1.match(expr_val) && (inst[input] == '0' || inst[input] == '1')) {
            Common.logWarning(vo, inst, ['eqn_s1', input], 'Equation uses a constant value input');
        }
		if(Common.which_instance(inst)== '0') {
			if ((inst[input] == 'LUT_1.OUT') || (inst[input] == 'LUT_2.OUT') ||
				(inst[input] == 'FSM_0.OUT') || (inst[input] == 'FSM_1.OUT') || (inst[input] == 'FSM_2.OUT'))
			{
				Common.logWarning(vo, inst, [input],
                'Invalid connection for this FSM instance');
			}
		}
		if(Common.which_instance(inst)==1) {
			if ((inst[input] == 'LUT_2.OUT') ||	(inst[input] == 'FSM_1.OUT') || (inst[input] == 'FSM_2.OUT'))
			{
				Common.logWarning(vo, inst, [input],
				'Invalid connection for this FSM instance');
			}
		}
		if(Common.which_instance(inst)==2) {
			if ((inst[input] == 'FSM_2.OUT'))
			{
				Common.logWarning(vo, inst, [input],
				'Invalid connection for this FSM instance');
			}
		}
    }

    Common.validateNames(inst, vo);
}

// Determine if the State equation is a valid function of e1,e0,s1,s0
function validate_equation(expr)
{
    let e1,e0,s1,s0;
    try        { eval(expr);   }
    catch(err) { return false; }

    return true;
}

function validate_out_equation(expr)
{
    let e1,e0,s1,s0, xe0, xe1;
    try        { eval(expr);   }
    catch(err) { return false; }

    return true;
}

// Define the common/portable base Watchdog
exports = {
    displayName         : "FSM",
    description         : "FSM",
    defaultInstanceName : "FSM_",
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
    //maxInstances        : 3,
};
