"use strict";
/*global exports, system*/

let Common   = system.getScript('/utilities/clb_tool/clb_syscfg/source/Common.js');
let logError = Common.logError;

/* Intro splash on GUI */
let longDescription = 'A set of up to 8 instructions to execute upon on event.';

/* The up to 8 instructions that can be configured for an event program */
let config = [
    { name: 'instruct0', default: '', hidden: false, onChange: onChange },
    { name: 'instruct1', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct2', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct3', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct4', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct5', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct6', default: '', hidden: true,  onChange: onChange },
    { name: 'instruct7', default: '', hidden: true,  onChange: onChange }
];

/**
 * Validate this module's configuration
 *
 * @param inst - HLC Program instance that was changed
 * @param ui   - The UI object for changing ui poroperties
 */
function onChange(inst, ui) {
    let first_blank = 0;

    for (first_blank=0; first_blank < 8; ++first_blank) {
        let field = 'instruct'+first_blank;
        ui[field].hidden = false;
        if (inst[field] == '') break;
    }

    for (let i=first_blank+1; i < 8; ++i)
    {
        let field = 'instruct'+i;
        if (inst[field] == '') { ui[field].hidden = true; }
    }
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

    for (let i=0 ; i < 8; ++i) {
        let field = 'instruct'+i;
        instruction_validate(inst[field], inst, field, vo)
    }
}

function instruction_validate(instruction, inst, field, vo)
{
    if (instruction == '') { return true; }
    let re  = /\s*(\S+)\s*(.*)/i
    let found = instruction.match(re);

    let opcode   = found[1].toUpperCase();
    let operands = found[2].toUpperCase();

    operands = operands.split(',');
    for (let i = 0; i < operands.length; i++) {
        operands[i] = operands[i].trim();
    }

    let registers = ['R0', 'R1', 'R2', 'R3'];
    let counters  = ['C0', 'C1', 'C2'];
    let either    = registers.concat(counters);
    let itype;

    switch (opcode) {
		case 'MOV':
           itype = [ 2, either, either ];
           break;

        case 'ADD':
        case 'SUB':
           itype = [ 2, either, registers ];
           break;

        case 'MOV_T1':
        case 'MOV_T2':
           itype = [ 2, either, counters ];
           break;

        case 'PUSH':
        case 'PULL':
           itype = [ 1, either, 0 ]
           break;

        case 'INTR':
           if (operands.length != 1) {
               logError(vo, inst, field,
                   'Expected 1 operands. ' + operands.length + ' operands found.');
               return;
           }
           let opval = parseInt(operands[0]);
           if (opval < 0 || opval > 63 || isNaN(opval)) {
               logError(vo, inst, field,
                   'The operand for INTR must be a 6 bit value [0..63]');
               return;
           }
           return;

        default:
            logError(vo, inst, field, 'Invalid operation');
            return;
    }

    if (operands.length != itype[0]) {
        logError(vo, inst, field,
            'Expected ' + itype[0] +' operands. ' + operands.length + ' operands found.');
        return;
    }

    if (itype[0] >= 1 && !itype[1].includes(operands[0])) {
        logError(vo, inst, field, 'Src Operand is invalid');
        return;
    }

    if (itype[0] >= 2 && !itype[2].includes(operands[1])) {
        logError(vo, inst, field, 'Dest Operand is invalid');
        return;
    }
}

// Define the common/portable base Watchdog
exports = {
    displayName         : 'HLCProgram',
    description         : 'High Level Controller Event Program',
    defaultInstanceName : 'HLCP_',
    longDescription     : longDescription,
    config              : config,
    validate            : validate
};
