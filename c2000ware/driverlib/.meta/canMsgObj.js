let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");

let device_driverlib_peripheral = 
    system.getScript("/driverlib/device_driverlib_peripherals/" + 
        Common.getDeviceName().toLowerCase() + "_can.js");

function displayMsgObjConf(inst, ui)
{
    if (inst.msgObjs > 0) {
        ui.msgID.hidden = false;
        ui.flags.hidden = false;
        ui.msgLen.hidden = false;
        ui.msgIDMask.hidden = false;
        ui.msgType.hidden = false;
        ui.frameType.hidden = false;
    }
    else {
        ui.msgID.hidden = true;
        ui.flags.hidden = true;
        ui.msgLen.hidden = true;
        ui.msgIDMask.hidden = true;
        ui.msgType.hidden = true;
        ui.frameType.hidden = true;
    }
}
let config = [
    {
        name        : "msgID",
        displayName : "Message Identifier",
        description : 'Message ID of object 1',
        hidden      : false,
        default     : 0,
    },
    {
        name        : "frameType",
        displayName : "Type of Frame",
        description : 'Type of Frame',
        hidden      : false,
        default     : device_driverlib_peripheral.CAN_MsgFrameType[0].name,
        options     : device_driverlib_peripheral.CAN_MsgFrameType
    },
    {
        name        : "msgType",
        displayName : "Type of Message",
        description : 'Type of Message',
        hidden      : false,
        default     : device_driverlib_peripheral.CAN_MsgObjType[2].name,
        options     : device_driverlib_peripheral.CAN_MsgObjType
    },
    {
        name        : "flags",
        displayName : "Flags for Message Object",
        description : 'Flags for Message Object',
        hidden      : false,
        default     : [],
        minSelections: 0,
        options     : [
            {name: "CAN_MSG_OBJ_NO_FLAGS", displayName : "No Flags"},
            {name: "CAN_MSG_OBJ_TX_INT_ENABLE", displayName : "Enable Transmit Interrupts"},
            {name: "CAN_MSG_OBJ_RX_INT_ENABLE", displayName : "Enable Receive Interrupts"},
            {name: "CAN_MSG_OBJ_USE_ID_FILTER", displayName : "Use filtering based on the Message ID"},
            {name: "CAN_MSG_OBJ_USE_EXT_FILTER", displayName : "Use filtering based on the Extended Identifier"},
            {name: "CAN_MSG_OBJ_USE_DIR_FILTER", displayName : "Use filtering based on the direction of the transfer"},
            {name: "CAN_MSG_OBJ_FIFO", displayName : "Message object part of a FIFO and not final message"},
        ],
    },
    {
        name        : "msgIDMask",
        displayName : "Message Identifier Mask",
        description : 'CAN message identifier mask when filtering is enabled',
        hidden      : false,
        default     : 0,
    },
    {
        name        : "msgLen",
        displayName : "Message data length (0-8)",
        description : 'Number of data bytes in the Message',
        hidden      : false,
        default     : 0,
    },
];
var canMsgObj = {
    displayName: "canMsgObj",
    maxInstances: 32,
    defaultInstanceName: "msgObj",
    description: "Message Object",
    config: config,
};

exports = canMsgObj;