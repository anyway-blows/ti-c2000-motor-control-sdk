let CAN_MsgFrameType = [
	{ name: "CAN_MSG_FRAME_STD", displayName: "MSG FRAME STD" },
	{ name: "CAN_MSG_FRAME_EXT", displayName: "MSG FRAME EXT" },
]
let CAN_MsgObjType = [
	{ name: "CAN_MSG_OBJ_TYPE_TX", displayName: "MSG OBJ TYPE TX" },
	{ name: "CAN_MSG_OBJ_TYPE_TX_REMOTE", displayName: "MSG OBJ TYPE TX REMOTE" },
	{ name: "CAN_MSG_OBJ_TYPE_RX", displayName: "MSG OBJ TYPE RX" },
	{ name: "CAN_MSG_OBJ_TYPE_RXTX_REMOTE", displayName: "MSG OBJ TYPE RXTX REMOTE" },
]
let CAN_ClockSource = [
	{ name: "CAN_CLOCK_SOURCE_SYS", displayName: "CLOCK SOURCE SYS" },
	{ name: "CAN_CLOCK_SOURCE_XTAL", displayName: "CLOCK SOURCE XTAL" },
	{ name: "CAN_CLOCK_SOURCE_AUX", displayName: "CLOCK SOURCE AUX" },
]
exports = {
	CAN_MsgFrameType: CAN_MsgFrameType,
	CAN_MsgObjType: CAN_MsgObjType,
	CAN_ClockSource: CAN_ClockSource,
}
