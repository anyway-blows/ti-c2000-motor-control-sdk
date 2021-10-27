let LIN_LoopbackType = [
	{ name: "LIN_LOOPBACK_DIGITAL", displayName: "Digital Loopback Mode" },
	{ name: "LIN_LOOPBACK_ANALOG", displayName: "Analog Loopback Mode" },
]
let LIN_AnalogLoopback = [
	{ name: "LIN_ANALOG_LOOP_NONE", displayName: "Default path for digital loopback mode" },
	{ name: "LIN_ANALOG_LOOP_TX", displayName: "Analog loopback through transmit pin" },
	{ name: "LIN_ANALOG_LOOP_RX", displayName: "Analog loopback through receive pin" },
]
let LIN_CommMode = [
	{ name: "LIN_COMM_LIN_USELENGTHVAL", displayName: "COMM LIN USELENGTHVAL" },
	{ name: "LIN_COMM_LIN_ID4ID5LENCTL", displayName: "COMM LIN ID4ID5LENCTL" },
]
let LIN_SCICommMode = [
	{ name: "LIN_COMM_SCI_IDLELINE", displayName: "COMM SCI IDLELINE" },
	{ name: "LIN_COMM_SCI_ADDRBIT", displayName: "COMM SCI ADDRBIT" },
]
let LIN_LINMode = [
	{ name: "LIN_MODE_LIN_SLAVE", displayName: "The node is in slave mode" },
	{ name: "LIN_MODE_LIN_MASTER", displayName: "The node is in master mode" },
]
let LIN_InterruptLine = [
	{ name: "LIN_INTERRUPT_LINE0", displayName: "Interrupt line 0" },
	{ name: "LIN_INTERRUPT_LINE1", displayName: "Interrupt line 1" },
]
let LIN_MessageFilter = [
	{ name: "LIN_MSG_FILTER_IDBYTE", displayName: "LIN Message ID Byte Filtering" },
	{ name: "LIN_MSG_FILTER_IDSLAVE", displayName: "Slave Task ID Byte Filtering" },
]
let LIN_ChecksumType = [
	{ name: "LIN_CHECKSUM_CLASSIC", displayName: "Checksum Classic" },
	{ name: "LIN_CHECKSUM_ENHANCED", displayName: "Checksum Enhanced" },
]
let LIN_DebugMode = [
	{ name: "LIN_DEBUG_FROZEN", displayName: "Freeze module during debug" },
	{ name: "LIN_DEBUG_COMPLETE", displayName: "Complete Tx/Rx before Freezing" },
]
let LIN_PinSampleMask = [
	{ name: "LIN_PINMASK_NONE", displayName: "PINMASK NONE" },
	{ name: "LIN_PINMASK_CENTER", displayName: "PINMASK CENTER" },
	{ name: "LIN_PINMASK_CENTER_SCLK", displayName: "PINMASK CENTER SCLK" },
	{ name: "LIN_PINMASK_CENTER_2SCLK", displayName: "PINMASK CENTER 2SCLK" },
]
let LIN_SCIParityType = [
	{ name: "LIN_SCI_PAR_ODD", displayName: "Odd parity" },
	{ name: "LIN_SCI_PAR_EVEN", displayName: "Even parity" },
]
let LIN_SCIStopBits = [
	{ name: "LIN_SCI_STOP_ONE", displayName: "Use One Stop bit" },
	{ name: "LIN_SCI_STOP_TWO", displayName: "Use Two Stop bits" },
]
exports = {
	LIN_LoopbackType: LIN_LoopbackType,
	LIN_AnalogLoopback: LIN_AnalogLoopback,
	LIN_CommMode: LIN_CommMode,
	LIN_SCICommMode: LIN_SCICommMode,
	LIN_LINMode: LIN_LINMode,
	LIN_InterruptLine: LIN_InterruptLine,
	LIN_MessageFilter: LIN_MessageFilter,
	LIN_ChecksumType: LIN_ChecksumType,
	LIN_DebugMode: LIN_DebugMode,
	LIN_PinSampleMask: LIN_PinSampleMask,
	LIN_SCIParityType: LIN_SCIParityType,
	LIN_SCIStopBits: LIN_SCIStopBits,
}
