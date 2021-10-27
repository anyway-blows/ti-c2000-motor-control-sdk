let PMBus_Transaction = [
	{ name: "PMBUS_TRANSACTION_NONE", displayName: "No Transaction" },
	{ name: "PMBUS_TRANSACTION_QUICKCOMMAND", displayName: "Quick Command" },
	{ name: "PMBUS_TRANSACTION_WRITEBYTE", displayName: "Write single byte" },
	{ name: "PMBUS_TRANSACTION_READBYTE", displayName: "Read single byte" },
	{ name: "PMBUS_TRANSACTION_SENDBYTE", displayName: "Send Byte" },
	{ name: "PMBUS_TRANSACTION_RECEIVEBYTE", displayName: "Receive Byte" },
	{ name: "PMBUS_TRANSACTION_BLOCKWRITE", displayName: "Block Write (up to 255 bytes)" },
	{ name: "PMBUS_TRANSACTION_BLOCKREAD", displayName: "Block Read (up to 255 bytes)" },
	{ name: "PMBUS_TRANSACTION_WRITEWORD", displayName: "Write word" },
	{ name: "PMBUS_TRANSACTION_READWORD", displayName: "Read word" },
	{ name: "PMBUS_TRANSACTION_BLOCKWRPC", displayName: "Block write, then process call" },
]
let PMBus_ClockMode = [
	{ name: "PMBUS_CLOCKMODE_STANDARD", displayName: "Standard mode 100 kHz" },
	{ name: "PMBUS_CLOCKMODE_FAST", displayName: "Fast Mode 400 kHz" },
]
let PMBus_accessType = [
	{ name: "PMBUS_ACCESSTYPE_WRITE", displayName: "Slave last address for write transaction" },
	{ name: "PMBUS_ACCESSTYPE_READ", displayName: "Slave last address for read transaction" },
]
let PMBus_intEdge = [
	{ name: "PMBUS_INTEDGE_FALLING", displayName: "Interrupt generated on falling edge" },
	{ name: "PMBUS_INTEDGE_RISING", displayName: "Interrupt generated on rising edge" },
]
exports = {
	PMBus_Transaction: PMBus_Transaction,
	PMBus_ClockMode: PMBus_ClockMode,
	PMBus_accessType: PMBus_accessType,
	PMBus_intEdge: PMBus_intEdge,
}
