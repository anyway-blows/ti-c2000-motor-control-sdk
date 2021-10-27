let UPP_EmulationMode = [
	{ name: "UPP_EMULATIONMODE_HARDSTOP", displayName: "uPP stops immediately" },
	{ name: "UPP_EMULATIONMODE_RUNFREE", displayName: "uPP unaffected by suspend" },
	{ name: "UPP_EMULATIONMODE_SOFTSTOP", displayName: "uPP stops at DMA transaction finish" },
]
let UPP_OperationMode = [
	{ name: "UPP_RECEIVE_MODE", displayName: "uPP to be configured as Receiver" },
	{ name: "UPP_TRANSMIT_MODE", displayName: "uPP to be configured as Transmitter" },
]
let UPP_DataRate = [
	{ name: "UPP_DATA_RATE_SDR", displayName: "uPP to operate in Single Data Rate Mode" },
	{ name: "UPP_DATA_RATE_DDR", displayName: "uPP to operate in Double Data Rate Mode" },
]
let UPP_TxSDRInterleaveMode = [
	{ name: "UPP_TX_SDR_INTERLEAVE_DISABLE", displayName: "nterleaving disabled in Tx SDR" },
	{ name: "UPP_TX_SDR_INTERLEAVE_ENABLE", displayName: "nterleaving enabled in Tx SDR" },
]
let UPP_DDRDemuxMode = [
	{ name: "UPP_DDR_DEMUX_DISABLE", displayName: "Demultiplexing disabled in DDR mode" },
	{ name: "UPP_DDR_DEMUX_ENABLE", displayName: "Demultiplexing enabled in DDR mode" },
]
let UPP_SignalPolarity = [
	{ name: "UPP_SIGNAL_POLARITY_HIGH", displayName: "Signal polarity is active high" },
	{ name: "UPP_SIGNAL_POLARITY_LOW", displayName: "Signal polarity is active low" },
]
let UPP_SignalMode = [
	{ name: "UPP_SIGNAL_DISABLE", displayName: "Control Signal is disabled for uPP" },
	{ name: "UPP_SIGNAL_ENABLE", displayName: "Control Signal is enabled for uPP" },
]
let UPP_ClockPolarity = [
	{ name: "UPP_CLK_NOT_INVERTED", displayName: "uPP Clock is not inverted" },
	{ name: "UPP_CLK_INVERTED", displayName: "uPP clock is inverted" },
]
let UPP_TxIdleDataMode = [
	{ name: "UPP_TX_IDLE_DATA_IDLE", displayName: "ata lines will drive idle val" },
	{ name: "UPP_TX_IDLE_DATA_TRISTATED", displayName: "ata lines will be tristated" },
]
let UPP_DMAChannel = [
	{ name: "UPP_DMA_CHANNEL_I", displayName: "uPP internal DMA channel I" },
	{ name: "UPP_DMA_CHANNEL_Q", displayName: "uPP internal DMA channel Q" },
]
let UPP_ThresholdSize = [
	{ name: "UPP_THR_SIZE_64BYTE", displayName: "Tx threshold size is 64 bytes" },
	{ name: "UPP_THR_SIZE_128BYTE", displayName: "Tx threshold size is 128 bytes" },
	{ name: "UPP_THR_SIZE_256BYTE", displayName: "Tx threshold size is 256 bytes" },
]
let UPP_InputDelay = [
	{ name: "UPP_INPUT_DLY_4", displayName: "4 cycle delay for data & control pins" },
	{ name: "UPP_INPUT_DLY_6", displayName: "6 cycle delay for data & control pins" },
	{ name: "UPP_INPUT_DLY_9", displayName: "9 cycle delay for data & control pins" },
	{ name: "UPP_INPUT_DLY_14", displayName: "14 cycle delay for data & control pins" },
]
exports = {
	UPP_EmulationMode: UPP_EmulationMode,
	UPP_OperationMode: UPP_OperationMode,
	UPP_DataRate: UPP_DataRate,
	UPP_TxSDRInterleaveMode: UPP_TxSDRInterleaveMode,
	UPP_DDRDemuxMode: UPP_DDRDemuxMode,
	UPP_SignalPolarity: UPP_SignalPolarity,
	UPP_SignalMode: UPP_SignalMode,
	UPP_ClockPolarity: UPP_ClockPolarity,
	UPP_TxIdleDataMode: UPP_TxIdleDataMode,
	UPP_DMAChannel: UPP_DMAChannel,
	UPP_ThresholdSize: UPP_ThresholdSize,
	UPP_InputDelay: UPP_InputDelay,
}
