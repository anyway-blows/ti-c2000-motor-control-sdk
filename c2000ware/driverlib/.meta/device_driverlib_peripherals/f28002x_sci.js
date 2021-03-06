let SCI_ParityType = [
	{ name: "SCI_CONFIG_PAR_NONE", displayName: "No parity" },
	{ name: "SCI_CONFIG_PAR_EVEN", displayName: "Even parity" },
	{ name: "SCI_CONFIG_PAR_ODD", displayName: "Odd parity" },
]
let SCI_TxFIFOLevel = [
	{ name: "SCI_FIFO_TX0", displayName: "Transmit interrupt empty" },
	{ name: "SCI_FIFO_TX1", displayName: "Transmit interrupt 1/16 full" },
	{ name: "SCI_FIFO_TX2", displayName: "Transmit interrupt 2/16 full" },
	{ name: "SCI_FIFO_TX3", displayName: "Transmit interrupt 3/16 full" },
	{ name: "SCI_FIFO_TX4", displayName: "Transmit interrupt 4/16 full" },
	{ name: "SCI_FIFO_TX5", displayName: "Transmit interrupt 5/16 full" },
	{ name: "SCI_FIFO_TX6", displayName: "Transmit interrupt 6/16 full" },
	{ name: "SCI_FIFO_TX7", displayName: "Transmit interrupt 7/16 full" },
	{ name: "SCI_FIFO_TX8", displayName: "Transmit interrupt 8/16 full" },
	{ name: "SCI_FIFO_TX9", displayName: "Transmit interrupt 9/16 full" },
	{ name: "SCI_FIFO_TX10", displayName: "Transmit interrupt 10/16 full" },
	{ name: "SCI_FIFO_TX11", displayName: "Transmit interrupt 11/16 full" },
	{ name: "SCI_FIFO_TX12", displayName: "Transmit interrupt 12/16 full" },
	{ name: "SCI_FIFO_TX13", displayName: "Transmit interrupt 13/16 full" },
	{ name: "SCI_FIFO_TX14", displayName: "Transmit interrupt 14/16 full" },
	{ name: "SCI_FIFO_TX15", displayName: "Transmit interrupt 15/16 full" },
	{ name: "SCI_FIFO_TX16", displayName: "Transmit interrupt full" },
]
let SCI_RxFIFOLevel = [
	{ name: "SCI_FIFO_RX0", displayName: "Receive interrupt empty" },
	{ name: "SCI_FIFO_RX1", displayName: "Receive interrupt 1/16 full" },
	{ name: "SCI_FIFO_RX2", displayName: "Receive interrupt 2/16 full" },
	{ name: "SCI_FIFO_RX3", displayName: "Receive interrupt 3/16 full" },
	{ name: "SCI_FIFO_RX4", displayName: "Receive interrupt 4/16 full" },
	{ name: "SCI_FIFO_RX5", displayName: "Receive interrupt 5/16 full" },
	{ name: "SCI_FIFO_RX6", displayName: "Receive interrupt 6/16 full" },
	{ name: "SCI_FIFO_RX7", displayName: "Receive interrupt 7/16 full" },
	{ name: "SCI_FIFO_RX8", displayName: "Receive interrupt 8/16 full" },
	{ name: "SCI_FIFO_RX9", displayName: "Receive interrupt 9/16 full" },
	{ name: "SCI_FIFO_RX10", displayName: "Receive interrupt 10/16 full" },
	{ name: "SCI_FIFO_RX11", displayName: "Receive interrupt 11/16 full" },
	{ name: "SCI_FIFO_RX12", displayName: "Receive interrupt 12/16 full" },
	{ name: "SCI_FIFO_RX13", displayName: "Receive interrupt 13/16 full" },
	{ name: "SCI_FIFO_RX14", displayName: "Receive interrupt 14/16 full" },
	{ name: "SCI_FIFO_RX15", displayName: "Receive interrupt 15/16 full" },
	{ name: "SCI_FIFO_RX16", displayName: "Receive interrupt full" },
]
exports = {
	SCI_ParityType: SCI_ParityType,
	SCI_TxFIFOLevel: SCI_TxFIFOLevel,
	SCI_RxFIFOLevel: SCI_RxFIFOLevel,
}
