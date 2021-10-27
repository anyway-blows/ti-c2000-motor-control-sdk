let SPI_TransferProtocol = [
	{ name: "SPI_PROT_POL0PHA0", displayName: "PROT POL0PHA0" },
	{ name: "SPI_PROT_POL0PHA1", displayName: "PROT POL0PHA1" },
	{ name: "SPI_PROT_POL1PHA0", displayName: "PROT POL1PHA0" },
	{ name: "SPI_PROT_POL1PHA1", displayName: "PROT POL1PHA1" },
]
let SPI_Mode = [
	{ name: "SPI_MODE_SLAVE", displayName: "SPI slave" },
	{ name: "SPI_MODE_MASTER", displayName: "SPI master" },
	{ name: "SPI_MODE_SLAVE_OD", displayName: "SPI slave w/ output (TALK) disabled" },
	{ name: "SPI_MODE_MASTER_OD", displayName: "SPI master w/ output (TALK) disabled" },
]
let SPI_TxFIFOLevel = [
	{ name: "SPI_FIFO_TXEMPTY", displayName: "Transmit FIFO empty" },
	{ name: "SPI_FIFO_TX0", displayName: "Transmit FIFO empty" },
	{ name: "SPI_FIFO_TX1", displayName: "Transmit FIFO 1/16 full" },
	{ name: "SPI_FIFO_TX2", displayName: "Transmit FIFO 2/16 full" },
	{ name: "SPI_FIFO_TX3", displayName: "Transmit FIFO 3/16 full" },
	{ name: "SPI_FIFO_TX4", displayName: "Transmit FIFO 4/16 full" },
	{ name: "SPI_FIFO_TX5", displayName: "Transmit FIFO 5/16 full" },
	{ name: "SPI_FIFO_TX6", displayName: "Transmit FIFO 6/16 full" },
	{ name: "SPI_FIFO_TX7", displayName: "Transmit FIFO 7/16 full" },
	{ name: "SPI_FIFO_TX8", displayName: "Transmit FIFO 8/16 full" },
	{ name: "SPI_FIFO_TX9", displayName: "Transmit FIFO 9/16 full" },
	{ name: "SPI_FIFO_TX10", displayName: "Transmit FIFO 10/16 full" },
	{ name: "SPI_FIFO_TX11", displayName: "Transmit FIFO 11/16 full" },
	{ name: "SPI_FIFO_TX12", displayName: "Transmit FIFO 12/16 full" },
	{ name: "SPI_FIFO_TX13", displayName: "Transmit FIFO 13/16 full" },
	{ name: "SPI_FIFO_TX14", displayName: "Transmit FIFO 14/16 full" },
	{ name: "SPI_FIFO_TX15", displayName: "Transmit FIFO 15/16 full" },
	{ name: "SPI_FIFO_TX16", displayName: "Transmit FIFO full" },
	{ name: "SPI_FIFO_TXFULL", displayName: "Transmit FIFO full" },
]
let SPI_RxFIFOLevel = [
	{ name: "SPI_FIFO_RXEMPTY", displayName: "Receive FIFO empty" },
	{ name: "SPI_FIFO_RX0", displayName: "Receive FIFO empty" },
	{ name: "SPI_FIFO_RX1", displayName: "Receive FIFO 1/16 full" },
	{ name: "SPI_FIFO_RX2", displayName: "Receive FIFO 2/16 full" },
	{ name: "SPI_FIFO_RX3", displayName: "Receive FIFO 3/16 full" },
	{ name: "SPI_FIFO_RX4", displayName: "Receive FIFO 4/16 full" },
	{ name: "SPI_FIFO_RX5", displayName: "Receive FIFO 5/16 full" },
	{ name: "SPI_FIFO_RX6", displayName: "Receive FIFO 6/16 full" },
	{ name: "SPI_FIFO_RX7", displayName: "Receive FIFO 7/16 full" },
	{ name: "SPI_FIFO_RX8", displayName: "Receive FIFO 8/16 full" },
	{ name: "SPI_FIFO_RX9", displayName: "Receive FIFO 9/16 full" },
	{ name: "SPI_FIFO_RX10", displayName: "Receive FIFO 10/16 full" },
	{ name: "SPI_FIFO_RX11", displayName: "Receive FIFO 11/16 full" },
	{ name: "SPI_FIFO_RX12", displayName: "Receive FIFO 12/16 full" },
	{ name: "SPI_FIFO_RX13", displayName: "Receive FIFO 13/16 full" },
	{ name: "SPI_FIFO_RX14", displayName: "Receive FIFO 14/16 full" },
	{ name: "SPI_FIFO_RX15", displayName: "Receive FIFO 15/16 full" },
	{ name: "SPI_FIFO_RX16", displayName: "Receive FIFO full" },
	{ name: "SPI_FIFO_RXFULL", displayName: "Receive FIFO full" },
	{ name: "SPI_FIFO_RXDEFAULT", displayName: "To prevent interrupt at reset" },
]
let SPI_EmulationMode = [
	{ name: "SPI_EMULATION_STOP_MIDWAY", displayName: "EMULATION STOP MIDWAY" },
	{ name: "SPI_EMULATION_FREE_RUN", displayName: "EMULATION FREE RUN" },
	{ name: "SPI_EMULATION_STOP_AFTER_TRANSMIT", displayName: "EMULATION STOP AFTER TRANSMIT" },
]
let SPI_STEPolarity = [
	{ name: "SPI_STE_ACTIVE_LOW", displayName: "SPISTE is active low (normal)" },
	{ name: "SPI_STE_ACTIVE_HIGH", displayName: "SPISTE is active high (inverted)" },
]
exports = {
	SPI_TransferProtocol: SPI_TransferProtocol,
	SPI_Mode: SPI_Mode,
	SPI_TxFIFOLevel: SPI_TxFIFOLevel,
	SPI_RxFIFOLevel: SPI_RxFIFOLevel,
	SPI_EmulationMode: SPI_EmulationMode,
	SPI_STEPolarity: SPI_STEPolarity,
}
