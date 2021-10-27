let EMIF_AsyncCSOffset = [
	{ name: "EMIF_ASYNC_CS2_OFFSET", displayName: "sync chip select 2 offset" },
	{ name: "EMIF_ASYNC_CS3_OFFSET", displayName: "sync chip select 3 offset" },
	{ name: "EMIF_ASYNC_CS4_OFFSET", displayName: "sync chip select 4 offset" },
]
let EMIF_AsyncDataWidth = [
	{ name: "EMIF_ASYNC_DATA_WIDTH_8", displayName: "SRAM/FLASH with 8 bit data bus" },
	{ name: "EMIF_ASYNC_DATA_WIDTH_16", displayName: "SRAM/FLASH with 16 bit data bus" },
	{ name: "EMIF_ASYNC_DATA_WIDTH_32", displayName: "SRAM/FLASH with 32 bit data bus" },
]
let EMIF_AsyncMode = [
	{ name: "EMIF_ASYNC_STROBE_MODE", displayName: "nables ASRAM/FLASH strobe mode" },
	{ name: "EMIF_ASYNC_NORMAL_MODE", displayName: "isables ASRAM/FLASH strobe mode" },
]
let EMIF_AsyncWaitPolarity = [
	{ name: "EMIF_ASYNC_WAIT_POLARITY_LOW", displayName: "MxWAIT pin polarity is low" },
	{ name: "EMIF_ASYNC_WAIT_POLARITY_HIGH", displayName: "MxWAIT pin polarity is high" },
]
let EMIF_SyncNarrowMode = [
	{ name: "EMIF_SYNC_NARROW_MODE_TRUE", displayName: "MemBusWidth=SystemBusWidth/2" },
	{ name: "EMIF_SYNC_NARROW_MODE_FALSE", displayName: "MemBusWidth=SystemBusWidth" },
]
let EMIF_SyncBank = [
	{ name: "EMIF_SYNC_BANK_1", displayName: "1 Bank SDRAM device" },
	{ name: "EMIF_SYNC_BANK_2", displayName: "2 Bank SDRAM device" },
	{ name: "EMIF_SYNC_BANK_4", displayName: "4 Bank SDRAM device" },
]
let EMIF_SyncCASLatency = [
	{ name: "EMIF_SYNC_CAS_LAT_2", displayName: "SDRAM with CAS Latency 2" },
	{ name: "EMIF_SYNC_CAS_LAT_3", displayName: "SDRAM with CAS Latency 3" },
]
let EMIF_SyncPageSize = [
	{ name: "EMIF_SYNC_COLUMN_WIDTH_8", displayName: "256-word pages in SDRAM" },
	{ name: "EMIF_SYNC_COLUMN_WIDTH_9", displayName: "512-word pages in SDRAM" },
	{ name: "EMIF_SYNC_COLUMN_WIDTH_10", displayName: "1024-word pages in SDRAM" },
	{ name: "EMIF_SYNC_COLUMN_WIDTH_11", displayName: "2048-word pages in SDRAM" },
]
exports = {
	EMIF_AsyncCSOffset: EMIF_AsyncCSOffset,
	EMIF_AsyncDataWidth: EMIF_AsyncDataWidth,
	EMIF_AsyncMode: EMIF_AsyncMode,
	EMIF_AsyncWaitPolarity: EMIF_AsyncWaitPolarity,
	EMIF_SyncNarrowMode: EMIF_SyncNarrowMode,
	EMIF_SyncBank: EMIF_SyncBank,
	EMIF_SyncCASLatency: EMIF_SyncCASLatency,
	EMIF_SyncPageSize: EMIF_SyncPageSize,
}
