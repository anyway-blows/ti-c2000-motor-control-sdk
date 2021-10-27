let MemCfg_CLAMemoryType = [
	{ name: "MEMCFG_CLA_MEM_DATA", displayName: "Section is CLA data memory" },
	{ name: "MEMCFG_CLA_MEM_PROGRAM", displayName: "Section is CLA program memory" },
]
let MemCfg_LSRAMMasterSel = [
	{ name: "MEMCFG_LSRAMMASTER_CPU_ONLY", displayName: "CPU is the master of the section" },
	{ name: "MEMCFG_LSRAMMASTER_CPU_CLA1", displayName: "CPU and CLA1 share this section" },
]
let MemCfg_GSRAMMasterSel = [
	{ name: "MEMCFG_GSRAMMASTER_CPU1", displayName: "CPU1 is master of the section" },
	{ name: "MEMCFG_GSRAMMASTER_CPU2", displayName: "CPU2 is master of the section" },
]
let MemCfg_TestMode = [
	{ name: "MEMCFG_TEST_FUNCTIONAL", displayName: "TEST FUNCTIONAL" },
	{ name: "MEMCFG_TEST_WRITE_DATA", displayName: "TEST WRITE DATA" },
	{ name: "MEMCFG_TEST_WRITE_ECC", displayName: "TEST WRITE ECC" },
	{ name: "MEMCFG_TEST_WRITE_PARITY", displayName: "TEST WRITE PARITY" },
	{ name: "MEMCFG_TEST_FUNC_DIAG", displayName: "TEST FUNC DIAG" },
]
exports = {
	MemCfg_CLAMemoryType: MemCfg_CLAMemoryType,
	MemCfg_LSRAMMasterSel: MemCfg_LSRAMMasterSel,
	MemCfg_GSRAMMasterSel: MemCfg_GSRAMMasterSel,
	MemCfg_TestMode: MemCfg_TestMode,
}
