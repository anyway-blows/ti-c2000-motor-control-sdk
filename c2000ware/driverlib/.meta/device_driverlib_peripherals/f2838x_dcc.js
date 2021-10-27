let DCC_SingleShotMode = [
	{ name: "DCC_MODE_COUNTER_ZERO", displayName: "MODE COUNTER ZERO" },
	{ name: "DCC_MODE_COUNTER_ONE", displayName: "MODE COUNTER ONE" },
]
let DCC_Count1ClockSource = [
	{ name: "DCC_COUNT1SRC_PLL", displayName: "PLL021SSP Clock Out Source" },
	{ name: "DCC_COUNT1SRC_AUXPLL", displayName: "AUXPLL Clock Out Source" },
	{ name: "DCC_COUNT1SRC_INTOSC1", displayName: "Internal Oscillator1 Clock Source" },
	{ name: "DCC_COUNT1SRC_INTOSC2", displayName: "Internal Oscillator2 Clock Source" },
	{ name: "DCC_COUNT1SRC_CMCLK", displayName: "CM Clock Source" },
	{ name: "DCC_COUNT1SRC_CPU1SYSCLK", displayName: "CPU1 System Clock Source" },
	{ name: "DCC_COUNT1SRC_ENETRXCLK", displayName: "etherNET RXCLK Clock Source" },
	{ name: "DCC_COUNT1SRC_CPU2SYSCLK", displayName: "CPU2 System Clock Source" },
	{ name: "DCC_COUNT1SRC_CROSSBAR", displayName: "Input Crossbar Clock Source" },
	{ name: "DCC_COUNT1SRC_AUXCLKIN", displayName: "AUX Clock input Source" },
	{ name: "DCC_COUNT1SRC_ETPWM", displayName: "ETPWM Clock Source" },
	{ name: "DCC_COUNT1SRC_LSPCLK", displayName: "LSP Clock Source" },
	{ name: "DCC_COUNT1SRC_ECATMII0RXCLK", displayName: "MII0 RXCLK(etherCAT)Clock Source" },
	{ name: "DCC_COUNT1SRC_WDCLK", displayName: "Watch Dog Clock Source" },
	{ name: "DCC_COUNT1SRC_CANX", displayName: "CANxBIT Clock Source" },
	{ name: "DCC_COUNT1SRC_ECATMII1RXCLK", displayName: "MII1 RXCLK(etherCAT) Clk Src" },
]
let DCC_Count0ClockSource = [
	{ name: "DCC_COUNT0SRC_XTAL", displayName: "Accurate Clock Source" },
	{ name: "DCC_COUNT0SRC_INTOSC1", displayName: "Internal Oscillator1 Clock Source" },
	{ name: "DCC_COUNT0SRC_INTOSC2", displayName: "Internal Oscillator2 Clock Source" },
	{ name: "DCC_COUNT0SRC_CPU1SYSCLK", displayName: "CPU1 System Clock Source" },
	{ name: "DCC_COUNT0SRC_CPU2SYSCLK", displayName: "CPU2 System Clock Source" },
	{ name: "DCC_COUNT0SRC_XBAR", displayName: "Input XBAR Clock Source" },
]
exports = {
	DCC_SingleShotMode: DCC_SingleShotMode,
	DCC_Count1ClockSource: DCC_Count1ClockSource,
	DCC_Count0ClockSource: DCC_Count0ClockSource,
}
