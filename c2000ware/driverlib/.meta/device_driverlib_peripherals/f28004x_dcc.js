let DCC_SingleShotMode = [
	{ name: "DCC_MODE_COUNTER_ZERO", displayName: "MODE COUNTER ZERO" },
	{ name: "DCC_MODE_COUNTER_ONE", displayName: "MODE COUNTER ONE" },
]
let DCC_RevisionNumber = [
	{ name: "DCC_REVISION_MINOR", displayName: "The module minor revision number" },
	{ name: "DCC_REVISION_MAJOR", displayName: "The module major revision number" },
]
let DCC_Count1ClockSource = [
	{ name: "DCC_COUNT1SRC_PLL", displayName: "PLL021SSP Clock Out Source" },
]
let DCC_Count0ClockSource = [
	{ name: "DCC_COUNT0SRC_XTAL", displayName: "Accurate Clock Source" },
	{ name: "DCC_COUNT0SRC_INTOSC1", displayName: "Internal Oscillator1 Clock Source" },
	{ name: "DCC_COUNT0SRC_INTOSC2", displayName: "Internal Oscillator2 Clock Source" },
]
exports = {
	DCC_SingleShotMode: DCC_SingleShotMode,
	DCC_RevisionNumber: DCC_RevisionNumber,
	DCC_Count1ClockSource: DCC_Count1ClockSource,
	DCC_Count0ClockSource: DCC_Count0ClockSource,
}
