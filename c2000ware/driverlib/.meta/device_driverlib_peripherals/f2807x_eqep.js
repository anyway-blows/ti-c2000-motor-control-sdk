let EQEP_PositionResetMode = [
	{ name: "EQEP_POSITION_RESET_IDX", displayName: "POSITION RESET IDX" },
	{ name: "EQEP_POSITION_RESET_MAX_POS", displayName: "POSITION RESET MAX POS" },
	{ name: "EQEP_POSITION_RESET_1ST_IDX", displayName: "POSITION RESET 1ST IDX" },
	{ name: "EQEP_POSITION_RESET_UNIT_TIME_OUT", displayName: "POSITION RESET UNIT TIME OUT" },
]
let EQEP_CAPCLKPrescale = [
	{ name: "EQEP_CAPTURE_CLK_DIV_1", displayName: "CAPCLK = SYSCLKOUT/1" },
	{ name: "EQEP_CAPTURE_CLK_DIV_2", displayName: "CAPCLK = SYSCLKOUT/2" },
	{ name: "EQEP_CAPTURE_CLK_DIV_4", displayName: "CAPCLK = SYSCLKOUT/4" },
	{ name: "EQEP_CAPTURE_CLK_DIV_8", displayName: "CAPCLK = SYSCLKOUT/8" },
	{ name: "EQEP_CAPTURE_CLK_DIV_16", displayName: "CAPCLK = SYSCLKOUT/16" },
	{ name: "EQEP_CAPTURE_CLK_DIV_32", displayName: "CAPCLK = SYSCLKOUT/32" },
	{ name: "EQEP_CAPTURE_CLK_DIV_64", displayName: "CAPCLK = SYSCLKOUT/64" },
	{ name: "EQEP_CAPTURE_CLK_DIV_128", displayName: "CAPCLK = SYSCLKOUT/128" },
]
let EQEP_UPEVNTPrescale = [
	{ name: "EQEP_UNIT_POS_EVNT_DIV_1", displayName: "UPEVNT = QCLK/1" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_2", displayName: "UPEVNT = QCLK/2" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_4", displayName: "UPEVNT = QCLK/4" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_8", displayName: "UPEVNT = QCLK/8" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_16", displayName: "UPEVNT = QCLK/16" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_32", displayName: "UPEVNT = QCLK/32" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_64", displayName: "UPEVNT = QCLK/64" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_128", displayName: "UPEVNT = QCLK/128" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_256", displayName: "UPEVNT = QCLK/256" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_512", displayName: "UPEVNT = QCLK/512" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_1024", displayName: "UPEVNT = QCLK/1024" },
	{ name: "EQEP_UNIT_POS_EVNT_DIV_2048", displayName: "UPEVNT = QCLK/2048" },
]
let EQEP_EmulationMode = [
	{ name: "EQEP_EMULATIONMODE_STOPIMMEDIATELY", displayName: "Counters stop immediately" },
	{ name: "EQEP_EMULATIONMODE_STOPATROLLOVER", displayName: "Counters stop at period rollover" },
	{ name: "EQEP_EMULATIONMODE_RUNFREE", displayName: "Counter unaffected by suspend" },
]
exports = {
	EQEP_PositionResetMode: EQEP_PositionResetMode,
	EQEP_CAPCLKPrescale: EQEP_CAPCLKPrescale,
	EQEP_UPEVNTPrescale: EQEP_UPEVNTPrescale,
	EQEP_EmulationMode: EQEP_EmulationMode,
}
