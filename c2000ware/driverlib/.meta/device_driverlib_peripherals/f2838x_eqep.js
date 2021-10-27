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
let EQEP_StrobeSource = [
	{ name: "EQEP_STROBE_FROM_GPIO", displayName: "Strobe signal comes from GPIO" },
	{ name: "EQEP_STROBE_OR_ADCSOCA", displayName: "Strobe signal is OR'd with ADCSOCA" },
	{ name: "EQEP_STROBE_OR_ADCSOCB", displayName: "Strobe signal is OR'd with ADCSOCB" },
]
let EQEP_QMAMode = [
	{ name: "EQEP_QMA_MODE_BYPASS", displayName: "QMA module is bypassed" },
	{ name: "EQEP_QMA_MODE_1", displayName: "QMA mode-1 operation is selected" },
	{ name: "EQEP_QMA_MODE_2", displayName: "QMA mode-2 operation is selected" },
]
let EQEP_Source = [
	{ name: "EQEP_SOURCE_DEVICE_PIN", displayName: " signal comes from Device Pin" },
	{ name: "EQEP_SOURCE_CMPSS1", displayName: " signal comes from CMPSS1" },
	{ name: "EQEP_SOURCE_CMPSS2", displayName: " signal comes from CMPSS2" },
	{ name: "EQEP_SOURCE_CMPSS3", displayName: " signal comes from CMPSS3" },
	{ name: "EQEP_SOURCE_CMPSS4", displayName: " signal comes from CMPSS4" },
	{ name: "EQEP_SOURCE_CMPSS5", displayName: " signal comes from CMPSS5" },
	{ name: "EQEP_SOURCE_CMPSS6", displayName: " signal comes from CMPSS6" },
	{ name: "EQEP_SOURCE_CMPSS7", displayName: " signal comes from CMPSS7" },
	{ name: "EQEP_SOURCE_CMPSS8", displayName: " signal comes from CMPSS8" },
	{ name: "EQEP_SOURCE_PWMXBAR1", displayName: " signal comes from PWMXBAR1" },
	{ name: "EQEP_SOURCE_PWMXBAR2", displayName: " signal comes from PWMXBAR2" },
	{ name: "EQEP_SOURCE_PWMXBAR3", displayName: " signal comes from PWMXBAR3" },
	{ name: "EQEP_SOURCE_PWMXBAR4", displayName: " signal comes from PWMXBAR4" },
	{ name: "EQEP_SOURCE_PWMXBAR5", displayName: " signal comes from PWMXBAR5" },
	{ name: "EQEP_SOURCE_PWMXBAR6", displayName: " signal comes from PWMXBAR6" },
	{ name: "EQEP_SOURCE_PWMXBAR7", displayName: " signal comes from PWMXBAR7" },
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
	EQEP_StrobeSource: EQEP_StrobeSource,
	EQEP_QMAMode: EQEP_QMAMode,
	EQEP_Source: EQEP_Source,
	EQEP_EmulationMode: EQEP_EmulationMode,
}
