let CPUTimer_EmulationMode = [
	{ name: "CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT", displayName: "EMULATIONMODE STOPAFTERNEXTDECREMENT" },
	{ name: "CPUTIMER_EMULATIONMODE_STOPATZERO", displayName: "EMULATIONMODE STOPATZERO" },
	{ name: "CPUTIMER_EMULATIONMODE_RUNFREE", displayName: "EMULATIONMODE RUNFREE" },
]
let CPUTimer_ClockSource = [
	{ name: "CPUTIMER_CLOCK_SOURCE_SYS", displayName: "CLOCK SOURCE SYS" },
	{ name: "CPUTIMER_CLOCK_SOURCE_INTOSC1", displayName: "CLOCK SOURCE INTOSC1" },
	{ name: "CPUTIMER_CLOCK_SOURCE_INTOSC2", displayName: "CLOCK SOURCE INTOSC2" },
	{ name: "CPUTIMER_CLOCK_SOURCE_XTAL", displayName: "CLOCK SOURCE XTAL" },
	{ name: "CPUTIMER_CLOCK_SOURCE_AUX", displayName: "CLOCK SOURCE AUX" },
]
let CPUTimer_Prescaler = [
	{ name: "CPUTIMER_CLOCK_PRESCALER_1", displayName: "Prescaler value of / 1" },
	{ name: "CPUTIMER_CLOCK_PRESCALER_2", displayName: "Prescaler value of / 2" },
	{ name: "CPUTIMER_CLOCK_PRESCALER_4", displayName: "Prescaler value of / 4" },
	{ name: "CPUTIMER_CLOCK_PRESCALER_8", displayName: "Prescaler value of / 8" },
	{ name: "CPUTIMER_CLOCK_PRESCALER_16", displayName: "Prescaler value of / 16" },
]
exports = {
	CPUTimer_EmulationMode: CPUTimer_EmulationMode,
	CPUTimer_ClockSource: CPUTimer_ClockSource,
	CPUTimer_Prescaler: CPUTimer_Prescaler,
}
