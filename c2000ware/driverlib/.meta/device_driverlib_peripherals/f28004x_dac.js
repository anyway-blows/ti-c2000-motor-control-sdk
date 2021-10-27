let DAC_ReferenceVoltage = [
	{ name: "DAC_REF_VDAC", displayName: "VDAC reference voltage" },
	{ name: "DAC_REF_ADC_VREFHI", displayName: "ADC VREFHI reference voltage" },
]
let DAC_GainMode = [
	{ name: "DAC_GAIN_ONE", displayName: "Gain set to 1" },
	{ name: "DAC_GAIN_TWO", displayName: "Gain set to 2" },
]
let DAC_LoadMode = [
	{ name: "DAC_LOAD_SYSCLK", displayName: "Load on next SYSCLK" },
	{ name: "DAC_LOAD_PWMSYNC", displayName: "Load on next PWMSYNC specified by SYNCSEL" },
]
exports = {
	DAC_ReferenceVoltage: DAC_ReferenceVoltage,
	DAC_GainMode: DAC_GainMode,
	DAC_LoadMode: DAC_LoadMode,
}
