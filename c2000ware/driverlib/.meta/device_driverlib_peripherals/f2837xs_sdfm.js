let SDFM_OutputThresholdStatus = [
	{ name: "SDFM_OUTPUT_WITHIN_THRESHOLD", displayName: "SDFM output is within threshold" },
	{ name: "SDFM_OUTPUT_ABOVE_THRESHOLD", displayName: "SDFM output is above threshold" },
	{ name: "SDFM_OUTPUT_BELOW_THRESHOLD", displayName: "SDFM output is below threshold" },
]
let SDFM_FilterNumber = [
	{ name: "SDFM_FILTER_1", displayName: "Digital filter 1" },
	{ name: "SDFM_FILTER_2", displayName: "Digital filter 2" },
	{ name: "SDFM_FILTER_3", displayName: "Digital filter 3" },
	{ name: "SDFM_FILTER_4", displayName: "Digital filter 4" },
]
let SDFM_FilterType = [
	{ name: "SDFM_FILTER_SINC_FAST", displayName: "FILTER SINC FAST" },
	{ name: "SDFM_FILTER_SINC_1", displayName: "FILTER SINC 1" },
	{ name: "SDFM_FILTER_SINC_2", displayName: "FILTER SINC 2" },
	{ name: "SDFM_FILTER_SINC_3", displayName: "FILTER SINC 3" },
]
let SDFM_ModulatorClockMode = [
	{ name: "SDFM_MODULATOR_CLK_EQUAL_DATA_RATE", displayName: "MODULATOR CLK EQUAL DATA RATE" },
	{ name: "SDFM_MODULATOR_CLK_HALF_DATA_RATE", displayName: "MODULATOR CLK HALF DATA RATE" },
	{ name: "SDFM_MODULATOR_CLK_OFF", displayName: "MODULATOR CLK OFF" },
	{ name: "SDFM_MODULATOR_CLK_DOUBLE_DATA_RATE", displayName: "MODULATOR CLK DOUBLE DATA RATE" },
]
let SDFM_OutputDataFormat = [
	{ name: "SDFM_DATA_FORMAT_16_BIT", displayName: "DATA FORMAT 16 BIT" },
	{ name: "SDFM_DATA_FORMAT_32_BIT", displayName: "DATA FORMAT 32 BIT" },
]
exports = {
	SDFM_OutputThresholdStatus: SDFM_OutputThresholdStatus,
	SDFM_FilterNumber: SDFM_FilterNumber,
	SDFM_FilterType: SDFM_FilterType,
	SDFM_ModulatorClockMode: SDFM_ModulatorClockMode,
	SDFM_OutputDataFormat: SDFM_OutputDataFormat,
}
