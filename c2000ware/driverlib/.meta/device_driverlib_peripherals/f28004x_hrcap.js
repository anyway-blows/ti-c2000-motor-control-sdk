let HRCAP_CalibrationClockSource = [
	{ name: "HRCAP_CALIBRATION_CLOCK_SYSCLK", displayName: "Use SYSCLK for period match." },
	{ name: "HRCAP_CALIBRATION_CLOCK_HRCLK", displayName: "Use HRCLK for period match." },
]
let HRCAP_ContinuousCalibrationMode = [
	{ name: "HRCAP_CONTINUOUS_CALIBRATION_DISABLED", displayName: "CONTINUOUS CALIBRATION DISABLED" },
	{ name: "HRCAP_CONTINUOUS_CALIBRATION_ENABLED", displayName: "CONTINUOUS CALIBRATION ENABLED" },
]
exports = {
	HRCAP_CalibrationClockSource: HRCAP_CalibrationClockSource,
	HRCAP_ContinuousCalibrationMode: HRCAP_ContinuousCalibrationMode,
}
