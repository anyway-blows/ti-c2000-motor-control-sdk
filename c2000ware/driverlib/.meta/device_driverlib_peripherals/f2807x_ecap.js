let ECAP_EmulationMode = [
	{ name: "ECAP_EMULATION_STOP", displayName: "EMULATION STOP" },
	{ name: "ECAP_EMULATION_RUN_TO_ZERO", displayName: "EMULATION RUN TO ZERO" },
	{ name: "ECAP_EMULATION_FREE_RUN", displayName: "EMULATION FREE RUN" },
]
let ECAP_CaptureMode = [
	{ name: "ECAP_CONTINUOUS_CAPTURE_MODE", displayName: "CONTINUOUS CAPTURE MODE" },
	{ name: "ECAP_ONE_SHOT_CAPTURE_MODE", displayName: "ONE SHOT CAPTURE MODE" },
]
let ECAP_Events = [
	{ name: "ECAP_EVENT_1", displayName: "eCAP event 1" },
	{ name: "ECAP_EVENT_2", displayName: "eCAP event 2" },
	{ name: "ECAP_EVENT_3", displayName: "eCAP event 3" },
	{ name: "ECAP_EVENT_4", displayName: "eCAP event 4" },
]
let ECAP_SyncOutMode = [
	{ name: "ECAP_SYNC_OUT_SYNCI", displayName: "SYNC OUT SYNCI" },
	{ name: "ECAP_SYNC_OUT_COUNTER_PRD", displayName: "SYNC OUT COUNTER PRD" },
	{ name: "ECAP_SYNC_OUT_DISABLED", displayName: "SYNC OUT DISABLED" },
]
let ECAP_APWMPolarity = [
	{ name: "ECAP_APWM_ACTIVE_HIGH", displayName: "APWM is active high" },
	{ name: "ECAP_APWM_ACTIVE_LOW", displayName: "APWM is active low" },
]
let ECAP_EventPolarity = [
	{ name: "ECAP_EVNT_RISING_EDGE", displayName: "Rising edge polarity" },
	{ name: "ECAP_EVNT_FALLING_EDGE", displayName: "Falling edge polarity" },
]
exports = {
	ECAP_EmulationMode: ECAP_EmulationMode,
	ECAP_CaptureMode: ECAP_CaptureMode,
	ECAP_Events: ECAP_Events,
	ECAP_SyncOutMode: ECAP_SyncOutMode,
	ECAP_APWMPolarity: ECAP_APWMPolarity,
	ECAP_EventPolarity: ECAP_EventPolarity,
}
