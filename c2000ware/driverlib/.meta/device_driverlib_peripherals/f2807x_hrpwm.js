let HRPWM_Channel = [
	{ name: "HRPWM_CHANNEL_A", displayName: "HRPWM A" },
	{ name: "HRPWM_CHANNEL_B", displayName: "HRPWM B" },
]
let HRPWM_MEPEdgeMode = [
	{ name: "HRPWM_MEP_CTRL_DISABLE", displayName: "MEP CTRL DISABLE" },
	{ name: "HRPWM_MEP_CTRL_RISING_EDGE", displayName: "MEP CTRL RISING EDGE" },
	{ name: "HRPWM_MEP_CTRL_FALLING_EDGE", displayName: "MEP CTRL FALLING EDGE" },
	{ name: "HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE", displayName: "MEP CTRL RISING AND FALLING EDGE" },
]
let HRPWM_MEPCtrlMode = [
	{ name: "HRPWM_MEP_DUTY_PERIOD_CTRL", displayName: "MEP DUTY PERIOD CTRL" },
	{ name: "HRPWM_MEP_PHASE_CTRL", displayName: "MEP PHASE CTRL" },
]
let HRPWM_LoadMode = [
	{ name: "HRPWM_LOAD_ON_CNTR_ZERO", displayName: "LOAD ON CNTR ZERO" },
	{ name: "HRPWM_LOAD_ON_CNTR_PERIOD", displayName: "LOAD ON CNTR PERIOD" },
	{ name: "HRPWM_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "LOAD ON CNTR ZERO PERIOD" },
]
let HRPWM_ChannelBOutput = [
	{ name: "HRPWM_OUTPUT_ON_B_NORMAL", displayName: "ePWMxB output is normal." },
	{ name: "HRPWM_OUTPUT_ON_B_INV_A", displayName: "ePWMxB output is inverted" },
]
let HRPWM_SyncPulseSource = [
	{ name: "HRPWM_PWMSYNC_SOURCE_PERIOD", displayName: "PWMSYNC SOURCE PERIOD" },
	{ name: "HRPWM_PWMSYNC_SOURCE_ZERO", displayName: "PWMSYNC SOURCE ZERO" },
	{ name: "HRPWM_PWMSYNC_SOURCE_COMPC_UP", displayName: "PWMSYNC SOURCE COMPC UP" },
	{ name: "HRPWM_PWMSYNC_SOURCE_COMPC_DOWN", displayName: "PWMSYNC SOURCE COMPC DOWN" },
	{ name: "HRPWM_PWMSYNC_SOURCE_COMPD_UP", displayName: "PWMSYNC SOURCE COMPD UP" },
	{ name: "HRPWM_PWMSYNC_SOURCE_COMPD_DOWN", displayName: "PWMSYNC SOURCE COMPD DOWN" },
]
let HRPWM_CounterCompareModule = [
	{ name: "HRPWM_COUNTER_COMPARE_A", displayName: "counter compare A" },
	{ name: "HRPWM_COUNTER_COMPARE_B", displayName: "counter compare B" },
]
let HRPWM_MEPDeadBandEdgeMode = [
	{ name: "HRPWM_DB_MEP_CTRL_DISABLE", displayName: "DB MEP CTRL DISABLE" },
	{ name: "HRPWM_DB_MEP_CTRL_RED", displayName: "DB MEP CTRL RED" },
	{ name: "HRPWM_DB_MEP_CTRL_FED", displayName: "DB MEP CTRL FED" },
	{ name: "HRPWM_DB_MEP_CTRL_RED_FED", displayName: "DB MEP CTRL RED FED" },
]
let HRPWM_LockRegisterGroup = [
	{ name: "HRPWM_REGISTER_GROUP_HRPWM", displayName: "HRPWM register group" },
	{ name: "HRPWM_REGISTER_GROUP_GLOBAL_LOAD", displayName: "Global load register group" },
	{ name: "HRPWM_REGISTER_GROUP_TRIP_ZONE", displayName: "Trip zone register group" },
	{ name: "HRPWM_REGISTER_GROUP_TRIP_ZONE_CLEAR", displayName: "Trip zone clear group" },
	{ name: "HRPWM_REGISTER_GROUP_DIGITAL_COMPARE", displayName: "Digital compare group" },
]
exports = {
	HRPWM_Channel: HRPWM_Channel,
	HRPWM_MEPEdgeMode: HRPWM_MEPEdgeMode,
	HRPWM_MEPCtrlMode: HRPWM_MEPCtrlMode,
	HRPWM_LoadMode: HRPWM_LoadMode,
	HRPWM_ChannelBOutput: HRPWM_ChannelBOutput,
	HRPWM_SyncPulseSource: HRPWM_SyncPulseSource,
	HRPWM_CounterCompareModule: HRPWM_CounterCompareModule,
	HRPWM_MEPDeadBandEdgeMode: HRPWM_MEPDeadBandEdgeMode,
	HRPWM_LockRegisterGroup: HRPWM_LockRegisterGroup,
}
