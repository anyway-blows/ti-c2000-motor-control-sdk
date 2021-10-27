let GPIO_Direction = [
	{ name: "GPIO_DIR_MODE_IN", displayName: "Pin is a GPIO input" },
	{ name: "GPIO_DIR_MODE_OUT", displayName: "Pin is a GPIO output" },
]
let GPIO_IntType = [
	{ name: "GPIO_INT_TYPE_FALLING_EDGE", displayName: "Interrupt on falling edge" },
	{ name: "GPIO_INT_TYPE_RISING_EDGE", displayName: "Interrupt on rising edge" },
	{ name: "GPIO_INT_TYPE_BOTH_EDGES", displayName: "Interrupt on both edges" },
]
let GPIO_QualificationMode = [
	{ name: "GPIO_QUAL_SYNC", displayName: "Synchronization to SYSCLKOUT" },
	{ name: "GPIO_QUAL_3SAMPLE", displayName: "Qualified with 3 samples" },
	{ name: "GPIO_QUAL_6SAMPLE", displayName: "Qualified with 6 samples" },
	{ name: "GPIO_QUAL_ASYNC", displayName: "No synchronization" },
]
let GPIO_AnalogMode = [
	{ name: "GPIO_ANALOG_DISABLED", displayName: "Pin is in digital mode" },
	{ name: "GPIO_ANALOG_ENABLED", displayName: "Pin is in analog mode" },
]
let GPIO_CoreSelect = [
	{ name: "GPIO_CORE_CPU1", displayName: "CPU1 selected as master core" },
	{ name: "GPIO_CORE_CPU1_CLA1", displayName: "CPU1's CLA1 selected as master core" },
	{ name: "GPIO_CORE_CPU2", displayName: "CPU2 selected as master core" },
	{ name: "GPIO_CORE_CPU2_CLA1", displayName: "CPU2's CLA1 selected as master core" },
]
let GPIO_Port = [
	{ name: "GPIO_PORT_A", displayName: "GPIO port A" },
	{ name: "GPIO_PORT_B", displayName: "GPIO port B" },
	{ name: "GPIO_PORT_C", displayName: "GPIO port C" },
	{ name: "GPIO_PORT_D", displayName: "GPIO port D" },
	{ name: "GPIO_PORT_E", displayName: "GPIO port E" },
	{ name: "GPIO_PORT_F", displayName: "GPIO port F" },
]
let GPIO_ExternalIntNum = [
	{ name: "GPIO_INT_XINT1", displayName: "External Interrupt 1" },
	{ name: "GPIO_INT_XINT2", displayName: "External Interrupt 2" },
	{ name: "GPIO_INT_XINT3", displayName: "External Interrupt 3" },
	{ name: "GPIO_INT_XINT4", displayName: "External Interrupt 4" },
	{ name: "GPIO_INT_XINT5", displayName: "External Interrupt 5" },
]
exports = {
	GPIO_Direction: GPIO_Direction,
	GPIO_IntType: GPIO_IntType,
	GPIO_QualificationMode: GPIO_QualificationMode,
	GPIO_AnalogMode: GPIO_AnalogMode,
	GPIO_CoreSelect: GPIO_CoreSelect,
	GPIO_Port: GPIO_Port,
	GPIO_ExternalIntNum: GPIO_ExternalIntNum,
}
