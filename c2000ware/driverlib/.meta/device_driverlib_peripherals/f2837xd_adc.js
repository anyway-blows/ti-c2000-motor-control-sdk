let ADC_ClkPrescale = [
	{ name: "ADC_CLK_DIV_1_0", displayName: "ADCCLK = (input clock) / 1.0" },
	{ name: "ADC_CLK_DIV_2_0", displayName: "ADCCLK = (input clock) / 2.0" },
	{ name: "ADC_CLK_DIV_2_5", displayName: "ADCCLK = (input clock) / 2.5" },
	{ name: "ADC_CLK_DIV_3_0", displayName: "ADCCLK = (input clock) / 3.0" },
	{ name: "ADC_CLK_DIV_3_5", displayName: "ADCCLK = (input clock) / 3.5" },
	{ name: "ADC_CLK_DIV_4_0", displayName: "ADCCLK = (input clock) / 4.0" },
	{ name: "ADC_CLK_DIV_4_5", displayName: "ADCCLK = (input clock) / 4.5" },
	{ name: "ADC_CLK_DIV_5_0", displayName: "ADCCLK = (input clock) / 5.0" },
	{ name: "ADC_CLK_DIV_5_5", displayName: "ADCCLK = (input clock) / 5.5" },
	{ name: "ADC_CLK_DIV_6_0", displayName: "ADCCLK = (input clock) / 6.0" },
	{ name: "ADC_CLK_DIV_6_5", displayName: "ADCCLK = (input clock) / 6.5" },
	{ name: "ADC_CLK_DIV_7_0", displayName: "ADCCLK = (input clock) / 7.0" },
	{ name: "ADC_CLK_DIV_7_5", displayName: "ADCCLK = (input clock) / 7.5" },
	{ name: "ADC_CLK_DIV_8_0", displayName: "ADCCLK = (input clock) / 8.0" },
	{ name: "ADC_CLK_DIV_8_5", displayName: "ADCCLK = (input clock) / 8.5" },
]
let ADC_Resolution = [
	{ name: "ADC_RESOLUTION_12BIT", displayName: "12-bit conversion resolution" },
	{ name: "ADC_RESOLUTION_16BIT", displayName: "16-bit conversion resolution" },
]
let ADC_SignalMode = [
	{ name: "ADC_MODE_SINGLE_ENDED", displayName: "Sample on single pin with VREFLO" },
	{ name: "ADC_MODE_DIFFERENTIAL", displayName: "Sample on pair of pins" },
]
let ADC_Trigger = [
	{ name: "ADC_TRIGGER_SW_ONLY", displayName: "Software only" },
	{ name: "ADC_TRIGGER_CPU1_TINT0", displayName: "CPU1 Timer 0, TINT0" },
	{ name: "ADC_TRIGGER_CPU1_TINT1", displayName: "CPU1 Timer 1, TINT1" },
	{ name: "ADC_TRIGGER_CPU1_TINT2", displayName: "CPU1 Timer 2, TINT2" },
	{ name: "ADC_TRIGGER_GPIO", displayName: "GPIO, ADCEXTSOC" },
	{ name: "ADC_TRIGGER_EPWM1_SOCA", displayName: "ePWM1, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM1_SOCB", displayName: "ePWM1, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM2_SOCA", displayName: "ePWM2, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM2_SOCB", displayName: "ePWM2, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM3_SOCA", displayName: "ePWM3, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM3_SOCB", displayName: "ePWM3, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM4_SOCA", displayName: "ePWM4, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM4_SOCB", displayName: "ePWM4, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM5_SOCA", displayName: "ePWM5, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM5_SOCB", displayName: "ePWM5, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM6_SOCA", displayName: "ePWM6, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM6_SOCB", displayName: "ePWM6, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM7_SOCA", displayName: "ePWM7, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM7_SOCB", displayName: "ePWM7, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM8_SOCA", displayName: "ePWM8, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM8_SOCB", displayName: "ePWM8, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM9_SOCA", displayName: "ePWM9, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM9_SOCB", displayName: "ePWM9, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM10_SOCA", displayName: "ePWM10, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM10_SOCB", displayName: "ePWM10, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM11_SOCA", displayName: "ePWM11, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM11_SOCB", displayName: "ePWM11, ADCSOCB" },
	{ name: "ADC_TRIGGER_EPWM12_SOCA", displayName: "ePWM12, ADCSOCA" },
	{ name: "ADC_TRIGGER_EPWM12_SOCB", displayName: "ePWM12, ADCSOCB" },
	{ name: "ADC_TRIGGER_CPU2_TINT0", displayName: "CPU2 Timer 0, TINT0" },
	{ name: "ADC_TRIGGER_CPU2_TINT1", displayName: "CPU2 Timer 1, TINT1" },
	{ name: "ADC_TRIGGER_CPU2_TINT2", displayName: "CPU2 Timer 2, TINT2" },
]
let ADC_Channel = [
	{ name: "ADC_CH_ADCIN0", displayName: "single-ended, ADCIN0" },
	{ name: "ADC_CH_ADCIN1", displayName: "single-ended, ADCIN1" },
	{ name: "ADC_CH_ADCIN2", displayName: "single-ended, ADCIN2" },
	{ name: "ADC_CH_ADCIN3", displayName: "single-ended, ADCIN3" },
	{ name: "ADC_CH_ADCIN4", displayName: "single-ended, ADCIN4" },
	{ name: "ADC_CH_ADCIN5", displayName: "single-ended, ADCIN5" },
	{ name: "ADC_CH_ADCIN6", displayName: "single-ended, ADCIN6" },
	{ name: "ADC_CH_ADCIN7", displayName: "single-ended, ADCIN7" },
	{ name: "ADC_CH_ADCIN8", displayName: "single-ended, ADCIN8" },
	{ name: "ADC_CH_ADCIN9", displayName: "single-ended, ADCIN9" },
	{ name: "ADC_CH_ADCIN10", displayName: "single-ended, ADCIN10" },
	{ name: "ADC_CH_ADCIN11", displayName: "single-ended, ADCIN11" },
	{ name: "ADC_CH_ADCIN12", displayName: "single-ended, ADCIN12" },
	{ name: "ADC_CH_ADCIN13", displayName: "single-ended, ADCIN13" },
	{ name: "ADC_CH_ADCIN14", displayName: "single-ended, ADCIN14" },
	{ name: "ADC_CH_ADCIN15", displayName: "single-ended, ADCIN15" },
	{ name: "ADC_CH_ADCIN0_ADCIN1", displayName: "differential, ADCIN0 and ADCIN1" },
	{ name: "ADC_CH_ADCIN2_ADCIN3", displayName: "differential, ADCIN2 and ADCIN3" },
	{ name: "ADC_CH_ADCIN4_ADCIN5", displayName: "differential, ADCIN4 and ADCIN5" },
	{ name: "ADC_CH_ADCIN6_ADCIN7", displayName: "differential, ADCIN6 and ADCIN7" },
	{ name: "ADC_CH_ADCIN8_ADCIN9", displayName: "differential, ADCIN8 and ADCIN9" },
	{ name: "ADC_CH_ADCIN10_ADCIN11", displayName: "differential, ADCIN10 and ADCIN11" },
	{ name: "ADC_CH_ADCIN12_ADCIN13", displayName: "differential, ADCIN12 and ADCIN13" },
	{ name: "ADC_CH_ADCIN14_ADCIN15", displayName: "differential, ADCIN14 and ADCIN15" },
]
let ADC_PulseMode = [
	{ name: "ADC_PULSE_END_OF_ACQ_WIN", displayName: "PULSE END OF ACQ WIN" },
	{ name: "ADC_PULSE_END_OF_CONV", displayName: "PULSE END OF CONV" },
]
let ADC_IntNumber = [
	{ name: "ADC_INT_NUMBER1", displayName: "ADCINT1 Interrupt" },
	{ name: "ADC_INT_NUMBER2", displayName: "ADCINT2 Interrupt" },
	{ name: "ADC_INT_NUMBER3", displayName: "ADCINT3 Interrupt" },
	{ name: "ADC_INT_NUMBER4", displayName: "ADCINT4 Interrupt" },
]
let ADC_PPBNumber = [
	{ name: "ADC_PPB_NUMBER1", displayName: "Post-processing block 1" },
	{ name: "ADC_PPB_NUMBER2", displayName: "Post-processing block 2" },
	{ name: "ADC_PPB_NUMBER3", displayName: "Post-processing block 3" },
	{ name: "ADC_PPB_NUMBER4", displayName: "Post-processing block 4" },
]
let ADC_SOCNumber = [
	{ name: "ADC_SOC_NUMBER0", displayName: "SOC/EOC number 0" },
	{ name: "ADC_SOC_NUMBER1", displayName: "SOC/EOC number 1" },
	{ name: "ADC_SOC_NUMBER2", displayName: "SOC/EOC number 2" },
	{ name: "ADC_SOC_NUMBER3", displayName: "SOC/EOC number 3" },
	{ name: "ADC_SOC_NUMBER4", displayName: "SOC/EOC number 4" },
	{ name: "ADC_SOC_NUMBER5", displayName: "SOC/EOC number 5" },
	{ name: "ADC_SOC_NUMBER6", displayName: "SOC/EOC number 6" },
	{ name: "ADC_SOC_NUMBER7", displayName: "SOC/EOC number 7" },
	{ name: "ADC_SOC_NUMBER8", displayName: "SOC/EOC number 8" },
	{ name: "ADC_SOC_NUMBER9", displayName: "SOC/EOC number 9" },
	{ name: "ADC_SOC_NUMBER10", displayName: "SOC/EOC number 10" },
	{ name: "ADC_SOC_NUMBER11", displayName: "SOC/EOC number 11" },
	{ name: "ADC_SOC_NUMBER12", displayName: "SOC/EOC number 12" },
	{ name: "ADC_SOC_NUMBER13", displayName: "SOC/EOC number 13" },
	{ name: "ADC_SOC_NUMBER14", displayName: "SOC/EOC number 14" },
	{ name: "ADC_SOC_NUMBER15", displayName: "SOC/EOC number 15" },
]
let ADC_IntSOCTrigger = [
	{ name: "ADC_INT_SOC_TRIGGER_NONE", displayName: "No ADCINT will trigger the SOC" },
	{ name: "ADC_INT_SOC_TRIGGER_ADCINT1", displayName: "ADCINT1 will trigger the SOC" },
	{ name: "ADC_INT_SOC_TRIGGER_ADCINT2", displayName: "ADCINT2 will trigger the SOC" },
]
let ADC_PriorityMode = [
	{ name: "ADC_PRI_ALL_ROUND_ROBIN", displayName: "Round robin mode is used for all" },
	{ name: "ADC_PRI_SOC0_HIPRI", displayName: "SOC 0 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC1_HIPRI", displayName: "SOC 0-1 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC2_HIPRI", displayName: "SOC 0-2 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC3_HIPRI", displayName: "SOC 0-3 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC4_HIPRI", displayName: "SOC 0-4 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC5_HIPRI", displayName: "SOC 0-5 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC6_HIPRI", displayName: "SOC 0-6 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC7_HIPRI", displayName: "SOC 0-7 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC8_HIPRI", displayName: "SOC 0-8 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC9_HIPRI", displayName: "SOC 0-9 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC10_HIPRI", displayName: "SOC 0-10 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC11_HIPRI", displayName: "SOC 0-11 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC12_HIPRI", displayName: "SOC 0-12 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC13_HIPRI", displayName: "SOC 0-13 hi pri, others in round robin" },
	{ name: "ADC_PRI_THRU_SOC14_HIPRI", displayName: "SOC 0-14 hi pri, SOC15 in round robin" },
	{ name: "ADC_PRI_ALL_HIPRI", displayName: "All priorities based on SOC number" },
]
exports = {
	ADC_ClkPrescale: ADC_ClkPrescale,
	ADC_Resolution: ADC_Resolution,
	ADC_SignalMode: ADC_SignalMode,
	ADC_Trigger: ADC_Trigger,
	ADC_Channel: ADC_Channel,
	ADC_PulseMode: ADC_PulseMode,
	ADC_IntNumber: ADC_IntNumber,
	ADC_PPBNumber: ADC_PPBNumber,
	ADC_SOCNumber: ADC_SOCNumber,
	ADC_IntSOCTrigger: ADC_IntSOCTrigger,
	ADC_PriorityMode: ADC_PriorityMode,
}