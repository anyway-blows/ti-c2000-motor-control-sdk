let FSI_DataWidth = [
	{ name: "FSI_DATA_WIDTH_1_LANE", displayName: "DATA WIDTH 1 LANE" },
	{ name: "FSI_DATA_WIDTH_2_LANE", displayName: "DATA WIDTH 2 LANE" },
]
let FSI_TxSubmoduleInReset = [
	{ name: "FSI_TX_MASTER_CORE_RESET", displayName: "TX MASTER CORE RESET" },
	{ name: "FSI_TX_CLOCK_RESET", displayName: "TX CLOCK RESET" },
	{ name: "FSI_TX_PING_TIMEOUT_CNT_RESET", displayName: "TX PING TIMEOUT CNT RESET" },
]
let FSI_TxStartMode = [
	{ name: "FSI_TX_START_FRAME_CTRL", displayName: "TX START FRAME CTRL" },
	{ name: "FSI_TX_START_EXT_TRIG", displayName: "TX START EXT TRIG" },
	{ name: "FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG", displayName: "TX START FRAME CTRL OR UDATA TAG" },
]
let FSI_FrameType = [
	{ name: "FSI_FRAME_TYPE_PING", displayName: "FRAME TYPE PING" },
	{ name: "FSI_FRAME_TYPE_ERROR", displayName: "FRAME TYPE ERROR" },
	{ name: "FSI_FRAME_TYPE_1WORD_DATA", displayName: "FRAME TYPE 1WORD DATA" },
	{ name: "FSI_FRAME_TYPE_2WORD_DATA", displayName: "FRAME TYPE 2WORD DATA" },
	{ name: "FSI_FRAME_TYPE_4WORD_DATA", displayName: "FRAME TYPE 4WORD DATA" },
	{ name: "FSI_FRAME_TYPE_6WORD_DATA", displayName: "FRAME TYPE 6WORD DATA" },
	{ name: "FSI_FRAME_TYPE_NWORD_DATA", displayName: "FRAME TYPE NWORD DATA" },
]
let FSI_FrameTag = [
	{ name: "FSI_FRAME_TAG0", displayName: "FRAME TAG0" },
	{ name: "FSI_FRAME_TAG1", displayName: "FRAME TAG1" },
	{ name: "FSI_FRAME_TAG2", displayName: "FRAME TAG2" },
	{ name: "FSI_FRAME_TAG3", displayName: "FRAME TAG3" },
	{ name: "FSI_FRAME_TAG4", displayName: "FRAME TAG4" },
	{ name: "FSI_FRAME_TAG5", displayName: "FRAME TAG5" },
	{ name: "FSI_FRAME_TAG6", displayName: "FRAME TAG6" },
	{ name: "FSI_FRAME_TAG7", displayName: "FRAME TAG7" },
	{ name: "FSI_FRAME_TAG8", displayName: "FRAME TAG8" },
	{ name: "FSI_FRAME_TAG9", displayName: "FRAME TAG9" },
	{ name: "FSI_FRAME_TAG10", displayName: "FRAME TAG10" },
	{ name: "FSI_FRAME_TAG11", displayName: "FRAME TAG11" },
	{ name: "FSI_FRAME_TAG12", displayName: "FRAME TAG12" },
	{ name: "FSI_FRAME_TAG13", displayName: "FRAME TAG13" },
	{ name: "FSI_FRAME_TAG14", displayName: "FRAME TAG14" },
	{ name: "FSI_FRAME_TAG15", displayName: "FRAME TAG15" },
]
let FSI_PingTimeoutMode = [
	{ name: "FSI_PINGTIMEOUT_ON_HWINIT_PING_FRAME", displayName: "PINGTIMEOUT ON HWINIT PING FRAME" },
	{ name: "FSI_PINGTIMEOUT_ON_HWSWINIT_PING_FRAME", displayName: "PINGTIMEOUT ON HWSWINIT PING FRAME" },
]
let FSI_ECCComputeWidth = [
	{ name: "FSI_32BIT_ECC_COMPUTE", displayName: "32BIT ECC COMPUTE" },
	{ name: "FSI_16BIT_ECC_COMPUTE", displayName: "16BIT ECC COMPUTE" },
]
let FSI_InterruptNum = [
	{ name: "FSI_INT1", displayName: "INT1" },
	{ name: "FSI_INT2", displayName: "INT2" },
]
let FSI_RxSubmoduleInReset = [
	{ name: "FSI_RX_MASTER_CORE_RESET", displayName: "RX MASTER CORE RESET" },
	{ name: "FSI_RX_FRAME_WD_CNT_RESET", displayName: "RX FRAME WD CNT RESET" },
	{ name: "FSI_RX_PING_WD_CNT_RESET", displayName: "RX PING WD CNT RESET" },
]
let FSI_RxDelayTapType = [
	{ name: "FSI_RX_DELAY_CLK", displayName: "RX DELAY CLK" },
	{ name: "FSI_RX_DELAY_D0", displayName: "RX DELAY D0" },
	{ name: "FSI_RX_DELAY_D1", displayName: "RX DELAY D1" },
]
let FSI_ExtFrameTriggerSrc = [
	{ name: "FSI_EXT_TRIGSRC_EPWM1_SOCA", displayName: "EXT TRIGSRC EPWM1 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM1_SOCB", displayName: "EXT TRIGSRC EPWM1 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM2_SOCA", displayName: "EXT TRIGSRC EPWM2 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM2_SOCB", displayName: "EXT TRIGSRC EPWM2 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM3_SOCA", displayName: "EXT TRIGSRC EPWM3 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM3_SOCB", displayName: "EXT TRIGSRC EPWM3 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM4_SOCA", displayName: "EXT TRIGSRC EPWM4 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM4_SOCB", displayName: "EXT TRIGSRC EPWM4 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM5_SOCA", displayName: "EXT TRIGSRC EPWM5 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM5_SOCB", displayName: "EXT TRIGSRC EPWM5 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM6_SOCA", displayName: "EXT TRIGSRC EPWM6 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM6_SOCB", displayName: "EXT TRIGSRC EPWM6 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_EPWM7_SOCA", displayName: "EXT TRIGSRC EPWM7 SOCA" },
	{ name: "FSI_EXT_TRIGSRC_EPWM7_SOCB", displayName: "EXT TRIGSRC EPWM7 SOCB" },
	{ name: "FSI_EXT_TRIGSRC_CLB1_CLBOUT30", displayName: "EXT TRIGSRC CLB1 CLBOUT30" },
	{ name: "FSI_EXT_TRIGSRC_CLB1_CLBOUT31", displayName: "EXT TRIGSRC CLB1 CLBOUT31" },
	{ name: "FSI_EXT_TRIGSRC_CLB2_CLBOUT30", displayName: "EXT TRIGSRC CLB2 CLBOUT30" },
	{ name: "FSI_EXT_TRIGSRC_CLB2_CLBOUT31", displayName: "EXT TRIGSRC CLB2 CLBOUT31" },
	{ name: "FSI_EXT_TRIGSRC_ADC_SOCA", displayName: "EXT TRIGSRC ADC SOCA" },
	{ name: "FSI_EXT_TRIGSRC_ADC_SOCB", displayName: "EXT TRIGSRC ADC SOCB" },
	{ name: "FSI_EXT_TRIGSRC_CPU1_TIMER0INT", displayName: "EXT TRIGSRC CPU1 TIMER0INT" },
	{ name: "FSI_EXT_TRIGSRC_CPU1_TIMER1INT", displayName: "EXT TRIGSRC CPU1 TIMER1INT" },
	{ name: "FSI_EXT_TRIGSRC_CPU1_TIMER2INT", displayName: "EXT TRIGSRC CPU1 TIMER2INT" },
]
exports = {
	FSI_DataWidth: FSI_DataWidth,
	FSI_TxSubmoduleInReset: FSI_TxSubmoduleInReset,
	FSI_TxStartMode: FSI_TxStartMode,
	FSI_FrameType: FSI_FrameType,
	FSI_FrameTag: FSI_FrameTag,
	FSI_PingTimeoutMode: FSI_PingTimeoutMode,
	FSI_ECCComputeWidth: FSI_ECCComputeWidth,
	FSI_InterruptNum: FSI_InterruptNum,
	FSI_RxSubmoduleInReset: FSI_RxSubmoduleInReset,
	FSI_RxDelayTapType: FSI_RxDelayTapType,
	FSI_ExtFrameTriggerSrc: FSI_ExtFrameTriggerSrc,
}
