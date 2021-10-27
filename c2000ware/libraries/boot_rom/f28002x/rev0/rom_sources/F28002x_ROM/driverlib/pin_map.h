//###########################################################################
//
// FILE:   pin_map.h
//
// TITLE:  Definitions of pin mux info for gpio.c.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:  $
//###########################################################################

#ifndef PIN_MAP_H
#define PIN_MAP_H

//*****************************************************************************
// 0x00000003 = MUX register value
// 0x0000000C = GMUX register value
// 0x0000FF00 = Shift amount within mux registers
// 0xFFFF0000 = Offset of MUX register
//*****************************************************************************
#define GPIO_0_GPIO00              0x00060000U
#define GPIO_0_EPWM1A              0x00060001U
#define GPIO_0_SDAA                0x00060006U
#define GPIO_0_SPISTEA             0x00060007U
#define GPIO_0_FSI_RXCLK           0x00060009U
#define GPIO_0_OUTPUTXBAR2_8       0x0006000BU
#define GPIO_0_HIBASESEL1          0x0006000FU

#define GPIO_1_GPIO01              0x00060200U
#define GPIO_1_EPWM1B              0x00060201U
#define GPIO_1_EMU0                0x00060202U
#define GPIO_1_SCLA                0x00060206U
#define GPIO_1_SPISOMIA            0x00060207U
#define GPIO_1_OUTPUTXBAR2_7       0x0006020BU
#define GPIO_1_HIADDR2             0x0006020DU
#define GPIO_1_FSI_TDM_TX1         0x0006020EU
#define GPIO_1_HIDATA10            0x0006020FU

#define GPIO_2_GPIO02              0x00060400U
#define GPIO_2_EPWM2A              0x00060401U
#define GPIO_2_EMU1                0x00060402U
#define GPIO_2_OUTPUTXBAR1         0x00060405U
#define GPIO_2_PMB1SDA             0x00060406U
#define GPIO_2_SPISIMOA            0x00060407U
#define GPIO_2_SCITXDA             0x00060409U
#define GPIO_2_FSI_RX1             0x0006040AU
#define GPIO_2_SDAB                0x0006040BU
#define GPIO_2_HIADDR1             0x0006040DU
#define GPIO_2_CANTXA              0x0006040EU
#define GPIO_2_HIDATA9             0x0006040FU

#define GPIO_3_GPIO03              0x00060600U
#define GPIO_3_EPWM2B              0x00060601U
#define GPIO_3_OUTPUTXBAR2         0x00060602U
#define GPIO_3_PMB1SCL             0x00060606U
#define GPIO_3_SPICLKA             0x00060607U
#define GPIO_3_SCIRXDA             0x00060609U
#define GPIO_3_FSI_RX0             0x0006060AU
#define GPIO_3_SCLB                0x0006060BU
#define GPIO_3_HINOE               0x0006060DU
#define GPIO_3_CANRXA              0x0006060EU
#define GPIO_3_HIDATA4             0x0006060FU

#define GPIO_4_GPIO04              0x00060800U
#define GPIO_4_EPWM3A              0x00060801U
#define GPIO_4_OUTPUTXBAR3         0x00060805U
#define GPIO_4_CANTXA              0x00060806U
#define GPIO_4_SPICLKB             0x00060807U
#define GPIO_4_EQEP2S              0x00060809U
#define GPIO_4_FSI_RXCLK           0x0006080AU
#define GPIO_4_OUTPUTXBAR2_6       0x0006080BU
#define GPIO_4_HIBASESEL2          0x0006080DU
#define GPIO_4_HINWE               0x0006080FU

#define GPIO_5_GPIO05              0x00060A00U
#define GPIO_5_EPWM3B              0x00060A01U
#define GPIO_5_OUTPUTXBAR3         0x00060A03U
#define GPIO_5_CANRXA              0x00060A06U
#define GPIO_5_SPISTEA             0x00060A07U
#define GPIO_5_FSI_TX1             0x00060A09U
#define GPIO_5_OUTPUTXBAR2_5       0x00060A0AU
#define GPIO_5_HIADDR7             0x00060A0DU
#define GPIO_5_HIDATA4             0x00060A0EU
#define GPIO_5_HIDATA15            0x00060A0FU

#define GPIO_6_GPIO06              0x00060C00U
#define GPIO_6_EPWM4A              0x00060C01U
#define GPIO_6_OUTPUTXBAR4         0x00060C02U
#define GPIO_6_EPWMSYNCO           0x00060C03U
#define GPIO_6_EQEP1A              0x00060C05U
#define GPIO_6_SPISOMIB            0x00060C07U
#define GPIO_6_FSI_TX0             0x00060C09U
#define GPIO_6_FSI_TX1             0x00060C0BU
#define GPIO_6_HINBYTEN1           0x00060C0DU
#define GPIO_6_OUTPUTXBAR2_8       0x00060C0EU
#define GPIO_6_HIDATA14            0x00060C0FU

#define GPIO_7_GPIO07              0x00060E00U
#define GPIO_7_EPWM4B              0x00060E01U
#define GPIO_7_OUTPUTXBAR5         0x00060E03U
#define GPIO_7_EQEP1B              0x00060E05U
#define GPIO_7_SPISIMOB            0x00060E07U
#define GPIO_7_FSI_TXCLK           0x00060E09U
#define GPIO_7_OUTPUTXBAR2_2       0x00060E0AU
#define GPIO_7_HIADDR6             0x00060E0DU
#define GPIO_7_HIDATA14            0x00060E0FU

#define GPIO_8_GPIO08              0x00061000U
#define GPIO_8_EPWM5A              0x00061001U
#define GPIO_8_ADCSOCAO            0x00061003U
#define GPIO_8_EQEP1S              0x00061005U
#define GPIO_8_SCITXDA             0x00061006U
#define GPIO_8_SPISIMOA            0x00061007U
#define GPIO_8_SCLA                0x00061009U
#define GPIO_8_FSI_TX1             0x0006100AU
#define GPIO_8_OUTPUTXBAR2_5       0x0006100BU
#define GPIO_8_HIADDR0             0x0006100DU
#define GPIO_8_FSI_TDM_CLK         0x0006100EU
#define GPIO_8_HIDATA8             0x0006100FU

#define GPIO_9_GPIO09              0x00061200U
#define GPIO_9_EPWM5B              0x00061201U
#define GPIO_9_OUTPUTXBAR6         0x00061203U
#define GPIO_9_EQEP1I              0x00061205U
#define GPIO_9_SCIRXDA             0x00061206U
#define GPIO_9_SPICLKA             0x00061207U
#define GPIO_9_FSI_TX0             0x0006120AU
#define GPIO_9_LINRXB              0x0006120BU
#define GPIO_9_HIBASESEL0          0x0006120DU
#define GPIO_9_SCLB                0x0006120EU
#define GPIO_9_HINRDY              0x0006120FU

#define GPIO_10_GPIO10              0x00061400U
#define GPIO_10_EPWM6A              0x00061401U
#define GPIO_10_ADCSOCBO            0x00061403U
#define GPIO_10_EQEP1A              0x00061405U
#define GPIO_10_SPISOMIA            0x00061407U
#define GPIO_10_SDAA                0x00061409U
#define GPIO_10_FSI_TXCLK           0x0006140AU
#define GPIO_10_LINTXB              0x0006140BU
#define GPIO_10_HINWE               0x0006140DU
#define GPIO_10_FSI_TDM_TX0         0x0006140EU

#define GPIO_11_GPIO11              0x00061600U
#define GPIO_11_EPWM6B              0x00061601U
#define GPIO_11_OUTPUTXBAR7         0x00061603U
#define GPIO_11_EQEP1B              0x00061605U
#define GPIO_11_SPISTEA             0x00061607U
#define GPIO_11_FSI_RX1             0x00061609U
#define GPIO_11_LINRXB              0x0006160AU
#define GPIO_11_EQEP2A              0x0006160BU
#define GPIO_11_SPISIMOA            0x0006160DU
#define GPIO_11_HIDATA6             0x0006160EU
#define GPIO_11_HINBYTEN0           0x0006160FU

#define GPIO_12_GPIO12              0x00061800U
#define GPIO_12_EPWM7A              0x00061801U
#define GPIO_12_EQEP1S              0x00061805U
#define GPIO_12_PMB1CTL             0x00061807U
#define GPIO_12_FSI_RX0             0x00061809U
#define GPIO_12_LINTXB              0x0006180AU
#define GPIO_12_SPICLKA             0x0006180BU
#define GPIO_12_CANRXA              0x0006180DU
#define GPIO_12_HIDATA13            0x0006180EU
#define GPIO_12_HIINT               0x0006180FU

#define GPIO_13_GPIO13              0x00061A00U
#define GPIO_13_EPWM7B              0x00061A01U
#define GPIO_13_EQEP1I              0x00061A05U
#define GPIO_13_PMB1ALRT            0x00061A07U
#define GPIO_13_FSI_RXCLK           0x00061A09U
#define GPIO_13_LINRXB              0x00061A0AU
#define GPIO_13_SPISOMIA            0x00061A0BU
#define GPIO_13_CANTXA              0x00061A0DU
#define GPIO_13_HIDATA11            0x00061A0EU
#define GPIO_13_HIDATA5             0x00061A0FU

#define GPIO_14_GPIO14              0x00061C00U
#define GPIO_14_SDAB                0x00061C05U
#define GPIO_14_OUTPUTXBAR3         0x00061C06U
#define GPIO_14_PMB1SDA             0x00061C07U
#define GPIO_14_SPICLKB             0x00061C09U
#define GPIO_14_EQEP2A              0x00061C0AU
#define GPIO_14_LINTXB              0x00061C0BU
#define GPIO_14_EPWM3A              0x00061C0DU
#define GPIO_14_OUTPUTXBAR2_7       0x00061C0EU
#define GPIO_14_HIDATA15            0x00061C0FU

#define GPIO_15_GPIO15              0x00061E00U
#define GPIO_15_SCLB                0x00061E05U
#define GPIO_15_OUTPUTXBAR4         0x00061E06U
#define GPIO_15_PMB1SCL             0x00061E07U
#define GPIO_15_SPISTEB             0x00061E09U
#define GPIO_15_EQEP2B              0x00061E0AU
#define GPIO_15_LINRXB              0x00061E0BU
#define GPIO_15_EPWM3B              0x00061E0DU
#define GPIO_15_OUTPUTXBAR2_6       0x00061E0EU
#define GPIO_15_HIDATA12            0x00061E0FU

#define GPIO_16_GPIO16              0x00080000U
#define GPIO_16_SPISIMOA            0x00080001U
#define GPIO_16_OUTPUTXBAR7         0x00080003U
#define GPIO_16_EPWM5A              0x00080005U
#define GPIO_16_SCITXDA             0x00080006U
#define GPIO_16_EQEP1S              0x00080009U
#define GPIO_16_PMB1SCL             0x0008000AU
#define GPIO_16_XCLKOUT             0x0008000BU
#define GPIO_16_EQEP2B              0x0008000DU
#define GPIO_16_SPISOMIB            0x0008000EU
#define GPIO_16_HIDATA1             0x0008000FU

#define GPIO_17_GPIO17              0x00080200U
#define GPIO_17_SPISOMIA            0x00080201U
#define GPIO_17_OUTPUTXBAR8         0x00080203U
#define GPIO_17_EPWM5B              0x00080205U
#define GPIO_17_SCIRXDA             0x00080206U
#define GPIO_17_EQEP1I              0x00080209U
#define GPIO_17_PMB1SDA             0x0008020AU
#define GPIO_17_CANTXA              0x0008020BU
#define GPIO_17_HIDATA2             0x0008020FU

#define GPIO_18_GPIO18              0x00080400U
#define GPIO_18_SPICLKA             0x00080401U
#define GPIO_18_CANRXA              0x00080403U
#define GPIO_18_EPWM6A              0x00080405U
#define GPIO_18_SCLA                0x00080406U
#define GPIO_18_EQEP2A              0x00080409U
#define GPIO_18_PMB1CTL             0x0008040AU
#define GPIO_18_XCLKOUT             0x0008040BU
#define GPIO_18_LINTXB              0x0008040DU
#define GPIO_18_FSI_TDM_CLK         0x0008040EU
#define GPIO_18_HIINT               0x0008040FU

#define GPIO_19_GPIO19              0x00080600U
#define GPIO_19_SPISTEA             0x00080601U
#define GPIO_19_CANTXA              0x00080603U
#define GPIO_19_EPWM6B              0x00080605U
#define GPIO_19_SDAA                0x00080606U
#define GPIO_19_EQEP2B              0x00080609U
#define GPIO_19_PMB1ALRT            0x0008060AU
#define GPIO_19_OUTPUTXBAR2_1       0x0008060BU
#define GPIO_19_LINRXB              0x0008060DU
#define GPIO_19_FSI_TDM_TX0         0x0008060EU
#define GPIO_19_HINBYTEN0           0x0008060FU

#define GPIO_22_GPIO22              0x00080C00U
#define GPIO_22_EQEP1S              0x00080C01U
#define GPIO_22_SPICLKB             0x00080C06U
#define GPIO_22_LINTXA              0x00080C09U
#define GPIO_22_OUTPUTXBAR2_1       0x00080C0AU
#define GPIO_22_LINTXB              0x00080C0BU
#define GPIO_22_HIADDR5             0x00080C0DU
#define GPIO_22_EPWM4A              0x00080C0EU
#define GPIO_22_HIDATA13            0x00080C0FU

#define GPIO_23_GPIO23              0x00080E00U
#define GPIO_23_EQEP1I              0x00080E01U
#define GPIO_23_SPISTEB             0x00080E06U
#define GPIO_23_LINRXA              0x00080E09U
#define GPIO_23_LINRXB              0x00080E0BU
#define GPIO_23_HIADDR3             0x00080E0DU
#define GPIO_23_EPWM4B              0x00080E0EU
#define GPIO_23_HIDATA11            0x00080E0FU

#define GPIO_24_GPIO24              0x00081000U
#define GPIO_24_OUTPUTXBAR1         0x00081001U
#define GPIO_24_EQEP2A              0x00081002U
#define GPIO_24_SPISIMOB            0x00081006U
#define GPIO_24_LINTXB              0x00081009U
#define GPIO_24_PMB1SCL             0x0008100AU
#define GPIO_24_SCITXDA             0x0008100BU
#define GPIO_24_ERRORSTS            0x0008100DU
#define GPIO_24_HIDATA3             0x0008100FU

#define GPIO_25_GPIO25              0x00081200U
#define GPIO_25_OUTPUTXBAR2         0x00081201U
#define GPIO_25_EQEP2B              0x00081202U
#define GPIO_25_EQEP1A              0x00081205U
#define GPIO_25_SPISOMIB            0x00081206U
#define GPIO_25_FSI_TX1             0x00081209U
#define GPIO_25_PMB1SDA             0x0008120AU
#define GPIO_25_SCIRXDA             0x0008120BU
#define GPIO_25_HIBASESEL0          0x0008120EU

#define GPIO_26_GPIO26              0x00081400U
#define GPIO_26_OUTPUTXBAR3         0x00081401U
#define GPIO_26_EQEP2I              0x00081402U
#define GPIO_26_SPICLKB             0x00081406U
#define GPIO_26_FSI_TX0             0x00081409U
#define GPIO_26_PMB1CTL             0x0008140AU
#define GPIO_26_SDAA                0x0008140BU
#define GPIO_26_HIDATA0             0x0008140EU
#define GPIO_26_HIADDR1             0x0008140FU

#define GPIO_27_GPIO27              0x00081600U
#define GPIO_27_OUTPUTXBAR4         0x00081601U
#define GPIO_27_EQEP2S              0x00081602U
#define GPIO_27_SPISTEB             0x00081606U
#define GPIO_27_FSI_TXCLK           0x00081609U
#define GPIO_27_PMB1ALRT            0x0008160AU
#define GPIO_27_SCLA                0x0008160BU
#define GPIO_27_HIDATA1             0x0008160EU
#define GPIO_27_HIADDR4             0x0008160FU

#define GPIO_28_GPIO28              0x00081800U
#define GPIO_28_SCIRXDA             0x00081801U
#define GPIO_28_EPWM7A              0x00081803U
#define GPIO_28_OUTPUTXBAR5         0x00081805U
#define GPIO_28_EQEP1A              0x00081806U
#define GPIO_28_EQEP2S              0x00081809U
#define GPIO_28_LINTXA              0x0008180AU
#define GPIO_28_SPICLKB             0x0008180BU
#define GPIO_28_ERRORSTS            0x0008180DU
#define GPIO_28_SDAB                0x0008180EU
#define GPIO_28_HINOE               0x0008180FU

#define GPIO_29_GPIO29              0x00081A00U
#define GPIO_29_SCITXDA             0x00081A01U
#define GPIO_29_EPWM7B              0x00081A03U
#define GPIO_29_OUTPUTXBAR6         0x00081A05U
#define GPIO_29_EQEP1B              0x00081A06U
#define GPIO_29_EQEP2I              0x00081A09U
#define GPIO_29_LINRXA              0x00081A0AU
#define GPIO_29_SPISTEB             0x00081A0BU
#define GPIO_29_ERRORSTS            0x00081A0DU
#define GPIO_29_SCLB                0x00081A0EU
#define GPIO_29_HINCS               0x00081A0FU

#define GPIO_30_GPIO30              0x00081C00U
#define GPIO_30_CANRXA              0x00081C01U
#define GPIO_30_SPISIMOB            0x00081C03U
#define GPIO_30_OUTPUTXBAR7         0x00081C05U
#define GPIO_30_EQEP1S              0x00081C06U
#define GPIO_30_FSI_RXCLK           0x00081C09U
#define GPIO_30_EPWM1A              0x00081C0BU
#define GPIO_30_HIDATA8             0x00081C0EU

#define GPIO_31_GPIO31              0x00081E00U
#define GPIO_31_CANTXA              0x00081E01U
#define GPIO_31_SPISOMIB            0x00081E03U
#define GPIO_31_OUTPUTXBAR8         0x00081E05U
#define GPIO_31_EQEP1I              0x00081E06U
#define GPIO_31_FSI_RX1             0x00081E09U
#define GPIO_31_EPWM1B              0x00081E0BU
#define GPIO_31_HIDATA10            0x00081E0EU

#define GPIO_32_GPIO32              0x00460000U
#define GPIO_32_SDAA                0x00460001U
#define GPIO_32_SPICLKB             0x00460003U
#define GPIO_32_LINTXA              0x00460006U
#define GPIO_32_FSI_RX0             0x00460009U
#define GPIO_32_CANTXA              0x0046000AU
#define GPIO_32_ADCSOCBO            0x0046000DU
#define GPIO_32_HIINT               0x0046000FU

#define GPIO_33_GPIO33              0x00460200U
#define GPIO_33_SCLA                0x00460201U
#define GPIO_33_SPISTEB             0x00460203U
#define GPIO_33_OUTPUTXBAR4         0x00460205U
#define GPIO_33_LINRXA              0x00460206U
#define GPIO_33_FSI_RXCLK           0x00460209U
#define GPIO_33_CANRXA              0x0046020AU
#define GPIO_33_EQEP2B              0x0046020BU
#define GPIO_33_ADCSOCAO            0x0046020DU
#define GPIO_33_HIDATA0             0x0046020FU

#define GPIO_34_GPIO34              0x00460400U
#define GPIO_34_OUTPUTXBAR1         0x00460401U
#define GPIO_34_PMB1SDA             0x00460406U
#define GPIO_34_HINBYTEN1           0x0046040DU
#define GPIO_34_SDAB                0x0046040EU
#define GPIO_34_HIDATA9             0x0046040FU

#define GPIO_35_GPIO35              0x00460600U
#define GPIO_35_SCIRXDA             0x00460601U
#define GPIO_35_SDAA                0x00460603U
#define GPIO_35_CANRXA              0x00460605U
#define GPIO_35_PMB1SCL             0x00460606U
#define GPIO_35_LINRXA              0x00460607U
#define GPIO_35_EQEP1A              0x00460609U
#define GPIO_35_PMB1CTL             0x0046060AU
#define GPIO_35_HINWE               0x0046060EU
#define GPIO_35_TDI                 0x0046060FU

#define GPIO_37_GPIO37              0x00460A00U
#define GPIO_37_OUTPUTXBAR2         0x00460A01U
#define GPIO_37_SCLA                0x00460A03U
#define GPIO_37_SCITXDA             0x00460A05U
#define GPIO_37_CANTXA              0x00460A06U
#define GPIO_37_LINTXA              0x00460A07U
#define GPIO_37_EQEP1B              0x00460A09U
#define GPIO_37_PMB1ALRT            0x00460A0AU
#define GPIO_37_HINRDY              0x00460A0EU
#define GPIO_37_TDO                 0x00460A0FU

#define GPIO_39_GPIO39              0x00460E00U
#define GPIO_39_FSI_RXCLK           0x00460E07U
#define GPIO_39_EQEP2I              0x00460E09U
#define GPIO_39_OUTPUTXBAR2_2       0x00460E0BU
#define GPIO_39_EPWMSYNCO           0x00460E0DU
#define GPIO_39_EQEP1I              0x00460E0EU
#define GPIO_39_HIDATA7             0x00460E0FU

#define GPIO_40_GPIO40              0x00461000U
#define GPIO_40_SPISIMOB            0x00461001U
#define GPIO_40_EMU0                0x00461003U
#define GPIO_40_EPWM2B              0x00461005U
#define GPIO_40_PMB1SDA             0x00461006U
#define GPIO_40_FSI_RX0             0x00461007U
#define GPIO_40_EQEP1A              0x0046100AU
#define GPIO_40_LINTXB              0x0046100BU
#define GPIO_40_HINBYTEN1           0x0046100EU
#define GPIO_40_HIDATA5             0x0046100FU

#define GPIO_41_GPIO41              0x00461200U
#define GPIO_41_EMU1                0x00461203U
#define GPIO_41_EPWM2A              0x00461205U
#define GPIO_41_PMB1SCL             0x00461206U
#define GPIO_41_FSI_RX1             0x00461207U
#define GPIO_41_EQEP1B              0x0046120AU
#define GPIO_41_LINRXB              0x0046120BU
#define GPIO_41_HIADDR4             0x0046120DU
#define GPIO_41_SPISOMIB            0x0046120EU
#define GPIO_41_HIDATA12            0x0046120FU

#define GPIO_42_GPIO42              0x00461400U
#define GPIO_42_LINRXA              0x00461402U
#define GPIO_42_OUTPUTXBAR5         0x00461403U
#define GPIO_42_PMB1CTL             0x00461405U
#define GPIO_42_SDAA                0x00461406U
#define GPIO_42_EQEP1S              0x0046140AU
#define GPIO_42_OUTPUTXBAR2_3       0x0046140BU
#define GPIO_42_HIDATA2             0x0046140EU
#define GPIO_42_HIADDR6             0x0046140FU

#define GPIO_43_GPIO43              0x00461600U
#define GPIO_43_OUTPUTXBAR6         0x00461603U
#define GPIO_43_PMB1ALRT            0x00461605U
#define GPIO_43_SCLA                0x00461606U
#define GPIO_43_EQEP1I              0x0046160AU
#define GPIO_43_OUTPUTXBAR2_4       0x0046160BU
#define GPIO_43_HIDATA3             0x0046160EU
#define GPIO_43_HIADDR7             0x0046160FU

#define GPIO_44_GPIO44              0x00461800U
#define GPIO_44_OUTPUTXBAR7         0x00461803U
#define GPIO_44_EQEP1A              0x00461805U
#define GPIO_44_FSI_TXCLK           0x00461807U
#define GPIO_44_OUTPUTXBAR2_3       0x0046180AU
#define GPIO_44_HIDATA7             0x0046180DU
#define GPIO_44_HIDATA5             0x0046180FU

#define GPIO_45_GPIO45              0x00461A00U
#define GPIO_45_OUTPUTXBAR8         0x00461A03U
#define GPIO_45_FSI_TX0             0x00461A07U
#define GPIO_45_OUTPUTXBAR2_4       0x00461A0AU
#define GPIO_45_HIDATA6             0x00461A0FU

#define GPIO_46_GPIO46              0x00461C00U
#define GPIO_46_LINTXA              0x00461C03U
#define GPIO_46_FSI_TX1             0x00461C07U
#define GPIO_46_HINWE               0x00461C0FU

#define GPIO_224_GPIO224             0x01C60000U
#define GPIO_224_HIADDR3             0x01C6000FU

#define GPIO_225_GPIO225             0x01C60200U
#define GPIO_225_HINWE               0x01C6020FU

#define GPIO_226_GPIO226             0x01C60400U
#define GPIO_226_HIADDR1             0x01C6040FU

#define GPIO_227_GPIO227             0x01C60600U
#define GPIO_227_HIBYTEN0            0x01C6060FU

#define GPIO_228_GPIO228             0x01C60800U
#define GPIO_228_HIADDR0             0x01C6080FU

#define GPIO_230_GPIO230             0x01C60C00U
#define GPIO_230_HIBASESEL2          0x01C60C0FU

#define GPIO_231_GPIO231             0x01C60E00U
#define GPIO_231_HIBASESEL1          0x01C60E0FU

#define GPIO_232_GPIO232             0x01C61000U
#define GPIO_232_HIBASESEL0          0x01C6100FU

#define GPIO_233_GPIO233             0x01C61200U
#define GPIO_233_HIADDR4             0x01C6120FU

#define GPIO_237_GPIO237             0x01C61A00U
#define GPIO_237_HIADDR6             0x01C61A0FU

#define GPIO_238_GPIO238             0x01C61C00U
#define GPIO_238_HINCS               0x01C61C0FU

#define GPIO_239_GPIO239             0x01C61E00U
#define GPIO_239_HIADDR5             0x01C61E0FU

#define GPIO_241_GPIO241             0x01C80200U
#define GPIO_241_HIBYTEN1            0x01C8020FU

#define GPIO_242_GPIO242             0x01C80400U
#define GPIO_242_HIADDR2             0x01C8040FU

#define GPIO_244_GPIO244             0x01C80800U
#define GPIO_244_HIADDR7             0x01C8080FU

#define GPIO_245_GPIO245             0x01C80A00U
#define GPIO_245_HINOE               0x01C80A0FU


#endif // PIN_MAP_H
