let EPWM_EmulationMode = [
	{ name: "EPWM_EMULATION_STOP_AFTER_NEXT_TB", displayName: "EMULATION STOP AFTER NEXT TB" },
	{ name: "EPWM_EMULATION_STOP_AFTER_FULL_CYCLE", displayName: "EMULATION STOP AFTER FULL CYCLE" },
	{ name: "EPWM_EMULATION_FREE_RUN", displayName: "EMULATION FREE RUN" },
]
let EPWM_SyncCountMode = [
	{ name: "EPWM_COUNT_MODE_DOWN_AFTER_SYNC", displayName: "Count down after sync event" },
	{ name: "EPWM_COUNT_MODE_UP_AFTER_SYNC", displayName: "Count up after sync event" },
]
let EPWM_ClockDivider = [
	{ name: "EPWM_CLOCK_DIVIDER_1", displayName: "Divide clock by 1" },
	{ name: "EPWM_CLOCK_DIVIDER_2", displayName: "Divide clock by 2" },
	{ name: "EPWM_CLOCK_DIVIDER_4", displayName: "Divide clock by 4" },
	{ name: "EPWM_CLOCK_DIVIDER_8", displayName: "Divide clock by 8" },
	{ name: "EPWM_CLOCK_DIVIDER_16", displayName: "Divide clock by 16" },
	{ name: "EPWM_CLOCK_DIVIDER_32", displayName: "Divide clock by 32" },
	{ name: "EPWM_CLOCK_DIVIDER_64", displayName: "Divide clock by 64" },
	{ name: "EPWM_CLOCK_DIVIDER_128", displayName: "Divide clock by 128" },
]
let EPWM_HSClockDivider = [
	{ name: "EPWM_HSCLOCK_DIVIDER_1", displayName: "Divide clock by 1" },
	{ name: "EPWM_HSCLOCK_DIVIDER_2", displayName: "Divide clock by 2" },
	{ name: "EPWM_HSCLOCK_DIVIDER_4", displayName: "Divide clock by 4" },
	{ name: "EPWM_HSCLOCK_DIVIDER_6", displayName: "Divide clock by 6" },
	{ name: "EPWM_HSCLOCK_DIVIDER_8", displayName: "Divide clock by 8" },
	{ name: "EPWM_HSCLOCK_DIVIDER_10", displayName: "Divide clock by 10" },
	{ name: "EPWM_HSCLOCK_DIVIDER_12", displayName: "Divide clock by 12" },
	{ name: "EPWM_HSCLOCK_DIVIDER_14", displayName: "Divide clock by 14" },
]
let EPWM_SyncInPulseSource = [
	{ name: "EPWM_SYNC_IN_PULSE_SRC_DISABLE", displayName: "SYNC IN PULSE SRC DISABLE" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM1" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM2" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM3" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM4" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM5" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM6", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM6" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7", displayName: "SYNC IN PULSE SRC SYNCOUT EPWM7" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP1", displayName: "SYNC IN PULSE SRC SYNCOUT ECAP1" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP2", displayName: "SYNC IN PULSE SRC SYNCOUT ECAP2" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_ECAP3", displayName: "SYNC IN PULSE SRC SYNCOUT ECAP3" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT5", displayName: "SYNC IN PULSE SRC INPUTXBAR OUT5" },
	{ name: "EPWM_SYNC_IN_PULSE_SRC_INPUTXBAR_OUT6", displayName: "SYNC IN PULSE SRC INPUTXBAR OUT6" },
]
let EPWM_OneShotSyncOutTrigger = [
	{ name: "EPWM_OSHT_SYNC_OUT_TRIG_SYNC", displayName: "Trigger is OSHT sync" },
	{ name: "EPWM_OSHT_SYNC_OUT_TRIG_RELOAD", displayName: "Trigger is OSHT reload" },
]
let EPWM_PeriodLoadMode = [
	{ name: "EPWM_PERIOD_SHADOW_LOAD", displayName: "PERIOD SHADOW LOAD" },
	{ name: "EPWM_PERIOD_DIRECT_LOAD", displayName: "PERIOD DIRECT LOAD" },
]
let EPWM_TimeBaseCountMode = [
	{ name: "EPWM_COUNTER_MODE_UP", displayName: "Up - count mode." },
	{ name: "EPWM_COUNTER_MODE_DOWN", displayName: "Down - count mode." },
	{ name: "EPWM_COUNTER_MODE_UP_DOWN", displayName: "Up - down - count mode." },
	{ name: "EPWM_COUNTER_MODE_STOP_FREEZE", displayName: "Stop - Freeze counter." },
]
let EPWM_PeriodShadowLoadMode = [
	{ name: "EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO", displayName: "SHADOW LOAD MODE COUNTER ZERO" },
	{ name: "EPWM_SHADOW_LOAD_MODE_COUNTER_SYNC", displayName: "SHADOW LOAD MODE COUNTER SYNC" },
	{ name: "EPWM_SHADOW_LOAD_MODE_SYNC", displayName: "SHADOW LOAD MODE SYNC" },
]
let EPWM_CurrentLink = [
	{ name: "EPWM_LINK_WITH_EPWM_1", displayName: "link current ePWM with ePWM1" },
	{ name: "EPWM_LINK_WITH_EPWM_2", displayName: "link current ePWM with ePWM2" },
	{ name: "EPWM_LINK_WITH_EPWM_3", displayName: "link current ePWM with ePWM3" },
	{ name: "EPWM_LINK_WITH_EPWM_4", displayName: "link current ePWM with ePWM4" },
	{ name: "EPWM_LINK_WITH_EPWM_5", displayName: "link current ePWM with ePWM5" },
	{ name: "EPWM_LINK_WITH_EPWM_6", displayName: "link current ePWM with ePWM6" },
	{ name: "EPWM_LINK_WITH_EPWM_7", displayName: "link current ePWM with ePWM7" },
]
let EPWM_LinkComponent = [
	{ name: "EPWM_LINK_TBPRD", displayName: "link TBPRD:TBPRDHR registers" },
	{ name: "EPWM_LINK_COMP_A", displayName: "link COMPA registers" },
	{ name: "EPWM_LINK_COMP_B", displayName: "link COMPB registers" },
	{ name: "EPWM_LINK_COMP_C", displayName: "link COMPC registers" },
	{ name: "EPWM_LINK_COMP_D", displayName: "link COMPD registers" },
	{ name: "EPWM_LINK_GLDCTL2", displayName: "link GLDCTL2 registers" },
]
let EPWM_CounterCompareModule = [
	{ name: "EPWM_COUNTER_COMPARE_A", displayName: "counter compare A" },
	{ name: "EPWM_COUNTER_COMPARE_B", displayName: "counter compare B" },
	{ name: "EPWM_COUNTER_COMPARE_C", displayName: "counter compare C" },
	{ name: "EPWM_COUNTER_COMPARE_D", displayName: "counter compare D" },
]
let EPWM_CounterCompareLoadMode = [
	{ name: "EPWM_COMP_LOAD_ON_CNTR_ZERO", displayName: "COMP LOAD ON CNTR ZERO" },
	{ name: "EPWM_COMP_LOAD_ON_CNTR_PERIOD", displayName: "COMP LOAD ON CNTR PERIOD" },
	{ name: "EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "COMP LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_COMP_LOAD_FREEZE", displayName: "COMP LOAD FREEZE" },
	{ name: "EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO", displayName: "COMP LOAD ON SYNC CNTR ZERO" },
	{ name: "EPWM_COMP_LOAD_ON_SYNC_CNTR_PERIOD", displayName: "COMP LOAD ON SYNC CNTR PERIOD" },
	{ name: "EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO_PERIOD", displayName: "COMP LOAD ON SYNC CNTR ZERO PERIOD" },
	{ name: "EPWM_COMP_LOAD_ON_SYNC_ONLY", displayName: "COMP LOAD ON SYNC ONLY" },
]
let EPWM_ActionQualifierModule = [
	{ name: "EPWM_ACTION_QUALIFIER_A", displayName: "Action Qualifier A" },
	{ name: "EPWM_ACTION_QUALIFIER_B", displayName: "Action Qualifier B" },
]
let EPWM_ActionQualifierLoadMode = [
	{ name: "EPWM_AQ_LOAD_ON_CNTR_ZERO", displayName: "AQ LOAD ON CNTR ZERO" },
	{ name: "EPWM_AQ_LOAD_ON_CNTR_PERIOD", displayName: "AQ LOAD ON CNTR PERIOD" },
	{ name: "EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "AQ LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_AQ_LOAD_FREEZE", displayName: "AQ LOAD FREEZE" },
	{ name: "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO", displayName: "AQ LOAD ON SYNC CNTR ZERO" },
	{ name: "EPWM_AQ_LOAD_ON_SYNC_CNTR_PERIOD", displayName: "AQ LOAD ON SYNC CNTR PERIOD" },
	{ name: "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD", displayName: "AQ LOAD ON SYNC CNTR ZERO PERIOD" },
	{ name: "EPWM_AQ_LOAD_ON_SYNC_ONLY", displayName: "AQ LOAD ON SYNC ONLY" },
]
let EPWM_ActionQualifierTriggerSource = [
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1", displayName: "Digital compare event A 1" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_2", displayName: "Digital compare event A 2" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_1", displayName: "Digital compare event B 1" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_DCB_2", displayName: "Digital compare event B 2" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_1", displayName: "Trip zone 1" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_2", displayName: "Trip zone 2" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_TZ_3", displayName: "Trip zone 3" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN", displayName: "ePWM sync" },
	{ name: "EPWM_AQ_TRIGGER_EVENT_TRIG_DC_EVTFILT", displayName: "Digital compare filter event" },
]
let EPWM_ActionQualifierOutputEvent = [
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO", displayName: "AQ OUTPUT ON TIMEBASE ZERO" },
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD", displayName: "AQ OUTPUT ON TIMEBASE PERIOD" },
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA", displayName: "AQ OUTPUT ON TIMEBASE UP CMPA" },
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA", displayName: "AQ OUTPUT ON TIMEBASE DOWN CMPA" },
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB", displayName: "AQ OUTPUT ON TIMEBASE UP CMPB" },
	{ name: "EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB", displayName: "AQ OUTPUT ON TIMEBASE DOWN CMPB" },
	{ name: "EPWM_AQ_OUTPUT_ON_T1_COUNT_UP", displayName: "AQ OUTPUT ON T1 COUNT UP" },
	{ name: "EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN", displayName: "AQ OUTPUT ON T1 COUNT DOWN" },
	{ name: "EPWM_AQ_OUTPUT_ON_T2_COUNT_UP", displayName: "AQ OUTPUT ON T2 COUNT UP" },
	{ name: "EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN", displayName: "AQ OUTPUT ON T2 COUNT DOWN" },
]
let EPWM_ActionQualifierOutput = [
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE", displayName: "No change in the output pins" },
	{ name: "EPWM_AQ_OUTPUT_LOW", displayName: "Set output pins to low" },
	{ name: "EPWM_AQ_OUTPUT_HIGH", displayName: "Set output pins to High" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE", displayName: "Toggle the output pins" },
]
let EPWM_ActionQualifierSWOutput = [
	{ name: "EPWM_AQ_SW_DISABLED", displayName: "Software forcing disabled" },
	{ name: "EPWM_AQ_SW_OUTPUT_LOW", displayName: "Set output pins to low" },
	{ name: "EPWM_AQ_SW_OUTPUT_HIGH", displayName: "Set output pins to High" },
]
let EPWM_ActionQualifierEventAction = [
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_ZERO", displayName: "AQ OUTPUT NO CHANGE ZERO" },
	{ name: "EPWM_AQ_OUTPUT_LOW_ZERO", displayName: "AQ OUTPUT LOW ZERO" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_ZERO", displayName: "AQ OUTPUT HIGH ZERO" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_ZERO", displayName: "AQ OUTPUT TOGGLE ZERO" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_PERIOD", displayName: "AQ OUTPUT NO CHANGE PERIOD" },
	{ name: "EPWM_AQ_OUTPUT_LOW_PERIOD", displayName: "AQ OUTPUT LOW PERIOD" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_PERIOD", displayName: "AQ OUTPUT HIGH PERIOD" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_PERIOD", displayName: "AQ OUTPUT TOGGLE PERIOD" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPA", displayName: "AQ OUTPUT NO CHANGE UP CMPA" },
	{ name: "EPWM_AQ_OUTPUT_LOW_UP_CMPA", displayName: "AQ OUTPUT LOW UP CMPA" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_UP_CMPA", displayName: "AQ OUTPUT HIGH UP CMPA" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_UP_CMPA", displayName: "AQ OUTPUT TOGGLE UP CMPA" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPA", displayName: "AQ OUTPUT NO CHANGE DOWN CMPA" },
	{ name: "EPWM_AQ_OUTPUT_LOW_DOWN_CMPA", displayName: "AQ OUTPUT LOW DOWN CMPA" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_DOWN_CMPA", displayName: "AQ OUTPUT HIGH DOWN CMPA" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPA", displayName: "AQ OUTPUT TOGGLE DOWN CMPA" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_UP_CMPB", displayName: "AQ OUTPUT NO CHANGE UP CMPB" },
	{ name: "EPWM_AQ_OUTPUT_LOW_UP_CMPB", displayName: "AQ OUTPUT LOW UP CMPB" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_UP_CMPB", displayName: "AQ OUTPUT HIGH UP CMPB" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_UP_CMPB", displayName: "AQ OUTPUT TOGGLE UP CMPB" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_CMPB", displayName: "AQ OUTPUT NO CHANGE DOWN CMPB" },
	{ name: "EPWM_AQ_OUTPUT_LOW_DOWN_CMPB", displayName: "AQ OUTPUT LOW DOWN CMPB" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_DOWN_CMPB", displayName: "AQ OUTPUT HIGH DOWN CMPB" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_DOWN_CMPB", displayName: "AQ OUTPUT TOGGLE DOWN CMPB" },
]
let EPWM_AdditionalActionQualifierEventAction = [
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_UP_T1", displayName: "AQ OUTPUT NO CHANGE UP T1" },
	{ name: "EPWM_AQ_OUTPUT_LOW_UP_T1", displayName: "AQ OUTPUT LOW UP T1" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_UP_T1", displayName: "AQ OUTPUT HIGH UP T1" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_UP_T1", displayName: "AQ OUTPUT TOGGLE UP T1" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T1", displayName: "AQ OUTPUT NO CHANGE DOWN T1" },
	{ name: "EPWM_AQ_OUTPUT_LOW_DOWN_T1", displayName: "AQ OUTPUT LOW DOWN T1" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_DOWN_T1", displayName: "AQ OUTPUT HIGH DOWN T1" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_DOWN_T1", displayName: "AQ OUTPUT TOGGLE DOWN T1" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_UP_T2", displayName: "AQ OUTPUT NO CHANGE UP T2" },
	{ name: "EPWM_AQ_OUTPUT_LOW_UP_T2", displayName: "AQ OUTPUT LOW UP T2" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_UP_T2", displayName: "AQ OUTPUT HIGH UP T2" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_UP_T2", displayName: "AQ OUTPUT TOGGLE UP T2" },
	{ name: "EPWM_AQ_OUTPUT_NO_CHANGE_DOWN_T2", displayName: "AQ OUTPUT NO CHANGE DOWN T2" },
	{ name: "EPWM_AQ_OUTPUT_LOW_DOWN_T2", displayName: "AQ OUTPUT LOW DOWN T2" },
	{ name: "EPWM_AQ_OUTPUT_HIGH_DOWN_T2", displayName: "AQ OUTPUT HIGH DOWN T2" },
	{ name: "EPWM_AQ_OUTPUT_TOGGLE_DOWN_T2", displayName: "AQ OUTPUT TOGGLE DOWN T2" },
]
let EPWM_ActionQualifierOutputModule = [
	{ name: "EPWM_AQ_OUTPUT_A", displayName: "ePWMxA output" },
	{ name: "EPWM_AQ_OUTPUT_B", displayName: "ePWMxB output" },
]
let EPWM_ActionQualifierContForce = [
	{ name: "EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO", displayName: "AQ SW SH LOAD ON CNTR ZERO" },
	{ name: "EPWM_AQ_SW_SH_LOAD_ON_CNTR_PERIOD", displayName: "AQ SW SH LOAD ON CNTR PERIOD" },
	{ name: "EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "AQ SW SH LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_AQ_SW_IMMEDIATE_LOAD", displayName: "AQ SW IMMEDIATE LOAD" },
]
let EPWM_DeadBandOutput = [
	{ name: "EPWM_DB_OUTPUT_A", displayName: "DB output is ePWMA" },
	{ name: "EPWM_DB_OUTPUT_B", displayName: "DB output is ePWMB" },
]
let EPWM_DeadBandDelayMode = [
	{ name: "EPWM_DB_RED", displayName: "DB RED (Rising Edge Delay) mode" },
	{ name: "EPWM_DB_FED", displayName: "DB FED (Falling Edge Delay) mode" },
]
let EPWM_DeadBandPolarity = [
	{ name: "EPWM_DB_POLARITY_ACTIVE_HIGH", displayName: "DB polarity is not inverted" },
	{ name: "EPWM_DB_POLARITY_ACTIVE_LOW", displayName: "DB polarity is inverted" },
]
let EPWM_DeadBandControlLoadMode = [
	{ name: "EPWM_DB_LOAD_ON_CNTR_ZERO", displayName: "DB LOAD ON CNTR ZERO" },
	{ name: "EPWM_DB_LOAD_ON_CNTR_PERIOD", displayName: "DB LOAD ON CNTR PERIOD" },
	{ name: "EPWM_DB_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "DB LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_DB_LOAD_FREEZE", displayName: "DB LOAD FREEZE" },
]
let EPWM_RisingEdgeDelayLoadMode = [
	{ name: "EPWM_RED_LOAD_ON_CNTR_ZERO", displayName: "RED LOAD ON CNTR ZERO" },
	{ name: "EPWM_RED_LOAD_ON_CNTR_PERIOD", displayName: "RED LOAD ON CNTR PERIOD" },
	{ name: "EPWM_RED_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "RED LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_RED_LOAD_FREEZE", displayName: "RED LOAD FREEZE" },
]
let EPWM_FallingEdgeDelayLoadMode = [
	{ name: "EPWM_FED_LOAD_ON_CNTR_ZERO", displayName: "FED LOAD ON CNTR ZERO" },
	{ name: "EPWM_FED_LOAD_ON_CNTR_PERIOD", displayName: "FED LOAD ON CNTR PERIOD" },
	{ name: "EPWM_FED_LOAD_ON_CNTR_ZERO_PERIOD", displayName: "FED LOAD ON CNTR ZERO PERIOD" },
	{ name: "EPWM_FED_LOAD_FREEZE", displayName: "FED LOAD FREEZE" },
]
let EPWM_DeadBandClockMode = [
	{ name: "EPWM_DB_COUNTER_CLOCK_FULL_CYCLE", displayName: "DB COUNTER CLOCK FULL CYCLE" },
	{ name: "EPWM_DB_COUNTER_CLOCK_HALF_CYCLE", displayName: "DB COUNTER CLOCK HALF CYCLE" },
]
let EPWM_TripZoneDigitalCompareOutput = [
	{ name: "EPWM_TZ_DC_OUTPUT_A1", displayName: "Digital Compare output 1 A" },
	{ name: "EPWM_TZ_DC_OUTPUT_A2", displayName: "Digital Compare output 2 A" },
	{ name: "EPWM_TZ_DC_OUTPUT_B1", displayName: "Digital Compare output 1 B" },
	{ name: "EPWM_TZ_DC_OUTPUT_B2", displayName: "Digital Compare output 2 B" },
]
let EPWM_TripZoneDigitalCompareOutputEvent = [
	{ name: "EPWM_TZ_EVENT_DC_DISABLED", displayName: "Event is disabled" },
	{ name: "EPWM_TZ_EVENT_DCXH_LOW", displayName: "Event when DCxH low" },
	{ name: "EPWM_TZ_EVENT_DCXH_HIGH", displayName: "Event when DCxH high" },
	{ name: "EPWM_TZ_EVENT_DCXL_LOW", displayName: "Event when DCxL low" },
	{ name: "EPWM_TZ_EVENT_DCXL_HIGH", displayName: "Event when DCxL high" },
	{ name: "EPWM_TZ_EVENT_DCXL_HIGH_DCXH_LOW", displayName: "Event when DCxL high DCxH low" },
]
let EPWM_TripZoneEvent = [
	{ name: "EPWM_TZ_ACTION_EVENT_TZA", displayName: "TZ1 - TZ6, DCAEVT2, DCAEVT1" },
	{ name: "EPWM_TZ_ACTION_EVENT_TZB", displayName: "TZ1 - TZ6, DCBEVT2, DCBEVT1" },
	{ name: "EPWM_TZ_ACTION_EVENT_DCAEVT1", displayName: "DCAEVT1 (Digital Compare A event 1)" },
	{ name: "EPWM_TZ_ACTION_EVENT_DCAEVT2", displayName: "DCAEVT2 (Digital Compare A event 2)" },
	{ name: "EPWM_TZ_ACTION_EVENT_DCBEVT1", displayName: "DCBEVT1 (Digital Compare B event 1)" },
	{ name: "EPWM_TZ_ACTION_EVENT_DCBEVT2", displayName: "DCBEVT2 (Digital Compare B event 2)" },
]
let EPWM_TripZoneAction = [
	{ name: "EPWM_TZ_ACTION_HIGH_Z", displayName: "high impedance output" },
	{ name: "EPWM_TZ_ACTION_HIGH", displayName: "high voltage state" },
	{ name: "EPWM_TZ_ACTION_LOW", displayName: "low voltage state" },
	{ name: "EPWM_TZ_ACTION_DISABLE", displayName: "disable action" },
]
let EPWM_TripZoneAdvancedEvent = [
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_TZB_D", displayName: "TZ ADV ACTION EVENT TZB D" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_TZB_U", displayName: "TZ ADV ACTION EVENT TZB U" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_TZA_D", displayName: "TZ ADV ACTION EVENT TZA D" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_TZA_U", displayName: "TZ ADV ACTION EVENT TZA U" },
]
let EPWM_TripZoneAdvancedAction = [
	{ name: "EPWM_TZ_ADV_ACTION_HIGH_Z", displayName: "high impedance output" },
	{ name: "EPWM_TZ_ADV_ACTION_HIGH", displayName: "high voltage state" },
	{ name: "EPWM_TZ_ADV_ACTION_LOW", displayName: "low voltage state" },
	{ name: "EPWM_TZ_ADV_ACTION_TOGGLE", displayName: "toggle the output" },
	{ name: "EPWM_TZ_ADV_ACTION_DISABLE", displayName: "disable action" },
]
let EPWM_TripZoneAdvDigitalCompareEvent = [
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U", displayName: "TZ ADV ACTION EVENT DCXEVT1 U" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D", displayName: "TZ ADV ACTION EVENT DCXEVT1 D" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U", displayName: "TZ ADV ACTION EVENT DCXEVT2 U" },
	{ name: "EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D", displayName: "TZ ADV ACTION EVENT DCXEVT2 D" },
]
let EPWM_CycleByCycleTripZoneClearMode = [
	{ name: "EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO", displayName: "TZ CBC PULSE CLR CNTR ZERO" },
	{ name: "EPWM_TZ_CBC_PULSE_CLR_CNTR_PERIOD", displayName: "TZ CBC PULSE CLR CNTR PERIOD" },
	{ name: "EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO_PERIOD", displayName: "TZ CBC PULSE CLR CNTR ZERO PERIOD" },
]
let EPWM_ADCStartOfConversionType = [
	{ name: "EPWM_SOC_A", displayName: "SOC A" },
	{ name: "EPWM_SOC_B", displayName: "SOC B" },
]
let EPWM_ADCStartOfConversionSource = [
	{ name: "EPWM_SOC_DCxEVT1", displayName: "SOC DCXEVT1" },
	{ name: "EPWM_SOC_TBCTR_ZERO", displayName: "SOC TBCTR ZERO" },
	{ name: "EPWM_SOC_TBCTR_PERIOD", displayName: "SOC TBCTR PERIOD" },
	{ name: "EPWM_SOC_TBCTR_ZERO_OR_PERIOD", displayName: "SOC TBCTR ZERO OR PERIOD" },
	{ name: "EPWM_SOC_TBCTR_U_CMPA", displayName: "SOC TBCTR U CMPA" },
	{ name: "EPWM_SOC_TBCTR_U_CMPC", displayName: "SOC TBCTR U CMPC" },
	{ name: "EPWM_SOC_TBCTR_D_CMPA", displayName: "SOC TBCTR D CMPA" },
	{ name: "EPWM_SOC_TBCTR_D_CMPC", displayName: "SOC TBCTR D CMPC" },
	{ name: "EPWM_SOC_TBCTR_U_CMPB", displayName: "SOC TBCTR U CMPB" },
	{ name: "EPWM_SOC_TBCTR_U_CMPD", displayName: "SOC TBCTR U CMPD" },
	{ name: "EPWM_SOC_TBCTR_D_CMPB", displayName: "SOC TBCTR D CMPB" },
	{ name: "EPWM_SOC_TBCTR_D_CMPD", displayName: "SOC TBCTR D CMPD" },
]
let EPWM_DigitalCompareType = [
	{ name: "EPWM_DC_TYPE_DCAH", displayName: "Digital Compare A High" },
	{ name: "EPWM_DC_TYPE_DCAL", displayName: "Digital Compare A Low" },
	{ name: "EPWM_DC_TYPE_DCBH", displayName: "Digital Compare B High" },
	{ name: "EPWM_DC_TYPE_DCBL", displayName: "Digital Compare B Low" },
]
let EPWM_DigitalCompareTripInput = [
	{ name: "EPWM_DC_TRIP_TRIPIN1", displayName: "Trip 1" },
	{ name: "EPWM_DC_TRIP_TRIPIN2", displayName: "Trip 2" },
	{ name: "EPWM_DC_TRIP_TRIPIN3", displayName: "Trip 3" },
	{ name: "EPWM_DC_TRIP_TRIPIN4", displayName: "Trip 4" },
	{ name: "EPWM_DC_TRIP_TRIPIN5", displayName: "Trip 5" },
	{ name: "EPWM_DC_TRIP_TRIPIN6", displayName: "Trip 6" },
	{ name: "EPWM_DC_TRIP_TRIPIN7", displayName: "Trip 7" },
	{ name: "EPWM_DC_TRIP_TRIPIN8", displayName: "Trip 8" },
	{ name: "EPWM_DC_TRIP_TRIPIN9", displayName: "Trip 9" },
	{ name: "EPWM_DC_TRIP_TRIPIN10", displayName: "Trip 10" },
	{ name: "EPWM_DC_TRIP_TRIPIN11", displayName: "Trip 11" },
	{ name: "EPWM_DC_TRIP_TRIPIN12", displayName: "Trip 12" },
	{ name: "EPWM_DC_TRIP_TRIPIN14", displayName: "Trip 14" },
	{ name: "EPWM_DC_TRIP_TRIPIN15", displayName: "Trip 15" },
	{ name: "EPWM_DC_TRIP_COMBINATION", displayName: "All Trips (Trip1 - Trip 15) are selected" },
]
let EPWM_DigitalCompareBlankingPulse = [
	{ name: "EPWM_DC_WINDOW_START_TBCTR_PERIOD", displayName: "DC WINDOW START TBCTR PERIOD" },
	{ name: "EPWM_DC_WINDOW_START_TBCTR_ZERO", displayName: "DC WINDOW START TBCTR ZERO" },
	{ name: "EPWM_DC_WINDOW_START_TBCTR_ZERO_PERIOD", displayName: "DC WINDOW START TBCTR ZERO PERIOD" },
]
let EPWM_DigitalCompareFilterInput = [
	{ name: "EPWM_DC_WINDOW_SOURCE_DCAEVT1", displayName: "DC filter signal source is DCAEVT1" },
	{ name: "EPWM_DC_WINDOW_SOURCE_DCAEVT2", displayName: "DC filter signal source is DCAEVT2" },
	{ name: "EPWM_DC_WINDOW_SOURCE_DCBEVT1", displayName: "DC filter signal source is DCBEVT1" },
	{ name: "EPWM_DC_WINDOW_SOURCE_DCBEVT2", displayName: "DC filter signal source is DCBEVT2" },
]
let EPWM_DigitalCompareModule = [
	{ name: "EPWM_DC_MODULE_A", displayName: "Digital Compare Module A" },
	{ name: "EPWM_DC_MODULE_B", displayName: "Digital Compare Module B" },
]
let EPWM_DigitalCompareEvent = [
	{ name: "EPWM_DC_EVENT_1", displayName: "Digital Compare Event number 1" },
	{ name: "EPWM_DC_EVENT_2", displayName: "Digital Compare Event number 2" },
]
let EPWM_DigitalCompareEventSource = [
	{ name: "EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL", displayName: "DC EVENT SOURCE ORIG SIGNAL" },
	{ name: "EPWM_DC_EVENT_SOURCE_FILT_SIGNAL", displayName: "DC EVENT SOURCE FILT SIGNAL" },
]
let EPWM_DigitalCompareSyncMode = [
	{ name: "EPWM_DC_EVENT_INPUT_SYNCED", displayName: "DC EVENT INPUT SYNCED" },
	{ name: "EPWM_DC_EVENT_INPUT_NOT_SYNCED", displayName: "DC EVENT INPUT NOT SYNCED" },
]
let EPWM_GlobalLoadTrigger = [
	{ name: "EPWM_GL_LOAD_PULSE_CNTR_ZERO", displayName: "GL LOAD PULSE CNTR ZERO" },
	{ name: "EPWM_GL_LOAD_PULSE_CNTR_PERIOD", displayName: "GL LOAD PULSE CNTR PERIOD" },
	{ name: "EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD", displayName: "GL LOAD PULSE CNTR ZERO PERIOD" },
	{ name: "EPWM_GL_LOAD_PULSE_SYNC", displayName: "GL LOAD PULSE SYNC" },
	{ name: "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO", displayName: "GL LOAD PULSE SYNC OR CNTR ZERO" },
	{ name: "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD", displayName: "GL LOAD PULSE SYNC OR CNTR PERIOD" },
	{ name: "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD", displayName: "GL LOAD PULSE SYNC CNTR ZERO PERIOD" },
	{ name: "EPWM_GL_LOAD_PULSE_GLOBAL_FORCE", displayName: "GL LOAD PULSE GLOBAL FORCE" },
]
let EPWM_ValleyTriggerSource = [
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE", displayName: "VALLEY TRIGGER EVENT SOFTWARE" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO", displayName: "VALLEY TRIGGER EVENT CNTR ZERO" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_CNTR_PERIOD", displayName: "VALLEY TRIGGER EVENT CNTR PERIOD" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_CNTR_ZERO_PERIOD", displayName: "VALLEY TRIGGER EVENT CNTR ZERO PERIOD" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_DCAEVT1", displayName: "VALLEY TRIGGER EVENT DCAEVT1" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_DCAEVT2", displayName: "VALLEY TRIGGER EVENT DCAEVT2" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_DCBEVT1", displayName: "VALLEY TRIGGER EVENT DCBEVT1" },
	{ name: "EPWM_VALLEY_TRIGGER_EVENT_DCBEVT2", displayName: "VALLEY TRIGGER EVENT DCBEVT2" },
]
let EPWM_ValleyCounterEdge = [
	{ name: "EPWM_VALLEY_COUNT_START_EDGE", displayName: "Valley count start edge" },
	{ name: "EPWM_VALLEY_COUNT_STOP_EDGE", displayName: "Valley count stop edge" },
]
let EPWM_ValleyDelayMode = [
	{ name: "EPWM_VALLEY_DELAY_MODE_SW_DELAY", displayName: "VALLEY DELAY MODE SW DELAY" },
	{ name: "EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SW_DELAY", displayName: "VALLEY DELAY MODE VCNT DELAY SW DELAY" },
	{ name: "EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_1_SW_DELAY", displayName: "VALLEY DELAY MODE VCNT DELAY SHIFT 1 SW DELAY" },
	{ name: "EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_2_SW_DELAY", displayName: "VALLEY DELAY MODE VCNT DELAY SHIFT 2 SW DELAY" },
	{ name: "EPWM_VALLEY_DELAY_MODE_VCNT_DELAY_SHIFT_4_SW_DELAY", displayName: "VALLEY DELAY MODE VCNT DELAY SHIFT 4 SW DELAY" },
]
let EPWM_DigitalCompareEdgeFilterMode = [
	{ name: "EPWM_DC_EDGEFILT_MODE_RISING", displayName: "Digital Compare Edge filter low" },
	{ name: "EPWM_DC_EDGEFILT_MODE_FALLING", displayName: "Digital Compare Edge filter high" },
	{ name: "EPWM_DC_EDGEFILT_MODE_BOTH", displayName: "Digital Compare Edge filter both" },
]
let EPWM_DigitalCompareEdgeFilterEdgeCount = [
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_0", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_1", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_2", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_3", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_4", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_5", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_6", displayName: "Digital Compare Edge filter edge" },
	{ name: "EPWM_DC_EDGEFILT_EDGECNT_7", displayName: "Digital Compare Edge filter edge" },
]
let EPWM_LockRegisterGroup = [
	{ name: "EPWM_REGISTER_GROUP_GLOBAL_LOAD", displayName: "Global load register group" },
	{ name: "EPWM_REGISTER_GROUP_TRIP_ZONE", displayName: "Trip zone register group" },
	{ name: "EPWM_REGISTER_GROUP_TRIP_ZONE_CLEAR", displayName: "Trip zone clear group" },
	{ name: "EPWM_REGISTER_GROUP_DIGITAL_COMPARE", displayName: "Digital compare group" },
]
exports = {
	EPWM_EmulationMode: EPWM_EmulationMode,
	EPWM_SyncCountMode: EPWM_SyncCountMode,
	EPWM_ClockDivider: EPWM_ClockDivider,
	EPWM_HSClockDivider: EPWM_HSClockDivider,
	EPWM_SyncInPulseSource: EPWM_SyncInPulseSource,
	EPWM_OneShotSyncOutTrigger: EPWM_OneShotSyncOutTrigger,
	EPWM_PeriodLoadMode: EPWM_PeriodLoadMode,
	EPWM_TimeBaseCountMode: EPWM_TimeBaseCountMode,
	EPWM_PeriodShadowLoadMode: EPWM_PeriodShadowLoadMode,
	EPWM_CurrentLink: EPWM_CurrentLink,
	EPWM_LinkComponent: EPWM_LinkComponent,
	EPWM_CounterCompareModule: EPWM_CounterCompareModule,
	EPWM_CounterCompareLoadMode: EPWM_CounterCompareLoadMode,
	EPWM_ActionQualifierModule: EPWM_ActionQualifierModule,
	EPWM_ActionQualifierLoadMode: EPWM_ActionQualifierLoadMode,
	EPWM_ActionQualifierTriggerSource: EPWM_ActionQualifierTriggerSource,
	EPWM_ActionQualifierOutputEvent: EPWM_ActionQualifierOutputEvent,
	EPWM_ActionQualifierOutput: EPWM_ActionQualifierOutput,
	EPWM_ActionQualifierSWOutput: EPWM_ActionQualifierSWOutput,
	EPWM_ActionQualifierEventAction: EPWM_ActionQualifierEventAction,
	EPWM_AdditionalActionQualifierEventAction: EPWM_AdditionalActionQualifierEventAction,
	EPWM_ActionQualifierOutputModule: EPWM_ActionQualifierOutputModule,
	EPWM_ActionQualifierContForce: EPWM_ActionQualifierContForce,
	EPWM_DeadBandOutput: EPWM_DeadBandOutput,
	EPWM_DeadBandDelayMode: EPWM_DeadBandDelayMode,
	EPWM_DeadBandPolarity: EPWM_DeadBandPolarity,
	EPWM_DeadBandControlLoadMode: EPWM_DeadBandControlLoadMode,
	EPWM_RisingEdgeDelayLoadMode: EPWM_RisingEdgeDelayLoadMode,
	EPWM_FallingEdgeDelayLoadMode: EPWM_FallingEdgeDelayLoadMode,
	EPWM_DeadBandClockMode: EPWM_DeadBandClockMode,
	EPWM_TripZoneDigitalCompareOutput: EPWM_TripZoneDigitalCompareOutput,
	EPWM_TripZoneDigitalCompareOutputEvent: EPWM_TripZoneDigitalCompareOutputEvent,
	EPWM_TripZoneEvent: EPWM_TripZoneEvent,
	EPWM_TripZoneAction: EPWM_TripZoneAction,
	EPWM_TripZoneAdvancedEvent: EPWM_TripZoneAdvancedEvent,
	EPWM_TripZoneAdvancedAction: EPWM_TripZoneAdvancedAction,
	EPWM_TripZoneAdvDigitalCompareEvent: EPWM_TripZoneAdvDigitalCompareEvent,
	EPWM_CycleByCycleTripZoneClearMode: EPWM_CycleByCycleTripZoneClearMode,
	EPWM_ADCStartOfConversionType: EPWM_ADCStartOfConversionType,
	EPWM_ADCStartOfConversionSource: EPWM_ADCStartOfConversionSource,
	EPWM_DigitalCompareType: EPWM_DigitalCompareType,
	EPWM_DigitalCompareTripInput: EPWM_DigitalCompareTripInput,
	EPWM_DigitalCompareBlankingPulse: EPWM_DigitalCompareBlankingPulse,
	EPWM_DigitalCompareFilterInput: EPWM_DigitalCompareFilterInput,
	EPWM_DigitalCompareModule: EPWM_DigitalCompareModule,
	EPWM_DigitalCompareEvent: EPWM_DigitalCompareEvent,
	EPWM_DigitalCompareEventSource: EPWM_DigitalCompareEventSource,
	EPWM_DigitalCompareSyncMode: EPWM_DigitalCompareSyncMode,
	EPWM_GlobalLoadTrigger: EPWM_GlobalLoadTrigger,
	EPWM_ValleyTriggerSource: EPWM_ValleyTriggerSource,
	EPWM_ValleyCounterEdge: EPWM_ValleyCounterEdge,
	EPWM_ValleyDelayMode: EPWM_ValleyDelayMode,
	EPWM_DigitalCompareEdgeFilterMode: EPWM_DigitalCompareEdgeFilterMode,
	EPWM_DigitalCompareEdgeFilterEdgeCount: EPWM_DigitalCompareEdgeFilterEdgeCount,
	EPWM_LockRegisterGroup: EPWM_LockRegisterGroup,
}
