expRemoveAll();

expAdd("motorVars.flagEnableSys",getDecimal())
expAdd("motorVars.flagRunIdentAndOnLine",getDecimal())
expAdd("motorVars.flagMotorIdentified",getDecimal())

expAdd("motorVars.flagEnableRsRecalc",getDecimal())
expAdd("motorVars.flagEnableUserParams",getDecimal())
expAdd("motorVars.flagEnableOffsetCalc",getDecimal())

expAdd("motorVars.flagEnableForceAngle",getDecimal())
expAdd("motorVars.flagEnablePowerWarp",getDecimal())
expAdd("motorVars.flagBypassLockRotor",getDecimal())
expAdd("motorVars.flagEnableSpeedCtrl",getDecimal())

expAdd("motorVars.ctrlState",getDecimal())
expAdd("motorVars.estState",getDecimal())
expAdd("motorVars.trajState",getDecimal())

expAdd("motorVars.RoverL_rps",getNatural())
expAdd("motorVars.Rr_Ohm",getNatural())
expAdd("motorVars.Rs_Ohm",getNatural())
expAdd("motorVars.Ls_d_H",getNatural())
expAdd("motorVars.Ls_q_H",getNatural())
expAdd("motorVars.flux_VpHz",getNatural())

expAdd("motorVars.speedRef_Hz",getNatural())
expAdd("motorVars.accelerationMax_Hzps",getNatural())
expAdd("motorVars.speed_Hz",getNatural())
expAdd("motorVars.speed_krpm",getNatural())
expAdd("motorVars.torque_Nm",getNatural())

expAdd("motorVars.VdcBus_V",getNatural())
expAdd("motorVars.Vbus_sf",getNatural())
expAdd("motorVars.offset_invVbus_invV",getNatural())

expAdd("flagEnableOffsetCalibration",getDecimal())
expAdd("offsetCalcCount",getDecimal())
expAdd("offsetCalcWaitTime",getDecimal())
expAdd("motorVars.offsets_I_A.value[0]",getNatural())
expAdd("motorVars.offsets_I_A.value[1]",getNatural())
expAdd("motorVars.offsets_I_A.value[2]",getNatural())
expAdd("motorVars.offsets_V_V.value[0]",getNatural())
expAdd("motorVars.offsets_V_V.value[1]",getNatural())
expAdd("motorVars.offsets_V_V.value[2]",getNatural())
expAdd("adcData.dcBus_V")
expAdd("adcData.I_A")
expAdd("adcData.V_V")

expAdd("AdcaResultRegs.ADCRESULT0")
expAdd("AdcbResultRegs.ADCRESULT0")
expAdd("AdccResultRegs.ADCRESULT0")
expAdd("AdcaResultRegs.ADCRESULT1")
expAdd("AdcbResultRegs.ADCRESULT1")
expAdd("AdccResultRegs.ADCRESULT1")
expAdd("AdcbResultRegs.ADCRESULT2")

expAdd("pwmData.Vabc_pu.value[0]")
expAdd("pwmData.Vabc_pu.value[1]")
expAdd("pwmData.Vabc_pu.value[2]")

expAdd("EPwm1Regs.TBPRD")
expAdd("EPwm1Regs.CMPA.bit.CMPA")	
expAdd("EPwm2Regs.CMPA.bit.CMPA")	
expAdd("EPwm3Regs.CMPA.bit.CMPA")

expAdd("motorVars.faultNow.bit")
expAdd("motorVars.faultNow.all")

expAdd("drvSPI8320Vars")