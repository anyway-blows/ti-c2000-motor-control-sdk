exports = {
	getDriverlibName : getDriverlibName
};


function getDriverlibName(interfaceName, peripheralName)
{
	interfaceName = interfaceName.toUpperCase()
	interfaceName = interfaceName.replace("-", "_")

	if (peripheralName.includes("SCI"))
	{
		return getSCIDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("UART"))
	{
		return getUARTDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("SPI"))
	{
		return getSPIDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("I2C"))
	{
		if (peripheralName.includes("CM-I2C"))
		{
			return getCMI2CDriverlibName(interfaceName, peripheralName);
		}
		else
		{
			return getI2CDriverlibName(interfaceName, peripheralName);
		}
	}
	else if (peripheralName.includes("CAN"))
	{
		if (peripheralName.includes("MCAN"))
		{
			return getMCANDriverlibName(interfaceName, peripheralName);
		}
		else
		{
			return getCANDriverlibName(interfaceName, peripheralName);
		}
	}
	else if (peripheralName.includes("EPWM"))
	{
		return getEPWMDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("OUTPUTXBAR"))
	{
		return getOUTPUTXBARDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("OTHER"))
	{
		return getOTHERDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("FSI"))
	{
		return getFSIDriverlibName(interfaceName, peripheralName);
	}	
	else if (peripheralName.includes("EQEP"))
	{
		return getEQEPDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("EMIF1"))
	{
		return getEMIF1DriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("EMIF2"))
	{
		return getEMIF2DriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("MCBSP"))
	{
		return getMCBSPDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("SD"))
	{
		return getSDDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("UPP"))
	{
		return getUPPDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("LIN"))
	{
		return getLINDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("DC-DC"))
	{
		return getDCDCDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("PMBUS"))
	{
		return getPMBUSDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("SSI"))
	{
		return getSSIDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("ECAT"))
	{
		return getECATDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("ETHERNET"))
	{
		return getETHERNETDriverlibName(interfaceName, peripheralName);
	}
	else if (peripheralName.includes("HIC"))
	{
		return getHICDriverlibName(interfaceName, peripheralName);
	}
	
	return "";
}


//SCI
function getSCIDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}
//SPI
function getSPIDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}

//I2C
function getI2CDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}


//CAN
function getCANDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}


//EPWM
function getEPWMDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("#", peripheralName.substring(4));

	return driverlibName;
}

//OUTPUTXBAR
function getOUTPUTXBARDriverlibName(interfaceName, peripheralName)
{
	let driverlibName = peripheralName;
	
	return driverlibName;
}

//OTHERS
function getOTHERDriverlibName(interfaceName, peripheralName)
{
	let driverlibName = interfaceName;
	if (interfaceName == "X2")
	{
		if (system.deviceData.deviceId.includes("F28004"))
		{
			driverlibName = "GPIO18_X2"
		}
		else
		{
			driverlibName = "GPIO18"
		}
	}
	else if (interfaceName == "X1")
	{
		driverlibName = "GPIO19"
	}
	return driverlibName;
}


//FSI
function getFSIDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("@", peripheralName[5])

	return driverlibName;
}


//EQEP
function getEQEPDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	var EQEPInstance = peripheralName.substring(4)
	driverlibName = interfaceName.replace("#", EQEPInstance)

	return driverlibName;
}


//MCBSP
function getMCBSPDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	var MCBSPInstance = peripheralName.substring(5);
	driverlibName = interfaceName.replace("@", MCBSPInstance)

	return driverlibName;
}

//EMIF1
function getEMIF1DriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = "";

	driverlibName = interfaceName

	return driverlibName;
}

//EMIF2
function getEMIF2DriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = "";

	driverlibName = interfaceName

	return driverlibName;
}


//SD
function getSDDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("#", peripheralName[2])

	return driverlibName;
}

//UPP
function getUPPDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("@", peripheralName[3]);	

	return driverlibName;
}


//LIN
function getLINDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";

	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}

//DC-DC
function getDCDCDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	if(interfaceName.includes("VFBSW"))
	{
		driverlibName = "GPIO22";
	}
	if(interfaceName.includes("VSW"))
	{
		driverlibName = "GPIO23";
	}

	return driverlibName;
}


//PMBUS
function getPMBUSDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("@", peripheralName[5])

	return driverlibName;
}

//CMI2C
function getCMI2CDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = interfaceName.replace("@" , peripheralName[6]);

	return driverlibName;
}


//MCAN
function getMCANDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName

	return driverlibName;
}

//UART
function getUARTDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	
	driverlibName = interfaceName.replace("@", peripheralName[4])

	return driverlibName;
}

//SSI
function getSSIDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = interfaceName.replace("@", peripheralName[3]);

	return driverlibName;
}


//ETHERNET
function getETHERNETDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = interfaceName.toUpperCase();
	return driverlibName;
}

//ECAT
function getECATDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = interfaceName;
	return driverlibName;
}

//HIC
function getHICDriverlibName(interfaceName, peripheralName)
{
	var driverlibName = "";
	driverlibName = interfaceName;
	return driverlibName;
}