exports = {
	getExtraInputOptions : getExtraInputOptions
};

function getExtraInputOptions()
{
	var deviceName = system.deviceData.deviceId;
	if (["F28002x", "F28004x", "F2838x"].includes(deviceName))
	{
		return [
			{name: "0xFFFE", displayName : "LOGIC HIGH ('1' selected input)"},
			{name: "0xFFFF", displayName : "LOGIC LOW ('0' selected input)"}
		];
	}
	else
	{
		return [];
	}
}