

exports = {
    boardName: boardName,  /* get /ti/boards name */
    peripheralCount: peripheralCount,
    typeMatches: typeMatches,
    peripheralNames: peripheralNames,
    stringOrEmpty: stringOrEmpty,
    removeNonNumber : removeNonNumber,
    gpioNameToNumber : gpioNameToNumber,
    printDebugObject : printDebugObject,
    getGPIOs : getGPIOs,
    getAIOs : getAIOs,
    getDeviceName: getDeviceName,
	getSDK: getSDK,
	onlyPinmux: onlyPinmux,
	CLB_isType1 : CLB_isType1,
	CLB_isType1_Type2 : CLB_isType1_Type2,
	CLB_isType2 : CLB_isType2,
	zero_to_15        : [
        { name: 0 },
        { name: 1 },
        { name: 2 },
        { name: 3 },
        { name: 4 },
        { name: 5 },
        { name: 6 },
        { name: 7 },
        { name: 8 },
        { name: 9 },
        { name: 10 },
        { name: 11 },
        { name: 12 },
        { name: 13 },
        { name: 14 },
        { name: 15 },
	],
	zero_to_16        : [
        { name: 0 },
        { name: 1 },
        { name: 2 },
        { name: 3 },
        { name: 4 },
        { name: 5 },
        { name: 6 },
        { name: 7 },
        { name: 8 },
        { name: 9 },
        { name: 10 },
        { name: 11 },
        { name: 12 },
        { name: 13 },
        { name: 14 },
        { name: 15 },
        { name: 16 },
	],
};

function onlyPinmux()
{
	return false;
}

function getSDK()
{
	var sdk = system.getScript("/.metadata/sdk.json");	    

    return sdk;
}

function getDeviceName()
{
	var deviceName = system.deviceData.device;
	return deviceName
}

function stringOrEmpty(stringToCheck, stringToAdd)
{
    if (stringToCheck != "")
    {
        return stringToAdd;
    }
    return "";
}

function printDebugObject(obj)
{
	if (obj == null)
	{
		console.log("Object is null");
		return;
	}
    var keys = Object.keys(obj);
    for (var key in keys)
	{
    	console.log(keys[key] + " : " + obj[keys[key]]);
    }
}

function removeNonNumber(stringToCheck)
{
	var numberOnly = "";
    if (stringToCheck != "")
    {
    	for (var i = 0; i < stringToCheck.length; i++)
    	{
    		if ("0123456789".includes(stringToCheck[i]))
    		{
    			numberOnly += stringToCheck[i];
    		}
    	}
        return numberOnly;
    }
    return "";
}

//As soon a number is detected and then more characters are detected, exit.
//Good for usecases like GPIO180_X2, the 2 is not added to the numbers
function gpioNameToNumber(stringToCheck)
{
	var numberOnly = "";
	var numbersDetected = false;
    if (stringToCheck != "")
    {
    	for (var i = 0; i < stringToCheck.length; i++)
    	{
    		if ("0123456789".includes(stringToCheck[i]))
    		{
    			numberOnly += stringToCheck[i];
    			numbersDetected = true;
    		}
    		else
    		{
    			if (numbersDetected)
        		{
        			return numberOnly;
        		}
    		}
    	}
        return numberOnly;
    }
    return "";
}

function getGPIOs()
{
	var allGpioNames = [];
	let gpio = system.deviceData.interfaces.GPIO;
	if (gpio != null)
	{
		for (var i = 0; i < gpio.peripherals.length; i++)
		{
			var gpioperiph = gpio.peripherals[i];
			allGpioNames.push(gpioperiph.name);
		}
	}
	return allGpioNames
}

function getAIOs()
{
	var allAioNames = [];
	let aio = system.deviceData.interfaces.AIO;
	if (aio != null)
	{
		for (var i = 0; i < aio.peripherals.length; i++)
		{
			var aioperiph = aio.peripherals[i];
			allAioNames.push(aioperiph.name);
		}
	}
	return allAioNames;
}

/*!
 *  ======== boardName ========
 *  Get the name of the board (or device)
 */
function boardName()
{
    let boardName = system.deviceData.deviceId;

    if (system.deviceData.board != null) {
        boardName = system.deviceData.board.source;

        /* Strip off everything up to and including the last '/' */
        boardName = boardName.replace(/.*\//, '');

        /* Strip off everything after and including the first '.' */
        boardName = boardName.replace(/\..*/, '');
    }
    return (boardName);
}


function peripheralCount(peripheralType)
{
	let peripherals = system.deviceData.peripherals
	let numberOfPeripherals = Object.keys(peripherals).length;
	var count = 0;

	//console.log(numberOfPeripherals);

	for (var peripheral in peripherals) {
	  	
	  	try
	  	{
	  		var interfaces = peripherals[peripheral]["interfaces"];
	  		for (var interfaceType in interfaces)
	  		{
	  			if (peripheralType == interfaceType)
	  			{
	  				peripheralNames = interfaces[interfaceType].peripherals;
	  				//console.log(peripheralNames);
	  				count = peripheralNames.length;
					//console.log(count);
	  				return count;
	  			}
			}
		}
		catch(err) {
		
		}
	}
	//console.log(peripheralType);
	//console.log(count);
	if (count == 0)
	{
		return -1;
	}
    return (count);
}


function peripheralNames(peripheralType)
{
	let peripherals = system.deviceData.peripherals
	let numberOfPeripherals = Object.keys(peripherals).length;
	var names = [];

	//console.log(numberOfPeripherals);

	for (var peripheral in peripherals) {
	  	
	  	try
	  	{
	  		var interfaces = peripherals[peripheral]["interfaces"];
	  		for (var interfaceType in interfaces)
	  		{
	  			if (peripheralType == interfaceType)
	  			{
	  				names = interfaces[interfaceType].peripherals;
	  				//console.log(names);
	  				return names;
	  			}
			}
		}
		catch(err) {
		
		}
	}

    return (names);
}


function CLB_isType1() {
    return ["F28004x"].includes(getDeviceName());
}

function CLB_isType2() {
    return ["F28002x", "F2838x"].includes(getDeviceName());
}

function CLB_isType1_Type2(){
    return (CLB_isType1() | CLB_isType2());
}



/*
 *  ======== typeMatches ========
 *  Check that HW signal type matches a specified array of types
 *
 *  Example: within a module's filterHardware(component) method:
 *      for (sig in component.signals) {
 *          let type = component.signals[sig].type;
 *          if (Common.typeMatches(type, ["PWM", "DOUT"])) {
 *              :
 *          }
 *      }
 *
 *  type      - a string or array of strings that are valid signal types
 *  nameArray - array of signal name types that must match one of the signal
 *              types named by type
 *
 *  Returns true iff nameArray contains at least one type name that's
 *          specified the type parameter.
 */
function typeMatches(type, nameArray)
{
    let options = {};

    if (type instanceof Array || typeof type == "object") {
        for (var i = 0; i < type.length; i++) {
            options[type[i]] = 1;
        }
    }
    else if (typeof type == "string" || type instanceof String) {
        options[type] = 1;
    }

    for (var i = 0; i < nameArray.length; i++) {
        let name = nameArray[i];
        if (name in options) {
            return (true);
        }
    }

    return (false);
}