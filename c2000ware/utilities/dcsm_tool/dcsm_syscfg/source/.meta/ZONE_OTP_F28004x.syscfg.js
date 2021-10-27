"use strict";
/*global exports, system*/
let Common   = system.getScript('/utilities/dcsm_tool/dcsm_syscfg/source/Common.js');

var ERRORSTS_options = [
    {name:0 , displayName: "GPIO24"},
    {name:1 , displayName: "GPIO28"},
    {name:2 , displayName: "GPIO29"},
    {name:3 , displayName: "ERRORSTS disabled (Default)"}
];

let REALIZED_BOOT_MODE = [
    {name: 0 , displayName:"Parallel IO"},
    {name: 1 , displayName:"SCI / Wait boot"},
    {name: 2 , displayName:"CAN"},
    {name: 3 , displayName:"Flash"},
    {name: 4 , displayName:"Wait"},
    {name: 5 , displayName:"RAM"},
    {name: 6 , displayName:"SPI Master"},
    {name: 7 , displayName:"I2C Master"},
]

var ALL_BOOT_OPTIONS = [];

let SCI_BOOT_OPTIONS =[
    {name: "0x01", displayName:"(SCI) SCIATX=GPIO29 , SCIARX=GPIO28"},
    {name: "0x21", displayName:"(SCI) SCIATX=GPIO16 , SCIARX=GPIO17"},
    {name: "0x41", displayName:"(SCI) SCIATX=GPIO8  , SCIARX=GPIO9"},
    {name: "0x61", displayName:"(SCI) SCIATX=GPIO48 , SCIARX=GPIO49"},
    {name: "0x81", displayName:"(SCI) SCIATX=GPIO24 , SCIARX=GPIO25"},
]

let CAN_BOOT_OPTIONS =[
    {name: "0x02", displayName:"(CAN) CANTXA=GPIO32 , CANRXA=GPIO33"},
    {name: "0x22", displayName:"(CAN) CANTXA=GPIO4  , CANRXA=GPIO5"},
    {name: "0x42", displayName:"(CAN) CANTXA=GPIO31 , CANRXA=GPIO30"},
    {name: "0x62", displayName:"(CAN) CANTXA=GPIO37 , CANRXA=GPIO35"},
]

let WAIT_BOOT_OPTIONS =[
    {name: "0x04", displayName:"(WAIT) Watchdog is enable"},
    {name: "0x24", displayName:"(WAIT) Watchdog is disabled"},
]

let SPI_BOOT_OPTIONS =[
    {name: "0x26", displayName:"(SPI) SPIA_SIMO=GPIO8  , SPIA_SOMI=GPIO10 , SPIA_CLK=GPIO9  , SPIA_STE=GPIO11"},
    {name: "0x46", displayName:"(SPI) SPIA_SIMO=GPIO54 , SPIA_SOMI=GPIO55 , SPIA_CLK=GPIO56 , SPIA_STE=GPIO57"},
    {name: "0x66", displayName:"(SPI) SPIA_SIMO=GPIO16 , SPIA_SOMI=GPIO17 , SPIA_CLK=GPIO56 , SPIA_STE=GPIO57"},
    {name: "0x86", displayName:"(SPI) SPIA_SIMO=GPIO8  , SPIA_SOMI=GPIO17 , SPIA_CLK=GPIO9  , SPIA_STE=GPIO11"},
]

let I2C_BOOT_OPTIONS =[
    {name: "0x07", displayName:"(I2C) SDAA=GPIO32 , SCLA=GPIO33"},
    {name: "0x47", displayName:"(I2C) SDAA=GPIO26 , SCLA=GPIO27"},
    {name: "0x67", displayName:"(I2C) SDAA=GPIO42 , SCLA=GPIO43"},
]

let PARALLEL_BOOT_OPTIONS =[
    {name: "0x00", displayName:"(PARALLEL) D0-D7=GPIO0-GPIO7 , DSP-Control=GPIO16 , Host-Control=GPIO11"},
]

let FLASH_BOOT_OPTIONS =[
    {name: "0x03", displayName:"(FLASH) Flash-Entry-Address=0x00080000, Bank 0 , Sector 0 "},
    {name: "0x23", displayName:"(FLASH) Flash-Entry-Address=0x0008EFF0, Bank 0 , Sector 14"},
    {name: "0x43", displayName:"(FLASH) Flash-Entry-Address=0x00090000, Bank 1 , Sector 0 "},
    {name: "0x63", displayName:"(FLASH) Flash-Entry-Address=0x0009EFF0, Bank 1 , Sector 14"},
] 

let RAM_BOOT_OPTIONS =[
    {name: "0x05", displayName:"(RAM) RAM Entry Point Address=0x00000000"},
]

ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(SCI_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(CAN_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(WAIT_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(SPI_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(I2C_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(PARALLEL_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(FLASH_BOOT_OPTIONS)
ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(RAM_BOOT_OPTIONS)

console.log(ALL_BOOT_OPTIONS);

// Device specific options
var pin0Defaults = "GPIO32";
var pin1Defaults = "GPIO24";

/* Intro splash on GUI */
let longDescription = 'ZONE Header can only be programmed once. After they are programmed they cannot be modified. ' + 
                        'This section cannot be modified by using a different LINKPOINTER value.';

var gpioOptions = [];
var gpios = Common.getGPIOs();
var unacceptableGPIOs = ["GPIO20", "GPIO21", "GPIO22", "GPIO23", 
                         "GPIO36", "GPIO28"]
for (var i = 60; i <= 223; i++)
{
    unacceptableGPIOs.push("GPIO" + i.toString());
}
for (var gpioIndex = 0; gpioIndex < gpios.length; gpioIndex++)
{
    if (!unacceptableGPIOs.includes(gpios[gpioIndex]))
    {
        gpioOptions.push({name: gpios[gpioIndex]});
    }
}


function onChangeuseZone(inst, ui)
{
    var bootConfigs = ['bootPinCount','BMSP0',
                        'BMSP1','BMSP2', 'ERRORSTSPIN',
                        'BOOTDEF0','BOOTDEF1','BOOTDEF2',
                        'BOOTDEF3','BOOTDEF4','BOOTDEF5',
                        'BOOTDEF6','BOOTDEF7'];
    for(var uiConfigIndex = 3; uiConfigIndex < config.length; uiConfigIndex++)
    {
        var configName = config[uiConfigIndex].name;
        if (bootConfigs.includes(configName))
        {
            ui[configName].hidden = !(inst.useZone & inst.configureBoot)
        }
        else
        {
            ui[configName].hidden = !inst.useZone;
        }
    }
    
    if (inst['bootPinCount'] <= 2)
    {
        ui['BMSP2'].hidden = true;
    }
    if (inst['bootPinCount'] <= 1)
    {
        ui['BMSP1'].hidden = true;
    }
    if (inst['bootPinCount'] <= 0)
    {
        ui['BMSP0'].hidden = true;
    }

    for (var i=Math.pow(2, inst["bootPinCount"]); i < 8; i++)
    {
        ui['BOOTDEF' + i.toString()].hidden = true;
    }
    
    if (inst["zone"] == 2)
    {
        ui["configureBoot"].hidden = true;
        for(var i = 0; i < bootConfigs.length; i++)
        {
            var configName = bootConfigs[i];
            ui[configName].hidden = true;
        }
    }
    
}

function onChangeZone(inst, ui)
{
    onChangeuseZone(inst, ui);
    
}

/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name: "useZone",
        displayName : "Configure this Section",
        description : 'Check to configure the header OTP values for this zone. Once you program these OTP values, you will not be able to change them.',
        hidden      : false,
        default     : true,
        onChange    : onChangeuseZone
    },
    {
        name        : 'zone',
        displayName : 'Zone',
        description : '',
        hidden      : true,
        readOnly    : false,
        default     : 3,
        onChange    : onChangeZone
    },
    {
        name: "selectedCPU",
        displayName : "Selected CPU",
        hidden      : true,
        default     : "CPU1",
        onChange    : onChangeuseZone
    },
    {
        name        : 'PASWDLOCK',
        displayName : 'Password Lock (PSWDLOCK)',
        description : 'Enabling this option will block access from reading the passwords',
        hidden      : false,
        default     : "DISABLE",
        options     : [
            {name: "DISABLE", displayName: "Disable PSWDLOCK permanently"},
            {name: "ENABLE", displayName: "Enable PSWDLOCK permanently"},
        ]
    },
    {
        name        : 'CRCLOCK',
        displayName : 'CRC Lock (CRCLOCK)',
        description : "Enabling this will disable the VCU's ability to calculate a CRC value on secure memories",
        hidden      : false,
        default     : "DISABLE",
        options     : [
            {name: "DISABLE", displayName: "Disable CRCLOCK permanently"},
            {name: "ENABLE", displayName: "Enable CRCLOCK permanently"},
        ]
    },
    {
        name        : 'configureBoot',
        displayName : 'Configure Boot Setting',
        readOnly    : false,
        hidden      : false,
        default     : false,
        onChange    : onChangeuseZone
    },
    {
        name        : 'bootPinCount',
        displayName : 'Number of Boot Pins',
        readOnly    : false,
        hidden      : true,
        default     : 2,
        options     : [
            {name: 0},
            {name: 1},
            {name: 2},
            {name: 3}
        ],
        onChange    : onChangeuseZone
    },
    {
        name        : 'BMSP0',
        displayName : 'Boot Pin 0 (BMSP0)',
        readOnly    : false,
        hidden      : true,
        default     : pin0Defaults,
        options     : gpioOptions,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BMSP1',
        displayName : 'Boot Pin 1 (BMSP1)',
        readOnly    : false,
        hidden      : true,
        default     : pin1Defaults,
        options     : gpioOptions,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BMSP2',
        displayName : 'Boot Pin 2 (BMSP2)',
        readOnly    : false,
        hidden      : true,
        default     : "GPIO0",
        options     : gpioOptions,
        onChange    : onChangeuseZone
    },
    {
        name        : 'ERRORSTSPIN',
        displayName : 'ERRORSTS PIN',
        readOnly    : false,
        hidden      : true,
        default     : 3,
        options     : ERRORSTS_options,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF0',
        displayName : 'BOOTDEF0',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b000",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF1',
        displayName : 'BOOTDEF1',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b001",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF2',
        displayName : 'BOOTDEF2',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b010",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF3',
        displayName : 'BOOTDEF3',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b011",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF4',
        displayName : 'BOOTDEF4',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b100",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF5',
        displayName : 'BOOTDEF5',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b101",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF6',
        displayName : 'BOOTDEF6',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b110",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    {
        name        : 'BOOTDEF7',
        displayName : 'BOOTDEF7',
        description : "Boot mode when tha value of the GPIOs used for valid BSMP2-BSMP1-BSMP0 are 0b111",
        readOnly    : false,
        hidden      : true,
        default     : "0x00",
        options     : ALL_BOOT_OPTIONS,
        onChange    : onChangeuseZone
    },
    
];

/**
 * Validate this module's configuration
 *
 * @param inst       - Watchdog instance to be validated
 * @param validation - Issue reporting object
 */
function validate(inst, vo)
{
    var format8Hex = new RegExp(/^0x[A-F0-9]{8}$/i);
    var sequenceOf1then0 = new RegExp(/^0x1F{0,7}[EC8]{0,1}0{0,7}$/i);
    
    if (inst["zone"] == 1 &&
        inst["useZone"] == true &&
        inst["configureBoot"] == true)
    {
        var errGPIO = ERRORSTS_options.find(input => {
            return input.name === inst["ERRORSTSPIN"]
        }).displayName;
        if (inst["bootPinCount"] > 0)
        {
            if (inst["BMSP0"] == errGPIO)
            {
                Common.logError(vo, inst, "BMSP0", 
                    'The BMSP0 pin is conflicting with ERRORSTS pin.');
            }
        }
        if (inst["bootPinCount"] > 1)
        {
            if (inst["BMSP1"] == errGPIO)
            {
                Common.logError(vo, inst, "BMSP1", 
                    'The BMSP1 pin is conflicting with ERRORSTS pin.');
            }
            if (inst["BMSP0"] == inst["BMSP1"])
            {
                Common.logError(vo, inst, "BMSP1", 
                    'The BMSP1 pin is conflicting with BMSP0 pin.');
            }
        }
        if (inst["bootPinCount"] > 2)
        {
            if (inst["BMSP2"] == errGPIO)
            {
                Common.logError(vo, inst, "BMSP2", 
                    'The BMSP2 pin is conflicting with ERRORSTS pin.');
            }
            if (inst["BMSP0"] == inst["BMSP2"])
            {
                Common.logError(vo, inst, "BMSP2", 
                    'The BMSP2 pin is conflicting with BMSP0 pin.');
            }
            if (inst["BMSP1"] == inst["BMSP2"])
            {
                Common.logError(vo, inst, "BMSP2", 
                    'The BMSP2 pin is conflicting with BMSP1 pin.');
            }
        }
        
    }
    //var format8HexResult = format8Hex.test();
    //var sequenceOf1then0Result = sequenceOf1then0.test(); 

}


// Define the common/portable base Watchdog
exports = {
    displayName         : 'ZONE OTP',
    description         : 'ZONE OTP',
    defaultInstanceName : 'ZONE_OTP',
    longDescription     : longDescription,
    config              : config,
    validate            : validate
};
