"use strict";
/*global exports, system*/
let Common   = system.getScript('/utilities/dcsm_tool/dcsm_syscfg/source/Common.js');

var ERRORSTS_options = [
    {name:0 , displayName: "GPIO24"},
    {name:1 , displayName: "GPIO28"},
    {name:2 , displayName: "GPIO29"},
    {name:3 , displayName: "ERRORSTS disabled (Default)"}
];


var MPOST_options = [
    {name:0 , displayName: "Run MPOST using INTOSC2 with PLL disabled (10MHz internal oscillator)"},
    {name:1 , displayName: "Run MPOST with PLL enabled for 95MHz"},
    {name:2 , displayName: "Run MPOST with PLL enabled for 47.5MHz"},
    {name:3 , displayName: "Disable MPOST"}
];

if ("F2838x".includes(Common.getDeviceName()))
{
    MPOST_options = [
        {name:0x0 , displayName: "MPOST will be run with PLL enabled for high speed (110MHz)"},
        {name:0xC , displayName: "MPOST will be run with PLL enabled for medium speed (80MHz)."},
        {name:0x3 , displayName: "MPOST will be run with PLL enabled for low speed (60MHz)"},
        {name:0x9 , displayName: "MPOST will be run using INTOSC2 with PLL disabled (10MHz)"},
        {name:0x1 , displayName: "Disable MPOST"}
    ];
}

// CJTAGNODEID: Boot ROM takes this values and programs the lower 4 bits of the CJTAGNODEID register

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

if ("F28004x".includes(Common.getDeviceName()))
{
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
    
    //console.log(ALL_BOOT_OPTIONS);
}
else if ("F28002x".includes(Common.getDeviceName()))
{
    let SCI_BOOT_OPTIONS =[
        {name: "0x01", displayName:"(SCI) SCIATX=GPIO29 , SCIARX=GPIO28"},
        {name: "0x21", displayName:"(SCI) SCIATX=GPIO16 , SCIARX=GPIO17"},
        {name: "0x41", displayName:"(SCI) SCIATX=GPIO8  , SCIARX=GPIO9"},
        {name: "0x61", displayName:"(SCI) SCIATX=GPIO2  , SCIARX=GPIO3"},
        {name: "0x81", displayName:"(SCI) SCIATX=GPIO16 , SCIARX=GPIO3"},
    ]
    
    let CAN_BOOT_OPTIONS =[
        {name: "0x02", displayName:"(CAN) CANTXA=GPIO4  , CANRXA=GPIO5"},
        {name: "0x22", displayName:"(CAN) CANTXA=GPIO32 , CANRXA=GPIO33"},
        {name: "0x42", displayName:"(CAN) CANTXA=GPIO2  , CANRXA=GPIO3"},
    ]
    
    let WAIT_BOOT_OPTIONS =[
        {name: "0x04", displayName:"(WAIT) Watchdog is enable"},
        {name: "0x24", displayName:"(WAIT) Watchdog is disabled"},
    ]
    
    let SPI_BOOT_OPTIONS =[
        {name: "0x06", displayName:"(SPI) SPIA_SIMO=GPIO2  , SPIA_SOMI=GPIO1  , SPIA_CLK=GPIO3  , SPIA_STE=GPIO5"},
        {name: "0x26", displayName:"(SPI) SPIA_SIMO=GPIO16 , SPIA_SOMI=GPIO1  , SPIA_CLK=GPIO3  , SPIA_STE=GPIO0"},
        {name: "0x46", displayName:"(SPI) SPIA_SIMO=GPIO8  , SPIA_SOMI=GPIO10 , SPIA_CLK=GPIO9  , SPIA_STE=GPIO11"},
        {name: "0x66", displayName:"(SPI) SPIA_SIMO=GPIO8  , SPIA_SOMI=GPIO17 , SPIA_CLK=GPIO9  , SPIA_STE=GPIO11"},
    ]
    
    let I2C_BOOT_OPTIONS =[
        {name: "0x07", displayName:"(I2C) SDAA=GPIO32 , SCLA=GPIO33"},
        {name: "0x27", displayName:"(I2C) SDAA=GPIO0  , SCLA=GPIO1"},
        {name: "0x47", displayName:"(I2C) SDAA=GPIO10 , SCLA=GPIO8"},
    ]
    
    let PARALLEL_BOOT_OPTIONS =[
        {name: "0x00", displayName:"(PARALLEL) D0=GPIO28, D1-D7=GPIO1-GPIO7 , DSP-Control=GPIO16 , Host-Control=GPIO29"},
        {name: "0x20", displayName:"(PARALLEL) D0-D7=GPIO0-GPIO7 , DSP-Control=GPIO16 , Host-Control=GPIO11"},
    ]
    
    let FLASH_BOOT_OPTIONS =[
        {name: "0x03", displayName:"(FLASH) Flash-Entry-Address=0x00080000, Bank 0 , Sector 0 "},
        {name: "0x23", displayName:"(FLASH) Flash-Entry-Address=0x00084000, Bank 0 , Sector 4"},
        {name: "0x43", displayName:"(FLASH) Flash-Entry-Address=0x00088000, Bank 1 , Sector 8 "},
        {name: "0x63", displayName:"(FLASH) Flash-Entry-Address=0x0008EFF0, Bank 0 , Sector 14"},
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
    
    //console.log(ALL_BOOT_OPTIONS);
}
else if ("F2838x".includes(Common.getDeviceName()))
{
    let SCI_BOOT_OPTIONS =[
        {name: "0x01", displayName:"(SCI) SCIATX=GPIO29 , SCIARX=GPIO28"},
        {name: "0x21", displayName:"(SCI) SCIATX=GPIO84 , SCIARX=GPIO85"},
        {name: "0x41", displayName:"(SCI) SCIATX=GPIO36 , SCIARX=GPIO35"},
        {name: "0x61", displayName:"(SCI) SCIATX=GPIO42 , SCIARX=GPIO43"},
        {name: "0x81", displayName:"(SCI) SCIATX=GPIO65 , SCIARX=GPIO64"},
        {name: "0xA1", displayName:"(SCI) SCIATX=GPIO135, SCIARX=GPIO136"},
        {name: "0xC1", displayName:"(SCI) SCIATX=GPIO8  , SCIARX=GPIO9"},
    ]
    
    let CAN_BOOT_OPTIONS =[
        {name: "0x02", displayName:"(CAN) CANTXA=GPIO37 , CANRXA=GPIO36"},
        {name: "0x22", displayName:"(CAN) CANTXA=GPIO71 , CANRXA=GPIO70"},
        {name: "0x42", displayName:"(CAN) CANTXA=GPIO63 , CANRXA=GPIO62"},
        {name: "0x62", displayName:"(CAN) CANTXA=GPIO19 , CANRXA=GPIO18"},
        {name: "0x82", displayName:"(CAN) CANTXA=GPIO4  , CANRXA=GPIO5"},
        {name: "0xA2", displayName:"(CAN) CANTXA=GPIO31 , CANRXA=GPIO30"},
    ]
    
    let WAIT_BOOT_OPTIONS =[
        {name: "0x04", displayName:"(WAIT) Watchdog is enable"},
        {name: "0x24", displayName:"(WAIT) Watchdog is disabled"},
    ]
    
    let SPI_BOOT_OPTIONS =[
        {name: "0x06", displayName:"(SPI) SPIA_SIMO=GPIO58 , SPIA_SOMI=GPIO59 , SPIA_CLK=GPIO60 , SPIA_STE=GPIO61"},
        {name: "0x26", displayName:"(SPI) SPIA_SIMO=GPIO16 , SPIA_SOMI=GPIO17 , SPIA_CLK=GPIO18 , SPIA_STE=GPIO19"},
        {name: "0x46", displayName:"(SPI) SPIA_SIMO=GPIO32 , SPIA_SOMI=GPIO33 , SPIA_CLK=GPIO34 , SPIA_STE=GPIO35"},
        {name: "0x66", displayName:"(SPI) SPIA_SIMO=GPIO16 , SPIA_SOMI=GPIO17 , SPIA_CLK=GPIO56 , SPIA_STE=GPIO57"},
        {name: "0x86", displayName:"(SPI) SPIA_SIMO=GPIO54 , SPIA_SOMI=GPIO55 , SPIA_CLK=GPIO56 , SPIA_STE=GPIO57"},
    ]
    
    let I2C_BOOT_OPTIONS =[
        {name: "0x07", displayName:"(I2C) SDAA=GPIO91 , SCLA=GPIO92"},
        {name: "0x27", displayName:"(I2C) SDAA=GPIO32 , SCLA=GPIO33"},
        {name: "0x47", displayName:"(I2C) SDAA=GPIO42 , SCLA=GPIO43"},
        {name: "0x67", displayName:"(I2C) SDAA=GPIO0  , SCLA=GPIO1"},
        {name: "0x87", displayName:"(I2C) SDAA=GPIO104, SCLA=GPIO105"},
    ]
    
    let PARALLEL_BOOT_OPTIONS =[
        {name: "0x00", displayName:"(PARALLEL) D0=GPIO89, D1=GPIO90, D2-D6=GPIO58-GPIO62, D7=GPIO88, DSP-Control=GPIO91 , Host-Control=GPIO92"},
    ]
    
    let FLASH_BOOT_OPTIONS =[
        {name: "0x03", displayName:"(FLASH) Flash-Entry-Address=0x00080000, CPU1 Bank 0 , Sector 0"},
        {name: "0x23", displayName:"(FLASH) Flash-Entry-Address=0x00088000, CPU1 Bank 0 , Sector 4"},
        {name: "0x43", displayName:"(FLASH) Flash-Entry-Address=0x000A8000, CPU1 Bank 0 , Sector 8"},
        {name: "0x63", displayName:"(FLASH) Flash-Entry-Address=0x000BE000, CPU1 Bank 0 , Sector 14"},
    ] 

    let SECURE_FLASH_BOOT_OPTIONS =[
        {name: "0x0A", displayName:"(SECURE FLASH) Flash-Entry-Address=0x00080000, CPU1 Bank 0 , Sector 0"},
        {name: "0x2A", displayName:"(SECURE FLASH) Flash-Entry-Address=0x00088000, CPU1 Bank 0 , Sector 4"},
        {name: "0x4A", displayName:"(SECURE FLASH) Flash-Entry-Address=0x000A8000, CPU1 Bank 0 , Sector 8"},
        {name: "0x6A", displayName:"(SECURE FLASH) Flash-Entry-Address=0x000BE000, CPU1 Bank 0 , Sector 13"},
    ] 
    
    let USB_BOOT_OPTIONS =[
        {name: "0x09", displayName:"(USB) USBDM=GPIO42, USBDP=GPIO43"},
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
    ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(SECURE_FLASH_BOOT_OPTIONS)
    ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(USB_BOOT_OPTIONS)
    ALL_BOOT_OPTIONS = ALL_BOOT_OPTIONS.concat(RAM_BOOT_OPTIONS)
}

// Device specific options
var pin0Defaults = "GPIO32";
var pin1Defaults = "GPIO24";
if ("F2838x".includes(Common.getDeviceName()))
{
    pin0Defaults = "GPIO84";
    pin1Defaults = "GPIO72";
}

/* Intro splash on GUI */
let longDescription = 'ZONE Header can ONLY be programmed once. After they are programmed they cannot be modified. ' + 
                        'This section cannot be modified by using a different LINKPOINTER value.';

var gpioOptions = [];
var gpios = Common.getGPIOs();
var unacceptableGPIOs = [];
if ("F28004x".includes(Common.getDeviceName()))
{
    unacceptableGPIOs = ["GPIO20", "GPIO21", 
                         "GPIO22", "GPIO23", 
                         "GPIO36", "GPIO38"]
    for (var i = 60; i <= 223; i++)
    {
        unacceptableGPIOs.push("GPIO" + i.toString());
    }
}
if ("F28002x".includes(Common.getDeviceName()))
{
    unacceptableGPIOs = ["GPIO20", "GPIO21", 
                         "GPIO36", "GPIO38"]
    for (var i = 63; i <= 223; i++)
    {
        unacceptableGPIOs.push("GPIO" + i.toString());
    }
}

if ("F2838x".includes(Common.getDeviceName()))
{
    unacceptableGPIOs = ["GPIO42", "GPIO43"]
    for (var i = 169; i <= 255; i++)
    {
        unacceptableGPIOs.push("GPIO" + i.toString());
    }
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
    if ("F28002x".includes(Common.getDeviceName()))
    {
        bootConfigs = bootConfigs.concat(['RUNMPOST', 'CJTAGNODEID']);
    }
    else if ("F2838x".includes(Common.getDeviceName()))
    {
        bootConfigs = bootConfigs.concat(['RUNMPOST', 
                    'CMACKEY0', 'CMACKEY1', 'CMACKEY2', 'CMACKEY3']);
    }
    
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

    if ("F2838x".includes(Common.getDeviceName()))
    {
        if (!inst['JTAGLOCK'])
        {
            ui['JTAGPSWDH0'].hidden = true;
            ui['JTAGPSWDH1'].hidden = true;
        }
    }

    for (var i=Math.pow(2, inst["bootPinCount"]); i < 8; i++)
    {
        ui['BOOTDEF' + i.toString()].hidden = true;
    }
    
    if ("F28004x".includes(Common.getDeviceName()))
    {
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
    if ("F2838x".includes(Common.getDeviceName()))
    {
        if (inst["zone"] == 2)
        {
            ui['JTAGLOCK'].hidden = true;
            ui['JTAGPSWDH0'].hidden = true;
            ui['JTAGPSWDH1'].hidden = true;
            ui['CMACKEY0'].hidden = true; 
            ui['CMACKEY1'].hidden = true; 
            ui['CMACKEY2'].hidden = true; 
            ui['CMACKEY3'].hidden = true;
        }
    }
    if ("F28002x".includes(Common.getDeviceName())){
    	ui['CJTAGNODEID'].hidden = true;
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
]

if ("F2838x".includes(Common.getDeviceName()))
{
    config = config.concat([
        {
            name        : 'JTAGLOCK',
            displayName : 'Enable JTAGLOCK',
            description : "Enable the JTAGLOCK feature which disables the JTAG access on a device to avoid any debug access to it. This can only be configured once",
            readOnly    : false,
            hidden      : false,
            default     : false,
            onChange    : onChangeuseZone
        },
        {
            name        : 'JTAGPSWDH0',
            displayName : 'JTAGPSWDH0 (JTAG Password)',
            description : "When the JTAGLOCK feature is enabled, the JTAGPSWDH0 represents the JTAG Lock permanent password 0. This can only be configured once",
            readOnly    : false,
            hidden      : true,
            default     : "0x4BFFFFFF",
            onChange    : onChangeuseZone
        },
        {
            name        : 'JTAGPSWDH1',
            displayName : 'JTAGPSWDH1 (JTAG Password)',
            description : "When the JTAGLOCK feature is enabled, the JTAGPSWDH1 represents the JTAG Lock permanent password 1. This can only be configured once",
            readOnly    : false,
            hidden      : true,
            default     : "0x3FFFFFFF",
            onChange    : onChangeuseZone
        },
    ]);
}

config = config.concat([
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
    }
]);

if (["F28002x", "F28004x"].includes(Common.getDeviceName()))
{
    config = config.concat([{
        name        : 'ERRORSTSPIN',
        displayName : 'ERRORSTS PIN',
        readOnly    : false,
        hidden      : true,
        default     : 3,
        options     : ERRORSTS_options,
        onChange    : onChangeuseZone
    }]);
}

config = config.concat([
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
    
]);

if ("F2838x".includes(Common.getDeviceName()))
{
    var mpost_cmac_config = [
        {
            name        : 'RUNMPOST',
            displayName : 'Run MPOST',
            readOnly    : false,
            hidden      : true,
            default     : 3,
            options     : MPOST_options,
            onChange    : onChangeuseZone
        },
        {
            name        : 'CMACKEY0',
            displayName : 'CMAC Key0',
            readOnly    : false,
            hidden      : true,
            default     : "0x00000000",
            onChange    : onChangeuseZone
        },
        {
            name        : 'CMACKEY1',
            displayName : 'CMAC Key1',
            readOnly    : false,
            hidden      : true,
            default     : "0x00000000",
            onChange    : onChangeuseZone
        },
        {
            name        : 'CMACKEY2',
            displayName : 'CMAC Key2',
            readOnly    : false,
            hidden      : true,
            default     : "0x00000000",
            onChange    : onChangeuseZone
        },
        {
            name        : 'CMACKEY3',
            displayName : 'CMAC Key3',
            readOnly    : false,
            hidden      : true,
            default     : "0x00000000",
            onChange    : onChangeuseZone
        },
    ]

    config = config.concat(mpost_cmac_config)
}

if ("F28002x".includes(Common.getDeviceName()))
{
    var mpost_cjtagnode_config = [
        {
            name        : 'RUNMPOST',
            displayName : 'Run MPOST',
            readOnly    : false,
            hidden      : true,
            default     : 3,
            options     : MPOST_options,
            onChange    : onChangeuseZone
        },
        {
            name        : 'CJTAGNODEID',
            displayName : 'CJTAG Node ID',
            readOnly    : false,
            hidden      : true,
            default     : 15,
            onChange    : onChangeuseZone
        },

    ]

    config = config.concat(mpost_cjtagnode_config)
}

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

        if ("F2838x".includes(Common.getDeviceName()))
        {
            var cmackkeys = ["CMACKEY0", "CMACKEY1", "CMACKEY2", "CMACKEY3"]
            for (var cmackkeys_i in cmackkeys)
            {
                var format8HexResult = format8Hex.test(inst[cmackkeys[cmackkeys_i]]);
                if (!format8HexResult)
                {
                    Common.logError(vo, inst, cmackkeys[cmackkeys_i], 'Invalid 8 digit hex value');
                }
            }
        }

        var errGPIO = "";
        if (["F28002x", "F28004x"].includes(Common.getDeviceName()))
        {
            errGPIO = ERRORSTS_options.find(input => {
                return input.name === inst["ERRORSTSPIN"]
            }).displayName;
        }
        
        if (inst["bootPinCount"] > 0)
        {
            if (["F28002x", "F28004x"].includes(Common.getDeviceName())){
                if (inst["BMSP0"] == errGPIO)
                {
                    Common.logError(vo, inst, "BMSP0", 
                        'The BMSP0 pin is conflicting with ERRORSTS pin.');
                }
            }
        }
        if (inst["bootPinCount"] > 1)
        {
            if (["F28002x", "F28004x"].includes(Common.getDeviceName())){
                if (inst["BMSP1"] == errGPIO)
                {
                    Common.logError(vo, inst, "BMSP1", 
                        'The BMSP1 pin is conflicting with ERRORSTS pin.');
                }
            }
            if (inst["BMSP0"] == inst["BMSP1"])
            {
                Common.logError(vo, inst, "BMSP1", 
                    'The BMSP1 pin is conflicting with BMSP0 pin.');
            }
        }
        if (inst["bootPinCount"] > 2)
        {
            if (["F28002x", "F28004x"].includes(Common.getDeviceName())){
                if (inst["BMSP2"] == errGPIO)
                {
                    Common.logError(vo, inst, "BMSP2", 
                        'The BMSP2 pin is conflicting with ERRORSTS pin.');
                }
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
        if ("F28002x".includes(Common.getDeviceName()))
        {
            if (!(inst["CJTAGNODEID"] >= 0 && inst["CJTAGNODEID"] < 16))
            {
                Common.logError(vo, inst, "CJTAGNODEID", 
                'Enter a valid 4-bit CJTAG NODE ID (0-15).');
            }
        }
    }
    if ("F2838x".includes(Common.getDeviceName()))
    {
        if (inst["zone"] == 1 &&
            inst["useZone"] == true &&
            inst["JTAGLOCK"] == true)
        {
            var jtagpassword = ["JTAGPSWDH0", "JTAGPSWDH1"]
            for (var jtagpassword_i in jtagpassword)
            {
                var format8HexResult = format8Hex.test(inst[jtagpassword[jtagpassword_i]]);
                if (!format8HexResult)
                {
                    Common.logError(vo, inst, jtagpassword[jtagpassword_i], 'Invalid 8 digit hex value');
                }
                if (inst["zone"] == 1)
                {
                    var mask = Common.getDefaultJTAGPSWDHValueMask(1)[jtagpassword_i]
                    
                    if (parseInt(inst[jtagpassword[jtagpassword_i]]) != 
                    ((parseInt(inst[jtagpassword[jtagpassword_i]]) & mask)>>> 0))
                    {
                        Common.logError(vo, inst, jtagpassword[jtagpassword_i], 
                            'This password value has a mask of 0x' + mask.toString(16) + 
                            ". Input a password that when the mask is applied, it does not change.");
                    }
                }
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
