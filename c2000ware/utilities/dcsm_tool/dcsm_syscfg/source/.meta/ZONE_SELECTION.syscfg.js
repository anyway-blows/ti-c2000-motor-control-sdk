"use strict";
/*global exports, system*/

var zoneOptions = [
    {name: "ZONE1", displayName: "Zone 1"},
    {name: "ZONE2", displayName: "Zone 2"},
    {name: "ZONE1EXEONLY", displayName: "Zone1 EXEONLY"},
    {name: "ZONE2EXEONLY", displayName: "Zone2 EXEONLY"},
    {name: "UNSECURE", displayName: "Unsecure"}
]

var claZoneOptions = [
    {name: "ZONE1", displayName: "Zone 1"},
    {name: "ZONE2", displayName: "Zone 2"},
    {name: "UNSECURE", displayName: "Unsecure"}
]

let Common   = system.getScript('/utilities/dcsm_tool/dcsm_syscfg/source/Common.js');
let device_ram = 
    system.getScript("/utilities/dcsm_tool/dcsm_syscfg/source/ram_sect_info/" + 
        Common.getDeviceName().toLowerCase() + "_ram.js");
let device_sect = 
    system.getScript("/utilities/dcsm_tool/dcsm_syscfg/source/ram_sect_info/" + 
        Common.getDeviceName().toLowerCase() + "_sect.js");
/* Intro splash on GUI */
let longDescription = 'ZONE Selection configurations';


/* Array of Watchdog configurables that are common across device families */
let config = [
    {
        name        : 'CLA',
        displayName : 'CLA',
        description : 'CLA Zone Selection.',
        default     : "UNSECURE",
        options     : claZoneOptions
    }
];

var dr = device_ram[Common.getDeviceName().toLowerCase() + "_ram"]
dr.forEach(element => {
    //console.log(element)
    element["options"] = zoneOptions
    //console.log(element)
});

var ds = device_sect[Common.getDeviceName().toLowerCase() + "_sect"]
ds.forEach(element => {
    //console.log(element)
    element["options"] = zoneOptions
    //console.log(element)
});

config = config.concat(dr)
config = config.concat(ds)

/**
 * Validate this module's configuration
 *
 * @param inst       - Watchdog instance to be validated
 * @param validation - Issue reporting object
 */
function validate(inst, vo)
{
    
}


// Define the common/portable base Watchdog
exports = {
    displayName         : 'CLA/Memory Ownership (Per LINKPOINTER)',
    description         : 'CLA/Memory Ownership for the different sections in the memory. These values can be updated per LINKPOINTER update!',
    defaultInstanceName : 'ZONE_SELECTION',
    longDescription     : longDescription,
    config              : config,
    validate            : validate,
};
