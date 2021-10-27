let Common   = system.getScript("/driverlib/Common.js");
let Pinmux   = system.getScript("/driverlib/pinmux.js");


/* Array of CAN configurables that are common across device families */
let config = [

];


/*
 *  ======== filterHardware ========
 *  Control RX, TX Pin usage by the user specified dataDirection.
 *
 *  param component - hardware object describing signals and
 *                     resources they're attached to
 *
 *  returns Boolean indicating whether or not to allow the component to
 *           be assigned to an instance's $hardware config
 */
function filterHardware(component)
{
    return (Common.typeMatches(component.type, ["ECAP"]));
}

var ecapModule = {
    peripheralName: "ECAP",
    displayName: "ECAP",
    maxInstances: 6,
    defaultInstanceName: "myECAP",
    description: "Enhanced Capture",
    filterHardware : filterHardware,
    config: config,
    templates: {
        boardc : "/driverlib/ecap/ecap.board.c.xdt",
        boardh : "/driverlib/ecap/ecap.board.h.xdt"
    },
};




exports = ecapModule;