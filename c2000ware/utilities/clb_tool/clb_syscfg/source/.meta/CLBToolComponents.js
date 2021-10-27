
exports = {
    displayName: "C2000 CLB Configuration Tool",
    templates:
    [
    
        {
            name      : "/utilities/clb_tool/clb_syscfg/source/templates/hfile/clb.h.xdt",
            outputPath: "clb_config.h"
        },
        {
            name      : "/utilities/clb_tool/clb_syscfg/source/templates/cfile/clb.c.xdt",
            outputPath: "clb_config.c"
        },
        {
            name      : "/utilities/clb_tool/clb_syscfg/source/templates/dot/clb.dot.xdt",
            outputPath: "clb.dot"
        },
        {
            name      : "/utilities/clb_tool/clb_syscfg/source/templates/systemc/clb.sim.xdt",
            outputPath: "clb_sim.cpp"
        }
    ],

    topModules:
    [
        {
            displayName: "Configurable Logic Block",
            description: "Configure instances of CLB tiles.",
            modules:
            [
                "/utilities/clb_tool/clb_syscfg/source/TILE",            
            ]
        }
    ],
}