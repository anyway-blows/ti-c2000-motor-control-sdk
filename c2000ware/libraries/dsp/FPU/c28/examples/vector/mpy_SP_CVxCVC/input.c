//#############################################################################
//! \file   input.c
//! \brief  Input Vector (64) 
//! \author Vishal Coelho 
//! \date   16-Sep-2016
//! 
//
//  Group:          C2000
//  Target Family:  $DEVICE$
//
//#############################################################################
// $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
// $Release Date: Feb 12, 2021 $
// $Copyright: Copyright (C) 2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################


float test_input[256] = {
     1.125541798361F, -2.052159867208F, -0.656497261735F, -0.685257998008F, 
    -0.832920102136F,  2.082120331303F,  3.066081352433F,  1.906094688038F, 
    -2.904472363724F, -2.761641029122F,  2.420081969933F, -0.632982095493F, 
     2.596757723074F,  0.168865823368F,  1.860978162734F, -0.522764360647F, 
    -2.521365114703F,  0.985579762252F, -1.496207478720F,  0.804080330174F, 
    -1.034483484073F, -1.307002572444F,  1.129264023384F, -0.429448362856F, 
    -2.283603987309F, -3.044284173343F,  1.390013368633F,  3.041462080758F, 
    -2.470788093372F, -2.091242556589F,  0.966085914025F, -2.474215675752F, 
    -0.036606235707F, -0.801673246622F,  1.753333687350F, -1.896778017650F, 
     1.351117811506F, -0.064794481300F,  2.536651094294F, -1.008492626689F, 
     2.456238535457F,  2.837677900566F, -1.041984270427F,  2.641024096838F, 
     1.248756893589F, -2.810613315736F, -1.898716856941F,  1.494506490946F, 
    -2.949698228501F, -1.450665427766F,  1.533563806602F, -0.484838130014F, 
     0.000140966971F,  0.300781743155F, -0.126152907751F,  2.781798514554F, 
     2.542944819714F, -0.516829035186F,  0.690312511118F,  3.035108159920F, 
     0.739319730211F, -1.247495349066F,  2.258442613615F,  1.263540748369F, 
     1.919446663704F,  1.045137828288F,  0.482055499655F,  0.245838830279F, 
    -1.992256881409F,  1.244733693668F, -1.634055370062F,  1.046325738726F, 
     2.428526098953F, -2.022353433370F, -2.961427640131F, -2.337254458161F, 
    -0.063451447323F,  3.135814603466F, -2.086475279162F, -2.066407283710F, 
     3.007639224657F, -2.936755657030F,  1.336398779371F,  0.384529638356F, 
     0.002963301960F,  2.399337984943F, -0.181657100285F,  1.062959787788F, 
    -2.766996260783F, -1.945065147247F,  1.143363194469F, -0.823621631786F, 
    -2.874989953878F, -0.246766213959F, -2.692687560146F,  3.026220496919F, 
     0.136029972074F, -2.158871355790F, -2.533819976840F,  2.233815670058F, 
     1.998986319111F,  0.909582411063F,  1.995207223290F, -0.777404630466F, 
     1.397629178494F, -1.941983696889F, -2.199960307358F, -0.450799740346F, 
     1.002829380022F, -0.112958721978F,  0.116835469570F, -2.383767537046F, 
     2.971786773162F,  0.562392112719F,  0.936141157905F, -1.720413547503F, 
     1.887032658351F, -0.724959422492F, -0.290297557685F,  0.521418820776F, 
    -0.424796710068F, -1.559448124614F,  2.044006859493F, -1.316700139197F, 
    -2.617136339275F,  0.735703724424F, -2.304854535250F, -1.474783538796F, 
     2.038116192857F,  0.641956106401F,  3.032663581446F, -0.711437566935F, 
     1.446695828572F,  2.613750073042F, -0.980949713858F, -3.134360348348F, 
     0.528223199640F, -0.235938890921F, -2.464459960441F, -0.475329001506F, 
     2.552909402350F, -0.245569714718F,  2.385434703481F,  1.697463637385F, 
     1.996548477839F, -1.115442532694F, -1.503390320754F,  1.789069753217F, 
     0.592857807815F, -0.179968310961F, -3.000141861658F, -2.916888773369F, 
    -0.469609541079F, -2.036541109058F, -1.176721938641F,  1.393346817152F, 
    -2.126954080797F, -0.166592419435F, -2.018371575567F, -2.182017050901F, 
    -0.484523505220F, -0.998243534661F, -2.549532255985F,  0.674746330099F, 
     0.619042467942F, -1.936821681751F, -0.182688285245F,  1.498080017780F, 
     1.231185846289F, -1.615723625583F,  1.255932401753F,  2.622754492824F, 
     0.870414424966F, -1.451028845398F, -2.930453524552F,  1.668185803492F, 
    -2.709271182567F, -1.956194492990F, -1.133488293325F, -1.335188356740F, 
     0.193925994974F, -2.569109877268F,  0.970411001736F,  0.478837660851F, 
    -0.580445703816F,  1.152105236146F,  2.010501317557F,  0.292753173010F, 
     1.371989703642F, -0.466659449502F,  2.944610585928F,  0.907560762018F, 
     0.196876741350F,  0.927508924982F, -1.098642082882F,  1.124795439057F, 
    -2.477904795224F,  0.853173064412F,  0.697173814338F,  2.797111446626F, 
     1.751766149438F, -1.828815818846F, -0.480959494881F,  1.314955719532F, 
    -2.570933118780F, -1.657312163116F, -1.467302897944F, -2.391403903697F, 
    -2.176139023271F,  0.674210543518F, -1.375984265469F, -0.313294089806F, 
    -0.376456174306F, -0.259335371867F,  0.170542876227F,  1.017528885748F, 
    -0.267510599955F,  1.698253975358F,  2.358529313082F, -0.941107977227F, 
     0.113424742018F,  1.017936328032F,  2.787363156495F, -0.526791115635F, 
     0.865251781672F,  2.148404228286F,  2.875775837998F,  2.091778066126F, 
    -1.629185744926F, -1.530326379057F,  1.106609071903F,  0.712894834484F, 
    -1.325346384019F,  0.516786742085F,  1.079502540584F,  0.255972804444F, 
     1.226103919619F,  2.324408059035F, -2.714381489745F, -1.477936964789F, 
    -1.540698885246F, -1.143074295932F, -1.733907623695F, -2.392545600836F, 
     1.054524124436F,  2.763530065736F,  2.163879737800F,  0.914529402260F, 
    -0.977271492027F, -0.129036363258F,  1.762556960417F,  0.875354282648F, 
     1.101643859379F,  0.280959608656F, -3.099399089331F,  0.925585328557F, 
};  

// End of File