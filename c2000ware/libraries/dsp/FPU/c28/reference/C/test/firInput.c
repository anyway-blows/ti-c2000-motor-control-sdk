//#############################################################################
//! \file firInput.c
//! \brief  Input Vector (512) 
//! \author Vishal Coelho 
//! \date   22-Jan-2016
//! 
//
//  Group:			C2000
//  Target Family:	$DEVICE$
//
//#############################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//#############################################################################


long double test_input[571] = {
	 8.506175670838, 11.478202551275,  7.262634956366,  2.067279320260, 
	 1.643755060659,  3.197307665970,  1.498297487314, -2.800755352218, 
	-5.467778824683, -5.149267582677, -3.770458657576, -3.430550569904, 
	-5.105138594285, -7.655210256168, -7.494464720085, -2.854021883573, 
	 1.986465726369,  1.682064869212, -1.552647446242, -0.704561794106, 
	 4.662182733463,  7.398064363860,  4.510645950188,  1.938025007615, 
	 4.501923648327,  8.285107030532,  7.317409083212,  2.254971429925, 
	-1.970886242832, -3.035716073850, -2.174624070013, -1.443914871747, 
	-2.402735126479, -4.740039773263, -5.432270679886, -2.672195815317, 
	-0.129623558814, -2.334530910237, -6.526825356245, -5.710216949903, 
	 0.037547905371,  2.937860270705, -0.286606507320, -3.390794300940, 
	-0.786802891253,  4.379244411920,  6.394210856744,  5.381822157884, 
	 5.099491334390,  6.570124062021,  7.742191822450,  6.611794092811, 
	 2.491373368083, -3.066554017348, -6.025808204523, -4.220842304665, 
	-1.657235578799, -3.562647508684, -7.755414551766, -7.362942868322, 
	-2.132721780990,  0.627577198335, -2.155088096199, -4.523028306983, 
	-1.547202261220,  3.007044625667,  3.305115560565,  0.061647064216, 
	-1.853337824055, -0.308089063797,  3.162382557843,  6.149690509376, 
	 6.762957833906,  5.073699143470,  4.011096952572,  5.453784692474, 
	 5.952282589861,  1.296023185266, -5.406399707319, -6.853449146673, 
	-2.917587916542, -1.299711753376, -5.320058091007, -8.859474965333, 
	-6.482644021803, -1.456669743388,  0.459314553426, -0.565825052411, 
	-0.653937028605,  1.371614411034,  3.588766035522,  4.043692888175, 
	 1.983030123537, -1.228636568884, -1.837725587658,  1.999368765085, 
	 5.985307729239,  4.696652643473,  0.259949768656, -0.355238744023, 
	 3.327291770415,  4.294327147737, -0.234133006352, -4.078303862381, 
	-2.206512534356,  1.584507176150,  1.318141729566, -2.488460663986, 
	-5.146191011958, -4.618466217587, -2.402903152791, -0.727513994618, 
	-1.190549391883, -3.381498384594, -4.071409245688, -1.205305466982, 
	 1.758956441750,  0.395775983109, -2.583347948262, -0.388930169163, 
	 6.574840244237, 10.157931940482,  6.759899054502,  2.439923445637, 
	 2.799127448145,  4.921754631690,  3.510067001415, -0.788230565067, 
	-3.687210721607, -3.738883602585, -2.760153622964, -2.756870564224, 
	-4.650047953282, -7.297764242118, -7.161336765058, -2.555078055968, 
	 2.147729322997,  1.527237131177, -2.232285675735, -2.087468695938, 
	 2.489940523070,  4.490931240555,  1.084588396391, -1.643031203698, 
	 1.229714511069,  5.810898189360,  6.068660528150,  2.515556225785, 
	-0.120427838464,  0.257090156178,  2.201061327106,  3.497711336071, 
	 2.512879167718, -0.422995218675, -2.178859118614, -0.774807206779, 
	 0.322811616396, -3.220139119929, -8.489655027708, -8.400713454201, 
	-3.015391245276, -0.161450420889, -3.208698376626, -6.018889418959, 
	-3.096018629092,  2.359279934226,  4.627241874740,  3.869219590941, 
	 3.908083842215,  5.835276247571,  7.644304738367,  7.331380124019, 
	 4.153630525751, -0.443567658747, -2.565505960604, -0.194634337473, 
	 2.540906256897,  0.345122456767, -4.595596596168, -5.326676709288, 
	-1.448258065416, -0.081097697496, -4.106154910341, -7.404171556025, 
	-4.944365272014, -0.466837491538,  0.142012374120, -2.516567131061, 
	-3.719723308193, -1.482906293576,  2.543685324501,  5.891637646657, 
	 6.674386197624,  5.025493112215,  3.973380319273,  5.503701509975, 
	 6.250617342401,  2.038429352056, -4.051615283093, -4.805635672861, 
	-0.226975367567,  1.837901223052, -2.057954134099, -5.872382560778, 
	-4.176355330540, -0.166482227708,  0.534850241330, -1.725611316790, 
	-2.885079629833, -1.611728220237,  0.267985009068,  0.818727591564, 
	-0.772649085383, -3.264308419350, -3.060144098057,  1.526023321308, 
	 6.073002568558,  5.091930303132,  0.713156088827, -0.026126617667, 
	 3.461696358262,  4.288762842576, -0.218211574497, -3.819084600682, 
	-1.484636988897,  2.921720282731,  3.306381610095,  0.045168113824, 
	-2.306434894037, -1.807045323657,  0.011302433887,  0.955880326704, 
	-0.473071993210, -3.723679043269, -5.412179486562, -3.347081779755, 
	-0.894821954117, -2.450506179694, -5.334519069681, -2.838697154837, 
	 4.525964074117,  8.506146339992,  5.428551239781,  1.328139487311, 
	 1.835380572469,  4.107749036069,  2.942491776601, -0.925279196009, 
	-3.159222372126, -2.320078346794, -0.301660776634,  0.752135520695, 
	-0.256452782576, -2.367405284105, -2.191367441745,  1.874365445427, 
	 5.462198552437,  3.252331799330, -2.389204082902, -4.184350535133, 
	-1.356526241364, -0.694684975950, -4.874103546973, -7.740790682361, 
	-4.399018694299,  1.151091919427,  2.713311170996,  0.614113810367, 
	-0.591180218503,  1.062824961045,  4.055561633103,  6.157979825600, 
	 5.765712161398,  3.261241062575,  1.823900395290,  3.455948904899, 
	 4.675056280935,  1.093502983807, -4.451982725902, -4.953591121597, 
	-0.522855511785,  1.015294757651, -3.639050672186, -8.210175940088, 
	-7.012094997816, -3.031298822292, -1.785452490871, -2.959807219481, 
	-2.656173761816,  0.198379396970,  3.486407328271,  5.019130671594, 
	 3.826340571320,  1.122176123532,  0.603535215119,  4.152314062259, 
	 7.581571786611,  5.611809673321,  0.504812321609, -0.677109238841, 
	 2.576038396070,  3.229984060860, -1.549453604951, -5.638550695397, 
	-4.033450744412, -0.511448780144, -0.980977564602, -4.824983961261, 
	-7.253550241347, -6.164001010457, -3.052299077683, -0.223747247132, 
	 0.573133247351, -0.448300942951, -0.262814275272,  3.018939011322, 
	 5.849301327798,  3.809999269898, -0.283485577516,  0.536981267545, 
	 6.082697250488,  8.411774579667,  4.084753112146, -0.751701129056, 
	-0.490164980638,  1.889820672303,  0.981839999247, -2.688607844346, 
	-4.942562733348, -4.404731192352, -2.924758027252, -2.508272369769, 
	-4.058380014064, -6.414623759987, -6.031622395692, -1.232515331977, 
	 3.590823790958,  3.002427175110, -0.813917016333, -0.791729949082, 
	 3.642146293549,  5.532741132044,  2.091777147922, -0.585370027114, 
	 2.383785995299,  7.018488637727,  7.164829681422,  3.212678531637, 
	-0.191604211091, -0.952516500203, -0.411985476537, -0.572058070648, 
	-2.777354421027, -6.385481662556, -8.002556026003, -5.506422571437, 
	-2.394929361041, -3.226715207239, -5.481925851355, -2.574466521789, 
	 4.924829510305,  8.774938799436,  5.380328440824,  0.897786492043, 
	 1.092643809718,  3.236880506082,  2.193865922743, -1.299783897658, 
	-2.971611889010, -1.496847547750,  1.092438283584,  2.522935941404, 
	 1.607090239767, -0.723476873601, -1.038759954432,  2.365334097446, 
	 5.260883430524,  2.474543971543, -3.507769577671, -5.342243159543, 
	-2.256068754947, -1.111141083218, -4.708699766140, -7.045379949796, 
	-3.366533958333,  2.228538756177,  3.509524623318,  0.842697301038, 
	-1.110564307703, -0.234887778158,  2.110980481272,  3.837003674748, 
	 3.425472245570,  1.275211756394,  0.509036145722,  3.013005864161, 
	 5.154614519860,  2.394217882236, -2.553732185716, -2.750524022911, 
	 1.686955132336,  2.987759090690, -2.052573365180, -7.048110362677, 
	-6.216762492904, -2.487010957030, -1.369853091726, -2.594523892111, 
	-2.343026570525,  0.365815447365,  3.340000854572,  4.359066822629, 
	 2.479968219780, -0.995314315966, -2.234857131032,  0.799057370301, 
	 4.062373360579,  2.371922503607, -1.986552054434, -2.011002697478, 
	 2.669409342282,  4.824361303834,  1.401187017276, -1.677363164125, 
	 0.446262090351,  3.929764031031,  2.891610033914, -1.939840536161, 
	-5.602746178887, -5.796700836636, -3.832420371139, -1.876081939376, 
	-1.605262319749, -2.810117849719, -2.533272473153,  1.005850281722, 
	 4.140715066610,  2.354779526870, -1.590706414722, -0.726970868081, 
	 4.808810820132,  7.159726353845,  2.978387573878, -1.517979110275, 
	-0.696593574247,  2.430375212433,  2.367425228448, -0.489659242155, 
	-2.105553918892, -1.230994128907,  0.205011069076,  0.183889655404, 
	-2.138304901284, -5.479844037571, -6.135642727364, -2.262944355649, 
	 1.884785714196,  0.954873705978, -2.856131877924, -2.539334592074, 
	 2.366168031010,  4.765981928027,  1.738251333031, -0.718426872913, 
	 2.240274383743,  6.661779499775,  6.478609341097,  2.203336530263, 
	-1.389420944093, -2.102651749950, -1.229859100268, -0.791928091288, 
	-2.218180678759, -4.999976339552, -5.897718231662, -2.930279257372, 
	 0.308119239445, -0.771056648171, -3.604145900900, -1.496646725811, 
	 5.130171113166,  8.193469305935,  4.230227454058, -0.524232698925, 
	-0.295943443164,  2.125878764499,  1.489581689989, -1.608775727441, 
	-3.028722612080, -1.536779723311,  0.807162110180,  1.773613863656, 
	 0.282084756942, -2.588671435803, -3.254276228835,  0.115815202663, 
	 3.362559524977,  1.306390847928, -3.649600547067, -4.306180281641, 
	-0.068977919081,  2.022609735552, -0.975133345969, -3.140227261035, 
	 0.274323267453,  5.232224096354,  5.619316961619,  1.943739517810, 
	-0.996308685875, -0.980563656341,  0.688222624972,  1.923585852716, 
	 0.000359976670, -0.000434091233, -0.000941633220, -0.001662963303, 
	-0.002401690879, -0.002895125406, -0.002849792236, -0.002018453018, 
	-0.000295152412,  0.002199321830,  0.005070891835,  0.007672335634, 
	 0.009204313082,  0.008887369916,  0.006181062702,  0.000998827742, 
	-0.006135795740, -0.014043791726, -0.020990186600, -0.024933498021, 
	-0.023892270448, -0.016363317305, -0.001706184865,  0.019595921893, 
	 0.045863956323,  0.074386051109,  0.101782198366,  0.124539463534, 
	 0.139593998881,  0.144858856161,  0.139593998881,  0.124539463534, 
	 0.101782198366,  0.074386051109,  0.045863956323,  0.019595921893, 
	-0.001706184865, -0.016363317305, -0.023892270448, -0.024933498021, 
	-0.020990186600, -0.014043791726, -0.006135795740,  0.000998827742, 
	 0.006181062702,  0.008887369916,  0.009204313082,  0.007672335634, 
	 0.005070891835,  0.002199321830, -0.000295152412, -0.002018453018, 
	-0.002849792236, -0.002895125406, -0.002401690879, -0.001662963303, 
	-0.000941633220, -0.000434091233,  0.000359976670, }; 


// End of File
