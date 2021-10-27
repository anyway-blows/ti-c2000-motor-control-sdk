//#############################################################################
//! \file   input.c
//! \brief  FID Input Vectors (256) 
//! \author Vishal Coelho 
//! \date   15-Apr-2016
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

#include "fpu64/fid.h"
#include <stdint.h>

const int32_t test_dividend[256] = {
#if SIGNED_DIV_TRUNCATED == 1
     -2011386176,    966094086,  -1528326274,    583801231, 
      1244885785,    284555011,   -526503188,   1381201848, 
      -838030456,   -775786639,   1223979313,     16047874, 
     -1026511220,    998474773,  -1447618878,   1808504307, 
     -1193041144,  -1788386941,  -1830937885,   1157715553, 
      1364412440,   1032619810,   1109170094,   1980817356, 
      -144203690,   1232640529,   -332567549,   1905805659, 
     -2141838352,   2067029921,    301481717,   -659403948, 
       246973732,   -859918155,  -1464271078,    709770343, 
       791144929,   1255888648,   -650153090,  -1073420847, 
      -665705391,   -736017410,   1836036382,   1099822722, 
      -909508144,    456039095,   1142587418,   1486712997, 
      1726496072,    411059068,  -1853123148,  -1210994484, 
      1586597389,   -368431511,    692388035,   1217558117, 
     -1082821352,    233754835,  -1161421884,  -2117793553, 
      1145207490,  -2053710555,   -459117253,  -1062831938, 
     -1270378650,    696985004,   1781303669,  -2117837431, 
      1058379586,   1287076260,   1751483114,   2038279531, 
     -1632673874,     81474513,   1382871286,    588508439, 
      1949530223,   1919534156,   2004054268,  -1858242561, 
      -268286019,   -769486587,  -1571734441,  -1569444829, 
      1314035195,    106361705,   1908069159,   2097404658, 
      -387001551,   -553243855,  -1173152740,   -231795267, 
     -1004071280,   -175679651,   -288169422,  -1032409489, 
     -1573184208,   -346911301,     29465671,   -754520291, 
       793232357,   -244411510,   -276324968,   1258526416, 
      1355304287,   1082815602,   1242344807,      5443669, 
       237004553,    561565850,  -1726623390,  -1092312481, 
       497035701,   -837690604,   1146617122,   -999743436, 
     -1977805363,   -873760513,    242085724,   2014659512, 
       812284448,    935793480,    253528138,    143255231, 
      1613722302,   -459138049,   -180101130,  -1253077303, 
      1104979820,    200540733,   -612513048,    863278933, 
     -1678376597,  -2119090624,    417909250,    683656000, 
       343625002,   1760728758,    584141975,    109784738, 
     -1032411010,  -1927706446,    996218218,  -1441857497, 
      -943187679,  -1033251244,    202275606,    177246718, 
      1237436702,   1587443772,   1234977173,   2016177288, 
     -1372379019,   1849408462,  -1953557817,  -1113917441, 
     -2109451583,    736988617,   1738659583,    311025689, 
     -1479782490,     10168050,    290910496,  -1338862529, 
      -755070440,    927873321,    227327072,  -1536388229, 
      -513834600,   -444231710,    329603073,  -2064154828, 
       333203825,   1856268083,  -1688468552,    997075621, 
      2020879981,    467659827,    943456620,   -847172311, 
      -175997478,  -1941202325,   -492408959,   -593923625, 
      -912319536,   1360270842,   -212480215,   1316976646, 
      1246283265,   -932188837,  -1854097702,  -1911557967, 
       590655238,   -325190953,   1741743877,   -355104101, 
     -1485808170,    171795529,   1877293130,    691296739, 
      -452442844,  -1035127740,   1494271348,   1911501286, 
      -528281677,  -1858517456,  -1367595890,    325333530, 
     -1349113517,   -895722491,   -164651407,   -657226887, 
      -780972301,   -172165570,  -1134322681,  -2028289554, 
       680552411,  -1465427540,   1299925970,   -392646891, 
      -741346236,   1056625213,   1058073440,  -1400352766, 
     -1642632568,  -1399998173,    547218192,   1468407466, 
        43305531,  -1435533789,    920434179,   1748211875, 
     -1208871197,   1593267286,  -1237655198,   1445964907, 
      1543189004,    100405481,    -97226059,   1674535303, 
     -1867983496,     40613748,    518823232,   1003164515, 
     -1159657358,  -2053544657,  -1550530912,   1157525670, 
      2017626088,   -486214281,   2119256219,   -745634379, 
     -1558381483,   -494979390,    269018852,    574818780, 
#elif SIGNED_DIV_MODULO == 1
      -168001697,   -582946045,   -943753676,  -1820188098, 
      -237833939,  -1435780524,   -434869848,   1806397406, 
        48680815,   1778727764,  -1752630387,   2117578538, 
     -1733304392,   -802525405,   1225676768,    439805758, 
      -146422507,   -867019267,  -1575510894,   -880429836, 
     -1431825776,   -785532592,  -1675725814,   1426306933, 
      2025475455,  -1210015671,    885096674,  -1979921115, 
       499483753,    727413573,  -1987704282,  -2133161814, 
     -1535603906,   1556519593,   -961911974,    136114114, 
        95439765,    290415874,   -717119722,   -371873077, 
      -367871884,   2078428236,  -1899495252,   -444344922, 
      1251212372,    404523482,   -817920290,   1725730341, 
     -1747506272,   -777272532,   1661987479,    676163888, 
       792494009,   -111889804,  -1540844607,   1936707618, 
      1643366686,   -268699399,   1438628505,   -751003376, 
      -568484879,   1266325863,  -1720955960,   1940387014, 
     -2141153197,   -878768137,  -1939372930,   -245928179, 
      1244880673,   1776057519,    142826388,   1305998384, 
       269124182,   1077506156,  -2107839578,    -99708303, 
     -1072326519,   -824984799,   2005531131,  -1250675748, 
        87944835,  -1178770399,    288612670,   2139596052, 
     -1581126339,   1952821061,  -1615433788,  -1347596645, 
       629224350,  -1597001652,  -1798214748,    683874990, 
     -2029806479,   2083831376,    168925131,   -541874907, 
       887919574,   1921617220,   -505614937,    828535934, 
       438392948,   1182222871,    394424909,   -531803677, 
      1506026844,  -1177958386,   1275426119,   2134087671, 
      -939262875,    903579407,    707132293,   -365778600, 
        -7430714,   1928968705,   1946400476,   1000398005, 
      -495295609,  -1975329639,    356197039,    277938679, 
      -621985922,   1632950091,    534852848,    532642715, 
      -877283959,  -1826735804,   -886026008,  -1139283847, 
      -661865710,   1496754694,  -1458723873,  -1469461315, 
        37191279,    443674416,  -1454288947,    581771895, 
      1477213524,   1212337033,  -1011179809,   -795861897, 
     -1360664660,   -225600577,   -744409674,   -945681834, 
      1854411865,   -430817806,   -517873598,    398772464, 
     -1853249672,  -1265994602,    960433191,    322756539, 
     -1287491593,   1475236849,   -327577152,    192203920, 
       120118998,  -1352451612,  -1796628998,   -154216715, 
     -2016251886,   -279240917,    248528116,    596064706, 
     -2000520977,    901515556,  -1420242827,    401074668, 
       464135294,   1169775518,  -1905790309,   1523532904, 
      -496995038,   -431133280,   -749826191,    237898713, 
      -878663966,   -575010213,   -648332492,    559308059, 
       706273358,   2113537671,   1908517088,   -642751039, 
     -1318563787,   1802065792,   -907560733,    218439577, 
      1800983526,  -1760728190,  -1040686458,   -313326636, 
       333818988,   1715931324,  -1210211760,   2005886669, 
      -283547823,   1223302992,    108378537,   -724660400, 
      -293803050,    935880249,   1787617263,   1675157449, 
     -1568925533,  -1632462285,   1689865280,    657497395, 
     -1974490626,     20254909,   1694164733,   -490792508, 
      -893130384,  -1142304234,  -1284417025,   -514081544, 
       407120560,   -994774663,    525887142,   1308106635, 
     -1700930493,    984579623,    638190326,   -108857564, 
      1859343354,  -1733261199,    425660973,  -1144330861, 
     -2008634493,    342992926,   1469729872,    244370703, 
      1459786326,  -1267220336,    520785773,  -1400069628, 
      -903919596,  -2068014146,    865420405,   1941717656, 
      1069486950,   1102635741,    180771202,   -936032834, 
     -1095754189,   -917756266,   1989147145,  -1156654683, 
       160294250,  -1266979910,   -283290483,  -1536687877, 
      -534363618,   1260833291,   1343532989,   1734268737, 
#else //SIGNED_DIV_EUCLIDEAN == 1
       574370748,   1287268571,  -1251922723,   1701907385, 
      1055314141,    156275851,   2018485138,    278671093, 
     -1212512652,   1536581616,   1554775230,   -801860968, 
      -743097154,   1460346450,    -32071439,  -1924564176, 
      1194079802,   -314939705,   -945056524,   -715077645, 
      -571488555,   1266123760,  -1981307989,    973537382, 
      1600667564,   -919918505,    673662439,  -1151538177, 
       523788190,  -1824829524,   2004701220,    472317367, 
      -499439500,  -2016187471,   1535732501,    444714594, 
      1493963163,     19826629,  -2113506859,   1799905591, 
      -383519983,    998003287,  -1498491417,   1353916613, 
      1759229478,   1193753780,   1104335461,   -407173536, 
       867158536,    281152584,    363939860,   -666086725, 
       877175084,  -1456579408,  -2140981354,    266291632, 
       199647796,    218303574,    835529025,   1836073144, 
      1907868748,   1732002810,    447133095,   1830556967, 
     -1645203293,   -683752318,  -1989727594,    164797252, 
      1018889942,  -1369138926,   -316768177,  -1725855483, 
      -845156936,   1203117456,    158743781,   1155653004, 
       596758413,   1688558857,  -1886820419,  -1392550608, 
      -359351930,   1029898114,   1687727982,  -2036444542, 
     -1556654940,   -326041995,   1136305675,    104956921, 
      1092989113,  -1418204887,    741856841,    509609649, 
     -2118079918,   1031760769,   2111987795,  -1596948911, 
      -598792485,  -1326830222,    920830941,  -1383550652, 
      2090999458,  -2074472941,  -1976559113,   1307089681, 
      1545296099,    284552440,   1091405653,     83815283, 
       368956899,  -1328291539,     14780973,  -1928806966, 
     -1906554266,   -707821778,    562231649,   1683531895, 
       744732839,    795715879,    840717484,   1287766199, 
       689793940,     85627025,   -722686613,   1864757601, 
     -1088873217,     48130997,   1032157462,  -1055838111, 
      1484275235,    163871994,   1761521282,   -641211086, 
      1869706133,   1837041088,    376670179,  -1710418609, 
      -571763486,   -962736097,  -1004603582,  -1174507558, 
     -2141810941,   1687953807,   -199726686,    336815653, 
      -791968181,   2121786321,   2078304109,   1995217999, 
       714103764,    971341971,   -712781801,     97808286, 
      -973435588,    937930785,   1194076311,  -1799315476, 
     -1195804192,  -1271484292,    533002460,    967248605, 
      1436083250,  -2066328601,  -1279395031,   -132744718, 
      -522288719,   -685503694,  -1873244897,   1122429289, 
      -418063159,    748568631,    219368285,  -1926476800, 
      -826798653,   1999070051,   1853171657,   -523871221, 
       505389626,    268578870,   1419747436,   1965801113, 
     -1823189337,   1666552093,    268209075,  -1312734741, 
     -1196351154,    883745568,    415587202,    370638811, 
      2012167044,    351085338,  -1718844661,  -1431929977, 
     -1708994138,  -1519357116,    736865062,    600664841, 
      -547885564,  -1447949585,   -474513106,   1288389542, 
      -430382782,   1095728126,   -879564585,    601320807, 
      1654065582,  -1247072433,    -84368242,  -1662205776, 
     -1578960373,  -1872596046,  -1806903928,    545263982, 
      -380817716,    595238168,   1531400493,   1132046146, 
      2045092899,   1209193259,   1870815613,   1027150736, 
     -1058195041,    941672802,    831130355,   1173895954, 
      1061789297,   -780036212,     44800620,   1172811585, 
       312740302,   1949272346,  -1410586212,   1749061414, 
      1083274171,   -918370387,    547223254,   -159652855, 
     -1591546323,    212105510,   2016527187,   -248504705, 
       390687103,  -1670870066,  -1289124592,  -1447558155, 
     -1989096304,   -976140136,  -1159067406,   -607545936, 
     -1570486150,   2141447488,     58071628,   -481696454, 
     -1074374224,   -580883906,   -433544560,   1830947820, 
#endif
}; 

const int16_t test_divisor[256] = {
#if SIGNED_DIV_TRUNCATED == 1
            2726,       -12125,       -22327,       -22766, 
          -23793,        13749,        -2303,       -25345, 
           13164,       -20973,        19900,          914, 
            3174,       -19147,        18650,         1735, 
            4654,        -5110,        14493,       -27975, 
            6216,        23722,        -3356,         9999, 
          -12880,         7041,       -14490,        19631, 
           19409,        29758,        -3648,        -2826, 
            6541,        22453,       -30723,       -20495, 
           29070,        29354,        -3082,        20370, 
           28106,        11319,        -8367,        -6181, 
           -4010,        11707,        -2290,        29704, 
           -9523,       -10550,        25942,         2977, 
           16336,       -24584,        -3065,       -27869, 
           10704,        13346,        27455,        10490, 
           12458,        23181,        -2104,        -2722, 
           20060,        21283,       -20288,       -31086, 
          -29045,       -23401,       -21534,         8247, 
          -30834,        -1814,        11694,       -25245, 
          -17298,       -13823,       -21446,       -11554, 
           19733,       -13132,        18063,         3458, 
            3584,        15115,        17932,        26269, 
          -23713,        19272,       -20355,       -30869, 
          -24420,       -24004,       -24359,        28524, 
          -14862,        29010,         9053,        24413, 
           -8712,       -17289,       -20492,         2991, 
          -16049,       -12726,       -31749,         5733, 
           30313,        22927,       -32248,         8783, 
           -9222,       -25293,         2675,        -5478, 
            1120,        25304,       -22977,        -4281, 
          -28899,        -7797,        14572,       -26534, 
           10956,       -13343,         6459,       -22816, 
           -4170,       -31937,       -17759,       -15488, 
             746,       -18672,       -10086,        16240, 
           -5661,       -29114,        -7208,        -1673, 
           21319,       -12869,        21088,         4304, 
          -29204,       -15728,         5839,        -1329, 
          -19749,       -17104,        18362,         7687, 
          -23323,        14164,        -6455,        -2466, 
           13584,        -6475,       -31825,       -27876, 
            5968,        -3539,        27958,       -26549, 
           -8165,         3014,       -25449,        26506, 
            8734,        26568,         8555,       -31836, 
          -12028,       -25437,         8484,       -28789, 
           11402,        -1479,       -12744,         1070, 
           13567,        20552,       -12072,       -12367, 
          -10160,        10898,        23662,        17155, 
           24630,        24328,       -21444,        22951, 
           30119,        17708,        24576,       -28350, 
            9620,       -11528,         9197,        24886, 
           -8282,        17480,       -21752,         1292, 
            8352,        14018,       -12688,       -15487, 
           27262,         7538,       -26662,         8368, 
          -20183,        18150,        23887,       -10907, 
          -23894,        17397,       -11888,       -16228, 
          -19656,       -28245,         3401,        -6304, 
           16391,         -840,        -7551,       -28744, 
          -18764,         2875,        -5856,        26277, 
          -29079,        -3703,         2478,       -23982, 
            2683,        23420,       -19791,       -22570, 
          -28746,        10555,       -31549,       -13691, 
           31052,        17343,       -16798,        11934, 
          -23734,         8507,        23396,        26200, 
           -9938,         -898,        11764,        13377,
#elif SIGNED_DIV_MODULO == 1
            2646,        20831,        13658,       -29936, 
          -23203,       -17476,       -16600,       -21609, 
          -17364,       -14716,        29596,       -10049, 
          -13283,        -6264,       -12961,        16862, 
           -9192,       -24583,         7679,        -9469, 
           -8983,       -28280,        24061,        -2760, 
          -27681,        26536,       -14307,         7462, 
           10607,       -19659,        30145,        10820, 
            2706,        24180,         3738,       -31366, 
           -1136,        20183,        15466,         4738, 
          -32179,        14305,        -3315,        10460, 
           16594,        19971,       -30857,        18337, 
            4413,       -27780,       -16278,       -24022, 
            4225,         2685,       -28251,        32009, 
          -16312,       -12093,       -13064,       -30013, 
            1828,       -15990,        -5983,        29327, 
           27477,       -24822,         6025,        -9198, 
           14372,         1543,       -15674,         -454, 
           23319,        14706,       -19719,       -22460, 
           -8489,        23741,        12108,         8794, 
          -23506,       -27571,        24650,        -5215, 
            -809,        -2601,         1027,       -14943, 
          -17591,        26183,        26783,         6792, 
           -8832,         6461,        11041,        25857, 
          -27044,         2556,        -4690,         7677, 
            3858,       -17967,       -25918,       -32114, 
          -28891,       -11623,        18315,       -10782, 
            7835,        32301,         9699,         2606, 
          -17547,        15717,        25492,        23579, 
            6360,        10141,        27197,        -4379, 
          -13779,         8642,       -13408,         7997, 
          -29653,        32414,       -19218,         7036, 
           -9986,        14269,       -30933,       -28388, 
           27987,       -27016,       -10984,         1715, 
          -16604,         2811,        18406,         1434, 
           28307,       -23127,        -5455,       -14399, 
            6429,       -30378,       -28594,       -11608, 
          -26321,       -21624,        -8444,       -30162, 
           13712,         9262,       -21361,       -28695, 
           -6118,        -2421,       -19486,        24218, 
            6418,       -31260,        26176,        -3084, 
          -28965,       -25804,        32664,        24007, 
            7550,       -31002,       -11631,        -2374, 
          -26279,         4652,       -11412,        -3245, 
            5101,       -27863,       -29010,       -13045, 
            1423,         4055,       -16938,        27047, 
           21346,        -3635,        31591,         5129, 
          -17405,        20354,        -3194,       -16386, 
           29847,       -23419,          823,        30927, 
            9720,         7514,        -1989,         5097, 
           26955,        -8112,       -17776,        -5012, 
          -14838,        -3633,         8356,         2270, 
           -7508,        24474,       -13085,        -6552, 
            1161,       -28716,       -17605,       -25003, 
          -26294,        25576,       -30580,        22218, 
             476,       -25316,         -627,         6516, 
          -26856,        31340,        10028,        -2548, 
           23838,       -15543,        21230,       -11209, 
           28920,       -16770,        29956,          709, 
            4232,        32355,        17758,       -12203, 
          -28975,       -29880,        20508,        -5763, 
           -7592,         1515,        25699,        -6166, 
            6841,       -26555,       -10749,       -23276, 
          -16720,        -7932,       -15053,       -18639,
#else //SIGNED_DIV_EUCLIDEAN == 1
            -297,         7155,       -32453,         5749, 
           18183,        10300,         1855,        21349, 
           30305,       -12207,        19469,       -14079, 
          -31792,       -26598,       -11225,       -12743, 
          -31587,       -22138,        -3671,        17482, 
           11878,        14059,        -2636,        27476, 
           32042,        28350,        -2522,        26536, 
           -7442,         6751,         3953,        22664, 
          -14103,        10703,         6701,        10253, 
          -12456,       -11034,       -20434,       -26170, 
          -13988,        -9514,         2343,        32166, 
          -30921,        13731,        26551,        23974, 
          -24958,        29836,        -3874,        24625, 
           23917,        -9493,         8594,        23890, 
          -31392,       -27734,        -8080,       -22991, 
          -30533,        18497,       -11322,        20805, 
          -21390,        11541,        24614,        16852, 
          -17719,        -9245,        -8968,       -15241, 
          -10663,       -27069,        -3175,        -2972, 
          -30865,         8984,       -28871,       -21680, 
           12102,         3576,       -32372,       -13888, 
           -8087,       -23168,       -27883,        -2573, 
           -8494,        21269,         2459,        20808, 
           -2597,       -31848,       -32441,       -21874, 
           -8786,        14300,       -22320,       -14499, 
            9498,       -13946,       -11670,       -22583, 
           -7362,        25934,        25412,        -6973, 
           11498,       -16239,        29489,         8171, 
          -19206,       -25582,         4315,       -14790, 
          -28104,       -22384,       -29523,        11854, 
           18464,        20143,       -15398,        25945, 
            7076,       -31824,       -10222,         2206, 
            8372,        -3491,        20388,       -23277, 
           31117,        21872,       -10479,         7617, 
          -12861,       -26921,         1410,        21317, 
           17296,        29336,       -10909,        -7226, 
          -22911,       -10899,         3517,         3296, 
          -22255,       -25095,        -6649,        21717, 
          -20644,           51,       -24490,        23895, 
           17474,         4217,        -7238,         -647, 
           25355,        26544,         -106,         1915, 
           26849,         5150,        18073,        10576, 
           -1989,       -18366,         6723,       -20693, 
          -19824,        23723,       -24534,         9540, 
           -4108,         7379,        15563,       -12893, 
          -29952,        21987,        -8565,        10573, 
           25933,       -14805,        32629,        21911, 
           19093,        10263,         2862,        -7427, 
           21117,         6247,        18457,        31959, 
           18890,        23128,         -943,        24494, 
          -10889,       -19313,         -370,       -30723, 
           21467,       -15450,        11632,        19258, 
           11917,         9880,       -17214,        -1481, 
           28600,       -16967,       -19064,       -14912, 
           18075,       -11049,         6755,       -20709, 
          -27036,       -12523,       -17637,        26816, 
           28631,       -30679,         6137,       -29896, 
           -4922,         1414,        22304,         8193, 
          -16043,        26521,        17517,         4106, 
           26027,        -8646,       -10635,         7789, 
          -27073,        -8160,       -20488,        20350, 
           14630,        11447,        27130,        21034, 
            3193,       -31749,       -26563,          967, 
           29653,        22482,        28508,        31025, 
#endif
}; 

// End of File
