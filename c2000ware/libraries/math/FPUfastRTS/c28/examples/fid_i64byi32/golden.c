//#############################################################################
//! \file   golden.c
//! \brief  FID Ouput Vector (256) 
//! \author Vishal Coelho 
//! \date   11-Apr-2016
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

const int64_t test_quotient[256] = {
#if SIGNED_DIV_TRUNCATED == 1
             -7163019628L,          -6391907001L,           2496768298L,           2639968280L, 
             -3375049579L,           4816299215L,          -2874806593L,           3155246019L, 
             -1328501109L,            798505279L,          -9520261848L,           5504586222L, 
             -4963432016L,           4351452095L,          -8193467005L,           4205157584L, 
             -8418966266L,           1235179654L,           3770416793L,          -4204922913L, 
             -5638903997L,            865838080L,          -8139024361L,         -53629692405L, 
                 4711820L,           -399250681L,           7043278491L,           4063923684L, 
             -3826774157L,           1306270640L,          13211866340L,           5088732322L, 
              3787116388L,          -3445763918L,        -137212003723L,           9796735813L, 
             -4408744584L,           3159669844L,           -711523608L,           1797482175L, 
             81315177796L,           6197427515L,          -3480071359L,         637724376243L, 
             -3905769623L,          -3506722227L,           4875516170L,         206163575554L, 
              1301538395L,          -5109036383L,           1083825416L,          -2796202652L, 
            -20120158609L,          -3223619357L,          -9878088475L,           4882126411L, 
               167971862L,          -6098718954L,          -8128572385L,           6686792079L, 
              1139336070L,          22529370152L,           1930279456L,          -1978904224L, 
             -3452101700L,          -9831809007L,          12946663179L,          -4534221624L, 
              4352572312L,           1253201044L,           -781470427L,           5945313133L, 
              3517890837L,           1670338792L,           -276070995L,        -217915683673L, 
            -20877697669L,           -528983762L,         -26833963206L,           7938058865L, 
              -296546980L,         -95019321694L,           3424451065L,          72314085643L, 
             -5957332511L,          18718809681L,           1745572141L,           5630077014L, 
              2165668454L,          12139102533L,           2314739110L,         -84372741405L, 
            -12258120653L,           -998412323L,          44427465507L,           1456893727L, 
              3838987130L,           5339784215L,          -5462079449L,           4664260884L, 
            -13995590331L,            299995782L,          -1891159820L,          36372336932L, 
            -26017651414L,           3223376831L,           1701195887L,           1207207616L, 
              1891041003L,           4803989756L,            253702603L,           2358754617L, 
             -4954913201L,           1343446410L,           -642739646L,           -679006859L, 
             -3540544864L,           2270812781L,          -3506615764L,         -10300556298L, 
             -5150124650L,           2758920427L,         -16389555965L,           2597135413L, 
              2046371487L,          11960935977L,           5517816827L,          -5947610948L, 
              -156946118L,          -3266941365L,           1236091718L,           7516147376L, 
            -11862252629L,          -2718265466L,          -5324896663L,          -5584715627L, 
             -3390438669L,         -14209968120L,           -354367814L,           2553017424L, 
              4865794044L,           6100641211L,           9014038163L,         -36406024149L, 
             -2519294672L,         -23473808410L,           4557092972L,          23044209591L, 
             -2454984961L,         -53490210332L,          -2662277678L,           3649002230L, 
             57370234405L,          -5532219674L,          -1806022717L,           4121509189L, 
             12646768657L,           -584253204L,           2441586585L,          23281983256L, 
              1884095105L,           6398375370L,           6765728792L,          25007850361L, 
            -19106807555L,           3403012642L,           1546728213L,           5525131683L, 
               482080517L,          -5364305119L,          -5109881742L,           4191976187L, 
             11004623894L,          41327072195L,          -3593824488L,           1776432469L, 
             -5507893351L,           5643257323L,         -15500204125L,           2220894577L, 
            -13633645339L,           2359097134L,           6191562411L,          40353444760L, 
             -1229390954L,            739721728L,          -3559504237L,           2209830005L, 
             -2950864519L,            792718624L,           -974394071L,           -749708683L, 
             -3426150799L,         395725375101L,         -23270498266L,          15773558395L, 
              2984223367L,          -6022322341L,          34523854758L,           7859822922L, 
             -7927925277L,          11213548096L,         -22480152194L,            941728063L, 
             -5343056969L,           3471378126L,           2180761364L,           5798486036L, 
                63406147L,          -1338709056L,          -4061674959L,           -691537804L, 
              1960224412L,         -26927709824L,          12527365651L,         -18046019254L, 
              4767070464L,           2876818211L,           5062502558L,          -4529232296L, 
            -11194089708L,           4326326210L,         -92454455181L,           4786575460L, 
                13039457L,           2530312664L,           2791569327L,         -17163121392L, 
             76624128440L,            266296509L,           4825258238L,          -6381699076L, 
             12968845314L,          17004576219L,          -2888041081L,         -11327124728L, 
            -30037745121L,          -7889348519L,          59697076133L,          -5706135805L, 
            -23275339512L,        -223478453721L,         131886608083L,          -4703206041L, 
             -1221298453L,           4266869301L,          -4763068758L,           5858871948L, 
              3431397454L,          -8268045083L,          -3019865152L,          -2513741523L, 
              4710588866L,         -14506272259L,           1771384430L,           6933399438L,
#elif SIGNED_DIV_MODULO == 1
              6986147365L,            385363112L,           5085319311L,           1619088797L, 
             10460151556L,            427990904L,          -3013848951L,          -1181007421L, 
               936747338L,           -759650386L,          -2198450187L,           -837636328L, 
              4542581952L,          10460139241L,           3952822531L,        -157841535420L, 
             -5340182925L,          -3859818639L,          -2229317344L,          -4989654045L, 
             21048131206L,          22528296304L,         -10061420024L,          -7873398352L, 
              1302059815L,          -3185952812L,            865543279L,          15494689065L, 
            -24346817433L,          82646253653L,           8099424724L,           2400041826L, 
             -2300897796L,          47645280376L,           3116000060L,          40427194155L, 
            -27415246877L,           7016973347L,          -4696166699L,           4333161393L, 
             -8530586190L,           6625488985L,           5783990834L,         -64572972666L, 
             25331372848L,          -1518944515L,          14164217237L,          -3597025929L, 
              4320469522L,          -5079730334L,           6071252834L,            158483659L, 
            -21669693155L,           8215479346L,           5268270358L,           2318458108L, 
              3166551485L,          -3327208254L,           2791977687L,        -117866689231L, 
              5071949800L,           9257104382L,           4208470616L,          -4507632782L, 
              3799189876L,            217576250L,           3477470113L,         226189685576L, 
              2440505581L,          -5534995711L,          -5976822782L,           3531377335L, 
             15222413021L,           1355801103L,          -4936578475L,          -1667347690L, 
             -6752064534L,         -10144034415L,           7261310164L,           3699910038L, 
              4957639878L,           -704291719L,          -5010413663L,           9440000850L, 
              -208423595L,           6712477518L,          -1413431716L,           2195424005L, 
              5660765571L,          -1389643406L,          -2042698814L,         -40721235343L, 
              2376288170L,          -1023892324L,            799511736L,          22459050043L, 
              8806906268L,         -23254307319L,         -11350724157L,           -723655805L, 
              4713023555L,           6570516789L,          -2283693051L,          29934864015L, 
              2739823107L,           1634336697L,          10362352043L,          55988300255L, 
              4144546979L,          30176556462L,          -4653592711L,            585796391L, 
              1891318046L,           -926126560L,          -1747365018L,          -2892122920L, 
              1565504518L,        -115161312439L,           -936024757L,          -1732065439L, 
             12464229057L,          -2149032142L,           6197149237L,          -1243139010L, 
             -1625702139L,          -3377028388L,           4995055556L,           -230407409L, 
             67612978415L,          -2693427532L,          -2416294940L,           6435982577L, 
             -5446237399L,           5134990813L,           2955847824L,           4244737883L, 
               400712217L,          -7157920622L,          -7796957035L,           3275636188L, 
              -659053293L,           1535174647L,          -7666114498L,           1422162309L, 
            -14415725056L,           6991886407L,           -708037111L,          -3083688219L, 
            -53605322083L,          -1141983716L,         -16456487280L,           5047413205L, 
              6099623284L,          -9711936443L,          -7814805643L,           6314334757L, 
            -80678626865L,            656376792L,          -5449496781L,          -9139453623L, 
             -3420034530L,           5701137098L,          -6785962390L,          -4073597664L, 
               271140287L,           3523008887L,         -13207416896L,          -3223183639L, 
               -64128693L,           3807515876L,          -4504447071L,          -4476230716L, 
             -3414047546L,          -8236045501L,           3149048659L,        -109426363381L, 
              -369911384L,          -3489957686L,           6779691572L,          -2158316696L, 
              -266164452L,           6836867815L,         -15019552759L,          -5941049051L, 
              7891652217L,           2863518338L,           3222411734L,          -1575608075L, 
             12868704950L,          -1044978953L,           -377122029L,          12466642887L, 
             -4381148355L,          -3597741595L,           1863090315L,        -326386142541L, 
             -3594617937L,           4185147594L,           1962419193L,          -5410428560L, 
            -41227703093L,            524752122L,         103691192784L,           2716413820L, 
               376216156L,           4276287911L,           8026877293L,         -18924069039L, 
             76861229352L,           2941288175L,         -10394918783L,           -558515848L, 
             14228195821L,           2894713416L,           5533306703L,          -4876778366L, 
             -3407200514L,          -1728847960L,         -17870779921L,           5182522668L, 
            -91305715077L,          -2315352091L,           -515542895L,         -12663122372L, 
             17824836527L,           4583803197L,           3064019081L,         -12614412738L, 
             -1653254803L,         -36939654338L,            308551208L,          -6465497683L, 
              4014836681L,           2025703089L,          -3402590201L,           1680522091L, 
             48985572212L,           -514690140L,          -3848933444L,             79781969L, 
              -104475766L,             64792495L,           7405349729L,           1219463312L, 
            -16944150687L,          16063332088L,           3442192973L,          -6101079550L, 
             -4983450404L,          24947476463L,           7982652643L,           -981676891L, 
              8835126500L,            283331379L,            -57211108L,          -3387409380L, 
#else //SIGNED_DIV_EUCLIDEAN == 1
               989658733L,           7908432074L,          -3249975019L,          10965394060L, 
             -4812793069L,          -2700544842L,          -1929640615L,         -14165188077L, 
             -2638411778L,           -626577934L,          -4479191388L,         -13141134533L, 
              9281251374L,          13936313023L,         -11310884744L,           4778075428L, 
               673222297L,          -8833707060L,         -13059879874L,          -2408198147L, 
             -7164986939L,           8417803445L,          20955018864L,         153611516224L, 
              3035292000L,          13422713038L,           1638921016L,           3235651641L, 
            -32849968172L,           -447018214L,         -16883479611L,           8584597331L, 
              6656988676L,          -6365725805L,           -860027563L,           5994469935L, 
             -2239744351L,         -11874901681L,          -7459766943L,          10344448932L, 
              2512607677L,           -988293898L,          -8205199136L,           6019604996L, 
            -10654535508L,          -3767053229L,          -5207717839L,           -248437194L, 
             -1992211419L,          10164818380L,          -3035112617L,         -11262223586L, 
              4124769488L,           4154600494L,          -5356921671L,         -15232994385L, 
             -4190688706L,          -4446931949L,           2512552425L,          -3794939399L, 
             10101516729L,           7503221223L,          -2314790049L,            924810754L, 
              1462701806L,          -8486374750L,           2373726781L,          -5750776740L, 
              3979995981L,           4206296475L,          -2548500684L,          -4832207828L, 
              1191566960L,          -1320788867L,          15781270048L,         145723824052L, 
             41645097271L,           3356199300L,          -3598411505L,          -6259119762L, 
             28690690826L,           4643849223L,           3345431173L,         -39975886146L, 
             -8121687466L,           4124819715L,           8173559198L,          -4562495022L, 
            -12438472806L,           6246372333L,          -5027592000L,          -1276538504L, 
             -2994787589L,            103502834L,           5843588623L,          -6061639082L, 
               -11720052L,           1758068975L,          30732808867L,           3194896204L, 
               974041689L,           3714140302L,          -6995999265L,          22446653351L, 
             -7331287289L,           3287451686L,        -120567337406L,           4952752711L, 
             -7320858664L,          -4918960190L,             41894810L,          -4604277517L, 
             13343662357L,           8298888575L,          -1913008223L,           3666667726L, 
             -1006193201L,           8378759204L,          -4649916776L,           4028492549L, 
              4782769718L,         -16422054877L,           2744734141L,         -28077793435L, 
              -847586459L,          11617032767L,          15980266389L,          -6962083437L, 
             -2454600967L,          27568145305L,          -7363005460L,          -5094758208L, 
             -3450885348L,            458966691L,          -1605558420L,           6737523524L, 
             -3399280289L,           2583091329L,          10415019416L,            276562816L, 
              2556402142L,            440509604L,         -19800155517L,          46982890233L, 
             12260652841L,          -4910341908L,            624825477L,            672358717L, 
            -22131526301L,          -3577334792L,         138245334786L,          -4557532248L, 
              2552409533L,          -4655769270L,         -11411203419L,           -343406000L, 
              -740628887L,          -4968184364L,           6304090766L,           4383229306L, 
            -28726697360L,           8186789753L,          -1135601187L,           4359460320L, 
             -3586238640L,          -3428065631L,          14956321939L,          -4623107698L, 
             -1904733651L,            -35617755L,         -14538665189L,            -18009460L, 
             -3227611030L,           -377668966L,           -934707260L,           2597609417L, 
             -5992876708L,          10520120910L,            336268045L,          -1197540468L, 
            -24888757944L,           1239523316L,          -3689289018L,         -11703516198L, 
            -67094075531L,           1976573020L,           7926214879L,          28258887051L, 
             -4331759063L,           3184632457L,          -5352752429L,           3214627965L, 
              2420109561L,         -12018746331L,           3545460703L,          -5946111695L, 
              6395249380L,          -7053740717L,          -2552210743L,           2859330120L, 
              5203353769L,            105224442L,          -2339196089L,          12228745448L, 
             -3499986960L,          -4189677409L,           2483025534L,           3703194925L, 
             13105103653L,          -2676721430L,         -13952993980L,           8285003733L, 
             12748782794L,           5880440842L,           1347625700L,          -2280748429L, 
              1915994073L,           8336085299L,          -5407240284L,           4769082430L, 
             -2477723023L,          89988937461L,           9812143623L,            -69078768L, 
            -13442755704L,          -8112992156L,           5196150518L,           3880045039L, 
              -557695312L,          -1147841550L,          21567716474L,         -10856147146L, 
              1063424099L,           4856591381L,            594875134L,          -5017579403L, 
              -816670787L,         -16523368597L,        5853494180453L,            464747430L, 
              2602609766L,          16212099300L,         -14247713671L,          -1540304594L, 
             -8556555459L,          -5761850879L,          -3951861235L,           6304356639L, 
             -7082999013L,          58275962464L,           1490210886L,           3402828605L, 
             -1718161033L,            793016721L,           3057408169L,         -49322351731L, 
#endif
}; 

const int32_t test_remainder[256] = {
#if SIGNED_DIV_TRUNCATED == 1
      -165621232,    -67053901,   -397586476,    567655248, 
      1056054381,   -893987408,    864757256,    142684436, 
     -1870966914,  -1233690437,    127536880,    394866294, 
        22406752,    672089006,   -163896011,   -169993648, 
        31484042,    283964670,    752919857,   -717378650, 
       120076575,   1371691008,    887765917,    -65934675, 
       132871500,    836902976,    -49383622,    216568024, 
      1322198880,   -634403776,    195212660,   -622536408, 
       116475204,   -803353238,     -1836508,    120789597, 
      -584718536,  -1030143364,   -289892976,  -1301409315, 
        58768368,    628069814,   -185697607,     -3741666, 
       384157189,   -212984567,   1756095580,     35703226, 
      1322607098,    421561620,    -53914856,    539905612, 
       298472757,    150185327,    356420222,    310426123, 
      -746964730,    359632024,    366647483,   -682342134, 
      -836948312,      7491744,   -183706496,    162437024, 
      1915152752,    740811103,     80534206,   -484027376, 
      -215832640,   1491729804,    249832243,   1343433919, 
       889060785,    374066560,   1150410841,      -873451, 
       372374900,   1028239194,   -270011080,      1811694, 
       761521992,     -3010782,     84225515,    -16650624, 
      -141391732,   -205691885,   -634081421,   -646423256, 
      1123541314,   -274403825,   -166669276,    -19571601, 
       176661213,   -472542653,     38942567,    117275753, 
       735614542,    -35460959,    310148632,   1078950852, 
       -93445320,   -481759076,   -469062720,      3469700, 
      -125396066,    139352747,    752746126,    989105984, 
      -319879224,   -487803420,    239628525,   -356560974, 
      -770248428,   -449710798,  -1222850420,     41518030, 
       471659680,    760165470,   1435491180,    -16970552, 
      -977283822,    -16545656,    120425561,   1331587880, 
      -407551051,   -126526576,     -7812351,    379427372, 
       456505568,   -194824083,    -72918482,   -797914736, 
       244549804,     -5825714,    608243587,   -592087615, 
       638025405,    102638696,   1445713930,    733135312, 
       485919820,    -76833294,    749291124,    -50404879, 
     -1244429872,   -129624714,    533346652,   -276998990, 
       351821755,     13058964,    322977216,   1233786172, 
      -116839412,    169046510,    289352424,    701690314, 
      -340353193,   1270050444,    761410036,     96157160, 
       -21737198,   -470723064,   -501281464,     45611257, 
        26442865,    257607582,   -860570543,    950011297, 
        44239940,    151796167,  -1155005948,   -795196578, 
       346752226,    131890761,    429494520,    -22242411, 
      -856547796,    247190142,    312541941,  -1820010978, 
      -244761548,   -913795074,    536396934,     15824016, 
       668884678,    216003584,    -17865836,    172135323, 
       923386353,  -1035017152,   -318463305,   -527215034, 
      -572844014,      4431545,    219787488,     59382742, 
       201780853,   -452507119,     42998408,   -297935948, 
       -49148091,    -66913792,   -108218586,   1092551558, 
     -1156383828,   1434801508,    745752128,    356912424, 
      -204409666,   1640738624,    864307223,    870963720, 
     -1310677180,     21192576,   -192429669,    -12623222, 
      1415420160,   -500732025,   -523769574,  -1225915752, 
       321613260,   -506825260,     -5208870,    103214436, 
      -231236628,    193638864,    239643972,    179189104, 
       -12569312,     73999840,   -902639550,    460654832, 
      -204321884,    142970442,  -1342859271,     34753024, 
       239501919,   -918809089,    -26938180,   -746662965, 
         7464352,      4346818,    -16624461,    342569851, 
       365555176,   -995777928,   -964912708,    -80492416, 
      1035695846,   -328205711,   -595785984,  -1645699150, 
        -5651366,    423211888,    678605064,    409971180, 
#elif SIGNED_DIV_MODULO == 1
       375003872,   -303981688,    -58563429,    459547946, 
      -448184216,     89134560,   -236452222,  -1134872361, 
      1091567298,    742021818,   1706391002,   1039027704, 
     -1556123136,    143162253,   1227819647,      8626120, 
      1533210711,   -654261343,    886085856,    439741655, 
        55628296,   -188891616,   -195166200,    706433968, 
     -1276020028,   -311292792,    262918797,     88438056, 
       -15488057,    -13519971,   -440748308,   1185866252, 
       190371716,     19311088,   1296284960,   -126799552, 
       -53438310,   -921911282,  -1159719486,   1228668565, 
       240783996,     21867586,   1317457288,     46624668, 
       193507424,   -529975244,    134571679,   -851812040, 
       -69804368,    266759462,     52285182,  -1109459376, 
      -134996245,    556381588,    206546374,    -44288276, 
       -78627297,   -324117992,    -53272534,    -45354127, 
      -412480840,    599303600,   -428107112,   -187816636, 
      1153695368,   1900173468,  -1395556869,     -9381816, 
      1303336263,   -588715751,   1372187272,     24247875, 
       281180093,    -99199266,   -323698982,   1410406960, 
       386138746,   -246876365,    116518888,   -456875426, 
       263389276,   1842160080,   1402590718,    -54324118, 
       522287415,   -127224712,   2012691808,   -518272802, 
      -702473668,    528360652,  -1400049612,    110396800, 
        90046490,    144664148,   -279107816,   -217600580, 
      -358304064,    -22127813,    180981525,  -1115821804, 
       693154129,    449474117,   -211147242,     38636728, 
      1547155592,    356199141,    483663428,   -121710523, 
      -838897057,     16772466,   -914068349,   -439334403, 
       233659024,    968261472,   1147789660,   1769911608, 
     -1219988200,      6173719,    392505376,   1455962186, 
       344120305,   -440646844,    536629080,    160622540, 
       371337388,    -40024080,    820214940,  -1448995808, 
        14071783,  -1218194656,   -497215068,    290087590, 
      -300619734,   1093341556,    691233472,   -173809186, 
      -609913844,   -624188716,    574653782,    -74623432, 
       629758442,    106947656,    691480162,    445143688, 
      -269106688,   -945599784,   1443475600,     51186586, 
      -164506433,  -1749057236,    106146208,   -529408025, 
        79455700,   -590809122,   -298436514,    394550755, 
        17206677,    559261968,    -70915471,    105254990, 
      -833728654,   1206918596,     49833002,   1175534336, 
      -336770051,    544760923,      6078464,     58901090, 
      -302797221,  -1238093052,    270845910,  -1283004364, 
       -76057922,     28417786,   1905443340,     28510985, 
      2037117168,   -312413464,    -60594428,    218586784, 
     -1470593936,    963405948,   -149945231,    445850814, 
      -245406311,   -713563130,   1305177774,    495917540, 
      -279311992,  -1008008669,    782889471,    258572666, 
     -1067518502,   -649074317,    688347446,     -2999311, 
     -1082620791,   -679366588,   -669100313,     58282560, 
       -82764809,      9431250,    -20178128,  -1464288480, 
       169521784,   -727625756,   -924393230,    298719258, 
       -77745160,   -438813113,      8054560,    765830744, 
       286359470,   -836916280,   -986508673,   -177315374, 
      -410081372,    791221064,    271113653,   -503485940, 
        41493211,  -1524422549,   1407109517,    -18092028, 
       108297216,     95873943,    676783853,   -105354370, 
     -1593494903,    -37826542,   -433018056,    680264780, 
     -1957941688,   1570123115,    587222265,    302854859, 
        23084868,   -101036548,   -319746296,    233286463, 
      1577766002,   -151723382,    640060291,   -884880000, 
       180946841,   -162059144,     54533478,   -811573262, 
       -16279984,   -177105474,    484726032,   -967236618, 
       -61258492,    510092367,   1170355500,   -356583272, 
#else //SIGNED_DIV_EUCLIDEAN == 1
      1019587866,    632949208,     92473723,    420482892, 
       517155305,    193916174,   1204774950,    506767089, 
       385950054,   1113887168,    583616048,    137943561, 
       156468726,    477453141,    454876224,    178730492, 
       492449930,     77560100,      7395154,    724868730, 
        55946440,     51645877,     22031392,      6773888, 
       886227456,    290808816,    981539816,    520875604, 
         2461908,    666772000,    164746454,     79872568, 
       806090456,    727323896,    259914310,     66795555, 
       992573770,    651942229,    594995179,    371431188, 
       539364408,   1796255840,    755065440,   1230515732, 
       306904488,    473122591,    730367455,    687894976, 
       570257147,    277851804,    363792417,    111478466, 
       209572928,    391037826,    754161585,    166756848, 
      1052086860,    409394776,    218541057,   1587407181, 
       239149454,    835795119,     63208323,   1358099182, 
        59269166,    135076658,   1231581228,     96137028, 
      1614486136,    936107039,    432052876,    301423644, 
       885217728,    986319173,     68215648,      5705928, 
        13230713,    669673564,    718982855,   1248469758, 
        98411092,    351622996,    550382720,     54228564, 
       615050168,     51672205,    438285164,    372729022, 
       355785496,    494826971,    960664448,    947487984, 
      1418604556,   1338428490,   1047128032,    236128028, 
       866856088,    332198072,     28838353,     71661776, 
       363160352,   1185429480,    900931427,     86866828, 
       409469084,   1289063332,     23835530,    698724722, 
       250087712,    101835572,    136556212,     91810147, 
       396119266,    176999617,     97481201,    369852182, 
       579101424,    477751364,    379233744,    645119911, 
      1672175252,    413055185,    179162958,    281069172, 
       274995391,     19119448,     17850435,    229934311, 
      1744333626,      9696668,    117129156,    443840384, 
       182767732,   1573173616,   1148864544,    565673312, 
       814961581,   1548327393,    731964768,    115233152, 
       893752684,    575977288,    123096893,      5893990, 
       248579855,    193236956,    625879915,    877083575, 
        84419249,    205306672,     31299100,    164386432, 
       292609188,    394828142,    396432464,    123675696, 
       214095677,    404452836,    991746750,     24702992, 
        92247616,     28113794,    199965906,    440450400, 
       361262512,     35746272,     76483520,    991111210, 
        91772578,    844501801,    233571393,    513264080, 
         1625500,   1413829488,    657497536,    465757190, 
       511607216,    189375610,    135406615,    584403580, 
        68319616,   1586168556,    418381240,    242736950, 
        35759934,    606409780,    283118536,     28233941, 
         5185442,    520894519,    982898034,    594484501, 
        70824319,    319883151,    271492410,    374681678, 
       679325332,    554863741,    596319118,    665864680, 
       805211046,   1650948754,   1304346295,    281361472, 
       702494944,    753819126,    611326682,     25870843, 
         1811123,   1323868102,     55522972,    309899594, 
        94575074,    140316308,      3824340,     16840128, 
      1068734290,    618890354,    917348056,    540777450, 
      1271589381,      2663885,    118512723,     90450288, 
       131135208,    946348796,   1337326916,    696817385, 
      1140087216,     71204334,    215222398,    207948672, 
      1971042637,     88591879,    845556468,    178087476, 
       385144184,    100249376,       827623,     70933966, 
      1488464102,      8453936,    356702480,    295326306, 
       743834722,    322641311,    238136370,    397439495, 
       251154416,     84606912,   1215405940,   1299536704, 
        88433378,    295477089,    609043746,    112101620, 
#endif
}; 


// End of File
