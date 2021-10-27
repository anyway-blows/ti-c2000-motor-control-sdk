//#############################################################################
//! \file   input.c
//! \brief  FID Input Vectors (256) 
//! \author Vishal Coelho 
//! \date   12-Apr-2016
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

const int64_t test_dividend[256] = {
#if SIGNED_DIV_TRUNCATED == 1
    -2415501404604182528L, -7742750994199947264L, -4864261170820929536L,  9118959538562191360L, 
     7550323573896173568L,  5364071019257268224L, -6730363632386193408L,   928594177727928320L, 
    -8230229812930392064L,  3256837159013918720L,  5945027540759646208L,  7589029285193203712L, 
    -6507918195231541248L, -3599659658380410880L,  5162723233171666944L,  -235594058129778688L, 
      907103284638085120L,  6976864797695277056L,  9007377743734767616L, -8870074602653085696L, 
    -2742592315335659520L,   639121001494302720L,  4571430889937526784L,  8670212744503072768L, 
     3599754677185445888L, -4182458370546724864L,  8322026386724718592L,  7520197338136756224L, 
     -785379136409192448L,  4301246379472193536L,  1074339629511962624L,  9110743242340347904L, 
    -2042048417509849088L,  7040811846776397824L,  7254543014169565184L, -2102931868691736576L, 
     4197570834500429824L,  7612043002373978112L,  7454828714686580736L, -7720841499446888448L, 
    -8608328852535441408L,  1335718508901908480L, -9051997773500751872L, -5564858984132495360L, 
    -8841890748296046592L,  5391453109591689216L, -5703190698725490688L,  8339452647316000768L, 
     4984795047417763840L,  5839750945156501504L, -1364134769269243904L, -3685324504916393984L, 
     3325772237319546880L,  6613446794504630272L,  6858369216489484288L,  7324510041386676224L, 
     6338057060794726400L,  -273911556350351360L,  -211390281676652544L, -8018173261698148352L, 
     -793559802623277056L,  1762684232156250112L, -7192690521392058368L, -2881671767010308096L, 
    -4352229842685050880L,  1479066209427810304L, -1191824163236552704L, -1566125163552802816L, 
    -8223318594512492544L, -4432833426542804992L, -4719082204382838784L,  3857123498845155328L, 
     2930278807102883840L, -7024864330103730176L, -6839699755158951936L, -7785104859264950272L, 
     4339413717459427328L,  1035784576265709568L,  6168059756986341376L, -7265721187866269696L, 
    -4490647876487632896L, -8224960024220887040L,  2887909759531802624L, -4105570417798160384L, 
     3857773112970795008L, -6819843900886206464L,   442671569906827264L, -8376848844183001088L, 
    -7354611937605240832L,  -573046447058808832L,  3885739537589702656L,  1869511242005579776L, 
    -5443321021337495552L,  2191224657789782016L,  3673671172618817536L, -3415808133974087680L, 
     5622358147454746624L, -5939437823658602496L, -4772067167786231808L,  1747459491757785088L, 
     6792698332237305856L,  5878393932101144576L,  1661758933458800640L, -9181168815572105216L, 
     8200168838055159808L,  5395246548336209920L, -1857062700831152128L,  6888927316698263552L, 
     3217044391206694912L, -4752515732098953216L, -6116115323525969920L,  6661576789559631872L, 
    -3197909060803584000L, -3225381952413136896L, -1841475357370710016L,  4860829941261303808L, 
    -6196855549788696576L,  3473343868865404928L,  2643724986985111552L,  5927926246970304512L, 
    -2240413693164738560L, -7159169185956759552L,  6631295961145104384L,  3932534203444725760L, 
     1383696092439093248L,  4448208982099404800L,  4679399045748606976L,  1016196269592637440L, 
     5550851430922653696L,  3142328738793185280L,   647618716459995136L,  3429731486543808512L, 
     8396659795568500736L,  2356613809711515648L,  5886946265544024064L, -5737576906576291840L, 
     4638902895224997888L, -5135933271042641920L, -1541228914755637248L,  5365670797129355264L, 
    -1845377881590558720L, -8391253270532511744L, -5514740071306577920L, -8432692268018223104L, 
     2567107961849653248L, -4579245174915655680L, -1477880273781118976L, -6053552917757816832L, 
     7091958972393617408L, -2932772018550185984L, -5439509779522326528L,  5448925917849038848L, 
    -8771980517024546816L, -8489568965556822016L,   550056745949272064L,  7994210791515803648L, 
     1885526714334519296L,  1866381200619474944L,  2653863306893309952L,  7783245357756123136L, 
    -5178789902090483712L,  7100232679160793088L, -8322765404877826048L,  5756117245354883072L, 
    -8622159585753927680L, -1567189836033069056L,  1746289755589593088L, -5593139864715182080L, 
     5461918816386422784L, -5331326391345465344L, -6006010497934940160L,  2100010167351228416L, 
     8136055432757422080L,  2980347219090862080L, -5259196708322107392L,  7629385200130392064L, 
    -2209979891796467712L, -8578062779436347392L, -5029595179684962304L,  7505731834170624000L, 
     6806628429131053056L,  1521278428753682432L,  1121308772339056640L, -9203290332007032832L, 
    -6883179100808572928L, -5263945043566383104L,  6028758082664773632L, -2073150579289755648L, 
    -3355915262350364672L, -4394410391903367168L, -3753470012718272512L, -1324881255853686784L, 
    -5674101961532823552L,  1244755671323758592L,  2908419106753163264L,  3423336438228838400L, 
    -2698013573074737152L,  8426297933757450240L, -4727188881312817152L,  4579804749485039616L, 
    -2024944620399552512L,  6359637362929438720L, -2361478754666176512L,  3829654500917762048L, 
    -2847157284449552384L,  6957335380440872960L,  4621424366449952768L, -1209639081273120768L, 
    -1832486681189593088L, -2343882975160195072L, -1046847277740677120L, -2207938694288689152L, 
    -7882064221184079872L, -1101625323641102336L,  1965072112416000000L, -3601096609316511744L, 
    -1722118747009200128L, -7360033103305725952L,  -426486110159869952L, -3829353296440922112L, 
    -8923011937316364288L,  7091218294087450624L, -5343825360412276736L,  3387639369102219264L, 
    -6375373644378961920L,  8749583511454468096L,    49111694996627456L,  7220206776482480128L, 
     4178023045692583936L,   -52026352501698560L,  1977753750629146624L, -2780130713933862912L, 
      483573295055286272L,  5477118416205473792L, -4174942229652340736L, -3824312629566736384L, 
    -2801103305830369280L,  6935475284389773312L, -7501937557007259648L, -5410791465498697728L, 
    -3638452463668379648L, -2738928386700290048L,  4182753130549000192L,  7126443440785614848L, 
    -8265502891810074624L, -7774210923877494784L, -8956151466095450112L, -5656876175879145472L, 
    -8738420421843482624L,  -576168361800726528L, -4733755158113992704L,  7746098958469871616L, 
     3981427893206140928L,  -209449230873946112L, -7783156189549729792L,   554621829142339584L, 
#elif SIGNED_DIV_MODULO == 1
    -4934559764453236736L, -7064784114020253696L, -6818351924660658176L,  4460867351996821504L, 
    -6671942645898389504L, -5771025303075352576L, -1909685790839920640L, -3025349019585937408L, 
     -761517986820741120L, -7324917345986291712L, -6078799340842815488L,  2608344496985577472L, 
     5306805230144043008L, -2354548217051471872L, -3567655531289116672L,  7335124348236763136L, 
     4139328670105075712L, -1721018230593730560L, -5785581160686338048L, -8271810800178847744L, 
     8189690354799351808L, -1464301806440376320L,  1773547909473769472L,   249846182581620736L, 
     7301728549262262272L, -5795364530107080704L, -7498569911289604096L, -8180100463785015296L, 
    -8344628931915919360L,  3370581357436813312L,  -451850262985175040L,  7877020069997946880L, 
    -1635064093711196160L,  6825401505689554944L,  3873819520434309120L,  8160225904696301568L, 
     -365775659049185280L, -1206210651747700736L,   222699207403874304L,   451760120414783488L, 
     -570907676399052800L,  8458622495921104896L,  6568855420251392000L, -3587262186559238144L, 
     4064230888834787328L,  6541552348495439872L, -3547483573547270144L, -6330871608757190656L, 
    -7801006546935269376L,  4576476086262730752L,   631116460454578176L,  -850040099925747712L, 
     5394323894187747328L, -5187654520809119744L, -3569265592140302336L,  4271112854713587712L, 
     -739235492364613632L, -1751649592278706176L, -4580598290638673920L,  1496856075245195264L, 
     5687637318793148416L, -8293034634455085056L, -6479485518466983936L,  -158457150666471424L, 
     7225490857160812544L, -3449164027949301760L, -4006150129302294528L, -8029644333870198784L, 
    -8107065403561334784L,  1458890853876586496L,  6215870050654228480L, -2815094336875315200L, 
    -2795993476305461248L, -8457335502814932992L, -2004822129934614528L,  6757162295310868480L, 
    -8757417567818323968L, -6062001505306677248L,  6104535770018572288L, -8986684553989222400L, 
     -614467683386290176L,  6117373958575706112L,  7383600007077345280L,   130108343947087872L, 
     6248456986227456000L,  1248047184669831168L,  5053691223457335296L, -3454688185375469568L, 
    -7238641439403270144L,  3589412121301458944L,  4459523416621707264L, -1901294649487226880L, 
    -2829993716494348288L, -8220626152486959104L, -2308307455500019712L, -2456071780001703936L, 
    -3981307945256904704L,  7644121733944872960L,  2954178596306956288L, -1373660017876008960L, 
     2307693115815223296L,  1846928221180276736L,  8863817208615514112L,  4876045249196046336L, 
     4037998212198164480L,  7021975760814624768L, -9026908662909325312L,  1319594672698406912L, 
     8699546866817435648L,   620361004169123840L, -8066451246342195200L,  1193413682800623616L, 
    -5015179764220227584L, -1096133232601360384L, -7398884353370091520L, -6878500465386553344L, 
     5564067415883816960L, -4521608925869561856L,    -1319608568135680L,   424194131675019264L, 
    -8861723631416008704L,  6954673348474374144L,  -735966812674383872L,  8853308326420432896L, 
     1281585904203843584L,  6904695044730974208L, -4768032938141114368L, -7054783973969172480L, 
    -6430450586608906240L, -8125903289382838272L, -5530224114162006016L,  6792863857343989760L, 
     4996745107082627072L,  7966609242639251456L, -7733742165763844096L, -7934102329209028608L, 
     8804723852065103872L,   400326091932588032L,  5307111201587576832L, -3509413908075153408L, 
    -7037139423371571200L,  -268804383098028032L, -1562850429505861632L,  7670913155614666752L, 
    -6281897289065121792L, -5467468895070218240L, -6454708793512912896L,  8443009162292996096L, 
     4587360757377898496L, -5415005188362897408L,  3056649634964031488L, -5173141069455841280L, 
     -935245692834564096L,  8123475441268199424L,  1511521279830411264L, -6498294384648706048L, 
     2389421857791354880L, -5823586806459289600L, -7694274163184986112L, -5277102271188121600L, 
      487659632430313472L,  3075151208945942528L,  -756331076183635968L, -5248861962407114752L, 
    -5160003341508329472L,  7195862677562042368L, -3660702666024013824L, -4610568205732659200L, 
     7316285820628926464L, -7014784869689571328L,  3047374925478043648L, -3182573193365248000L, 
    -3207994392814712832L,  6004197493190049792L,   926240335395264512L,  2670675848295815168L, 
     6366510428623271936L,  1752421186183962624L, -3117748659302696960L, -5323572141799061504L, 
     2318525981899601920L, -4370743812936044544L,  8774176542728497152L, -6246877097166950400L, 
     5433349555008907264L,  7829408696059998208L,  -621512956030840832L,  2094906447315357696L, 
    -1651189080741842944L,  8655664984783474688L,  2826039982055309312L,  5006629329744015360L, 
    -1564748012473749504L, -6473502822586851328L,  -104800097855332352L,  6432276146575695872L, 
     3279571576997666816L,  -997448118412795904L,   -53913173653215232L, -6343316172658276352L, 
     3649972833487902720L, -5447474779121272832L,  3271984037121372160L, -6926598832594800640L, 
     1761138492526864384L,  5034453593622546432L,  1468019010856933376L,  3392986265649479680L, 
    -1341024211455315968L, -4117211182080618496L,  8818288642235318272L,  8576210149275930624L, 
     4206853272661485568L,  4146409412127365120L,  2674124602870165504L, -7639724194130900992L, 
      514742122783952896L,  6484819004998309888L, -5349813354767114240L,  6027281068185305088L, 
    -5415787532280102912L,  -452025891937650688L, -3690664187742736384L, -2859800850085855232L, 
     7894282967043424256L, -8667733123695319040L,  4058951378308214784L, -2283417079331100672L, 
     1623832894697738240L, -3136929994462162944L,  6731712603417892864L,  6547760077856333824L, 
      887275105969649664L, -8422531227705325568L,   619448930774583296L,  6000131954817581056L, 
     1222550043397703680L, -5179154405905020928L, -3258844986578569216L, -8023534784934502400L, 
    -2327562604735418368L, -4454383812514916352L, -4416090579889053696L,  -507557482034231296L, 
    -7474009922324703232L, -8058544380984496128L,  5838038155551715328L, -7960725990345713664L, 
    -3732971126705997824L,  5304743288388698112L,  -804787261779017728L,  7566435657759322112L, 
      934716804966576128L, -1305705956851380224L, -1862132886761506816L, -8055755797454604288L, 
#else //SIGNED_DIV_EUCLIDEAN == 1
    -5888414969252755456L, -5647203931255740416L, -2572427159025807360L, -1763624495745503232L, 
    -5058464156320495616L,  3118552656351037440L,  8434088360898011136L, -1260555871235354624L, 
     5604382974975438848L,  5500619788251543552L,  9135584226726189056L, -5316270439644264448L, 
    -3154045522987499520L,  4839352907600918528L,  4736376734108397568L,  8509051143024459776L, 
     1289285944571785216L,  2001584581323974656L, -2488609288867119104L, -2886038030307729408L, 
     1844452193699209216L,   472771994710771712L,  -277318780839583744L,  2871241237297434624L, 
    -8517747376665636864L, -5719915323499661312L,  7908954416111089664L,  1530705398432475136L, 
    -8939100087967821824L, -3806019058977081344L, -8388469040244785152L,  5436837098481516544L, 
     3899013952099768320L,   670747005178030080L,  1601078726014629888L, -5332079951068506112L, 
    -1587987818243231744L,   291839730576570368L,  6241176891698108416L,  2949830625446938624L, 
     -347240003565731840L,  6507360273238966272L,  -340746029107693568L, -4200399762822338560L, 
     4264808065985370112L,  8660179367550742528L, -2294358204065814528L, -4821976993371623424L, 
    -4746259673972889600L, -8874001791261433856L,  8903379679115345920L,  5582344943547613184L, 
     1171096434965354496L, -2050742552085803008L,  2813652600165050368L,  9107308056998479872L, 
     3943283502467817472L, -7335034473611026432L,  -972348762733830144L,  3556436112365168640L, 
    -3081419508180277248L,  7779893092311689216L,  2444027345677846528L,  7972414442593619968L, 
    -1366531113410197504L, -3680316628515717120L,  7176312958738974720L, -8902882051041538048L, 
    -6492237073408585728L,  8470050463035934720L,  3956263009775884288L, -3570058871744655360L, 
     6052306055727800320L,  5679602405275623424L,  7549423383529451520L,  2634434386613280768L, 
     2373828893216641024L, -7037966339080847360L,  7728675933940426752L,  2287047128160833536L, 
    -4471769219630336000L,  8325081356362940416L, -8371026208818276352L, -8837334410094151680L, 
    -7599133872666785792L,   329142574747686912L,  6985113878923194368L, -1804477215544801280L, 
     1131215660452620288L,  1810655768995801088L, -7838928099759374336L, -2001656190791139328L, 
    -2675052241462257664L, -4707812138341632000L, -7230601427387205632L, -7152001883862913024L, 
    -4294318159515406336L, -4309300060165670912L,  8044279760542033920L, -5789013895014209536L, 
      137153302978756608L, -6500739349064640512L,  7760545852470294528L,  7922162090302111744L, 
    -6700753698814482432L,  6854242126145179648L, -8995460584428740608L,  4095843233668153344L, 
     6983666157587980288L, -2076241820464476160L, -4677532677068804096L, -7162436239122876416L, 
     6396982916898371584L,  2945188904369774592L,  5158857083952932864L, -2609057925221009408L, 
    -5826673245392416768L, -9035592348680757248L, -7638523982404102144L, -3145774832973635584L, 
    -3674035171120689152L,   121216512831545344L, -2091763102367426560L,  8066694509390864384L, 
    -6280355842616399872L,  5159737512264435712L, -5949770845318643712L,  5631670929269751808L, 
     1150778197360050176L,  -552023141143595008L, -2589708125822758912L, -2365344623897413632L, 
    -2160620369914519552L, -4495347368161947648L,  8707562462655258624L,  2961892676751652864L, 
       97519824326002688L, -5570534295979229184L, -7297824014682159104L,  7164961592977240064L, 
    -2527942428851390464L,  -853833247587997696L, -8028140313055436800L,  6470419241124896768L, 
    -4068743361431683072L,  7598575771030929408L, -8120806937503641600L,  2139201116096548864L, 
     1179544452544030720L,  3104030241789806592L, -3035541432687278080L, -8526190691520585728L, 
    -3951565236523732992L,  7955140896489021440L,  5531288070536450048L, -3116121112642824192L, 
     4379616938211559424L,  1244046278279852032L, -8927049680069013504L, -1374013942280742912L, 
    -8681435564770084864L,  5242134132545824768L,  8869468270137257984L,  3503851911867199488L, 
     5159284589875947520L,  1807384500351528960L, -3973254135693516800L,  4542599587080921088L, 
     3576348630184933376L, -1060260787984633856L, -1491819402385008640L, -6442131060844920832L, 
      743256672612136960L, -5601164457028726784L,  4596608970540578816L,  3978229392683847680L, 
     3772304624399341568L,  4358732331130685440L,  2890367660061505536L,  7214450529803151360L, 
     -164553997665492992L, -2918695354472347648L,  8535469662786467840L, -6918556124481441792L, 
    -5375155049243930624L, -6364676950752002048L,  5674308231129651200L, -7980678084158617600L, 
     -173340775973013504L,  1815494895621857280L, -2489938319437666304L,  3478297975299325952L, 
     6583774767618975744L,  7390475291101118464L,  8328622734861850624L,  7379561379532244992L, 
    -9102787024790794240L, -8449859415309910016L,  6841753074164774912L, -1025628240463499264L, 
    -1059940091410581504L, -3124903886851545088L,  8110537265978517504L,  3926228294152232960L, 
     8096405493317296128L,   571036309262508032L, -8821178011922892800L,  5910172278096365568L, 
    -3920005836889573376L,  6776958678934827008L,  6644716665918961664L,   934002343965669376L, 
     -788810479286028288L,  6456811144455262208L, -2807993781576906752L, -4824543029912934400L, 
    -8283735324170434560L, -6733827159294779392L,  1441306388126627840L,  7459299456639174656L, 
     5002136664085577728L,  3765775772733980672L, -1431263366079385600L,  8445962682579369984L, 
     4714789374144813056L, -2471799534089781248L,   933666127329589248L, -5138719748295268352L, 
    -4502023941038114816L,  4339659983190228992L, -3402590077680336896L, -1431369475806953472L, 
     4582945456769808384L, -2710028999788853248L,  -987665837872291840L, -8597380446344308736L, 
     4206124219313866752L,  2238989382063820800L, -6902869412121651200L, -5248911763648468992L, 
    -3534815472743970816L, -8652410086236432384L, -5507181506214017024L, -7979511965376866304L, 
    -9220563527899006976L,  2516679149009727488L,  5584824474597457920L, -1949118660730118144L, 
    -4376751049921730560L, -3987662997398525952L,  3941042146223450112L, -5642034653951107072L, 
     1477223361924476928L, -4658310957294987264L,  8041061134976933888L,  1190863272955351040L, 
#endif
}; 

const int64_t test_divisor[256] = {
#if SIGNED_DIV_TRUNCATED == 1
     3063088654995032064L,  1986947083846514688L, -1484013202390663168L, -7179516211910420480L, 
     5840505559165552640L, -4624514261179719680L,   169311578085447680L,  5610257568331294720L, 
      181332060042676224L, -3639542544007452672L,  6676367759143741440L, -8164633739365277696L, 
     4256712154275889152L,  7394925112886585344L,  -663282770752708608L,  -565135003717634048L, 
    -6386002134059876352L,  8518612698334025728L,  6940673719952101376L,  -209752807896012800L, 
    -1714135411534573568L, -6888451171551422464L,  7847711280844220416L, -9120418502698928128L, 
    -5785112808820443136L, -3245501131568693248L, -8297592549240469504L, -6557137620988602368L, 
     4231119274740602880L,  -326883607771303936L, -2987178138299842560L, -4855790927282059264L, 
     -906413850532521984L, -5802405093022752768L, -3241926519756926976L, -4353957654202449920L, 
     6088849147907002368L,  3622203526862391296L, -3070693091661637632L,  1480028242717124608L, 
    -3913694357278033920L, -4353972813308157952L, -4429039907968618496L,  3266550096812666880L, 
      365735279268104192L, -7807515538240731136L, -8193208909501542400L, -4450676943464509440L, 
    -1108143530724941824L, -3979163463680966656L,  3297684776184072192L,  8293278856686450688L, 
     5053579420280512512L,  2511321450937483264L,  4677752482569428992L,  4552962734148321280L, 
     1586994673906796544L,  5037000342504681472L, -1983241584607184896L,  1942259880029464576L, 
    -4659041138303741952L, -3870086386904516608L, -8866748572821934080L, -2816639429525204992L, 
    -6607702093755629568L, -1631896588528996352L, -6399759352743962624L,  6069143225541978112L, 
     4412452484394209280L, -7399536800310235136L,  5914636524176982016L, -5033902305947336704L, 
    -7250652362018013184L,  3002686479326846976L,  8385449122091780096L,  5802690074000531456L, 
     2273612551796754432L, -3167882410864398336L, -4106835494880485376L, -1210533289554012160L, 
    -2761559778613794816L,  6968427095804590080L, -9109841673839521792L,  3623848727813763072L, 
    -2987208413590710272L, -3597416328057665536L,  2731975703306584064L,  7768777592126029824L, 
     7250931441956820992L,  9171339817846663168L, -7878901547856281600L, -6832330456121989120L, 
     8883365897874077696L, -7558695626752731136L,  3434021126557087744L,  7913003906092294144L, 
    -6607321736460509184L,  7091217182894735360L, -8858828194540204032L, -2901933004111566848L, 
    -4827400628596877312L,  8938628113097795584L,  6393602796843053056L,  5432780045437186048L, 
     7383827054420291584L,  5167020060108552192L,  6207528082614079488L, -3300246763851411456L, 
     4477865513683343360L,  3034383976919216128L, -3888874506028259328L, -2999952797537320960L, 
     7538023249642649600L, -8628446454527944704L,  3622160754493247488L, -5372248672836974592L, 
     5979500217839114240L, -5197697319127855104L, -7385295975206856704L,  2204493211295807488L, 
    -7308327310222043136L,  5516716553040019456L,  7432656556281270272L, -3458527547192125440L, 
    -4028968754786301952L, -9098267301959960576L,   -76154326895511552L,  9010906921399252992L, 
     4389232147543963648L, -3491603086802722816L,  1852191462058575872L,  5196075673144465408L, 
    -7165949030050490368L,  1463367098906390528L,  6832143113357793280L,  3500753907844825088L, 
    -4741362000578498560L, -2901262368247910400L,   838216139546302464L, -7976875868443533312L, 
    -1651935452152080384L, -4842060077574985728L,  -203401982676254720L,  5645932023916498944L, 
    -2253307573601165312L,   331638230647959552L, -7478348982195357696L,  7546474324285771776L, 
    -5393263607513511936L, -2175525551458844672L,  2956649031817504768L,  4766146182593570816L, 
    -6030804879470483456L,   320600012569303040L,  9137375387616440320L,  3829717590673514496L, 
    -7737167656001480704L, -8424478666703712256L,  -163151066105391104L,  -985121800823754752L, 
     -243523477256839168L, -6163220963645968384L, -2570433794048706560L,  7023080222960973824L, 
     4507502128242903040L, -1535303123212695552L,  7514369899137970176L, -7483717271999725568L, 
    -5878492906010404864L,  8238080378669826048L, -7363035294876340224L, -2065333069184389120L, 
    -3888115027325898752L, -7875157420570742784L, -5633487965861826560L, -1522128824139024384L, 
    -3819830767934169088L,  3728797394453694464L, -4801444392386285568L,  8475958310517932032L, 
    -3588588966703663104L, -6365686226136561664L,  1023949605205555200L,  5359585075284779008L, 
    -1035373435629350912L,  9146241066660882432L, -1169768949864232960L, -3607423633520265216L, 
    -4676058866440194048L,  8500725479974516736L, -5111944037633605632L, -1925670713137512448L, 
    -5081625063201323008L, -4242296073937805312L, -1504489163869505536L,  9181608235155580928L, 
     7582172667824574464L,   930211910578173952L,  1777082138758914048L, -7763400062781546496L, 
     1413685368566685696L,  7344982736178036736L,  -676800575976439808L, -1874255110446372864L, 
    -7296404586514456576L,  2808265700661944320L,  9070264159799504896L,  3284863662744133632L, 
    -1319355525772204032L,  2855580361382234112L,  1637066053863213056L,  4520465213346809856L, 
     2598598153457385472L,    67813666128273408L,  8080438997848449024L,  1943246071532709888L, 
     2562690828431435776L,  3738369454939949056L,  6658335470337064960L, -2219836589700515840L, 
     3912976542356328448L,   433875454513088512L, -2518175385926178816L, -1205017450441240576L, 
     3460565313251328000L, -5038707005350731776L,  8836056911285483520L,  8774580077798719488L, 
    -3882785042522421248L, -2980379203939225600L,  9157787682718769152L,  5330565355401541632L, 
     5440342317204856832L,  2441774736992016384L,  5745252933081763840L,  -957510515407036416L, 
     6098743592415139840L, -6886846295157757952L,   244753758383478784L,  3983228549675960320L, 
    -4645879771412074496L,   588457134260135936L, -2172847833230028800L,  5566512420488075264L, 
     3151824921000558592L,  8907179801857912832L,  8057931255149375488L,  1406976595281752064L, 
    -7744197966464159744L, -1589651083137105920L, -5887370153411747840L,  9142781665829031936L, 
      376058909805015040L,  7107211279887460352L,  2734757874370533376L,  -622511381888434176L, 
#elif SIGNED_DIV_MODULO == 1
     3988033325263450112L,  4056875866223273984L,  4448851156101777408L,  3493318892245583872L, 
    -4419549597544499200L, -9041302176891824128L, -2765142056988108800L,  7307876067708446720L, 
     4486912936415311872L, -3800282002694365184L, -5822851196776067072L, -6217360227925245952L, 
    -7825292942620342272L, -1162537044038014976L, -4212872241946376192L, -4849549759573352448L, 
    -6520013447187269632L,  -538188635376433152L,  4216701588059084800L, -2003325286978424832L, 
    -6345375758996795392L, -6611454011406635008L,  7461994683592300544L, -2722410262065274880L, 
    -6281826511949111296L,  6859919014154123264L,  5602505513326723072L,  4319395336044124160L, 
     -144415460454957056L, -1810697726755909632L,  6688809163019552768L, -5887529287780812800L, 
     7850341112413511680L,  5940519779107162112L,  3441755832170940416L,  -568882859797016576L, 
     7156211816267962368L,  8897951128810442752L, -8626887314881234944L,  5991000682095616000L, 
     2938897089392658432L,  -954080519462375424L,  1359884825279467520L,  5495575830591795200L, 
     2448953674096476160L,  5344092836122605568L, -8124697107950766080L, -2040107435563522048L, 
    -1471420808556904448L,  3026597681064577024L,  2480546679291228160L,  7107590376133251072L, 
     2056950538850213888L,  2127808082372554752L,  6957405450844510208L,  4922758185397760000L, 
     4613364360750946304L, -7090524567367510016L, -5458584785535287296L, -7078750019073335296L, 
     6298821457551130624L, -2184576987608569856L,  -847705713732888576L, -7860389279294386176L, 
    -8759160272691951616L,  8506061537825239040L, -6301508600158461952L,  7999290791756038144L, 
    -4855826809780398080L, -7829647390355267584L,  3183087444703401984L, -7667804885749800960L, 
    -8076582378127486976L,  3600230751367581696L, -9185391014932713472L, -6329865790478628864L, 
    -4158812536429203456L, -3408421536516820992L,  3948267359718658048L,  1324228746601467904L, 
      280264369545725952L,  1426485810692276224L, -9132195067320713216L, -2725788450719965184L, 
      825209739433699328L, -7525090012726607872L, -4199883248032313344L,  1673636475761412096L, 
    -9138012976126300160L, -8656121092542720000L, -2274351185788971008L, -6170153463178070016L, 
    -3574521162504607744L, -4243221771109445632L, -3556354259051081728L,  1549931043011287040L, 
      710541544438900736L,  5361928488089106432L, -2347921173393506304L,  4648750252783691776L, 
     2814587992192460800L,   958106841877739520L,  6473327559112595456L, -1378918826292742144L, 
     1372449307262201856L,  4948412004232548352L,   728913269699305472L,  8933210698051692544L, 
    -7275669437397762048L, -3670813737772539904L, -2216306017065658368L,  -503302288867459072L, 
    -5117703808424407040L, -3306717530916071424L, -5038711387777630208L, -8822011461507045376L, 
    -8937938464259647488L,  5236105599356975104L, -8879393947164325888L,  7429004844349696000L, 
    -5596654786616924160L,  2011335717005586432L,   421897519504953344L,  3813501497939208192L, 
    -2708051372953317376L,  4775758308157984768L,   102853549141069824L,  4902672389785755648L, 
    -2535838052562014208L,  7840190093916155904L,  7398974572839766016L,   953971148960679936L, 
      387843665397471232L, -5920950728634718208L,  2009279427244075008L,  4205122944020690944L, 
     6934896812793565184L,  -492125597796872192L,  4652523068354682880L, -5883620423777921024L, 
    -8829796825663481856L, -6838735082574407680L,  6641791269774391296L,  5473320813135663104L, 
    -5762906339968124928L, -6065012652048953344L,  3238665400432392192L, -1846620517537118208L, 
     8034052938045966336L,  -438384601117691904L, -1803431403023798272L, -5718057890475941888L, 
    -3005605553484541952L, -1088713572751278080L, -6678021041949188096L,  8860687015111036928L, 
    -1113259864440385536L,  5604893417119897600L,  5859885378259210240L, -4952437205711417344L, 
     2605756896660877312L, -4467620058569627648L, -6934003229343299584L, -1714481440356136960L, 
    -7974666083773708288L, -7198652373221875712L,   545216053955620864L, -5321849332119238656L, 
    -8744357622196142080L,  9114904313169752064L,  4512826424654239744L,  8895667721291315200L, 
     8326528171493013504L,  -523744209569079296L,  8993444418029613056L,  6510245437798942720L, 
    -3521908175878789120L,  8333060065128849408L, -7126923897931143168L, -7060052566724415488L, 
    -9210543901740742656L, -3631122886330118144L, -2413178870058113024L, -7003470144675756032L, 
     3909090708590604288L,  8439639200999714816L, -3278203987704201216L,  5330809993432342528L, 
     3163356354083305472L, -7201344359938693120L,  4131379807416942592L,  7057762998034894848L, 
    -9214275473564266496L, -7342844744569532416L, -2390992654399393792L,  7649665412683718656L, 
     9204718659372103680L, -6078314917744558080L, -6039144301476972544L, -7322058594431025152L, 
     1958079424463890432L, -1625339807202322432L, -4505012963861835776L, -5502943302595434496L, 
    -5761116583314980864L,  7079352109392025600L,  2424156578921164800L,  2258369032153917440L, 
     2919541915808393216L, -5956740126119000064L,  2321498926244204544L,  6944381098169755648L, 
    -5936107298127814656L,  2685364094628061184L,  7085802410712877056L, -4602163142037399552L, 
     1590499591969091584L,  -242148879284334592L,  6255037956758388736L, -8270126334893842432L, 
     4417489343071035392L,  1005643933705791488L, -1301003881200087040L,  5159970155523252224L, 
    -1906151578045253632L, -2633658887005108224L,  2941747405287401472L, -1952184165105999872L, 
    -5931042539232032768L, -8221805503833493504L, -4664789748879396864L,  7412147886438141952L, 
     8899504271784974336L,  -293137389472497664L, -6553213709981093888L,   215210297262925824L, 
     9103977656441010176L,  4843541323200393216L,  8596621264188633088L,  7929896489073700864L, 
    -4860891140066615296L, -7424405747258929152L, -1664369929546678272L, -6803664496917661696L, 
     4695927289664319488L, -2802002521838510080L,  -926326777363599360L, -5546281422988142592L, 
     4362986990623387648L, -5471904538466461696L,  4946685687599095808L, -4772538974990694400L, 
     2098012540304080896L, -2845007981022390272L,  4106831863461797888L, -7060987233267769344L,
#else //SIGNED_DIV_EUCLIDEAN == 1
     7002551751604414464L,  9088270636865056768L,  8231913322416250880L,  -398887161432401920L, 
    -6917635276176689152L, -8039902989988349952L, -2090663943372752896L,  1074443360432848896L, 
     7792443799908605952L,  8011577036766480384L,  5991722696106225664L, -5836879559385843712L, 
    -1971807643705573376L,  7614486624801804288L,   630330638601805824L,  -895375464960755712L, 
    -3955045875549530112L,  2264782354376335360L, -5870261501108094976L,  7879451623415357440L, 
     3947116716320628736L,  6131103188357552128L,   -64884535386372096L,  8544843335554830336L, 
     6333029185470918656L, -7122332962620618752L,  8046087374449266688L,  3474708856005433344L, 
    -3558159420925456384L,  6652902228123199488L,  8272599074991986688L,  3354378604733243392L, 
     1165743888977594368L, -6456838397445408768L,  8968996296008921088L,  5811164523273056256L, 
     7604116419815438336L,  3393571910624587776L, -3633440597633886208L,  4275907662309222400L, 
    -4128190352078766080L,  2125779251687903232L, -3784310442373466112L,   337122926909444096L, 
     4407365891176230912L,  4501394515374581760L, -1780858964727197696L, -6254766642738442240L, 
    -8810147127740684288L, -7270257212569022464L,  7968975487180951552L,  2131511249136234496L, 
     2458655989814337536L, -9100015854369693696L, -5259799870567913472L, -7729126268561080320L, 
    -6244871746816892928L, -7529895199993460736L, -6749049360097099776L, -7898966362370084864L, 
    -4264014128316043264L, -8484271780608735232L,  4502483197156483072L,  8205723012265830400L, 
    -5139990850980151296L,  1855103639552499712L, -1178871578978828288L, -6021744996157806592L, 
    -5628212613637615616L, -5734708370020683776L,  7930101337714606080L,  -174436068812804096L, 
     5466220634988972032L, -5814430064483375104L, -5021489381909895168L,  6452825537045446656L, 
     -105384948997079040L,  8960297038866522112L, -6046351691093331968L,  8302936021079783424L, 
    -2170534438386294784L,  -965994025700059136L, -2584549204007737344L, -6558426042299813888L, 
     5840957050902568960L, -5871386685987483648L,  6142263888385769472L, -1728540292803383296L, 
    -1448976827005995008L,   613431393561860096L, -1285415674513098752L, -9178706798569351168L, 
     3037430021781696512L, -4486470918682306560L,  1695687017144567808L, -2622211648256155648L, 
      886433929406547968L,  5177790237820160000L,  1086150546113529856L, -3658130676681693184L, 
     -726528252742303744L,  7764572519561744384L,  7123782722506354688L,  7000056340238927872L, 
     8676298643877904384L,  4654209038079322112L, -8936648938270449664L,  9082105395601852416L, 
     -816025560205668352L,   378126062653667328L, -7583585181068851200L,  5259251183740344320L, 
     4015742383024422912L,   837511171394863104L,  4503508552213911552L,  3455504853535096832L, 
     2053003650360801280L, -7704439058164566016L, -2268904225909757952L, -1753000571591188480L, 
     3456513989647042560L, -1752169838262149120L,  1004732815242428416L,  2181613041242744832L, 
    -1522561889239480320L,  6361890165494030336L,   950190268984446976L,  5776593769535664128L, 
    -3405361159542749184L,  6629077624761909248L, -5213694544194983936L, -7095435504747386880L, 
    -7639322314385186816L, -6370904462157117440L,  5574238324049496064L,  8568000438887884800L, 
    -5593197849170616320L,  5129109615238363136L, -8734849523859793920L, -7122677564683960320L, 
    -5780793002332973056L,  7847342423473747968L,   916986485770952704L, -8648326935469727744L, 
     6946221769558276096L,  8304951596676980736L,  -317268572846972928L, -1237750990157340672L, 
     2210061789617975296L, -4975894778353541120L,  3699466452725325824L,  8872745603027716096L, 
    -6207049027828981760L,  2412550201039253504L,  2615586768112664576L,  2518482449529040896L, 
     8987526126703718400L, -2086069166835365888L, -4395406924952678400L,  -778293278050463744L, 
    -7113539666604107776L, -2617503414210719744L,  5263760047724804096L, -3594984322401261568L, 
     4255184232426805248L,  2676214000597796864L,  6260189718997852160L,   380192249907759104L, 
    -2187615006119720960L,  2199686604553641984L,  1932484912780132352L,   655867878857420800L, 
     4244810653099415552L, -2281617184182640640L, -6977630735430834176L, -2454849930322728960L, 
     5732873868705986560L,  7336343228515923968L, -2827622743921278976L, -8930013618704680960L, 
    -8692413027269296128L,  6449761522808870912L,  8108432918530299904L,  4846066093656942592L, 
     8592542253152286720L,  -134336992274008064L,  1013522322382735360L,  2808280223789019136L, 
     8727690644434466816L,  7447497615281584128L, -8761378249192071168L,  1371017758879844352L, 
     4231235779645276160L, -6425548757958158336L, -8310582417380214784L,   644504973390485504L, 
    -8487936731887839232L,     -373602983788544L, -5343386737707452416L,  -442892799042185216L, 
     2841709381144770560L, -8319983472968916992L,  6609714595843829760L, -8494573147523627008L, 
     2592778870745176064L,   118696348747581440L,  1422537492347267072L,  6992324372631635968L, 
    -5822651168367073280L, -5507140067842740224L, -6789131327042738176L,  5943665725293613056L, 
     4341312117086087168L,  7258564434312224768L, -6277705913885208576L, -1433459976180975616L, 
     2377927921983195136L,  1785300075140370432L, -3767941747768817664L,  5202424882908350464L, 
    -2180917336795054080L,  8942450116910821376L, -5439428690395115520L,  5160482095563601920L, 
     6364491983886579712L,  -714163325455529984L,  3304281453302798336L,  5481540035414626304L, 
    -4877783821635575808L,  4652817060270385152L,  7748779312289314816L,  9124467972778092544L, 
     7064421797068734464L,    62754076564623360L, -9114901361178345472L, -8945077343493832704L, 
    -7453649775338221568L,  5293481213977495552L,   988608848222564352L,  4049413323369566208L, 
    -5008687960393248768L, -8989247848203476992L, -7005633451690356736L,  2453015696936448000L, 
    -8942782327881134080L,  8121470560522934272L,  9215362500540016640L, -2246470725442396160L, 
     7290260784311545856L,  1661130154657703936L,  6180686158218067968L,  -325660779775066112L, 
    -8126878324900976640L,  -731666641205368832L, -3344200531614496768L,  7566124820985886720L,  
#endif
}; 

// End of File
