//#############################################################################
//! \file iirOutput.c
//! \brief  Ouput Vector (512) 
//! \author Vishal Coelho 
//! \date   23-Feb-2016
//! 
//
//  Group:          C2000
//  Target Family:  $DEVICE$
//
//#############################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//#############################################################################


const double test_golden[512] = {
     0.013544649591L,  0.053789346410L,  0.124276243883L,  0.237723689586L, 
     0.412260934597L,  0.659519790596L,  0.977048032943L,  1.352794975672L, 
     1.769208561751L,  2.199186492579L,  2.602638061564L,  2.930354784324L, 
     3.130377356272L,  3.154010737097L,  2.964383542577L,  2.544255178908L, 
     1.896522552501L,  1.042440207534L,  0.026619620537L, -1.078191168518L, 
    -2.180491179217L, -3.187882989443L, -4.014819793845L, -4.580480069534L, 
    -4.814838615903L, -4.675753914120L, -4.159210416901L, -3.294817951666L, 
    -2.139154356370L, -0.775155814156L,  0.690449548792L,  2.137428943737L, 
     3.444334811616L,  4.499288598199L,  5.211868263141L,  5.522874423284L, 
     5.405710302136L,  4.864691326365L,  3.939533694887L,  2.709074040466L, 
     1.280295546389L, -0.231523643258L, -1.712650029619L, -3.050522695354L, 
    -4.140841814323L, -4.904784607643L, -5.298703950496L, -5.308970436141L, 
    -4.945263025302L, -4.241133470447L, -3.254139479443L, -2.059662276998L, 
    -0.743635789574L,  0.603018351297L,  1.891635481340L,  3.042232990375L, 
     3.983480295382L,  4.652039076014L,  5.000855318891L,  5.009491805236L, 
     4.682172022428L,  4.038418379486L,  3.113770993693L,  1.969152659099L, 
     0.689715198639L, -0.630468712337L, -1.900419549518L, -3.033032769612L, 
    -3.946747399742L, -4.575499820291L, -4.877940219096L, -4.839055217882L, 
    -4.468872180365L, -3.800853839138L, -2.886671952354L, -1.790105088689L, 
    -0.586067229933L,  0.640593787121L,  1.805617162307L,  2.837858326680L, 
     3.679335649941L,  4.277848164846L,  4.590120731300L,  4.594625326522L, 
     4.295321883107L,  3.712644153544L,  2.878565665330L,  1.842584940516L, 
     0.675214680894L, -0.539540423349L, -1.714311707534L, -2.764211590484L, 
    -3.612444847328L, -4.197279666574L, -4.475914896408L, -4.426901720974L, 
    -4.056218356882L, -3.400953053842L, -2.521077990553L, -1.486506664136L, 
    -0.374183826416L,  0.729186565408L,  1.735753711945L,  2.574532775744L, 
     3.199150392036L,  3.581176846651L,  3.705971888862L,  3.578462690046L, 
     3.226256022327L,  2.692307117905L,  2.025956605571L,  1.279207128527L, 
     0.503834382695L, -0.252765703652L, -0.948995682980L, -1.549728312580L, 
    -2.031535745449L, -2.386590866992L, -2.615893992409L, -2.719177883580L, 
    -2.696289242869L, -2.555469479831L, -2.311084447039L, -1.972039859879L, 
    -1.539777388869L, -1.019347241928L, -0.426232733557L,  0.218626085044L, 
     0.892980818613L,  1.568752251967L,  2.207619252908L,  2.763551932096L, 
     3.187691595309L,  3.432977213152L,  3.461642360516L,  3.252447862113L, 
     2.801169663880L,  2.119324335976L,  1.240084895670L,  0.224227452844L, 
    -0.848002764329L, -1.893554714680L, -2.833270031888L, -3.588887714782L, 
    -4.088418852486L, -4.282695695072L, -4.155239000058L, -3.717596424521L, 
    -3.002841699185L, -2.065581877384L, -0.980615606742L,  0.165418513767L, 
     1.283677563344L,  2.290653437564L,  3.116506321509L,  3.711386013315L, 
     4.043983785696L,  4.098150434686L,  3.877270080415L,  3.409712538087L, 
     2.741470946093L,  1.921558269731L,  0.998547492897L,  0.027744636582L, 
    -0.929498604843L, -1.817812769872L, -2.594219050452L, -3.222032598247L, 
    -3.665721325252L, -3.895382690666L, -3.892681574603L, -3.651695392657L, 
    -3.179106632625L, -2.495742803159L, -1.635378694246L, -0.642772547430L, 
     0.423740956182L,  1.491017391177L,  2.480705522835L,  3.322520934251L, 
     3.956513453565L,  4.328821333230L,  4.398005113288L,  4.150151706589L, 
     3.603233333301L,  2.796756182368L,  1.783835590088L,  0.633344248239L, 
    -0.570240989279L, -1.736883002230L, -2.781985486699L, -3.631444762036L, 
    -4.225212720210L, -4.521594293958L, -4.498435483937L, -4.153318603097L, 
    -3.508122326002L, -2.612003092821L, -1.533277223491L, -0.347299890805L, 
     0.865132953702L,  2.013633704334L,  3.005801678265L,  3.765080984928L, 
     4.239517081169L,  4.397184201062L,  4.224714700328L,  3.735561839667L, 
     2.974577637918L,  2.010561802638L,  0.925472878747L, -0.192485759477L, 
    -1.255666944283L, -2.185068252448L, -2.915174512761L, -3.397261289584L, 
    -3.606390552730L, -3.546214913684L, -3.242201021116L, -2.730507311131L, 
    -2.057192315617L, -1.283170525422L, -0.477663576737L,  0.298218829447L, 
     0.998372279021L,  1.585095574548L,  2.025957438928L,  2.301433172869L, 
     2.410458299471L,  2.365774803527L,  2.187507897829L,  1.901515314698L, 
     1.537878696287L,  1.127355900310L,  0.699858873270L,  0.283162169451L, 
    -0.103194105290L, -0.452640696498L, -0.765377533242L, -1.038455265068L, 
    -1.266929544691L, -1.451620397777L, -1.596222392891L, -1.695234601118L, 
    -1.731817368696L, -1.689214015792L, -1.558454217064L, -1.334719805029L, 
    -1.014846621332L, -0.603599256398L, -0.119727176245L,  0.405198408558L, 
     0.930665123192L,  1.411231570846L,  1.803742167242L,  2.074522780599L, 
     2.199997587601L,  2.165677970486L,  1.972516883487L,  1.643583744033L, 
     1.217112069698L,  0.731636849651L,  0.221824071840L, -0.275285536685L, 
    -0.721845526227L, -1.091720913957L, -1.376572144107L, -1.578216923311L, 
    -1.700514850620L, -1.749575458065L, -1.734692090608L, -1.664339501137L, 
    -1.542671966626L, -1.369344934360L, -1.139008536891L, -0.842415560426L, 
    -0.474053071576L, -0.039746982228L,  0.445936034883L,  0.966009580997L, 
     1.497112315611L,  2.000470390425L,  2.425574477729L,  2.725352259502L, 
     2.863607570635L,  2.810565718013L,  2.542920342301L,  2.054850946769L, 
     1.366287757820L,  0.519637850900L, -0.426879835845L, -1.402656326553L, 
    -2.329157524660L, -3.127208917461L, -3.722715959537L, -4.052406427426L, 
    -4.074440260379L, -3.777409042404L, -3.177631796246L, -2.311290917202L, 
    -1.235526568878L, -0.033577025445L,  1.193338451725L,  2.345536219034L, 
     3.335943669113L,  4.088187379984L,  4.538053798784L,  4.645086363047L, 
     4.401063657320L,  3.826996688483L,  2.967003255363L,  1.885644130367L, 
     0.664209635953L, -0.606069908907L, -1.829656532791L, -2.912789126103L, 
    -3.774830640553L, -4.358414196571L, -4.628417834442L, -4.566501774549L, 
    -4.175450878107L, -3.488318591053L, -2.564688138710L, -1.475297479757L, 
    -0.294203542601L,  0.897473844712L,  2.012789326894L,  2.971842850794L, 
     3.712726000648L,  4.191864842287L,  4.381913191417L,  4.273698686543L, 
     3.877642034675L,  3.222417925744L,  2.354655584968L,  1.337436804716L, 
     0.242107975584L, -0.860634957112L, -1.900598179672L, -2.805435641737L, 
    -3.509297922495L, -3.967257430640L, -4.157354428500L, -4.071140042481L, 
    -3.711574969193L, -3.101975619673L, -2.289024169001L, -1.332746469917L, 
    -0.296683932378L,  0.753140309402L,  1.748381121969L,  2.623686648144L, 
     3.322061004402L,  3.797662678476L,  4.019721584307L,  3.975446061847L, 
     3.666602066841L,  3.105716213988L,  2.321358079174L,  1.365282993429L, 
     0.306802623155L, -0.780820661439L, -1.824206517880L, -2.744757283732L, 
    -3.463033462988L, -3.915553373257L, -4.066149143803L, -3.903866102913L, 
    -3.439740355553L, -2.710235474983L, -1.778758522730L, -0.728876117873L, 
     0.345528550076L,  1.349825505472L,  2.200361770380L,  2.834665608205L, 
     3.213366826889L,  3.318836818009L,  3.159819367593L,  2.775441404332L, 
     2.225157764444L,  1.570825423000L,  0.869888694293L,  0.179629727254L, 
    -0.445669495429L, -0.967853353524L, -1.371080225997L, -1.653968523782L, 
    -1.821106987449L, -1.882826183021L, -1.855650476531L, -1.757717738663L, 
    -1.604519409741L, -1.407711744343L, -1.173330400097L, -0.901420651790L, 
    -0.592154583479L, -0.252119150115L,  0.109136597286L,  0.485579388837L, 
     0.870045293833L,  1.243643464404L,  1.577139182734L,  1.843371170703L, 
     2.021850887697L,  2.091800222875L,  2.030407823317L,  1.823058130617L, 
     1.471945185836L,  0.994249544490L,  0.417829907436L, -0.219253202495L, 
    -0.870046544557L, -1.482996892329L, -2.005154156604L, -2.386078779373L, 
    -2.587388872494L, -2.591453414597L, -2.399048864369L, -2.022402751969L, 
    -1.487631748083L, -0.841587395860L, -0.145774809944L,  0.540801519471L, 
     1.170033463281L,  1.699855589874L,  2.092638083744L,  2.323825224284L, 
     2.387889866233L,  2.293621832102L,  2.057480928802L,  1.701890442316L, 
     1.254096504843L,  0.743494729055L,  0.201353946190L, -0.339189592128L, 
    -0.848350884580L, -1.304401543109L, -1.688585493017L, -1.977521991992L, 
    -2.147644631007L, -2.186691672228L, -2.094291604527L, -1.872369744472L, 
    -1.523862247523L, -1.063023809659L, -0.520056915592L,  0.067163988920L, 
     0.661088785775L,  1.223674917786L,  1.714840769250L,  2.096447950725L, 
     2.336878974998L,  2.413582005931L,  2.316999116640L,  2.053308855007L, 
     1.640243158434L,  1.101746654966L,  0.471124656631L, -0.203997424817L, 
    -0.868824658928L, -1.475233310840L, -1.984234738474L, -2.357982098299L, 
    -2.560144704616L, -2.568203103677L, -2.380425436925L, -2.010066147210L, 
    -1.479578440387L, -0.822990918504L, -0.088131623859L,  0.667606127767L, 
     1.382951495768L,  1.997192317332L,  2.456762221979L,  2.721426368919L, 
     2.764252357555L,  2.570834909634L,  2.146841445299L,  1.526429667854L, 
     0.766864631253L, -0.065723434351L, -0.901401927451L, -1.662793847043L, 
    -2.270774237596L, -2.662168120083L, -2.801427333738L, -2.678273184761L, 
    -2.303782259152L, -1.712938573341L, -0.965350815130L, -0.138001380347L, 
     0.684576904953L,  1.419334990613L,  1.995480483996L,  2.363518353965L, 
     2.496048011322L,  2.385560014438L,  2.048669426979L,  1.530225022392L, 
     0.893768126268L,  0.204343312891L, -0.477618284501L, -1.091653679995L, 
    -1.580216998218L, -1.902507590710L, -2.041888602839L, -1.999586250505L, 
    -1.787759227374L, -1.430469670330L, -0.964634203683L, -0.434997720370L, 
}; 


// End of File
