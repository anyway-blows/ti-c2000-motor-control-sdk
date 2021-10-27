//#############################################################################
//! \file   golden.c
//! \brief  Ouput Vector (1024) 
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

const float test_golden[514] = {
      52.477014878165F,   52.948992355413F,   54.403738872800F,   56.965986930740F,
      60.874745022984F,   66.544715152272F,   74.693827791907F,   86.622899818053F,
     104.898760471038F,  135.322549097098F,  194.282033604989F,  353.108820202073F,
    2178.567714361938F, -502.044491544014F, -210.791224201451F,  201.290636418757F,
    -133.334808121703F,  -99.983101084492F,  -82.051007839738F,  -70.270814135173F,
     -61.861206803053F,  -55.548815198492F,  -50.641558651124F,  -46.724642040473F,
     -43.532536287749F,  -40.886803104322F,  -38.662903005378F,  -36.771213822283F,
     -35.145583301915F,  -33.736123926230F,  -32.504514581280F,  -31.420843907055F,
     -30.461434267943F,  -29.607307797717F,  -28.843083587376F,  -28.156170876234F,
     -27.536169511607F,  -26.974418126571F,  -26.463649281796F,  -25.997723184894F,
     -25.571419896451F,  -25.180275594415F,  -24.820452395964F,  -24.488633999900F,
     -24.181941384223F,  -23.897864217403F,  -23.634204682093F,  -23.389031178216F,
     -23.160639945163F,  -22.947523074179F,  -22.748341709426F,  -22.561903486991F,
     -22.387143454519F,  -22.223107864483F,  -22.068940351744F,  -21.923870098664F,
     -21.787201664476F,  -21.658306214084F,  -21.536613928362F,  -21.421607415805F,
     -21.312815975954F,  -21.209810589892F,  -21.112199533454F,  -21.019624525480F,
     -20.931757337177F,  -20.848296800074F,  -20.768966159459F,  -20.693510728126F,
     -20.621695801799F,  -20.553304803184F,  -20.488137626222F,  -20.426009156090F,
     -20.366747943804F,  -20.310195017134F,  -20.256202811955F,  -20.204634210210F,
     -20.155361672455F,  -20.108266454448F,  -20.063237898582F,  -20.020172792055F,
     -19.978974784689F,  -19.939553860124F,  -19.901825854859F,  -19.865712020259F,
     -19.831138623199F,  -19.798036581507F,  -19.766341130791F,  -19.735991519614F,
     -19.706930730318F,  -19.679105223073F,  -19.652464700991F,  -19.626961894382F,
     -19.602552362404F,  -19.579194310559F,  -19.556848422643F,  -19.535477705875F,
     -19.515047348090F,  -19.495524585960F,  -19.476878583326F,  -19.459080318809F,
     -19.442102481939F,  -19.425919377127F,  -19.410506834853F,  -19.395842129510F,
     -19.381903903404F,  -19.368672096436F,  -19.356127881045F,  -19.344253602044F,
     -19.333032720986F,  -19.322449764750F,  -19.312490278062F,  -19.303140779694F,
     -19.294388722080F,  -19.286222454172F,  -19.278631187304F,  -19.271604963908F,
     -19.265134628909F,  -19.259211803669F,  -19.253828862328F,  -19.248978910445F,
     -19.244655765818F,  -19.240853941405F,  -19.237568630241F,  -19.234795692309F,
     -19.232531643267F,  -19.230773645014F,  -19.229519498021F,  -19.228767635403F,
     -19.228517118710F,    0.221351015806F,    0.442773645805F,    0.664339599805F,
       0.886120779136F,    1.108189373098F,    1.330617956269F,    1.553479586931F,
       1.776847906926F,    2.000797243238F,    2.225402711606F,    2.450740322497F,
       2.676887089759F,    2.903921142323F,    3.131921839291F,    3.360969888817F,
       3.591147471161F,    3.822538366361F,    4.055228086965F,    4.289304016302F,
       4.524855552812F,    4.761974260974F,    5.000754029424F,    5.241291236893F,
       5.483684926643F,    5.728036990122F,    5.974452360642F,    6.223039217927F,
       6.473909204447F,    6.727177654560F,    6.982963837528F,    7.241391215627F,
       7.502587718607F,    7.766686035954F,    8.033823928464F,    8.304144560857F,
       8.577796857247F,    8.854935881542F,    9.135723244986F,    9.420327543326F,
       9.708924826313F,   10.001699102542F,   10.298842882954F,   10.600557766672F,
      10.907055073244F,   11.218556525835F,   11.535294990388F,   11.857515276379F,
      12.185475005428F,   12.519445554759F,   12.859713083366F,   13.206579649664F,
      13.560364430520F,   13.921405052784F,   14.290059049875F,   14.666705457607F,
      15.051746565312F,   15.445609840495F,   15.848750047751F,   16.261651585566F,
      16.684831067999F,   17.118840182155F,   17.564268856919F,   18.021748783797F,
      18.491957336969F,   18.975621947073F,   19.473524991970F,   19.986509278113F,
      20.515484198443F,   21.061432667444F,   21.625418951611F,   22.208597534732F,
      22.812223182963F,   23.437662405644F,   24.086406545553F,   24.760086778391F,
      25.460491358031F,   26.189585514085F,   26.949534495274F,   27.742730360669F,
      28.571823257171F,   29.439758093817F,   30.349817742510F,   31.305674175139F,
      32.311449308641F,   33.371787799397F,   34.491944643895F,   35.677891256032F,
      36.936444776274F,   38.275426829269F,   39.703859935783F,   41.232212524065F,
      42.872707304089F,   44.639713162147F,   46.550248464212F,   48.624634913039F,
      50.887357780000F,   53.368213512847F,   56.103864553521F,   59.139982516570F,
      62.534260295484F,   66.360739771154F,   70.716189046681F,   75.729780091287F,
      81.578292711073F,   88.511015882619F,   96.892672940146F,  107.282369986808F,
     120.591726630873F,  138.441365569460F,  164.119514286771F,  206.038229471313F,
     300.500425424380F, -1635.689796200242F,  257.055459180032F,  723.085149745699F,
    -3193.678016941223F, -507.331965081419F, -267.826810174191F, -175.847750710729F,
    -126.187696855108F,  -94.491769721952F,  -72.045254740897F,  -54.926528688618F,
     -41.088739424727F,  -29.338819489982F,  -18.912522387464F,   -9.272512467120F,
      52.477014878165F,   53.754769824701F,   57.597311624729F,   64.077219010027F,
      73.443986065072F,   86.285124259508F,  103.777100753913F,  128.188226128903F,
     164.094743322133F,  221.888764303557F,  330.872647448791F,  618.119374957519F,
    3865.971619393218F,  880.284502460680F,  332.431119624207F, 1648.028831575302F,
     328.753215553985F,  229.016096609267F,  183.487282549070F,  155.254626404359F,
     135.532923819068F,  120.810503599408F,  109.328667476108F,  100.086922754092F,
      92.466748387178F,   86.062362625983F,   80.595281884793F,   75.867449864477F,
      71.733853488678F,   68.085707675151F,   64.839703006757F,   61.930894110971F,
      59.307859171360F,   56.929322808682F,   54.761748537718F,   52.777589463810F,
      50.953995554363F,   49.271843713187F,   47.715000022112F,   46.269751563642F,
      44.924363860063F,   43.668732561043F,   42.494106679137F,   41.392866729776F,
      40.358345426131F,   39.384681663222F,   38.466700767790F,   37.599815638884F,
      36.779944628948F,   36.003442934041F,   35.267044957591F,   34.567815643397F,
      33.903109182575F,   33.270533816426F,   32.667921705038F,   32.093303026395F,
      31.544883625049F,   31.021025652334F,   30.520230738512F,   30.041125316508F,
      29.582447781099F,   29.143037219604F,   28.721823492830F,   28.317818480073F,
      27.930108330881F,   27.557846590223F,   27.200248083628F,   26.856583465461F,
      26.526174347418F,   26.208388936043F,   25.902638117904F,   25.608371939455F,
      25.325076435676F,   25.052270767622F,   24.789504634179F,   24.536355927726F,
      24.292428607201F,   24.057350765327F,   23.830772869581F,   23.612366158907F,
      23.401821180309F,   23.198846451286F,   23.003167235674F,   22.814524421850F,
      22.632673493499F,   22.457383584186F,   22.288436607949F,   22.125626458929F,
      21.968758273797F,   21.817647751403F,   21.672120524597F,   21.532011579733F,
      21.397164719770F,   21.267432067324F,   21.142673604343F,   21.022756745437F,
      20.907555942137F,   20.796952315653F,   20.690833315888F,   20.589092404714F,
      20.491628761659F,   20.398347010346F,   20.309156964174F,   20.223973389843F,
      20.142715787487F,   20.065308186250F,   19.991678954259F,   19.921760622053F,
      19.855489718578F,   19.792806618960F,   19.733655403335F,   19.677983726057F,
      19.625742694694F,   19.576886758250F,   19.531373604115F,   19.489164063288F,
      19.450222023445F,   19.414514349499F,   19.382010811282F,   19.352684018071F,
      19.326509359664F,   19.303464953772F,   19.283531599497F,   19.266692736728F,
      19.252934411265F,   19.242245245544F,   19.234616414844F,   19.230041628882F,
      19.228517118710F,    0.000000000000F,   -0.173363654476F,   -0.334564372998F,
      -0.475594011903F,   -0.593730329473F,   -0.690042659658F,   -0.767350582352F,
      -0.828817751360F,   -0.877264274533F,   -0.914904057871F,   -0.943224857902F,
      -0.962752308959F,   -0.972150897566F,    2.177692574331F,    2.257627346214F,
      -1.448350343962F,    1.988406535086F,    2.022586442649F,    2.034401286436F,
       2.040493815180F,    2.044774345866F,    2.048567650231F,    2.052404202009F,
       2.056511153624F,    2.060983753336F,    2.065855178187F,    2.071128138381F,
       2.076790272114F,    2.082822035162F,    2.089200877125F,    2.095903485696F,
       2.102906987269F,    2.110189568447F,    2.117730770848F,    2.125511600467F,
       2.133514532547F,    2.141723459065F,    2.150123606555F,    2.158701440599F,
       2.167444566537F,    2.176341631899F,    2.185382233561F,    2.194556831142F,
       2.203856667271F,    2.213273694783F,    2.222800510648F,    2.232430296200F,
       2.242156763232F,    2.251974105419F,    2.261876954604F,    2.271860341446F,
       2.281919660006F,    2.292050635855F,    2.302249297339F,    2.312511949662F,
       2.322835151502F,    2.333215693867F,    2.343650580990F,    2.354137013007F,
       2.364672370271F,    2.375254199104F,    2.385880198852F,    2.396548210105F,
       2.407256203970F,    2.418002272283F,    2.428784618674F,    2.439601550397F,
       2.450451470856F,    2.461332872747F,    2.472244331781F,    2.483184500905F,
       2.494152104995F,    2.505145935974F,    2.516164848311F,    2.527207754864F,
       2.538273623050F,    2.549361471302F,    2.560470365788F,    2.571599417376F,
       2.582747778819F,    2.593914642140F,    2.605099236209F,    2.616300824483F,
       2.627518702908F,    2.638752197964F,    2.650000664840F,    2.661263485730F,
       2.672540068248F,    2.683829843936F,    2.695132266875F,    2.706446812386F,
       2.717772975802F,    2.729110271323F,    2.740458230943F,    2.751816403433F,
       2.763184353393F,    2.774561660350F,    2.785947917913F,    2.797342732976F,
       2.808745724964F,    2.820156525116F,    2.831574775813F,    2.843000129935F,
       2.854432250255F,    2.865870808863F,    2.877315486614F,    2.888765972609F,
       2.900221963692F,    2.911683163980F,    2.923149284404F,    2.934620042275F,
       2.946095160866F,    2.957574369013F,    2.969057400727F,    2.980543994823F,
       2.992033894563F,    3.003526847309F,    3.015022604187F,    3.026520919760F,
       3.038021551714F,    3.049524260547F,    3.061028809268F,    3.072534963105F,
       3.084042489211F,    3.095551156383F,    3.107060734781F,    3.118570995653F,
       3.130081711056F,    3.141592653590F, };

// End of File
