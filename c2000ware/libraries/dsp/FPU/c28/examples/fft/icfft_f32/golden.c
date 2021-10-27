//#############################################################################
//! \file   golden.c
//! \brief  Ouput Vector (256) 
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


const float test_golden[512] = {
     5.00000000000F, -0.00000000000F,  4.15811075152F,  2.43452074461F, 
     2.20332051542F,  3.70613998382F,  0.33181173441F,  3.58355735094F, 
    -0.68024686830F,  2.81694650762F, -1.03980605837F,  2.29430105164F, 
    -1.50247974576F,  2.16530009276F, -2.42475247350F,  1.80175213766F, 
    -3.29057961438F,  0.60483533182F, -3.20663027124F, -1.19211550919F, 
    -1.86829414597F, -2.57571741549F, -0.00579719200F, -2.64706853023F, 
     1.17547016338F, -1.48639216345F,  1.04518827966F, -0.12316432836F, 
     0.07643903128F,  0.37969205953F, -0.65445618723F, -0.15194404629F, 
    -0.49571623178F, -0.95555143938F,  0.23249598583F, -1.17089990412F, 
     0.68399604806F, -0.70649407652F,  0.44032806940F, -0.26699598437F, 
    -0.01536901164F, -0.53407083870F,  0.20254576812F, -1.35410566387F, 
     1.40998491312F, -1.78452662355F,  2.90058611065F, -0.98639722544F, 
     3.50057154296F,  0.90990413287F,  2.61682245380F,  2.83217598038F, 
     0.76571309921F,  3.68031112105F, -0.92485016060F,  3.24866807860F, 
    -1.74095224764F,  2.27679452197F, -1.92959711165F,  1.61587320246F, 
    -2.25557375494F,  1.36889998180F, -3.05442349448F,  0.86917333383F, 
    -3.76878963803F, -0.49751185418F, -3.45969593762F, -2.48221639778F, 
    -1.79057961438F, -4.02640271988F,  0.51142051674F, -4.16706705025F, 
     2.20703274067F, -2.91869330258F,  2.59660549639F, -1.26317925367F, 
     2.05981404439F, -0.25133136541F,  1.57454631815F, -0.09084847684F, 
     1.71169479997F, -0.09859688602F,  2.10866866503F,  0.47122028207F, 
     1.92955367463F,  1.58049412639F,  0.82272776725F,  2.40481381526F, 
    -0.61325497948F,  2.17749923260F, -1.34745201557F,  1.02114714676F, 
    -0.91461778743F, -0.08943620692F,  0.10445949691F, -0.22292727047F, 
     0.61385718463F,  0.62769692179F,  0.03550556774F,  1.54835892785F, 
    -1.16498712510F,  1.58923967945F, -1.96656935854F,  0.66014357270F, 
    -1.79784782060F, -0.43874298612F, -1.05718545665F, -0.85936149229F, 
    -0.64192111555F, -0.56195638952F, -0.97854059536F, -0.32143845989F, 
    -1.54558789267F, -0.88206567739F, -1.38324642983F, -2.12082224832F, 
    -0.08719472946F, -3.07772419275F,  1.71148112702F, -2.84523181109F, 
     2.87497905369F, -1.44495414071F,  2.78871580735F,  0.17670609741F, 
     1.88512648042F,  1.03237201135F,  1.15584999902F,  0.98603312848F, 
     1.15004582581F,  0.78108851866F,  1.46021337053F,  1.19729465428F, 
     1.19992883314F,  2.19761117081F, -0.02040488498F,  2.92890001270F, 
    -1.61375022053F,  2.57769792357F, -2.52818233729F,  1.21896114924F, 
    -2.24851074817F, -0.20290274554F, -1.28902480177F, -0.75555399610F, 
    -0.68092767970F, -0.39321717746F, -0.95910142290F,  0.04292801734F, 
    -1.65245270764F, -0.30306421759F, -1.78043218765F, -1.42659261322F, 
    -0.85597013072F, -2.45421311996F,  0.60721587416F, -2.50579097399F, 
     1.58649606042F, -1.56271316447F,  1.54521723175F, -0.47547221573F, 
     0.93340577110F, -0.10788988275F,  0.69508299841F, -0.48010561400F, 
     1.28065088165F, -0.77256937065F,  2.16087271369F, -0.18864092558F, 
     2.33807517695F,  1.19037218058F,  1.35065639214F,  2.42146491391F, 
    -0.23128896823F,  2.58330472369F, -1.31890033748F,  1.65583223416F, 
    -1.32313928975F,  0.52595012684F, -0.67428707976F,  0.11721152188F, 
    -0.33706490830F,  0.51020474014F, -0.81708192008F,  0.92303381319F, 
    -1.65661284055F,  0.55711065752F, -1.92068937346F, -0.55214938936F, 
    -1.17731114034F, -1.54135618359F,  0.02258969052F, -1.58005142728F, 
     0.65953737830F, -0.72181759717F,  0.24981696682F,  0.11615133593F, 
    -0.66192482305F,  0.03892448720F, -1.01997516583F, -0.94706756480F, 
    -0.27987479346F, -1.93222003987F,  1.07090881713F, -1.98059103215F, 
     2.00057154296F, -1.01703009607F,  1.93345464848F,  0.14699916805F, 
     1.26458980016F,  0.65630134188F,  0.88545602565F,  0.46472317204F, 
     1.21462055814F,  0.33674474903F,  1.72501560606F,  0.99902052275F, 
     1.45986792044F,  2.30463297745F,  0.03179916368F,  3.26903880504F, 
    -1.89621172384F,  2.96857026904F, -3.14451926768F,  1.42659525352F, 
    -3.05752774255F, -0.38488250675F, -2.04146741495F, -1.42988590307F, 
    -1.08944918202F, -1.50988165128F, -0.78679148328F, -1.30791619593F, 
    -0.79435780620F, -1.56482195452F, -0.31318361561F, -2.24290073349F, 
     0.96142114820F, -2.53528562559F,  2.38514666681F, -1.71808393605F, 
     2.89801574630F,  0.01778096192F,  2.03720712662F,  1.61285484366F, 
     0.42584727057F,  2.05208496946F, -0.75966676327F,  1.26650117619F, 
    -0.83568604874F,  0.14842079394F, -0.15896334617F, -0.32390323644F, 
     0.33637811858F,  0.01604027361F,  0.12229882996F,  0.47708869847F, 
    -0.41471347333F,  0.32812144446F, -0.44211068878F, -0.37387469520F, 
     0.36534102437F, -0.80117705892F,  1.37515500673F, -0.21311542893F, 
     1.53652681840F,  1.21012652578F,  0.39877683175F,  2.41574268729F, 
    -1.39667375325F,  2.39618327173F, -2.64947961607F,  1.09700639736F, 
    -2.65075564928F, -0.56269858583F, -1.73801279514F, -1.56340032862F, 
    -0.84046262170F, -1.68479164965F, -0.50262564636F, -1.55818057105F, 
    -0.37946331974F, -1.85873427788F,  0.29660032907F, -2.48889478370F, 
     1.76920311361F, -2.60619252450F,  3.31915168044F, -1.48788215805F, 
     3.81796282189F,  0.63394820587F,  2.76032892483F,  2.62568921161F, 
     0.76709249167F,  3.38106102355F, -0.94239013934F,  2.74403323706F, 
    -1.60195493676F,  1.54566308494F, -1.46178496319F,  0.74562731294F, 
    -1.34745201557F,  0.54251581818F, -1.70025164388F,  0.32475949981F, 
    -2.08889178783F, -0.52559769780F, -1.68976950599F, -1.82679795629F, 
    -0.24095224764F, -2.64006578597F,  1.51842955761F, -2.15057583806F, 
     2.40804812647F, -0.51717277718F,  1.85043585550F,  1.17077384763F, 
     0.38825091488F,  1.81820715499F, -0.82937567135F,  1.24980814061F, 
    -1.08765675887F,  0.25484350491F, -0.66347963566F, -0.25991115495F, 
    -0.38444575701F, -0.14782699560F, -0.67445187533F, -0.06147706331F, 
    -1.07415272943F, -0.64293989059F, -0.73111508518F, -1.71684772515F, 
     0.63775846020F, -2.33702322344F,  2.30187770401F, -1.67474744971F, 
     3.07643903128F,  0.12306208025F,  2.37054169328F,  1.96979757257F, 
     0.71169479997F,  2.76009403768F, -0.75950196771F,  2.29579835399F, 
    -1.32313928975F,  1.33579737045F, -1.23368544953F,  0.75717764171F, 
    -1.28004867650F,  0.68699405413F, -1.83696722121F,  0.47136647186F, 
    -2.39497669488F, -0.51019944281F, -2.06169436378F, -2.03825591350F, 
    -0.53441871889F, -3.10581003637F,  1.44878777411F, -2.81679353843F, 
     2.66709587883F, -1.25686760501F,  2.46948607390F,  0.53014648834F, 
     1.30901699437F,  1.45381065608F,  0.25072034399F,  1.29881225424F, 
    -0.04818816531F,  0.77683658501F,  0.11985230084F,  0.69580972167F, 
    -0.04013207956F,  1.10424127651F, -0.87608262993F,  1.27484499834F, 
    -1.82438600601F,  0.53234574217F, -1.92199833372F, -0.93029866991F, 
    -0.79293794238F, -2.10041829140F,  0.88660277476F, -2.04673858278F, 
     1.93500948774F, -0.79866264392F,  1.71164592259F,  0.66489829216F, 
     0.65116912486F,  1.31347491641F, -0.22252093396F,  0.97492791218F, 
    -0.30552697997F,  0.37969205953F,  0.05677272341F,  0.32811420551F, 
     0.02858480673F,  0.83880736092F, -0.75840460076F,  1.14106691827F, 
    -1.73734157440F,  0.51408039206F, -1.92085416903F, -0.88555108046F, 
    -0.89497669488F, -2.06375395733F,  0.70529990791F, -2.08615647143F, 
     1.73189117338F, -0.95888763857F,  1.56129217763F,  0.37611273251F, 
     0.62438877672F,  0.92924970933F, -0.07883942478F,  0.56246233986F, 
     0.01722177994F,  0.02446519423F,  0.52394691136F,  0.11295958822F, 
     0.56806474673F,  0.82298386589F, -0.24097243404F,  1.34447245792F, 
    -1.33738310710F,  0.91055104846F, -1.71280619860F, -0.36406285367F, 
    -0.91461778743F, -1.51291791093F,  0.46981367994F, -1.60813727089F, 
     1.33798531225F, -0.63925126235F,  1.09920576903F,  0.48801757051F, 
     0.19650180280F,  0.83172941895F, -0.38184760488F,  0.30188287885F, 
    -0.10337480057F, -0.31568090587F,  0.59620028629F, -0.20605067559F, 
     0.79350415213F,  0.61862468522F,  0.05677272341F,  1.31785352628F, 
    -1.07044632537F,  1.07773998661F, -1.57712582100F, -0.03991166950F, 
    -0.98279764896F, -1.11507543021F,  0.17263935843F, -1.24856649377F, 
     0.84353099006F, -0.43336189446F,  0.49269691363F,  0.44859968186F, 
    -0.39903208126F,  0.50227939047F, -0.83051983604F, -0.30172586847F, 
    -0.28537944186F, -1.11601394378F,  0.75788414389F, -1.07644551214F, 
     1.31447280748F, -0.16922503383F,  0.88477164624F,  0.76162232447F, 
    -0.04818816531F,  0.86913114678F, -0.51419900141F,  0.15861979190F, 
    -0.04558789267F, -0.51879441339F,  0.83551930008F, -0.33174700215F, 
     1.12808931467F,  0.67319940341F,  0.35730927643F,  1.58476900005F, 
    -0.92832458002F,  1.50767113544F, -1.66778850265F,  0.44011701257F, 
    -1.30349819720F, -0.72182509557F, -0.29796065705F, -1.05503467346F, 
     0.35391809732F, -0.48859456793F,  0.12091943751F,  0.17783860098F, 
    -0.55821994436F,  0.12163467569F, -0.75950196771F, -0.64983062220F, 
    -0.05322454543F, -1.30266199153F,  1.01593680623F, -1.03478132988F, 
     1.44247225745F,  0.07533740595F,  0.76287113985F,  1.09107924811F, 
    -0.45372003749F,  1.10499868505F, -1.12502094631F,  0.11870882423F,
}; 


// End of File
