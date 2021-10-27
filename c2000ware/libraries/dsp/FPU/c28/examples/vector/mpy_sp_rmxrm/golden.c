//#############################################################################
//! \file   golden.c
//! \brief  Ouput Vector (16x16) 
//! \author Vishal Coelho 
//! \date   30-Mar-2017
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


const float test_golden[256] = {
    -1.354913640661F, -0.622606092365F,  0.107507384494F,  1.316761415284F,
     1.454730826057F,  3.111972131413F, -2.405053146498F,  2.749790075358F,
    -2.798230541672F,  2.176566295027F,  1.559771120822F, -0.439506585339F,
    -2.590297097395F,  3.143724060299F, -1.913790842916F, -0.434807277875F,
     5.127412975275F,  1.234992128092F, -1.652998422937F, -1.798351812602F,
    -4.754361655239F, -7.175492281169F,  8.236914962282F,  1.785515724228F,
     3.412343156787F, -8.987965931883F, -4.843618339399F, -0.503007602812F,
     7.338871352110F, -10.740926550995F, -0.838065217076F, -1.102411021486F,
     2.742976249777F,  2.685724136083F, -2.130278657956F,  2.899270673776F,
    -5.200823676004F, -4.890429155146F,  4.131588571852F,  5.292405441094F,
     1.341245515694F, -7.502604770775F, -3.104966575038F, -3.614811590508F,
     7.103136468207F, -7.725477348098F, -5.287268107629F, -1.775633458276F,
     4.167983338295F, -10.294902237839F, -7.644524403232F, -10.471538046595,
    -9.499517344418F, -8.980401336410F,  7.700272279071F,  0.493571587501F,
     6.289298307445F,  0.078676425280F, -4.648874673351F, -0.024590903863F,
     0.428986847672F, -1.807521450104F, -7.319795432494F, -11.183149033305F,
    -5.902025743542F, -0.552300729398F,  0.595675679914F,  2.184834993212F,
     1.662677759696F,  3.808744110522F, -9.018465228779F, -6.129428368721F,
     0.836184677956F,  9.476373163638F,  4.301694067534F, -1.045045160699F,
    -4.755404943281F, 10.842792891319F,  0.733382007841F,  0.383781764365F,
     3.332923773968F, -0.123836867687F, -2.445505316619F,  2.821747299785F,
    -2.323369276042F,  0.489888703235F,  4.347243583628F, 15.157680821301F,
    -5.937162856929F, -6.994037251480F, -1.980314120443F, -2.987805557622F,
     2.310517405933F, -5.936120407645F, -9.823474217003F, -3.700185363592F,
     0.483103379887F,  2.862837399000F,  6.525305487411F,  1.611205250968F,
    11.770984800566F, 10.410129813266F, -0.462959225142F,  5.677775554379F,
    -10.513813872655F, -1.108227764891F,  2.764606374555F,  5.241229774615F,
    -5.874864051600F, -0.247926675468F,  7.135626852369F,  7.769887058964F,
    -2.478093228624F, -11.904678143394F, -4.962338871176F, -8.482933561953F,
    -2.383729865804F,  0.963280666487F, -3.002160185577F, -2.199731330836F,
     1.445927698198F, 11.990940833617F,  1.842673901073F,  1.309599339243F,
    -9.716947409769F, 12.323673595967F, -5.157673107298F, -9.115950883712F,
    -8.663590011673F, -6.526153073448F, -7.667071339140F,  0.414916034213F,
    -10.819886160640F, -4.685482265475F, -12.056093021169F, -10.67202584572F,
    10.481160952661F, 15.619395873593F,  3.283073895298F, -7.452786897767F,
    -2.507982227594F, 17.128047877414F, -10.348089599905F, -10.585943181942,
     2.297474847892F, -3.926347576014F, -0.762360898889F, -2.464054874341F,
     2.224701136543F,  3.726360600383F,  3.042692440648F,  9.727628059011F,
    -6.450016023035F, -1.042287889101F, -0.202308851604F,  1.700481381443F,
    -4.214417098390F, -0.490691541203F, -3.848084998816F, -2.553813109012F,
    -7.124487019604F, -4.606141315988F, -1.841162099705F,  0.323082096091F,
    -0.204514933729F,  3.991715308461F, -10.680709669192F, -5.545475256714F,
     1.210215932958F, 13.648437668091F,  5.026074592096F, -1.815387189801F,
    -7.680635572920F, 15.481251267987F, -3.160880349140F, -3.716328032114F,
     1.837247582567F,  1.027041937730F,  3.230419290913F, -0.328886146969F,
     6.037928386828F,  4.795005147142F,  2.189639504330F,  4.681113539205F,
    -6.012661297091F, -2.602206914358F,  0.418347096856F,  3.405389293677F,
    -2.547386644130F, -2.521024468635F,  3.633140359915F,  3.751741923979F,
     5.409004705746F, -6.550994288743F, -2.254536812108F, -3.417181611853F,
     2.500680399384F,  5.715387987158F,  7.281253828514F, 21.379369070140F,
    -12.396586984708F, -4.824160629230F, -1.437672302337F,  1.830741673336F,
    -5.646305518859F, -3.722142724271F, -9.745982877804F, -5.629000643267F,
    -4.622143750500F,  1.992462091162F, -2.951984411621F,  3.418336153757F,
    -7.933536910309F, -6.829481031086F, -6.193726110266F, -11.425948464018F,
    10.459423417997F,  4.610767943041F,  0.472649289162F, -5.653355742503F,
     5.225630802972F,  4.623575026711F, -2.175782033284F, -2.307473504409F,
     8.049162997524F, -4.010936070464F, -3.315515168620F, -5.494496968280F,
    -3.477473183545F, -4.232409730710F, 12.344461881592F, 12.270484654015F,
    -2.764317087030F, -10.182184807312F, -5.594060031133F,  1.247421480098F,
     2.992009673138F, -11.549229289268F, -5.802727950700F, -5.047844824736F,
     1.447586133850F,  5.926767085338F,  3.535510425775F,  4.135801390360F,
     3.491024715965F,  1.722184704406F,  1.585571973910F,  2.744969698184F,
    -3.077325881679F, -6.003678361470F, -0.389466653577F,  0.497230580300F,
     3.245824266053F, -5.976338406843F,  3.564418716547F,  5.626278513445F,
}; 


// End of File
