//#############################################################################
//! \file   golden.c
//! \brief  Arctangent Ouput Vector (256) 
//! \author Vishal Coelho 
//! \date   21-Sep-2016
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

#include "fastrts.h"

const float32_t test_golden[256] = {
    -0.785398163397F, -0.781476614873F, -0.777524310373F, -0.773541011593F, 
    -0.769526480406F, -0.765480478966F, -0.761402769806F, -0.757293115937F, 
    -0.753151280962F, -0.748977029183F, -0.744770125716F, -0.740530336613F, 
    -0.736257428981F, -0.731951171116F, -0.727611332627F, -0.723237684576F, 
    -0.718829999622F, -0.714388052157F, -0.709911618464F, -0.705400476865F, 
    -0.700854407884F, -0.696273194408F, -0.691656621853F, -0.687004478341F, 
    -0.682316554875F, -0.677592645520F, -0.672832547594F, -0.668036061856F, 
    -0.663202992706F, -0.658333148385F, -0.653426341181F, -0.648482387642F, 
    -0.643501108793F, -0.638482330354F, -0.633425882969F, -0.628331602434F, 
    -0.623199329934F, -0.618028912283F, -0.612820202165F, -0.607573058389F, 
    -0.602287346135F, -0.596962937215F, -0.591599710335F, -0.586197551356F, 
    -0.580756353568F, -0.575276017956F, -0.569756453483F, -0.564197577362F, 
    -0.558599315344F, -0.552961601994F, -0.547284380987F, -0.541567605392F, 
    -0.535811237960F, -0.530015251424F, -0.524179628783F, -0.518304363604F, 
    -0.512389460311F, -0.506434934483F, -0.500440813147F, -0.494407135071F, 
    -0.488333951056F, -0.482221324228F, -0.476069330323F, -0.469878057976F, 
    -0.463647609001F, -0.457378098670F, -0.451069655989F, -0.444722423961F, 
    -0.438336559858F, -0.431912235472F, -0.425449637370F, -0.418948967134F, 
    -0.412410441597F, -0.405834293075F, -0.399220769575F, -0.392570135012F, 
    -0.385882669398F, -0.379158669033F, -0.372398446677F, -0.365602331707F, 
    -0.358770670271F, -0.351903825415F, -0.345002177207F, -0.338066122837F, 
    -0.331096076704F, -0.324092470490F, -0.317055753209F, -0.309986391247F, 
    -0.302884868375F, -0.295751685750F, -0.288587361894F, -0.281392432649F, 
    -0.274167451120F, -0.266912987587F, -0.259629629408F, -0.252317980886F, 
    -0.244978663127F, -0.237612313865F, -0.230219587277F, -0.222801153759F, 
    -0.215357699698F, -0.207889927202F, -0.200398553826F, -0.192884312258F, 
    -0.185347949996F, -0.177790228993F, -0.170211925285F, -0.162613828598F, 
    -0.154996741924F, -0.147361481089F, -0.139708874289F, -0.132039761615F, 
    -0.124354994547F, -0.116655435441F, -0.108941956990F, -0.101215441667F, 
    -0.093476781159F, -0.085726875771F, -0.077966633832F, -0.070196971072F, 
    -0.062418809996F, -0.054633079239F, -0.046840712916F, -0.039042649955F, 
    -0.031239833430F, -0.023433209879F, -0.015623728620F, -0.007812341060F, 
     0.000000000000F,  0.007812341060F,  0.015623728620F,  0.023433209879F, 
     0.031239833430F,  0.039042649955F,  0.046840712916F,  0.054633079239F, 
     0.062418809996F,  0.070196971072F,  0.077966633832F,  0.085726875771F, 
     0.093476781159F,  0.101215441667F,  0.108941956990F,  0.116655435441F, 
     0.124354994547F,  0.132039761615F,  0.139708874289F,  0.147361481089F, 
     0.154996741924F,  0.162613828598F,  0.170211925285F,  0.177790228993F, 
     0.185347949996F,  0.192884312258F,  0.200398553826F,  0.207889927202F, 
     0.215357699698F,  0.222801153759F,  0.230219587277F,  0.237612313865F, 
     0.244978663127F,  0.252317980886F,  0.259629629408F,  0.266912987587F, 
     0.274167451120F,  0.281392432649F,  0.288587361894F,  0.295751685750F, 
     0.302884868375F,  0.309986391247F,  0.317055753209F,  0.324092470490F, 
     0.331096076704F,  0.338066122837F,  0.345002177207F,  0.351903825415F, 
     0.358770670271F,  0.365602331707F,  0.372398446677F,  0.379158669033F, 
     0.385882669398F,  0.392570135012F,  0.399220769575F,  0.405834293075F, 
     0.412410441597F,  0.418948967134F,  0.425449637370F,  0.431912235472F, 
     0.438336559858F,  0.444722423961F,  0.451069655989F,  0.457378098670F, 
     0.463647609001F,  0.469878057976F,  0.476069330323F,  0.482221324228F, 
     0.488333951056F,  0.494407135071F,  0.500440813147F,  0.506434934483F, 
     0.512389460311F,  0.518304363604F,  0.524179628783F,  0.530015251424F, 
     0.535811237960F,  0.541567605392F,  0.547284380987F,  0.552961601994F, 
     0.558599315344F,  0.564197577362F,  0.569756453483F,  0.575276017956F, 
     0.580756353568F,  0.586197551356F,  0.591599710335F,  0.596962937215F, 
     0.602287346135F,  0.607573058389F,  0.612820202165F,  0.618028912283F, 
     0.623199329934F,  0.628331602434F,  0.633425882969F,  0.638482330354F, 
     0.643501108793F,  0.648482387642F,  0.653426341181F,  0.658333148385F, 
     0.663202992706F,  0.668036061856F,  0.672832547594F,  0.677592645520F, 
     0.682316554875F,  0.687004478341F,  0.691656621853F,  0.696273194408F, 
     0.700854407884F,  0.705400476865F,  0.709911618464F,  0.714388052157F, 
     0.718829999622F,  0.723237684576F,  0.727611332627F,  0.731951171116F, 
     0.736257428981F,  0.740530336613F,  0.744770125716F,  0.748977029183F, 
     0.753151280962F,  0.757293115937F,  0.761402769806F,  0.765480478966F, 
     0.769526480406F,  0.773541011593F,  0.777524310373F,  0.781476614873F, 
}; 


// End of File
