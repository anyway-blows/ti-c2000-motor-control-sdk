//#############################################################################
//! \file   input.c
//! \brief  Input Vector (1024) 
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

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(test_input, "FFT_buffer_1")
#endif
float test_input[256] = {
       0.000000000000F, 1879.166852238198F,  331.667913895491F,  568.428690010465F,
     728.420021767419F, -1039.019110794615F,  506.740939322956F, 1984.062817636476F,
      20.552123993448F, -116.680943813521F, -238.155603607050F, -2156.519333532307F,
    -610.759283414733F, 1017.487192262008F, -664.557509830541F, -427.796733715564F,
    -133.259638208775F, -1649.778394209357F,  229.388458028851F, 2077.575127924918F,
     472.299870190386F,  632.291201947348F,  706.888103234807F, -1143.037454886403F,
     334.284423427136F, 1766.459338022879F, -212.809763633600F, -334.284423427118F,
    -410.612119502886F, -2260.537677624096F, -632.291201947330F, 1081.349704198890F,
    -523.925553535647F, -229.388458028843F,   96.128819820061F, -1420.389936180516F,
     427.796733715582F, 2218.207084219819F,  536.162382127264F,  610.759283414743F,
     602.869759143015F, -1315.493970782233F,  116.680943813548F, 1533.097450395835F,
    -430.413243247203F, -506.740939322948F, -514.630463594679F, -2282.069596156703F,
    -568.428690010439F, 1221.981660493788F, -325.517277848932F,   -0.000000000001F,
     325.517277848897F, -1221.981660493795F,  568.428690010493F, 2282.069596156701F,
     514.630463594653F,  506.740939322957F,  430.413243247181F, -1533.097450395831F,
    -116.680943813485F, 1315.493970782237F, -602.869759143040F, -610.759283414736F,
    -536.162382127292F, -2218.207084219823F, -427.796733715530F, 1420.389936180508F,
     -96.128819820096F,  229.388458028841F,  523.925553535614F, -1081.349704198894F,
     632.291201947387F, 2260.537677624096F,  410.612119502862F,  334.284423427128F,
     212.809763633578F, -1766.459338022874F, -334.284423427073F, 1143.037454886405F,
    -706.888103234834F, -632.291201947344F, -472.299870190417F, -2077.575127924925F,
    -229.388458028800F, 1649.778394209349F,  133.259638208740F,  427.796733715563F,
     664.557509830510F, -1017.487192262011F,  610.759283414792F, 2156.519333532310F,
     238.155603607028F,  116.680943813532F,  -20.552123993470F, -1984.062817636473F,
    -506.740939322894F, 1039.019110794615F, -728.420021767448F, -568.428690010464F,
    -331.667913895524F, -1879.166852238206F,    0.000000000051F, 1879.166852238189F,
     331.667913895457F,  568.428690010466F,  728.420021767389F, -1039.019110794615F,
     506.740939323017F, 1984.062817636481F,   20.552123993426F, -116.680943813511F,
    -238.155603607073F, -2156.519333532305F, -610.759283414674F, 1017.487192262006F,
    -664.557509830573F, -427.796733715565F, -133.259638208810F, -1649.778394209366F,
     229.388458028902F, 2077.575127924912F,  472.299870190354F,  632.291201947351F,
     706.888103234780F, -1143.037454886401F,  334.284423427199F, 1766.459338022883F,
    -212.809763633623F, -334.284423427109F, -410.612119502911F, -2260.537677624096F,
    -632.291201947273F, 1081.349704198885F, -523.925553535681F, -229.388458028846F,
      96.128819820025F, -1420.389936180524F,  427.796733715635F, 2218.207084219814F,
     536.162382127234F,  610.759283414748F,  602.869759142990F, -1315.493970782229F,
     116.680943813613F, 1533.097450395840F, -430.413243247226F, -506.740939322941F,
    -514.630463594706F, -2282.069596156706F, -568.428690010384F, 1221.981660493782F,
    -325.517277848967F,   -0.000000000005F,  325.517277848862F, -1221.981660493802F,
     568.428690010548F, 2282.069596156699F,  514.630463594626F,  506.740939322964F,
     430.413243247158F, -1533.097450395826F, -116.680943813420F, 1315.493970782241F,
    -602.869759143064F, -610.759283414731F, -536.162382127322F, -2218.207084219828F,
    -427.796733715477F, 1420.389936180500F,  -96.128819820132F,  229.388458028838F,
     523.925553535580F, -1081.349704198899F,  632.291201947444F, 2260.537677624096F,
     410.612119502837F,  334.284423427137F,  212.809763633556F, -1766.459338022869F,
    -334.284423427009F, 1143.037454886407F, -706.888103234861F, -632.291201947342F,
    -472.299870190449F, -2077.575127924932F, -229.388458028749F, 1649.778394209340F,
     133.259638208705F,  427.796733715561F,  664.557509830478F, -1017.487192262013F,
     610.759283414852F, 2156.519333532312F,  238.155603607005F,  116.680943813542F,
     -20.552123993492F, -1984.062817636469F, -506.740939322833F, 1039.019110794616F,
    -728.420021767477F, -568.428690010463F, -331.667913895558F, -1879.166852238214F,
       0.000000000101F, 1879.166852238181F,  331.667913895423F,  568.428690010466F,
     728.420021767360F, -1039.019110794615F,  506.740939323079F, 1984.062817636485F,
      20.552123993404F, -116.680943813501F, -238.155603607096F, -2156.519333532303F,
    -610.759283414614F, 1017.487192262004F, -664.557509830605F, -427.796733715567F,
    -133.259638208845F, -1649.778394209374F,  229.388458028954F, 2077.575127924905F,
     472.299870190322F,  632.291201947354F,  706.888103234753F, -1143.037454886399F,
     334.284423427262F, 1766.459338022889F, -212.809763633645F, -334.284423427100F,
    -410.612119502935F, -2260.537677624096F, -632.291201947216F, 1081.349704198880F,
    -523.925553535714F, -229.388458028849F,   96.128819819990F, -1420.389936180532F,
     427.796733715687F, 2218.207084219809F,  536.162382127205F,  610.759283414753F,
     602.869759142966F, -1315.493970782225F,  116.680943813677F, 1533.097450395845F,
    -430.413243247249F, -506.740939322934F, -514.630463594733F, -2282.069596156708F,
    -568.428690010330F, 1221.981660493775F, -325.517277849002F,   -0.000000000008F,
};

//End of file
