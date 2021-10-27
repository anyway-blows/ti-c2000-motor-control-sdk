//#############################################################################
//! \file wcfft2Input.c
//! \brief  Input Vector (1024) 
//! \author Vishal Coelho 
//! \date   16-Nov-2015
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


float test_input[2048] = {
	44.00000000000F,  0.00000000000F, 36.60686219503F, 21.57296232978F, 
	18.90075626927F, 33.10154934594F,  1.07001745882F, 31.34809938127F, 
	-8.26124411416F, 21.65970469025F, -7.93001519773F, 13.46263570741F, 
	-4.38622325073F, 12.92576991424F, -6.07890367911F, 18.34549009276F, 
	-16.40094187649F, 21.71982152599F, -30.59232917650F, 15.28500639872F, 
	-38.99981439277F, -1.97702528748F, -34.27469498393F, -22.84809938127F, 
	-17.09110291372F, -36.61626797170F,  3.95219892729F, -36.56731553153F, 
	18.23199246986F, -24.65324215126F, 20.43890141802F, -9.86472895136F, 
	13.91547489503F, -1.54404209429F,  7.56437066057F, -2.65283162923F, 
	 8.72403726008F, -8.01532400150F, 17.42234795605F, -8.79065319697F, 
	26.35234702788F,  0.27252776298F, 26.57575958864F, 16.33984329451F, 
	14.62035455802F, 30.20510290023F, -4.43201911051F, 32.96940877548F, 
	-20.26124411416F, 22.67621686378F, -24.60108740867F,  5.89954272486F, 
	-17.07638428796F, -6.94123927995F, -5.23220344947F, -9.06073772278F, 
	 1.24671109563F, -2.00765071103F, -2.22341256076F,  5.89420675126F, 
	-11.85253263511F,  6.13863730849F, -18.59593940542F, -3.33723400599F, 
	-15.26782192291F, -16.50060537574F, -2.24360890818F, -23.78500639872F, 
	12.58433949775F, -19.41015163980F, 19.59220118838F, -5.91344560986F, 
	14.60563593226F,  7.55951146000F,  2.06233409125F, 11.95718416849F, 
	-8.24652548840F,  5.00118817203F, -8.43312663165F, -7.16934380276F, 
	 1.66218599066F, -14.62391868273F, 14.43375341476F, -11.06638452103F, 
	20.26800753014F,  1.50706791473F, 14.75249512847F, 14.39420675126F, 
	 1.91547489503F, 18.79187945974F, -8.92233183016F, 12.42170169663F, 
	-10.28254054867F,  1.17442331602F, -2.24360890818F, -5.46283096673F, 
	 7.59905812351F, -1.73512294805F,  9.91859361312F,  9.84549009276F, 
	 0.83762840212F, 20.21275361126F, -14.91357359798F, 20.45718416849F, 
	-27.26782192291F,  8.51590679779F, -28.09908349589F, -9.17499060100F, 
	-17.07638428796F, -22.30659808551F, -1.73420736225F, -24.13527104864F, 
	 8.25328890437F, -16.16796077702F,  8.25814192672F, -6.61801785573F, 
	 1.93019352079F, -3.85371198048F, -1.93877342990F, -9.83158720775F, 
	 3.48546698146F, -18.50825608676F, 17.42234795605F, -20.45718416849F, 
	31.59091730649F, -10.22040432229F, 36.07890367911F,  9.16108771599F, 
	26.60563593225F, 27.89543301404F,  7.74874038079F, 36.21611987111F, 
	-10.28254054867F, 31.16149823802F, -18.91468111913F, 18.33158720775F, 
	-17.09110291372F,  7.36843060624F, -11.40781493752F,  4.61237105749F, 
	-10.48528137424F,  8.48528137424F, -17.90216813927F, 11.06638452103F, 
	-29.09110291372F,  4.63156939376F, -34.59343669764F, -11.83723400599F, 
	-27.25310329715F, -31.16149823802F, -7.93001519773F, -42.71047307286F, 
	14.60563593226F, -39.89543301404F, 29.58455047735F, -24.83984329451F, 
	31.59091730649F, -6.75015842619F, 23.91670115781F,  4.77842858997F, 
	15.48546698146F,  6.50825608676F, 13.73998214862F,  3.33723400599F, 
	18.90075626927F,  3.85371198048F, 23.93689750523F, 13.11237105749F, 
	20.25328890437F, 28.16796077702F,  4.76014583950F, 39.81402662716F, 
	-17.07638428796F, 39.27716083398F, -34.59343669764F, 24.85374617952F, 
	-39.26782192291F,  3.48409320221F, -30.59232917650F, -13.96283096673F, 
	-16.13293434636F, -20.21275361126F, -5.76016196540F, -16.33984329451F, 
	-4.40094187649F, -10.26487705195F, -8.73796210994F, -10.21592461178F, 
	-10.28254054867F, -18.14498606449F, -2.42797862840F, -28.10045727514F, 
	13.91547489503F, -30.79187945974F, 30.43125070699F, -20.88855995301F, 
	37.23857027861F, -1.50706791473F, 30.11250899328F, 17.56073772278F, 
	13.66218599065F, 26.62391868273F, -1.93877342990F, 22.84809938127F, 
	-8.24652548840F, 11.96937457645F, -4.43201911051F,  3.72157141003F, 
	 2.60563593226F,  4.44048854000F,  3.91344560986F, 12.40779881162F, 
	-4.38622325073F, 19.41015163980F, -17.92236448670F, 17.29065319697F, 
	-27.26782192291F,  4.50060537574F, -25.09029260717F, -12.34152157252F, 
	-11.85253263511F, -23.10920005697F,  4.27094064100F, -21.57296232978F, 
	13.24671109563F, -9.99234928897F, 10.44655212904F,  2.56638452103F, 
	-0.10582153948F,  6.94123927995F, -8.92233183016F,  0.59481047690F, 
	-8.26124411416F, -10.67621686378F,  2.06233409125F, -17.29065319697F, 
	14.62035455802F, -13.23454015175F, 20.08140638688F, -0.66108771599F, 
	14.35234702788F, 11.72747223702F,  1.74359237754F, 15.28500639872F, 
	-8.24652548840F,  8.01532400150F, -8.11438491795F, -3.84152157252F, 
	 1.91547489503F, -10.45595790571F, 13.94454821626F, -5.81402662716F, 
	18.23199246986F,  7.68267940278F, 10.44655212904F, 20.88855995301F, 
	-5.09110291372F, 24.61626797170F, -18.59593940542F, 16.35374617952F, 
	-22.02925164430F,  1.97702528748F, -14.91357359798F, -8.79065319697F, 
	-4.40094187649F, -9.71982152599F,  0.41544952265F, -2.66673451424F, 
	-4.38622325073F,  4.04479283424F, -14.42436839948F,  2.21611987111F, 
	-20.26124411416F, -9.65970469025F, -14.60873811970F, -24.85374617952F, 
	 1.93019352079F, -33.10154934594F, 20.92810661652F, -28.06731553153F, 
	32.00000000000F, -12.00000000000F, 30.11250899328F,  5.89420675126F, 
	18.90075626927F, 16.13098659746F,  7.56437066057F, 15.66934380276F, 
	 3.73875588584F,  9.65970469025F,  7.74874038079F,  6.96828250565F, 
	12.58433949775F, 12.92576991424F,  9.59985189941F, 24.83984329451F, 
	-4.40094187649F, 33.71982152599F, -24.09797597474F, 30.96376197724F, 
	-38.99981439278F, 14.99353746100F, -40.76904818569F, -7.16934380276F, 
	-29.09110291372F, -24.61626797170F, -11.72655665123F, -30.07296232978F, 
	 1.26142972139F, -24.65324215126F,  4.76014583950F, -16.35908215311F, 
	 1.91547489503F, -13.54404209429F,  1.07001745882F, -18.33158720775F, 
	 8.72403726008F, -24.98588674997F, 23.91670115781F, -24.46940877548F, 
	38.35234702788F, -11.72747223702F, 42.25451516715F,  9.84549009276F, 
	31.59091730649F, 30.20510290023F, 11.24673646801F, 39.46376197724F, 
	-8.26124411416F, 34.67621686378F, -18.10673420692F, 21.57829830337F, 
	-17.07638428796F, 10.02932346853F, -11.72655665123F,  6.61801785573F, 
	-10.75328890437F,  9.99234928897F, -17.90216813927F, 12.38855995301F, 
	-28.82309538358F,  6.13863730849F, -34.27469498393F, -9.83158720775F, 
	-27.26782192291F, -28.50060537574F, -8.73796210993F, -39.46376197724F, 
	12.58433949775F, -36.38071438828F, 26.08655439014F, -21.59220118838F, 
	26.60563593225F, -4.44048854000F, 17.74108966976F,  5.46283096673F, 
	 8.72403726008F,  5.00118817203F,  7.24562894686F, -0.67499060100F, 
	13.66218599066F, -2.62391868273F, 20.92810661652F,  4.61237105749F, 
	20.26800753014F, 18.47763066321F,  8.25814192672F, 30.07296232978F, 
	-10.08452510497F, 30.79187945974F, -24.60108740867F, 18.91605489838F, 
	-27.25310329715F,  1.17442331602F, -17.92236448670F, -11.95718416849F, 
	-4.40094187649F, -13.73512294805F,  3.42424041136F, -5.83326548576F, 
	 0.83762840212F,  3.24219086278F, -8.41922039623F,  4.77842858997F, 
	-15.26782192291F, -3.48409320221F, -12.42032791737F, -15.66934380276F, 
	-0.10582153948F, -22.30659808551F, 13.94454821626F, -17.64091784688F, 
	20.25328890437F, -4.16796077702F, 14.75249512847F,  9.06073772278F, 
	 1.93019352079F, 13.11685076799F, -8.43312663165F,  5.84716837077F, 
	-8.51453301854F, -6.50825608676F,  1.74359237754F, -13.96283096673F, 
	14.62035455802F, -10.22040432229F, 20.40014810059F,  2.66673451424F, 
	14.60563593226F, 15.89543301404F,  1.25438717904F, 20.53736429259F, 
	-10.28254054867F, 14.19093548954F, -12.42032791737F,  2.65283162923F, 
	-5.09110291372F, -4.63156939376F,  4.27094064100F, -1.88198214426F, 
	 6.48528137424F,  8.48528137424F, -2.22341256076F, 17.56073772278F, 
	-17.09110291372F, 16.63156939376F, -28.09908349589F,  3.84152157252F, 
	-27.25310329715F, -14.19093548954F, -14.42436839948F, -27.03171749435F, 
	 2.60563593226F, -27.89543301404F, 13.90579489884F, -18.34549009276F, 
	14.62035455802F, -6.75015842619F,  8.23794557929F, -1.71592461178F, 
	 3.48546698146F, -5.49174391324F,  7.24562894686F, -12.34152157252F, 
	18.90075626927F, -13.11685076799F, 30.43125070699F, -2.56638452103F, 
	32.25328890437F, 16.16796077702F, 20.43890141802F, 33.31967342540F, 
	-0.10582153948F, 39.27716083398F, -18.91468111913F, 31.34809938127F, 
	-27.26782192291F, 15.48409320221F, -24.09797597474F,  1.71592461178F, 
	-16.13293434636F, -3.24219086278F, -12.25451516715F, -0.66108771599F, 
	-16.40094187649F,  1.73512294805F, -24.41671768845F, -3.72157141003F, 
	-27.25310329715F, -18.14498606449F, -18.10673420692F, -34.59481047690F, 
	 1.91547489503F, -42.79187945975F, 23.93689750524F, -36.56731553153F, 
	37.23857027861F, -18.47763066321F, 36.60686219503F,  1.88198214427F, 
	25.66218599065F, 14.62391868273F, 13.73998214862F, 16.35374617952F, 
	 8.72403726008F, 11.96937457645F, 11.24673646801F, 10.21592461178F, 
	14.60563593225F, 16.44048854000F, 10.40779881162F, 28.08655439014F, 
	-4.38622325073F, 36.38071438828F, -24.41671768845F, 32.96940877548F, 
	-39.26782192291F, 16.50060537573F, -40.76904818569F, -5.84716837077F, 
	-28.82309538358F, -23.10920005697F, -11.40781493752F, -28.06731553153F, 
	 1.24671109563F, -21.99234928897F,  3.95219892729F, -13.11237105749F, 
	-0.10582153948F, -10.02932346853F, -2.42797862840F, -15.08394510162F, 
	 3.73875588584F, -22.67621686378F, 17.74108966976F, -23.78500639872F, 
	31.59091730649F, -13.23454015175F, 35.76016196540F,  5.83326548576F, 
	26.35234702788F, 23.72747223702F,  8.23794557929F, 30.96376197724F, 
	-8.24652548840F, 24.98588674997F, -14.60873811970F, 11.83723400599F, 
	-10.08452510497F,  1.54404209429F, -1.73420736225F,  0.68032657460F, 
	 1.26142972139F,  7.68267940278F, -5.23220344947F, 14.39420675126F, 
	-17.09110291372F, 12.61626797170F, -25.09029260717F,  0.67499060100F, 
	-22.02925164430F, -14.99353746100F, -8.41922039623F, -24.46940877548F, 
	 7.59905812351F, -21.71982152599F, 16.09420510116F, -9.16108771599F, 
	12.58433949774F,  4.04479283424F,  1.25438717904F,  8.71047307286F, 
	-8.26124411416F,  2.34029530975F, -8.11438491795F, -9.17499060100F, 
	 1.93019352079F, -16.13098659746F, 14.43375341476F, -12.38855995301F, 
	20.00000000000F,  0.00000000000F, 14.43375341476F, 12.38855995301F, 
	 1.93019352079F, 16.13098659746F, -8.11438491795F,  9.17499060100F, 
	-8.26124411416F, -2.34029530975F,  1.25438717904F, -8.71047307286F, 
	12.58433949775F, -4.04479283424F, 16.09420510116F,  9.16108771599F, 
	 7.59905812351F, 21.71982152599F, -8.41922039623F, 24.46940877548F, 
	-22.02925164430F, 14.99353746100F, -25.09029260717F, -0.67499060100F, 
	-17.09110291372F, -12.61626797170F, -5.23220344947F, -14.39420675126F, 
	 1.26142972139F, -7.68267940278F, -1.73420736225F, -0.68032657460F, 
	-10.08452510497F, -1.54404209429F, -14.60873811970F, -11.83723400599F, 
	-8.24652548840F, -24.98588674997F,  8.23794557929F, -30.96376197724F, 
	26.35234702788F, -23.72747223702F, 35.76016196540F, -5.83326548576F, 
	31.59091730649F, 13.23454015175F, 17.74108966976F, 23.78500639872F, 
	 3.73875588584F, 22.67621686378F, -2.42797862840F, 15.08394510162F, 
	-0.10582153948F, 10.02932346853F,  3.95219892729F, 13.11237105749F, 
	 1.24671109563F, 21.99234928897F, -11.40781493752F, 28.06731553153F, 
	-28.82309538358F, 23.10920005697F, -40.76904818569F,  5.84716837077F, 
	-39.26782192291F, -16.50060537574F, -24.41671768845F, -32.96940877548F, 
	-4.38622325073F, -36.38071438828F, 10.40779881162F, -28.08655439014F, 
	14.60563593226F, -16.44048854000F, 11.24673646801F, -10.21592461178F, 
	 8.72403726008F, -11.96937457645F, 13.73998214862F, -16.35374617952F, 
	25.66218599066F, -14.62391868273F, 36.60686219503F, -1.88198214426F, 
	37.23857027861F, 18.47763066321F, 23.93689750523F, 36.56731553153F, 
	 1.91547489503F, 42.79187945974F, -18.10673420692F, 34.59481047690F, 
	-27.25310329715F, 18.14498606449F, -24.41671768845F,  3.72157141003F, 
	-16.40094187649F, -1.73512294805F, -12.25451516715F,  0.66108771599F, 
	-16.13293434636F,  3.24219086278F, -24.09797597474F, -1.71592461178F, 
	-27.26782192291F, -15.48409320221F, -18.91468111913F, -31.34809938127F, 
	-0.10582153948F, -39.27716083398F, 20.43890141802F, -33.31967342540F, 
	32.25328890437F, -16.16796077702F, 30.43125070699F,  2.56638452103F, 
	18.90075626927F, 13.11685076799F,  7.24562894686F, 12.34152157252F, 
	 3.48546698146F,  5.49174391324F,  8.23794557929F,  1.71592461178F, 
	14.62035455802F,  6.75015842619F, 13.90579489884F, 18.34549009276F, 
	 2.60563593225F, 27.89543301404F, -14.42436839948F, 27.03171749435F, 
	-27.25310329715F, 14.19093548954F, -28.09908349589F, -3.84152157253F, 
	-17.09110291372F, -16.63156939376F, -2.22341256075F, -17.56073772278F, 
	 6.48528137424F, -8.48528137424F,  4.27094064100F,  1.88198214427F, 
	-5.09110291372F,  4.63156939376F, -12.42032791737F, -2.65283162923F, 
	-10.28254054867F, -14.19093548954F,  1.25438717904F, -20.53736429259F, 
	14.60563593226F, -15.89543301404F, 20.40014810059F, -2.66673451424F, 
	14.62035455802F, 10.22040432229F,  1.74359237754F, 13.96283096673F, 
	-8.51453301854F,  6.50825608676F, -8.43312663165F, -5.84716837077F, 
	 1.93019352079F, -13.11685076799F, 14.75249512847F, -9.06073772278F, 
	20.25328890437F,  4.16796077702F, 13.94454821626F, 17.64091784689F, 
	-0.10582153948F, 22.30659808551F, -12.42032791737F, 15.66934380276F, 
	-15.26782192291F,  3.48409320221F, -8.41922039623F, -4.77842858997F, 
	 0.83762840212F, -3.24219086278F,  3.42424041136F,  5.83326548576F, 
	-4.40094187649F, 13.73512294805F, -17.92236448670F, 11.95718416849F, 
	-27.25310329715F, -1.17442331602F, -24.60108740867F, -18.91605489838F, 
	-10.08452510497F, -30.79187945975F,  8.25814192672F, -30.07296232978F, 
	20.26800753014F, -18.47763066321F, 20.92810661652F, -4.61237105749F, 
	13.66218599066F,  2.62391868273F,  7.24562894686F,  0.67499060100F, 
	 8.72403726008F, -5.00118817203F, 17.74108966976F, -5.46283096673F, 
	26.60563593226F,  4.44048854000F, 26.08655439013F, 21.59220118838F, 
	12.58433949774F, 36.38071438828F, -8.73796210994F, 39.46376197724F, 
	-27.26782192291F, 28.50060537573F, -34.27469498394F,  9.83158720774F, 
	-28.82309538358F, -6.13863730849F, -17.90216813927F, -12.38855995301F, 
	-10.75328890437F, -9.99234928897F, -11.72655665123F, -6.61801785574F, 
	-17.07638428796F, -10.02932346853F, -18.10673420692F, -21.57829830337F, 
	-8.26124411416F, -34.67621686378F, 11.24673646801F, -39.46376197724F, 
	31.59091730650F, -30.20510290023F, 42.25451516715F, -9.84549009275F, 
	38.35234702788F, 11.72747223702F, 23.91670115781F, 24.46940877548F, 
	 8.72403726008F, 24.98588674997F,  1.07001745882F, 18.33158720775F, 
	 1.91547489503F, 13.54404209429F,  4.76014583950F, 16.35908215312F, 
	 1.26142972139F, 24.65324215126F, -11.72655665123F, 30.07296232978F, 
	-29.09110291372F, 24.61626797170F, -40.76904818569F,  7.16934380275F, 
	-38.99981439277F, -14.99353746100F, -24.09797597474F, -30.96376197724F, 
	-4.40094187649F, -33.71982152599F,  9.59985189941F, -24.83984329451F, 
	12.58433949775F, -12.92576991423F,  7.74874038079F, -6.96828250565F, 
	 3.73875588584F, -9.65970469025F,  7.56437066057F, -15.66934380276F, 
	18.90075626927F, -16.13098659746F, 30.11250899328F, -5.89420675126F, 
	32.00000000000F, 12.00000000000F, 20.92810661652F, 28.06731553153F, 
	 1.93019352079F, 33.10154934594F, -14.60873811970F, 24.85374617952F, 
	-20.26124411416F,  9.65970469025F, -14.42436839948F, -2.21611987111F, 
	-4.38622325073F, -4.04479283424F,  0.41544952265F,  2.66673451424F, 
	-4.40094187649F,  9.71982152599F, -14.91357359798F,  8.79065319697F, 
	-22.02925164430F, -1.97702528748F, -18.59593940542F, -16.35374617952F, 
	-5.09110291372F, -24.61626797170F, 10.44655212904F, -20.88855995301F, 
	18.23199246986F, -7.68267940278F, 13.94454821626F,  5.81402662716F, 
	 1.91547489503F, 10.45595790571F, -8.11438491795F,  3.84152157252F, 
	-8.24652548840F, -8.01532400150F,  1.74359237754F, -15.28500639872F, 
	14.35234702788F, -11.72747223702F, 20.08140638688F,  0.66108771599F, 
	14.62035455802F, 13.23454015175F,  2.06233409125F, 17.29065319697F, 
	-8.26124411416F, 10.67621686378F, -8.92233183015F, -0.59481047690F, 
	-0.10582153948F, -6.94123927995F, 10.44655212905F, -2.56638452103F, 
	13.24671109563F,  9.99234928897F,  4.27094064100F, 21.57296232978F, 
	-11.85253263511F, 23.10920005697F, -25.09029260717F, 12.34152157252F, 
	-27.26782192291F, -4.50060537574F, -17.92236448670F, -17.29065319697F, 
	-4.38622325073F, -19.41015163980F,  3.91344560986F, -12.40779881162F, 
	 2.60563593225F, -4.44048854000F, -4.43201911051F, -3.72157141003F, 
	-8.24652548840F, -11.96937457645F, -1.93877342990F, -22.84809938127F, 
	13.66218599066F, -26.62391868273F, 30.11250899328F, -17.56073772278F, 
	37.23857027861F,  1.50706791474F, 30.43125070699F, 20.88855995302F, 
	13.91547489503F, 30.79187945975F, -2.42797862840F, 28.10045727514F, 
	-10.28254054867F, 18.14498606449F, -8.73796210994F, 10.21592461178F, 
	-4.40094187649F, 10.26487705195F, -5.76016196540F, 16.33984329451F, 
	-16.13293434636F, 20.21275361126F, -30.59232917650F, 13.96283096673F, 
	-39.26782192291F, -3.48409320221F, -34.59343669764F, -24.85374617952F, 
	-17.07638428796F, -39.27716083398F,  4.76014583950F, -39.81402662716F, 
	20.25328890438F, -28.16796077702F, 23.93689750523F, -13.11237105749F, 
	18.90075626927F, -3.85371198048F, 13.73998214862F, -3.33723400599F, 
	15.48546698146F, -6.50825608676F, 23.91670115781F, -4.77842858997F, 
	31.59091730649F,  6.75015842619F, 29.58455047735F, 24.83984329451F, 
	14.60563593225F, 39.89543301404F, -7.93001519773F, 42.71047307286F, 
	-27.25310329715F, 31.16149823802F, -34.59343669764F, 11.83723400599F, 
	-29.09110291372F, -4.63156939376F, -17.90216813927F, -11.06638452103F, 
	-10.48528137424F, -8.48528137424F, -11.40781493752F, -4.61237105749F, 
	-17.09110291372F, -7.36843060625F, -18.91468111913F, -18.33158720775F, 
	-10.28254054867F, -31.16149823802F,  7.74874038079F, -36.21611987111F, 
	26.60563593226F, -27.89543301404F, 36.07890367911F, -9.16108771599F, 
	31.59091730649F, 10.22040432229F, 17.42234795605F, 20.45718416849F, 
	 3.48546698146F, 18.50825608676F, -1.93877342990F,  9.83158720775F, 
	 1.93019352079F,  3.85371198048F,  8.25814192672F,  6.61801785573F, 
	 8.25328890437F, 16.16796077702F, -1.73420736226F, 24.13527104864F, 
	-17.07638428796F, 22.30659808551F, -28.09908349589F,  9.17499060100F, 
	-27.26782192291F, -8.51590679779F, -14.91357359798F, -20.45718416849F, 
	 0.83762840212F, -20.21275361126F,  9.91859361312F, -9.84549009275F, 
	 7.59905812351F,  1.73512294805F, -2.24360890818F,  5.46283096673F, 
	-10.28254054867F, -1.17442331602F, -8.92233183016F, -12.42170169663F, 
	 1.91547489503F, -18.79187945975F, 14.75249512847F, -14.39420675126F, 
	20.26800753014F, -1.50706791473F, 14.43375341476F, 11.06638452103F, 
	 1.66218599065F, 14.62391868273F, -8.43312663165F,  7.16934380275F, 
	-8.24652548840F, -5.00118817203F,  2.06233409125F, -11.95718416849F, 
	14.60563593226F, -7.55951146000F, 19.59220118838F,  5.91344560987F, 
	12.58433949775F, 19.41015163980F, -2.24360890818F, 23.78500639872F, 
	-15.26782192291F, 16.50060537574F, -18.59593940542F,  3.33723400599F, 
	-11.85253263511F, -6.13863730849F, -2.22341256076F, -5.89420675126F, 
	 1.24671109563F,  2.00765071103F, -5.23220344947F,  9.06073772278F, 
	-17.07638428796F,  6.94123927995F, -24.60108740867F, -5.89954272486F, 
	-20.26124411416F, -22.67621686378F, -4.43201911051F, -32.96940877548F, 
	14.62035455802F, -30.20510290023F, 26.57575958864F, -16.33984329451F, 
	26.35234702788F, -0.27252776298F, 17.42234795605F,  8.79065319697F, 
	 8.72403726008F,  8.01532400150F,  7.56437066057F,  2.65283162923F, 
	13.91547489503F,  1.54404209429F, 20.43890141802F,  9.86472895136F, 
	18.23199246986F, 24.65324215126F,  3.95219892729F, 36.56731553153F, 
	-17.09110291372F, 36.61626797170F, -34.27469498394F, 22.84809938127F, 
	-38.99981439278F,  1.97702528747F, -30.59232917650F, -15.28500639872F, 
	-16.40094187649F, -21.71982152599F, -6.07890367911F, -18.34549009276F, 
	-4.38622325073F, -12.92576991424F, -7.93001519773F, -13.46263570741F, 
	-8.26124411416F, -21.65970469026F,  1.07001745882F, -31.34809938127F, 
	18.90075626927F, -33.10154934594F, 36.60686219504F, -21.57296232977F, 
	44.00000000000F,  0.00000000000F, 36.60686219503F, 21.57296232978F, 
	18.90075626927F, 33.10154934594F,  1.07001745881F, 31.34809938127F, 
	-8.26124411416F, 21.65970469025F, -7.93001519773F, 13.46263570741F, 
	-4.38622325073F, 12.92576991424F, -6.07890367911F, 18.34549009275F, 
	-16.40094187649F, 21.71982152599F, -30.59232917650F, 15.28500639872F, 
	-38.99981439277F, -1.97702528748F, -34.27469498393F, -22.84809938127F, 
	-17.09110291372F, -36.61626797170F,  3.95219892729F, -36.56731553153F, 
	18.23199246987F, -24.65324215125F, 20.43890141802F, -9.86472895136F, 
	13.91547489503F, -1.54404209429F,  7.56437066057F, -2.65283162923F, 
	 8.72403726008F, -8.01532400150F, 17.42234795605F, -8.79065319697F, 
	26.35234702788F,  0.27252776298F, 26.57575958864F, 16.33984329451F, 
	14.62035455801F, 30.20510290023F, -4.43201911051F, 32.96940877548F, 
	-20.26124411416F, 22.67621686378F, -24.60108740867F,  5.89954272485F, 
	-17.07638428796F, -6.94123927995F, -5.23220344947F, -9.06073772278F, 
	 1.24671109563F, -2.00765071103F, -2.22341256076F,  5.89420675126F, 
	-11.85253263511F,  6.13863730849F, -18.59593940542F, -3.33723400599F, 
	-15.26782192291F, -16.50060537574F, -2.24360890818F, -23.78500639872F, 
	12.58433949775F, -19.41015163980F, 19.59220118838F, -5.91344560986F, 
	14.60563593225F,  7.55951146000F,  2.06233409124F, 11.95718416849F, 
	-8.24652548840F,  5.00118817203F, -8.43312663165F, -7.16934380276F, 
	 1.66218599066F, -14.62391868273F, 14.43375341476F, -11.06638452103F, 
	20.26800753014F,  1.50706791473F, 14.75249512847F, 14.39420675126F, 
	 1.91547489503F, 18.79187945974F, -8.92233183016F, 12.42170169663F, 
	-10.28254054867F,  1.17442331602F, -2.24360890818F, -5.46283096673F, 
	 7.59905812351F, -1.73512294805F,  9.91859361312F,  9.84549009276F, 
	 0.83762840212F, 20.21275361126F, -14.91357359798F, 20.45718416849F, 
	-27.26782192291F,  8.51590679779F, -28.09908349589F, -9.17499060100F, 
	-17.07638428796F, -22.30659808551F, -1.73420736225F, -24.13527104864F, 
	 8.25328890437F, -16.16796077702F,  8.25814192672F, -6.61801785573F, 
	 1.93019352079F, -3.85371198048F, -1.93877342990F, -9.83158720775F, 
	 3.48546698147F, -18.50825608676F, 17.42234795606F, -20.45718416849F, 
	31.59091730650F, -10.22040432228F, 36.07890367911F,  9.16108771600F, 
	26.60563593225F, 27.89543301404F,  7.74874038079F, 36.21611987111F, 
	-10.28254054867F, 31.16149823802F, -18.91468111913F, 18.33158720775F, 
	-17.09110291372F,  7.36843060624F, -11.40781493752F,  4.61237105749F, 
	-10.48528137424F,  8.48528137424F, -17.90216813927F, 11.06638452103F, 
	-29.09110291372F,  4.63156939375F, -34.59343669764F, -11.83723400599F, 
	-27.25310329715F, -31.16149823802F, -7.93001519772F, -42.71047307286F, 
	14.60563593226F, -39.89543301404F, 29.58455047735F, -24.83984329451F, 
	31.59091730649F, -6.75015842619F, 23.91670115781F,  4.77842858997F, 
	15.48546698146F,  6.50825608676F, 13.73998214862F,  3.33723400599F, 
	18.90075626927F,  3.85371198048F, 23.93689750523F, 13.11237105749F, 
	20.25328890437F, 28.16796077702F,  4.76014583950F, 39.81402662716F, 
	-17.07638428796F, 39.27716083398F, -34.59343669765F, 24.85374617952F, 
	-39.26782192291F,  3.48409320221F, -30.59232917650F, -13.96283096674F, 
	-16.13293434636F, -20.21275361126F, -5.76016196540F, -16.33984329451F, 
	-4.40094187649F, -10.26487705195F, -8.73796210994F, -10.21592461178F, 
	-10.28254054867F, -18.14498606449F, -2.42797862840F, -28.10045727514F, 
	13.91547489503F, -30.79187945974F, 30.43125070699F, -20.88855995301F, 
	37.23857027861F, -1.50706791473F, 30.11250899328F, 17.56073772278F, 
	13.66218599065F, 26.62391868273F, -1.93877342990F, 22.84809938127F, 
	-8.24652548840F, 11.96937457645F, -4.43201911051F,  3.72157141003F, 
	 2.60563593226F,  4.44048854000F,  3.91344560986F, 12.40779881162F, 
	-4.38622325073F, 19.41015163980F, -17.92236448670F, 17.29065319696F, 
	-27.26782192291F,  4.50060537573F, -25.09029260717F, -12.34152157253F, 
	-11.85253263510F, -23.10920005697F,  4.27094064100F, -21.57296232977F, 
	13.24671109563F, -9.99234928897F, 10.44655212904F,  2.56638452103F, 
	-0.10582153948F,  6.94123927995F, -8.92233183016F,  0.59481047690F, 
	-8.26124411416F, -10.67621686378F,  2.06233409125F, -17.29065319697F, 
	14.62035455802F, -13.23454015175F, 20.08140638688F, -0.66108771599F, 
	14.35234702788F, 11.72747223702F,  1.74359237754F, 15.28500639872F, 
	-8.24652548840F,  8.01532400150F, -8.11438491794F, -3.84152157253F, 
	 1.91547489503F, -10.45595790571F, 13.94454821626F, -5.81402662715F, 
	18.23199246986F,  7.68267940278F, 10.44655212904F, 20.88855995302F, 
	-5.09110291372F, 24.61626797170F, -18.59593940542F, 16.35374617952F, 
	-22.02925164430F,  1.97702528747F, -14.91357359798F, -8.79065319697F, 
	-4.40094187649F, -9.71982152599F,  0.41544952265F, -2.66673451424F, 
	-4.38622325073F,  4.04479283424F, -14.42436839948F,  2.21611987111F, 
	-20.26124411416F, -9.65970469026F, -14.60873811970F, -24.85374617952F, 
	 1.93019352079F, -33.10154934594F, 20.92810661652F, -28.06731553153F, 
	32.00000000000F, -12.00000000000F, 30.11250899328F,  5.89420675126F, 
	18.90075626927F, 16.13098659746F,  7.56437066057F, 15.66934380276F, 
	 3.73875588584F,  9.65970469025F,  7.74874038079F,  6.96828250565F, 
	12.58433949775F, 12.92576991424F,  9.59985189941F, 24.83984329451F, 
	-4.40094187650F, 33.71982152599F, -24.09797597475F, 30.96376197724F, 
	-38.99981439278F, 14.99353746100F, -40.76904818569F, -7.16934380276F, 
	-29.09110291372F, -24.61626797170F, -11.72655665122F, -30.07296232978F, 
	 1.26142972139F, -24.65324215126F,  4.76014583950F, -16.35908215311F, 
	 1.91547489503F, -13.54404209429F,  1.07001745882F, -18.33158720775F, 
	 8.72403726008F, -24.98588674997F, 23.91670115781F, -24.46940877548F, 
	38.35234702788F, -11.72747223702F, 42.25451516715F,  9.84549009276F, 
	31.59091730649F, 30.20510290023F, 11.24673646801F, 39.46376197724F, 
	-8.26124411416F, 34.67621686378F, -18.10673420692F, 21.57829830337F, 
	-17.07638428796F, 10.02932346853F, -11.72655665123F,  6.61801785573F, 
	-10.75328890437F,  9.99234928897F, -17.90216813927F, 12.38855995301F, 
	-28.82309538358F,  6.13863730849F, -34.27469498393F, -9.83158720775F, 
	-27.26782192291F, -28.50060537574F, -8.73796210993F, -39.46376197724F, 
	12.58433949775F, -36.38071438827F, 26.08655439014F, -21.59220118838F, 
	26.60563593225F, -4.44048853999F, 17.74108966976F,  5.46283096674F, 
	 8.72403726008F,  5.00118817203F,  7.24562894686F, -0.67499060100F, 
	13.66218599066F, -2.62391868273F, 20.92810661652F,  4.61237105749F, 
	20.26800753013F, 18.47763066321F,  8.25814192672F, 30.07296232978F, 
	-10.08452510497F, 30.79187945974F, -24.60108740867F, 18.91605489838F, 
	-27.25310329715F,  1.17442331601F, -17.92236448670F, -11.95718416849F, 
	-4.40094187649F, -13.73512294805F,  3.42424041136F, -5.83326548576F, 
	 0.83762840212F,  3.24219086278F, -8.41922039623F,  4.77842858997F, 
	-15.26782192291F, -3.48409320221F, -12.42032791737F, -15.66934380276F, 
	-0.10582153948F, -22.30659808551F, 13.94454821626F, -17.64091784688F, 
	20.25328890437F, -4.16796077702F, 14.75249512847F,  9.06073772278F, 
	 1.93019352079F, 13.11685076799F, -8.43312663166F,  5.84716837077F, 
	-8.51453301854F, -6.50825608677F,  1.74359237754F, -13.96283096674F, 
	14.62035455802F, -10.22040432229F, 20.40014810059F,  2.66673451424F, 
	14.60563593225F, 15.89543301404F,  1.25438717904F, 20.53736429259F, 
	-10.28254054867F, 14.19093548954F, -12.42032791737F,  2.65283162923F, 
	-5.09110291372F, -4.63156939375F,  4.27094064100F, -1.88198214426F, 
	 6.48528137424F,  8.48528137424F, -2.22341256076F, 17.56073772278F, 
	-17.09110291372F, 16.63156939375F, -28.09908349589F,  3.84152157252F, 
	-27.25310329715F, -14.19093548955F, -14.42436839948F, -27.03171749435F, 
	 2.60563593226F, -27.89543301404F, 13.90579489884F, -18.34549009275F, 
	14.62035455802F, -6.75015842619F,  8.23794557929F, -1.71592461178F, 
	 3.48546698146F, -5.49174391324F,  7.24562894686F, -12.34152157252F, 
	18.90075626927F, -13.11685076799F, 30.43125070699F, -2.56638452102F, 
	32.25328890437F, 16.16796077702F, 20.43890141801F, 33.31967342540F, 
	-0.10582153948F, 39.27716083398F, -18.91468111913F, 31.34809938127F, 
	-27.26782192291F, 15.48409320221F, -24.09797597474F,  1.71592461178F, 
	-16.13293434636F, -3.24219086278F, -12.25451516715F, -0.66108771599F, 
	-16.40094187649F,  1.73512294805F, -24.41671768845F, -3.72157141003F, 
	-27.25310329715F, -18.14498606450F, -18.10673420691F, -34.59481047690F, 
	 1.91547489503F, -42.79187945975F, 23.93689750524F, -36.56731553153F, 
	37.23857027861F, -18.47763066321F, 36.60686219503F,  1.88198214427F, 
	25.66218599065F, 14.62391868273F, 13.73998214862F, 16.35374617952F, 
	 8.72403726008F, 11.96937457645F, 11.24673646801F, 10.21592461178F, 
	14.60563593225F, 16.44048854000F, 10.40779881162F, 28.08655439014F, 
	-4.38622325073F, 36.38071438828F, -24.41671768845F, 32.96940877548F, 
	-39.26782192291F, 16.50060537573F, -40.76904818569F, -5.84716837077F, 
	-28.82309538358F, -23.10920005697F, -11.40781493751F, -28.06731553153F, 
	 1.24671109563F, -21.99234928897F,  3.95219892729F, -13.11237105749F, 
	-0.10582153948F, -10.02932346853F, -2.42797862840F, -15.08394510162F, 
	 3.73875588584F, -22.67621686378F, 17.74108966976F, -23.78500639872F, 
	31.59091730649F, -13.23454015175F, 35.76016196540F,  5.83326548576F, 
	26.35234702788F, 23.72747223702F,  8.23794557929F, 30.96376197724F, 
	-8.24652548840F, 24.98588674997F, -14.60873811970F, 11.83723400599F, 
	-10.08452510497F,  1.54404209429F, -1.73420736225F,  0.68032657460F, 
	 1.26142972139F,  7.68267940278F, -5.23220344947F, 14.39420675126F, 
	-17.09110291372F, 12.61626797170F, -25.09029260717F,  0.67499060100F, 
	-22.02925164430F, -14.99353746100F, -8.41922039622F, -24.46940877548F, 
	 7.59905812351F, -21.71982152599F, 16.09420510116F, -9.16108771599F, 
	12.58433949774F,  4.04479283424F,  1.25438717903F,  8.71047307286F, 
	-8.26124411416F,  2.34029530974F, -8.11438491795F, -9.17499060100F, 
	 1.93019352079F, -16.13098659746F, 14.43375341476F, -12.38855995301F, 
	20.00000000000F,  0.00000000000F, 14.43375341476F, 12.38855995301F, 
	 1.93019352079F, 16.13098659746F, -8.11438491794F,  9.17499060100F, 
	-8.26124411416F, -2.34029530975F,  1.25438717904F, -8.71047307286F, 
	12.58433949775F, -4.04479283424F, 16.09420510116F,  9.16108771600F, 
	 7.59905812351F, 21.71982152599F, -8.41922039623F, 24.46940877548F, 
	-22.02925164430F, 14.99353746100F, -25.09029260717F, -0.67499060100F, 
	-17.09110291372F, -12.61626797170F, -5.23220344947F, -14.39420675126F, 
	 1.26142972139F, -7.68267940278F, -1.73420736226F, -0.68032657460F, 
	-10.08452510497F, -1.54404209429F, -14.60873811970F, -11.83723400599F, 
	-8.24652548840F, -24.98588674998F,  8.23794557930F, -30.96376197724F, 
	26.35234702788F, -23.72747223702F, 35.76016196540F, -5.83326548576F, 
	31.59091730649F, 13.23454015176F, 17.74108966976F, 23.78500639872F, 
	 3.73875588584F, 22.67621686378F, -2.42797862840F, 15.08394510162F, 
	-0.10582153948F, 10.02932346853F,  3.95219892729F, 13.11237105749F, 
	 1.24671109562F, 21.99234928897F, -11.40781493752F, 28.06731553153F, 
	-28.82309538359F, 23.10920005696F, -40.76904818569F,  5.84716837077F, 
	-39.26782192291F, -16.50060537574F, -24.41671768845F, -32.96940877548F, 
	-4.38622325073F, -36.38071438828F, 10.40779881162F, -28.08655439013F, 
	14.60563593226F, -16.44048854000F, 11.24673646801F, -10.21592461178F, 
	 8.72403726008F, -11.96937457645F, 13.73998214862F, -16.35374617952F, 
	25.66218599066F, -14.62391868273F, 36.60686219503F, -1.88198214426F, 
	37.23857027861F, 18.47763066321F, 23.93689750523F, 36.56731553153F, 
	 1.91547489503F, 42.79187945974F, -18.10673420692F, 34.59481047690F, 
	-27.25310329715F, 18.14498606449F, -24.41671768845F,  3.72157141003F, 
	-16.40094187649F, -1.73512294805F, -12.25451516715F,  0.66108771599F, 
	-16.13293434636F,  3.24219086278F, -24.09797597474F, -1.71592461178F, 
	-27.26782192291F, -15.48409320221F, -18.91468111912F, -31.34809938127F, 
	-0.10582153948F, -39.27716083398F, 20.43890141802F, -33.31967342540F, 
	32.25328890437F, -16.16796077701F, 30.43125070699F,  2.56638452103F, 
	18.90075626927F, 13.11685076800F,  7.24562894686F, 12.34152157252F, 
	 3.48546698146F,  5.49174391324F,  8.23794557929F,  1.71592461178F, 
	14.62035455802F,  6.75015842619F, 13.90579489884F, 18.34549009276F, 
	 2.60563593225F, 27.89543301404F, -14.42436839948F, 27.03171749434F, 
	-27.25310329715F, 14.19093548954F, -28.09908349589F, -3.84152157253F, 
	-17.09110291372F, -16.63156939376F, -2.22341256075F, -17.56073772278F, 
	 6.48528137424F, -8.48528137424F,  4.27094064100F,  1.88198214427F, 
	-5.09110291372F,  4.63156939376F, -12.42032791737F, -2.65283162923F, 
	-10.28254054867F, -14.19093548954F,  1.25438717904F, -20.53736429259F, 
	14.60563593226F, -15.89543301404F, 20.40014810059F, -2.66673451424F, 
	14.62035455801F, 10.22040432229F,  1.74359237754F, 13.96283096673F, 
	-8.51453301854F,  6.50825608676F, -8.43312663165F, -5.84716837077F, 
	 1.93019352079F, -13.11685076800F, 14.75249512847F, -9.06073772278F, 
	20.25328890438F,  4.16796077702F, 13.94454821626F, 17.64091784689F, 
	-0.10582153948F, 22.30659808551F, -12.42032791737F, 15.66934380276F, 
	-15.26782192291F,  3.48409320221F, -8.41922039623F, -4.77842858997F, 
	 0.83762840212F, -3.24219086278F,  3.42424041136F,  5.83326548576F, 
	-4.40094187650F, 13.73512294805F, -17.92236448670F, 11.95718416849F, 
	-27.25310329715F, -1.17442331602F, -24.60108740867F, -18.91605489839F, 
	-10.08452510497F, -30.79187945975F,  8.25814192672F, -30.07296232978F, 
	20.26800753014F, -18.47763066321F, 20.92810661652F, -4.61237105749F, 
	13.66218599066F,  2.62391868273F,  7.24562894686F,  0.67499060100F, 
	 8.72403726008F, -5.00118817203F, 17.74108966976F, -5.46283096673F, 
	26.60563593226F,  4.44048854000F, 26.08655439013F, 21.59220118838F, 
	12.58433949774F, 36.38071438828F, -8.73796210994F, 39.46376197724F, 
	-27.26782192291F, 28.50060537573F, -34.27469498394F,  9.83158720774F, 
	-28.82309538358F, -6.13863730849F, -17.90216813927F, -12.38855995301F, 
	-10.75328890437F, -9.99234928897F, -11.72655665123F, -6.61801785574F, 
	-17.07638428796F, -10.02932346853F, -18.10673420692F, -21.57829830337F, 
	-8.26124411416F, -34.67621686378F, 11.24673646801F, -39.46376197724F, 
	31.59091730650F, -30.20510290023F, 42.25451516715F, -9.84549009275F, 
	38.35234702788F, 11.72747223702F, 23.91670115781F, 24.46940877548F, 
	 8.72403726007F, 24.98588674997F,  1.07001745882F, 18.33158720775F, 
	 1.91547489503F, 13.54404209429F,  4.76014583950F, 16.35908215312F, 
	 1.26142972139F, 24.65324215126F, -11.72655665123F, 30.07296232977F, 
	-29.09110291372F, 24.61626797170F, -40.76904818569F,  7.16934380275F, 
	-38.99981439277F, -14.99353746101F, -24.09797597474F, -30.96376197724F, 
	-4.40094187649F, -33.71982152599F,  9.59985189941F, -24.83984329451F, 
	12.58433949775F, -12.92576991423F,  7.74874038079F, -6.96828250565F, 
	 3.73875588584F, -9.65970469025F,  7.56437066057F, -15.66934380276F, 
	18.90075626927F, -16.13098659746F, 30.11250899328F, -5.89420675126F, 
	32.00000000000F, 12.00000000000F, 20.92810661651F, 28.06731553153F, 
	 1.93019352079F, 33.10154934594F, -14.60873811970F, 24.85374617952F, 
	-20.26124411416F,  9.65970469025F, -14.42436839948F, -2.21611987111F, 
	-4.38622325073F, -4.04479283424F,  0.41544952265F,  2.66673451424F, 
	-4.40094187649F,  9.71982152599F, -14.91357359798F,  8.79065319697F, 
	-22.02925164430F, -1.97702528748F, -18.59593940542F, -16.35374617952F, 
	-5.09110291372F, -24.61626797170F, 10.44655212905F, -20.88855995301F, 
	18.23199246986F, -7.68267940278F, 13.94454821626F,  5.81402662716F, 
	 1.91547489503F, 10.45595790571F, -8.11438491795F,  3.84152157252F, 
	-8.24652548840F, -8.01532400150F,  1.74359237754F, -15.28500639872F, 
	14.35234702788F, -11.72747223702F, 20.08140638688F,  0.66108771599F, 
	14.62035455802F, 13.23454015175F,  2.06233409125F, 17.29065319697F, 
	-8.26124411416F, 10.67621686378F, -8.92233183015F, -0.59481047690F, 
	-0.10582153948F, -6.94123927995F, 10.44655212905F, -2.56638452102F, 
	13.24671109563F,  9.99234928898F,  4.27094064100F, 21.57296232978F, 
	-11.85253263511F, 23.10920005697F, -25.09029260718F, 12.34152157252F, 
	-27.26782192291F, -4.50060537574F, -17.92236448670F, -17.29065319697F, 
	-4.38622325073F, -19.41015163980F,  3.91344560986F, -12.40779881162F, 
	 2.60563593225F, -4.44048854000F, -4.43201911051F, -3.72157141003F, 
	-8.24652548840F, -11.96937457645F, -1.93877342990F, -22.84809938127F, 
	13.66218599066F, -26.62391868273F, 30.11250899328F, -17.56073772278F, 
	37.23857027861F,  1.50706791474F, 30.43125070699F, 20.88855995302F, 
}; 


// End of File
