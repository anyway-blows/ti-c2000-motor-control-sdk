//#############################################################################
//! \file   golden.c
//! \brief  Sine/Cosine Ouput Vector (512) 
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


// The data is organized sine first
const float32_t test_golden[256] = {
    -0.00000000000F, -1.00000000000F, -0.01227153829F, -0.99992470184F, 
    -0.02454122852F, -0.99969881870F, -0.03680722294F, -0.99932238459F, 
    -0.04906767433F, -0.99879545621F, -0.06132073630F, -0.99811811290F, 
    -0.07356456360F, -0.99729045668F, -0.08579731234F, -0.99631261218F, 
    -0.09801714033F, -0.99518472667F, -0.11022220729F, -0.99390697000F, 
    -0.12241067520F, -0.99247953460F, -0.13458070851F, -0.99090263543F, 
    -0.14673047446F, -0.98917650996F, -0.15885814333F, -0.98730141816F, 
    -0.17096188876F, -0.98527764239F, -0.18303988796F, -0.98310548743F, 
    -0.19509032202F, -0.98078528040F, -0.20711137619F, -0.97831737072F, 
    -0.21910124016F, -0.97570213004F, -0.23105810828F, -0.97293995221F, 
    -0.24298017990F, -0.97003125319F, -0.25486565960F, -0.96697647104F, 
    -0.26671275747F, -0.96377606580F, -0.27851968939F, -0.96043051942F, 
    -0.29028467725F, -0.95694033573F, -0.30200594932F, -0.95330604035F, 
    -0.31368174040F, -0.94952818059F, -0.32531029216F, -0.94560732538F, 
    -0.33688985339F, -0.94154406518F, -0.34841868025F, -0.93733901191F, 
    -0.35989503653F, -0.93299279883F, -0.37131719395F, -0.92850608047F, 
    -0.38268343237F, -0.92387953251F, -0.39399204006F, -0.91911385169F, 
    -0.40524131400F, -0.91420975570F, -0.41642956010F, -0.90916798309F, 
    -0.42755509343F, -0.90398929312F, -0.43861623854F, -0.89867446569F, 
    -0.44961132965F, -0.89322430120F, -0.46053871096F, -0.88763962040F, 
    -0.47139673683F, -0.88192126435F, -0.48218377208F, -0.87607009420F, 
    -0.49289819223F, -0.87008699111F, -0.50353838373F, -0.86397285612F, 
    -0.51410274419F, -0.85772861000F, -0.52458968268F, -0.85135519311F, 
    -0.53499761989F, -0.84485356525F, -0.54532498842F, -0.83822470555F, 
    -0.55557023302F, -0.83146961230F, -0.56573181078F, -0.82458930279F, 
    -0.57580819142F, -0.81758481315F, -0.58579785746F, -0.81045719825F, 
    -0.59569930449F, -0.80320753148F, -0.60551104140F, -0.79583690461F, 
    -0.61523159058F, -0.78834642763F, -0.62485948814F, -0.78073722857F, 
    -0.63439328416F, -0.77301045336F, -0.64383154289F, -0.76516726562F, 
    -0.65317284295F, -0.75720884651F, -0.66241577759F, -0.74913639452F, 
    -0.67155895485F, -0.74095112535F, -0.68060099780F, -0.73265427167F, 
    -0.68954054474F, -0.72424708295F, -0.69837624941F, -0.71573082528F, 
    -0.70710678119F, -0.70710678119F, -0.71573082528F, -0.69837624941F, 
    -0.72424708295F, -0.68954054474F, -0.73265427167F, -0.68060099780F, 
    -0.74095112535F, -0.67155895485F, -0.74913639452F, -0.66241577759F, 
    -0.75720884651F, -0.65317284295F, -0.76516726562F, -0.64383154289F, 
    -0.77301045336F, -0.63439328416F, -0.78073722857F, -0.62485948814F, 
    -0.78834642763F, -0.61523159058F, -0.79583690461F, -0.60551104140F, 
    -0.80320753148F, -0.59569930449F, -0.81045719825F, -0.58579785746F, 
    -0.81758481315F, -0.57580819142F, -0.82458930279F, -0.56573181078F, 
    -0.83146961230F, -0.55557023302F, -0.83822470555F, -0.54532498842F, 
    -0.84485356525F, -0.53499761989F, -0.85135519311F, -0.52458968268F, 
    -0.85772861000F, -0.51410274419F, -0.86397285612F, -0.50353838373F, 
    -0.87008699111F, -0.49289819223F, -0.87607009420F, -0.48218377208F, 
    -0.88192126435F, -0.47139673683F, -0.88763962040F, -0.46053871096F, 
    -0.89322430120F, -0.44961132965F, -0.89867446569F, -0.43861623854F, 
    -0.90398929312F, -0.42755509343F, -0.90916798309F, -0.41642956010F, 
    -0.91420975570F, -0.40524131400F, -0.91911385169F, -0.39399204006F, 
    -0.92387953251F, -0.38268343237F, -0.92850608047F, -0.37131719395F, 
    -0.93299279883F, -0.35989503653F, -0.93733901191F, -0.34841868025F, 
    -0.94154406518F, -0.33688985339F, -0.94560732538F, -0.32531029216F, 
    -0.94952818059F, -0.31368174040F, -0.95330604035F, -0.30200594932F, 
    -0.95694033573F, -0.29028467725F, -0.96043051942F, -0.27851968939F, 
    -0.96377606580F, -0.26671275747F, -0.96697647104F, -0.25486565960F, 
    -0.97003125319F, -0.24298017990F, -0.97293995221F, -0.23105810828F, 
    -0.97570213004F, -0.21910124016F, -0.97831737072F, -0.20711137619F, 
    -0.98078528040F, -0.19509032202F, -0.98310548743F, -0.18303988796F, 
    -0.98527764239F, -0.17096188876F, -0.98730141816F, -0.15885814333F, 
    -0.98917650996F, -0.14673047446F, -0.99090263543F, -0.13458070851F, 
    -0.99247953460F, -0.12241067520F, -0.99390697000F, -0.11022220729F, 
    -0.99518472667F, -0.09801714033F, -0.99631261218F, -0.08579731234F, 
    -0.99729045668F, -0.07356456360F, -0.99811811290F, -0.06132073630F, 
    -0.99879545621F, -0.04906767433F, -0.99932238459F, -0.03680722294F, 
    -0.99969881870F, -0.02454122852F, -0.99992470184F, -0.01227153829F
};

