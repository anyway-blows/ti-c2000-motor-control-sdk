//#############################################################################
//! \file   input.c
//! \brief  FID Input Vectors (256) 
//! \author Vishal Coelho 
//! \date   18-Apr-2016
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


#include <stdint.h>

#include "fastrts.h"

const float64_t test_dividend[256] = {
           1.314641e+308L,        4.433504e+307L,        1.471249e+307L,        1.533268e+308L, 
           1.234210e+308L,        1.722389e+308L,        1.770828e+308L,        9.612672e+307L, 
           1.628675e+308L,        8.568680e+307L,        1.599356e+308L,        4.287455e+306L, 
           2.314077e+307L,        1.331444e+308L,        7.029343e+307L,        7.076232e+307L, 
           1.081939e+308L,        4.293875e+306L,        6.969135e+307L,        1.020121e+307L, 
           1.239636e+308L,        9.355032e+307L,        1.367801e+307L,        7.303304e+307L, 
           7.733907e+307L,        1.185241e+308L,        3.926003e+307L,        1.457725e+307L, 
           8.407302e+307L,        1.309928e+308L,        1.103755e+308L,        1.618998e+308L, 
           4.544197e+307L,        3.956638e+307L,        4.239050e+307L,        1.456364e+308L, 
           1.262716e+308L,        1.579282e+308L,        1.278180e+308L,        1.188419e+308L, 
           7.902253e+307L,        1.011730e+308L,        1.780276e+308L,        1.720959e+308L, 
           6.054959e+307L,        2.794618e+307L,        7.497803e+307L,        9.967795e+307L, 
           5.348518e+306L,        1.558072e+308L,        1.607882e+308L,        8.937957e+307L, 
           3.762338e+307L,        1.297210e+308L,        1.257875e+308L,        8.282821e+307L, 
           3.066739e+307L,        1.428036e+308L,        1.597250e+308L,        2.561974e+307L, 
           1.689241e+308L,        4.689375e+304L,        7.888368e+307L,        2.007622e+306L, 
           4.215869e+307L,        1.270941e+308L,        1.247895e+308L,        4.766059e+307L, 
           5.870792e+307L,        1.676060e+307L,        7.478287e+306L,        1.364953e+308L, 
           5.840246e+307L,        9.380755e+307L,        1.711202e+308L,        1.202618e+308L, 
           8.505275e+307L,        1.167372e+308L,        1.196609e+308L,        4.137345e+307L, 
           1.446945e+308L,        9.038532e+307L,        7.802522e+307L,        1.274625e+308L, 
           9.981943e+307L,        4.584186e+307L,        1.351875e+307L,        1.059025e+308L, 
           1.212080e+308L,        1.057068e+308L,        9.594329e+307L,        8.496192e+307L, 
           5.821132e+307L,        7.747807e+307L,        1.375052e+308L,        1.791334e+308L, 
           4.764216e+307L,        1.423893e+308L,        2.419861e+307L,        6.388309e+306L, 
           8.160565e+307L,        1.012060e+308L,        4.150564e+307L,        2.602384e+307L, 
           1.217211e+308L,        6.365101e+307L,        9.176726e+307L,        1.248869e+308L, 
           1.070154e+308L,        9.098318e+307L,        1.395728e+308L,        1.231784e+308L, 
           5.743515e+307L,        1.495938e+308L,        1.187954e+308L,        1.055369e+308L, 
           3.429716e+306L,        1.504579e+308L,        2.049332e+307L,        1.510818e+308L, 
           6.067155e+307L,        7.719935e+307L,        1.426970e+308L,        2.544949e+307L, 
           1.093181e+308L,        1.672985e+308L,        1.144131e+308L,        8.587698e+306L, 
           1.255412e+308L,        4.622040e+307L,        1.242029e+308L,        6.528858e+307L, 
           9.741416e+307L,        3.993430e+307L,        1.627788e+308L,        6.715124e+307L, 
           1.416879e+307L,        5.560254e+306L,        1.126975e+308L,        1.684432e+308L, 
           5.751455e+307L,        1.339595e+308L,        5.194046e+307L,        2.235504e+307L, 
           9.241599e+307L,        6.966274e+307L,        1.759240e+308L,        5.245987e+307L, 
           1.064850e+308L,        1.027124e+308L,        1.107120e+308L,        1.008352e+308L, 
           5.031013e+307L,        2.729790e+307L,        1.317732e+308L,        7.136576e+306L, 
           1.441071e+308L,        7.306129e+307L,        1.492106e+308L,        1.745624e+308L, 
           7.594949e+307L,        9.690437e+306L,        1.414851e+308L,        1.132685e+308L, 
           9.147011e+307L,        1.020526e+308L,        5.558303e+307L,        1.343121e+308L, 
           6.052129e+307L,        1.588000e+308L,        5.998110e+307L,        1.430990e+308L, 
           1.446754e+308L,        9.031372e+306L,        7.235807e+307L,        1.735118e+308L, 
           7.449208e+307L,        5.941800e+307L,        1.144002e+308L,        1.542466e+308L, 
           1.417093e+308L,        9.678955e+307L,        1.333296e+308L,        1.329791e+308L, 
           7.996042e+307L,        1.528776e+308L,        8.890219e+307L,        1.704496e+308L, 
           1.434178e+308L,        2.391733e+307L,        4.274371e+307L,        2.296076e+307L, 
           9.601846e+307L,        1.019233e+307L,        8.423541e+307L,        1.766571e+308L, 
           1.226424e+308L,        1.565612e+308L,        8.920343e+307L,        5.171284e+307L, 
           3.290126e+307L,        1.727977e+308L,        1.359526e+308L,        1.708444e+308L, 
           1.206731e+308L,        3.344691e+307L,        5.210469e+307L,        1.046398e+307L, 
           7.921755e+306L,        5.025610e+307L,        3.064521e+307L,        1.217219e+308L, 
           1.622271e+308L,        6.327652e+307L,        1.800543e+307L,        1.425575e+308L, 
           8.420978e+307L,        1.235629e+308L,        1.227966e+308L,        7.576124e+307L, 
           1.185296e+308L,        1.138708e+308L,        9.440514e+307L,        3.438653e+307L, 
           9.202499e+307L,        1.345268e+308L,        7.547200e+307L,        1.325762e+308L, 
           1.234058e+308L,        1.718102e+308L,        5.535213e+307L,        6.153519e+307L, 
           6.957676e+307L,        4.705625e+306L,        1.138922e+308L,        1.016148e+308L, 
           6.570774e+307L,        7.576534e+307L,        8.308943e+307L,        5.652644e+307L, 
           4.056705e+307L,        2.084189e+307L,        2.408299e+307L,        1.701559e+308L, 
           8.852434e+307L,        1.345059e+308L,        5.589660e+307L,        1.385174e+306L, 
           1.106699e+308L,        6.428071e+307L,        1.166396e+308L,        1.520807e+308L, 
           1.822020e+307L,        1.057586e+308L,        1.046944e+308L,        9.380430e+307L, 
}; 

#include "fastrts.h"

const float64_t test_divisor[256] = {
           8.172310e+307L,        1.737834e+308L,        3.277320e+307L,        1.005615e+308L, 
           1.126209e+308L,        5.416449e+306L,        1.154610e+308L,        1.506757e+308L, 
           7.103333e+307L,        7.862071e+307L,        1.525932e+307L,        1.045138e+308L, 
           1.567017e+308L,        3.719554e+307L,        1.700798e+307L,        3.378804e+307L, 
           1.200565e+308L,        1.316668e+308L,        1.134363e+308L,        8.416910e+307L, 
           5.651270e+307L,        1.153160e+308L,        1.010352e+308L,        1.193177e+308L, 
           2.244906e+307L,        1.211694e+308L,        1.222991e+308L,        1.672071e+308L, 
           1.489963e+308L,        3.201698e+307L,        1.706443e+308L,        3.588606e+307L, 
           3.913230e+307L,        1.531983e+306L,        5.404343e+307L,        4.934143e+307L, 
           1.054254e+308L,        7.128803e+307L,        3.350950e+307L,        7.061633e+307L, 
           2.779123e+307L,        1.496228e+308L,        2.692083e+307L,        1.221615e+308L, 
           1.348432e+308L,        2.676767e+306L,        2.557156e+307L,        5.510937e+306L, 
           1.453786e+308L,        8.475396e+306L,        1.197590e+308L,        8.198064e+307L, 
           1.443541e+308L,        6.443632e+307L,        4.554284e+306L,        3.929544e+307L, 
           1.278007e+308L,        9.867911e+307L,        1.563441e+308L,        1.328200e+308L, 
           1.855761e+307L,        1.746339e+308L,        1.327979e+308L,        2.491167e+306L, 
           1.220852e+308L,        8.727849e+307L,        6.241294e+306L,        2.057273e+307L, 
           1.377484e+307L,        1.092682e+308L,        4.197000e+307L,        2.552930e+307L, 
           9.530887e+307L,        1.672396e+308L,        1.109002e+308L,        1.240214e+308L, 
           4.214258e+307L,        3.718418e+307L,        6.717308e+306L,        1.192527e+308L, 
           9.118881e+307L,        8.733201e+307L,        1.549063e+308L,        1.735700e+308L, 
           1.365686e+308L,        4.434540e+307L,        1.215721e+308L,        1.364998e+308L, 
           1.053789e+308L,        1.478493e+308L,        1.666165e+308L,        1.146150e+308L, 
           3.404447e+306L,        3.313879e+307L,        4.807159e+306L,        1.680841e+308L, 
           8.709236e+307L,        1.428781e+308L,        8.212498e+307L,        8.884366e+307L, 
           7.653030e+307L,        1.603133e+308L,        1.571696e+308L,        1.138210e+308L, 
           5.254285e+307L,        5.243482e+307L,        4.413337e+307L,        1.266201e+308L, 
           1.392514e+308L,        1.478879e+308L,        1.449282e+308L,        4.562023e+307L, 
           1.439053e+308L,        1.132023e+308L,        4.161688e+307L,        6.850591e+307L, 
           8.147175e+307L,        8.998525e+307L,        9.717818e+307L,        6.204651e+307L, 
           5.239018e+307L,        1.264729e+308L,        1.679186e+308L,        1.068144e+308L, 
           1.066195e+308L,        1.328012e+308L,        6.698981e+307L,        8.148410e+307L, 
           9.157010e+307L,        1.669537e+308L,        8.103781e+307L,        1.145912e+308L, 
           8.257953e+307L,        9.955165e+307L,        7.412367e+306L,        1.442113e+308L, 
           1.207172e+308L,        3.601275e+307L,        2.622182e+306L,        1.371938e+308L, 
           1.020212e+308L,        1.176076e+308L,        1.141068e+308L,        1.320100e+308L, 
           1.595141e+308L,        1.197100e+308L,        2.169359e+307L,        1.283078e+308L, 
           8.039772e+307L,        1.152290e+308L,        1.166516e+308L,        1.219511e+308L, 
           1.057540e+307L,        4.025992e+307L,        3.019793e+307L,        1.669211e+308L, 
           9.262272e+307L,        1.210501e+308L,        1.200500e+308L,        1.464426e+307L, 
           4.046130e+307L,        5.108850e+307L,        1.642667e+308L,        4.118543e+307L, 
           4.469432e+307L,        1.215763e+308L,        1.165787e+308L,        1.264995e+308L, 
           1.504855e+308L,        1.307454e+308L,        3.493707e+306L,        1.049397e+308L, 
           1.655454e+308L,        1.648089e+307L,        9.969298e+307L,        1.673298e+308L, 
           3.585552e+307L,        9.443634e+307L,        2.278380e+307L,        1.345982e+308L, 
           5.656777e+307L,        3.364817e+307L,        1.762450e+308L,        1.281444e+308L, 
           1.484265e+308L,        1.228220e+308L,        1.239333e+308L,        9.543519e+307L, 
           3.395010e+307L,        8.862910e+307L,        1.005881e+308L,        1.488994e+308L, 
           3.646955e+307L,        9.487258e+307L,        1.452197e+308L,        6.383688e+307L, 
           9.846955e+307L,        1.289733e+308L,        3.640551e+307L,        1.157277e+308L, 
           1.600234e+304L,        9.225347e+307L,        1.024378e+308L,        1.264983e+306L, 
           1.604392e+308L,        1.497826e+308L,        1.393510e+308L,        1.416826e+308L, 
           6.716713e+307L,        2.738102e+307L,        6.332559e+307L,        1.159688e+308L, 
           1.675336e+308L,        1.677626e+307L,        1.328096e+308L,        9.935375e+306L, 
           1.361139e+308L,        8.332116e+307L,        8.090419e+306L,        1.513711e+308L, 
           2.961008e+307L,        2.068676e+307L,        4.881831e+307L,        5.651701e+307L, 
           1.089563e+308L,        1.217877e+308L,        1.775478e+308L,        1.785578e+308L, 
           1.361818e+308L,        4.944728e+307L,        1.715024e+308L,        7.390000e+307L, 
           3.894222e+307L,        1.130983e+308L,        2.672997e+306L,        7.783973e+306L, 
           3.240560e+307L,        3.603747e+307L,        1.293184e+308L,        7.961400e+307L, 
           1.519936e+308L,        7.007230e+307L,        1.505662e+308L,        1.345427e+308L, 
           1.049603e+308L,        2.886621e+307L,        9.506924e+307L,        8.317403e+307L, 
           6.823034e+307L,        1.675989e+307L,        4.658640e+307L,        6.037312e+307L, 
           6.743367e+307L,        2.629300e+307L,        5.694550e+307L,        5.055570e+307L, 
}; 


// End of File
