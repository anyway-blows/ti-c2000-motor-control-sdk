//#############################################################################
//! \file input.c
//! \brief  Exp Input Vector (512) 
//! \author Vishal Coelho 
//! \date   27-Jan-2017
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

const float32_t test_input[512] = {
     1.00000000000F,  1.19335937500F,  1.38671875000F,  1.58007812500F, 
     1.77343750000F,  1.96679687500F,  2.16015625000F,  2.35351562500F, 
     2.54687500000F,  2.74023437500F,  2.93359375000F,  3.12695312500F, 
     3.32031250000F,  3.51367187500F,  3.70703125000F,  3.90039062500F, 
     4.09375000000F,  4.28710937500F,  4.48046875000F,  4.67382812500F, 
     4.86718750000F,  5.06054687500F,  5.25390625000F,  5.44726562500F, 
     5.64062500000F,  5.83398437500F,  6.02734375000F,  6.22070312500F, 
     6.41406250000F,  6.60742187500F,  6.80078125000F,  6.99414062500F, 
     7.18750000000F,  7.38085937500F,  7.57421875000F,  7.76757812500F, 
     7.96093750000F,  8.15429687500F,  8.34765625000F,  8.54101562500F, 
     8.73437500000F,  8.92773437500F,  9.12109375000F,  9.31445312500F, 
     9.50781250000F,  9.70117187500F,  9.89453125000F, 10.08789062500F, 
    10.28125000000F, 10.47460937500F, 10.66796875000F, 10.86132812500F, 
    11.05468750000F, 11.24804687500F, 11.44140625000F, 11.63476562500F, 
    11.82812500000F, 12.02148437500F, 12.21484375000F, 12.40820312500F, 
    12.60156250000F, 12.79492187500F, 12.98828125000F, 13.18164062500F, 
    13.37500000000F, 13.56835937500F, 13.76171875000F, 13.95507812500F, 
    14.14843750000F, 14.34179687500F, 14.53515625000F, 14.72851562500F, 
    14.92187500000F, 15.11523437500F, 15.30859375000F, 15.50195312500F, 
    15.69531250000F, 15.88867187500F, 16.08203125000F, 16.27539062500F, 
    16.46875000000F, 16.66210937500F, 16.85546875000F, 17.04882812500F, 
    17.24218750000F, 17.43554687500F, 17.62890625000F, 17.82226562500F, 
    18.01562500000F, 18.20898437500F, 18.40234375000F, 18.59570312500F, 
    18.78906250000F, 18.98242187500F, 19.17578125000F, 19.36914062500F, 
    19.56250000000F, 19.75585937500F, 19.94921875000F, 20.14257812500F, 
    20.33593750000F, 20.52929687500F, 20.72265625000F, 20.91601562500F, 
    21.10937500000F, 21.30273437500F, 21.49609375000F, 21.68945312500F, 
    21.88281250000F, 22.07617187500F, 22.26953125000F, 22.46289062500F, 
    22.65625000000F, 22.84960937500F, 23.04296875000F, 23.23632812500F, 
    23.42968750000F, 23.62304687500F, 23.81640625000F, 24.00976562500F, 
    24.20312500000F, 24.39648437500F, 24.58984375000F, 24.78320312500F, 
    24.97656250000F, 25.16992187500F, 25.36328125000F, 25.55664062500F, 
    25.75000000000F, 25.94335937500F, 26.13671875000F, 26.33007812500F, 
    26.52343750000F, 26.71679687500F, 26.91015625000F, 27.10351562500F, 
    27.29687500000F, 27.49023437500F, 27.68359375000F, 27.87695312500F, 
    28.07031250000F, 28.26367187500F, 28.45703125000F, 28.65039062500F, 
    28.84375000000F, 29.03710937500F, 29.23046875000F, 29.42382812500F, 
    29.61718750000F, 29.81054687500F, 30.00390625000F, 30.19726562500F, 
    30.39062500000F, 30.58398437500F, 30.77734375000F, 30.97070312500F, 
    31.16406250000F, 31.35742187500F, 31.55078125000F, 31.74414062500F, 
    31.93750000000F, 32.13085937500F, 32.32421875000F, 32.51757812500F, 
    32.71093750000F, 32.90429687500F, 33.09765625000F, 33.29101562500F, 
    33.48437500000F, 33.67773437500F, 33.87109375000F, 34.06445312500F, 
    34.25781250000F, 34.45117187500F, 34.64453125000F, 34.83789062500F, 
    35.03125000000F, 35.22460937500F, 35.41796875000F, 35.61132812500F, 
    35.80468750000F, 35.99804687500F, 36.19140625000F, 36.38476562500F, 
    36.57812500000F, 36.77148437500F, 36.96484375000F, 37.15820312500F, 
    37.35156250000F, 37.54492187500F, 37.73828125000F, 37.93164062500F, 
    38.12500000000F, 38.31835937500F, 38.51171875000F, 38.70507812500F, 
    38.89843750000F, 39.09179687500F, 39.28515625000F, 39.47851562500F, 
    39.67187500000F, 39.86523437500F, 40.05859375000F, 40.25195312500F, 
    40.44531250000F, 40.63867187500F, 40.83203125000F, 41.02539062500F, 
    41.21875000000F, 41.41210937500F, 41.60546875000F, 41.79882812500F, 
    41.99218750000F, 42.18554687500F, 42.37890625000F, 42.57226562500F, 
    42.76562500000F, 42.95898437500F, 43.15234375000F, 43.34570312500F, 
    43.53906250000F, 43.73242187500F, 43.92578125000F, 44.11914062500F, 
    44.31250000000F, 44.50585937500F, 44.69921875000F, 44.89257812500F, 
    45.08593750000F, 45.27929687500F, 45.47265625000F, 45.66601562500F, 
    45.85937500000F, 46.05273437500F, 46.24609375000F, 46.43945312500F, 
    46.63281250000F, 46.82617187500F, 47.01953125000F, 47.21289062500F, 
    47.40625000000F, 47.59960937500F, 47.79296875000F, 47.98632812500F, 
    48.17968750000F, 48.37304687500F, 48.56640625000F, 48.75976562500F, 
    48.95312500000F, 49.14648437500F, 49.33984375000F, 49.53320312500F, 
    49.72656250000F, 49.91992187500F, 50.11328125000F, 50.30664062500F, 
    50.50000000000F, 50.69335937500F, 50.88671875000F, 51.08007812500F, 
    51.27343750000F, 51.46679687500F, 51.66015625000F, 51.85351562500F, 
    52.04687500000F, 52.24023437500F, 52.43359375000F, 52.62695312500F, 
    52.82031250000F, 53.01367187500F, 53.20703125000F, 53.40039062500F, 
    53.59375000000F, 53.78710937500F, 53.98046875000F, 54.17382812500F, 
    54.36718750000F, 54.56054687500F, 54.75390625000F, 54.94726562500F, 
    55.14062500000F, 55.33398437500F, 55.52734375000F, 55.72070312500F, 
    55.91406250000F, 56.10742187500F, 56.30078125000F, 56.49414062500F, 
    56.68750000000F, 56.88085937500F, 57.07421875000F, 57.26757812500F, 
    57.46093750000F, 57.65429687500F, 57.84765625000F, 58.04101562500F, 
    58.23437500000F, 58.42773437500F, 58.62109375000F, 58.81445312500F, 
    59.00781250000F, 59.20117187500F, 59.39453125000F, 59.58789062500F, 
    59.78125000000F, 59.97460937500F, 60.16796875000F, 60.36132812500F, 
    60.55468750000F, 60.74804687500F, 60.94140625000F, 61.13476562500F, 
    61.32812500000F, 61.52148437500F, 61.71484375000F, 61.90820312500F, 
    62.10156250000F, 62.29492187500F, 62.48828125000F, 62.68164062500F, 
    62.87500000000F, 63.06835937500F, 63.26171875000F, 63.45507812500F, 
    63.64843750000F, 63.84179687500F, 64.03515625000F, 64.22851562500F, 
    64.42187500000F, 64.61523437500F, 64.80859375000F, 65.00195312500F, 
    65.19531250000F, 65.38867187500F, 65.58203125000F, 65.77539062500F, 
    65.96875000000F, 66.16210937500F, 66.35546875000F, 66.54882812500F, 
    66.74218750000F, 66.93554687500F, 67.12890625000F, 67.32226562500F, 
    67.51562500000F, 67.70898437500F, 67.90234375000F, 68.09570312500F, 
    68.28906250000F, 68.48242187500F, 68.67578125000F, 68.86914062500F, 
    69.06250000000F, 69.25585937500F, 69.44921875000F, 69.64257812500F, 
    69.83593750000F, 70.02929687500F, 70.22265625000F, 70.41601562500F, 
    70.60937500000F, 70.80273437500F, 70.99609375000F, 71.18945312500F, 
    71.38281250000F, 71.57617187500F, 71.76953125000F, 71.96289062500F, 
    72.15625000000F, 72.34960937500F, 72.54296875000F, 72.73632812500F, 
    72.92968750000F, 73.12304687500F, 73.31640625000F, 73.50976562500F, 
    73.70312500000F, 73.89648437500F, 74.08984375000F, 74.28320312500F, 
    74.47656250000F, 74.66992187500F, 74.86328125000F, 75.05664062500F, 
    75.25000000000F, 75.44335937500F, 75.63671875000F, 75.83007812500F, 
    76.02343750000F, 76.21679687500F, 76.41015625000F, 76.60351562500F, 
    76.79687500000F, 76.99023437500F, 77.18359375000F, 77.37695312500F, 
    77.57031250000F, 77.76367187500F, 77.95703125000F, 78.15039062500F, 
    78.34375000000F, 78.53710937500F, 78.73046875000F, 78.92382812500F, 
    79.11718750000F, 79.31054687500F, 79.50390625000F, 79.69726562500F, 
    79.89062500000F, 80.08398437500F, 80.27734375000F, 80.47070312500F, 
    80.66406250000F, 80.85742187500F, 81.05078125000F, 81.24414062500F, 
    81.43750000000F, 81.63085937500F, 81.82421875000F, 82.01757812500F, 
    82.21093750000F, 82.40429687500F, 82.59765625000F, 82.79101562500F, 
    82.98437500000F, 83.17773437500F, 83.37109375000F, 83.56445312500F, 
    83.75781250000F, 83.95117187500F, 84.14453125000F, 84.33789062500F, 
    84.53125000000F, 84.72460937500F, 84.91796875000F, 85.11132812500F, 
    85.30468750000F, 85.49804687500F, 85.69140625000F, 85.88476562500F, 
    86.07812500000F, 86.27148437500F, 86.46484375000F, 86.65820312500F, 
    86.85156250000F, 87.04492187500F, 87.23828125000F, 87.43164062500F, 
    87.62500000000F, 87.81835937500F, 88.01171875000F, 88.20507812500F, 
    88.39843750000F, 88.59179687500F, 88.78515625000F, 88.97851562500F, 
    89.17187500000F, 89.36523437500F, 89.55859375000F, 89.75195312500F, 
    89.94531250000F, 90.13867187500F, 90.33203125000F, 90.52539062500F, 
    90.71875000000F, 90.91210937500F, 91.10546875000F, 91.29882812500F, 
    91.49218750000F, 91.68554687500F, 91.87890625000F, 92.07226562500F, 
    92.26562500000F, 92.45898437500F, 92.65234375000F, 92.84570312500F, 
    93.03906250000F, 93.23242187500F, 93.42578125000F, 93.61914062500F, 
    93.81250000000F, 94.00585937500F, 94.19921875000F, 94.39257812500F, 
    94.58593750000F, 94.77929687500F, 94.97265625000F, 95.16601562500F, 
    95.35937500000F, 95.55273437500F, 95.74609375000F, 95.93945312500F, 
    96.13281250000F, 96.32617187500F, 96.51953125000F, 96.71289062500F, 
    96.90625000000F, 97.09960937500F, 97.29296875000F, 97.48632812500F, 
    97.67968750000F, 97.87304687500F, 98.06640625000F, 98.25976562500F, 
    98.45312500000F, 98.64648437500F, 98.83984375000F, 99.03320312500F, 
    99.22656250000F, 99.41992187500F, 99.61328125000F, 99.80664062500F, 
}; 


// End of File
