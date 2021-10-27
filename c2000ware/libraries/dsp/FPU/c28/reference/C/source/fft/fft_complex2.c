//#############################################################################
//! \file   reference/C/source/$FILENAME$
//! \brief  Complex Fast Fourier Transform (Radix 2)
//! \author Vishal Coelho 
//! \date   22-Oct-2015
//! 
//
//  Group:            C2000
//  Target Family:    $DEVICE$
//
//#############################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//#############################################################################


//*****************************************************************************
// the includes
//*****************************************************************************
#include "fft.h"
#include <math.h>

//*****************************************************************************
// defines
//*****************************************************************************
DSP_FILENUM(24)

//*****************************************************************************
// FFT_runCFFT2()
//*****************************************************************************
void FFT_runCFFT2(FFTHandle hnd)
{
    // Locals
    uint16_t i, k, p, r;
    uint16_t p_top, p_bot, p_sin, p_cos, in_sep;
    
    uint16_t curr_stage, n_groups, n_bflys, bfly;
    fsize_t Ar, Ai, Br, Bi, Cr, Ci, Dr, Di, S, C;
    fsize_t *tf          = FFT_getTwiddlePtr(hnd);
    fsize_t *io          = FFT_getIOBufferPtr(hnd);
    uint16_t N           = FFT_getSize(hnd);
    uint16_t n_stages    = log2(N);
    uint16_t tblsize     = FFT_getTfTblSize(hnd);
    uint16_t tf_sep      = tblsize/4U;
    uint16_t skipfactor  = tblsize/N; //The max skipfactor for the last stage
    FFT_setNStages(hnd, n_stages);
    FFT_setSkipFactor(hnd, skipfactor);
    
    //-------------------------------------------------------------------------
    // Stage 1
    //-------------------------------------------------------------------------
    curr_stage  = 1U;
    n_groups    = N >> curr_stage;
    n_bflys     = (N >> 1U)/n_groups;
    in_sep      = (1U << (curr_stage - 1U)) * 2U;
    p_top       = 0U;
    p_bot       = p_top + in_sep;
    bfly        = 0U;
#ifdef _DEBUG
    printf("******************Stage #%4d******************\n", curr_stage);   
    printf("n_groups=%4d, n_bflys=%4d, in_sep=%4d \n", n_groups, n_bflys,
           in_sep);
    printf("***********************************************\n");
#endif    
    // Iterate through the groups
    for (i = 0U; i < n_groups; i++)
    {
        Ar          = io[p_top];
        Ai          = io[p_top+1];
        Br          = io[p_bot];
        Bi          = io[p_bot+1];
        Cr          = Ar+Br;
        Ci          = Ai+Bi;
        Dr          = Ar-Br;
        Di          = Ai-Bi;
        io[p_top]   = Cr;
        io[p_top+1] = Ci;
        io[p_bot]   = Dr;
        io[p_bot+1] = Di;
        bfly++;
        // setup p_top and p_bot for next group
        p_top = p_top + in_sep + 2U;
        p_bot = p_top + in_sep;
#ifdef _DEBUG
        printf("Bfly #%4d, Ar+jAi=%10.7f+j%10.7f, "
               "Br+jBi=%10.7f+j%10.7f, "
               "Cr+jCi=%10.7f+j%10.7f, "
               "Dr+jDi=%10.7f+j%10.7f\n",         
                bfly, Ar, Ai, Br, Bi, Cr, Ci, Dr, Di);
#endif
    }
    
    //-------------------------------------------------------------------------
    // Stage 2
    //-------------------------------------------------------------------------
    curr_stage  = 2U;
    n_groups    = N >> curr_stage;
    n_bflys     = (N >> 1U)/n_groups;
    in_sep      = (1U << (curr_stage - 1U)) * 2U;
    p_top       = 0U;
    p_bot       = p_top + in_sep;
    bfly        = 0U;
#ifdef _DEBUG
    printf("******************Stage #%4d******************\n", curr_stage);   
    printf("n_groups=%4d, n_bflys=%4d, in_sep=%4d \n", n_groups, n_bflys,
           in_sep);
    printf("***********************************************\n");
#endif 
    // Iterate through the groups
    for (i = 0U; i < n_groups; i++)
    {
        //first bfly
        Ar          = io[p_top];
        Ai          = io[p_top+1];
        Br          = io[p_bot];
        Bi          = io[p_bot+1];
        Cr          = Ar+Br;
        Ci          = Ai+Bi;
        Dr          = Ar-Br;
        Di          = Ai-Bi;
        io[p_top]   = Cr;
        io[p_top+1] = Ci;
        io[p_bot]   = Dr;
        io[p_bot+1] = Di;
        bfly++;
#ifdef _DEBUG
        printf("Bfly #%4d, Ar+jAi=%10.7f+j%10.7f, "
               "Br+jBi=%10.7f+j%10.7f, "
               "Cr+jCi=%10.7f+j%10.7f, "
               "Dr+jDi=%10.7f+j%10.7f\n",         
                bfly, Ar, Ai, Br, Bi, Cr, Ci, Dr, Di);
#endif
        // second bfly
        p_top       = p_top + 2U;
        p_bot       = p_top + in_sep;
        Ar          = io[p_top];
        Ai          = io[p_top+1];
        Br          = io[p_bot];
        Bi          = io[p_bot+1];
        Cr          = Ar+Bi;
        Ci          = Ai-Br;
        Dr          = Ar-Bi;
        Di          = Ai+Br;
        io[p_top]   = Cr;
        io[p_top+1] = Ci;
        io[p_bot]   = Dr;
        io[p_bot+1] = Di;
        bfly++;
        // setup p_top and p_bot for next group
        p_top = p_top + in_sep + 2U;
        p_bot = p_top + in_sep;
#ifdef _DEBUG
        printf("Bfly #%4d, Ar+jAi=%10.7f+j%10.7f, "
               "Br+jBi=%10.7f+j%10.7f, "
               "Cr+jCi=%10.7f+j%10.7f, "
               "Dr+jDi=%10.7f+j%10.7f\n",         
                bfly, Ar, Ai, Br, Bi, Cr, Ci, Dr, Di);
#endif
    }
    //-------------------------------------------------------------------------
    // Stage 3 to n_stages
    //-------------------------------------------------------------------------
    for (k = 3U; k <= n_stages; k++)
    {
        curr_stage  = curr_stage + 1U;
        n_groups    = N >> curr_stage;
        n_bflys     = (N >> 1U)/n_groups;
        in_sep      = (1U << (curr_stage - 1U)) * 2U;
        skipfactor  = tblsize/(1U << curr_stage);
        // Setup data and twiddle pointers
        p_top       = 0U;
        p_bot       = p_top + in_sep;
        p_sin       = 0U;
        p_cos       = p_sin + tf_sep;
        bfly        = 0U;
#ifdef _DEBUG
        printf("******************Stage #%4d******************\n", curr_stage);   
        printf("n_groups=%4d, n_bflys=%4d, in_sep=%4d \n", n_groups, n_bflys,
               in_sep);
        printf("***********************************************\n");
#endif 
        // Iterate through the groups
        for (p = 0U; p < n_groups; p++)
        {
            // iterate through the butterflies
            for (r = 0U; r < n_bflys; r++)
            {
                Ar    = io[p_top];
                Ai    = io[p_top+1];
                Br    = io[p_bot];
                Bi    = io[p_bot+1];
                S     = -tf[p_sin];
                C     =  tf[p_cos];
                DSP_ASSERT(((C*C+S*S)-1.0) < 1e-14);
                Cr    = Ar+C*Br-S*Bi;
                Ci    = Ai+C*Bi+S*Br;
                Dr    = Ar-C*Br+S*Bi;
                Di    = Ai-C*Bi-S*Br;
                io[p_top]   = Cr;
                io[p_top+1] = Ci;
                io[p_bot]   = Dr;
                io[p_bot+1] = Di;
                bfly++;
                // setup p_top and p_bot for next butterfly
                p_top = p_top + 2U;
                p_bot = p_top + in_sep;
                // setup twiddle pointer for next butterfly
                p_sin = p_sin + skipfactor;
                p_cos = p_sin + tf_sep;
#ifdef _DEBUG
                printf("Bfly #%4d, Ar+jAi=%10.7f+j%10.7f, "
                       "Br+jBi=%10.7f+j%10.7f, "
                       "C+jS=%10.7f+j%10.7f, "
                       "Cr+jCi=%10.7f+j%10.7f, "
                       "Dr+jDi=%10.7f+j%10.7f\n",         
                        bfly, Ar, Ai, Br, Bi, C, S, Cr, Ci, Dr, Di);
#endif
            }
            // setup p_top and p_bot for next group
            p_top = p_top + in_sep;
            p_bot = p_top + in_sep;
            // reset twiddle pointers for next group
            p_sin = 0U;
            p_cos = p_sin + tf_sep;
        }
    }
}

// End of File
