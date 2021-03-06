%//#############################################################################
%//! \file /2837x_RFFT/matlab/RFFTforC28x.m
%//!
%//! \brief  MATLAB code for the Real Fast Fourier Transform
%//! \author C2000
%//
%//  Group:             C2000
%//  Target Family:     x86
%//
%//#############################################################################
%// $TI Release: $
%// $Release Date: $
%// $Copyright: $
%//#############################################################################

clear all
close all

disp('this is Matlab version of C28x RFFT code');
disp('The result is stored in array X');
disp('The signal is stored in array Rx');

RadStep = 0.1963495408494;

stages=input('Please enter stages (>=3) =');
N=2^stages;
halfSize=N-1;

Rad=[0:RadStep:(halfSize)*RadStep];

Rx=sin(Rad)+cos(Rad*2.3567);

X=fft(Rx);
figure,plot(real(X(1: (length(X)/2) )));
grid on;


% End of file
