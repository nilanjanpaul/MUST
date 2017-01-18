# Function File fDualTone (TONE_1, TONE_2, _spb)
# Generates a dual tone wavform of _spb samples.
# First _spb/2 have TONE_1 frequency followed by TONE_2 frequency

function [reference_] = fDualTone(TONE_1, TONE_2, _spb)

%_spb = 256
%TONE_1 = 4;
%TONE_2 = 14;

REFERENCE_ = zeros(1, _spb/2);
REFERENCE_( TONE_1/2 + 1) = .9;
ref4_ = (ifft(REFERENCE_)) * _spb/2;


REFERENCE_ = zeros(1,_spb/2);
REFERENCE_( TONE_2/2 + 1) = .9;
ref16_ = (ifft(REFERENCE_)) * _spb/2;

reference_ = [ref4_ ref16_];  % this is TD reference signal
%REFERENCE_ = fft(reference_, 1024);   % this is FD reference signal
