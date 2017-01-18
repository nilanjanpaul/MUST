# Function File fSingleTone (TONE_1, _spb)
# Generates a single tone wavform of _spb samples.

function [x] = fSingleTone(TONE_1, _spb)

%_spb = 256;
X = zeros(1,_spb);
X(TONE_1+1) = 0.5;
x = ifft(X) * _spb;
