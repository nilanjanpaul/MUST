#Comment   #! /usr/bin/octave -qf


x = fDualTone(4,14, 256, 0.2);

for c = 1:length(x)
    printf("(%f,%f)",real( x(c)),  imag(x(c)) );
    if (c != length(x))
      printf(" | ");
    endif
end
