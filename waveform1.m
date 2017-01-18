x = fSingleTone(96, 256);

for c = 1:length(x)
    printf("(%f,%f)",real( x(c)),  imag(x(c)) );
    if (c != length(x))
      printf(" | ");
    endif
end
