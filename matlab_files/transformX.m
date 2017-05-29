function X = transformX(T)

  P = T(1:3,4);
  Px = vec2skew(P);
  R = T(1:3,1:3);
  
  X = [R Px*R; zeros(3) R];
  
end