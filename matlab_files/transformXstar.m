function X = transformXstar(T)

  P = T(1:3,4);
  Px = vec2skew(P);
  R = T(1:3,1:3);
  
  X = [R zeros(3);Px*R R];
  
end