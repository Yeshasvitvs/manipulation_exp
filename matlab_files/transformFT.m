function X = transformFT(T,offset)

  P = T(1:3,4) + offset;
  R = T(1:3,1:3);
 
  newP = R'*P;
  Px = vec2skew(newP);
  
  X = [R Px*R; zeros(3) R];
  
end