function X = transformFT(T,offset)

  P = T(1:3,4) + offset;
  R = T(1:3,1:3);
 
  newP = R'*P;

  newT = [R,newP;0 0 0 1];
  X = transformXstar(newT);
  
end