function X = transformXinv(T)

  P = T(1:3,4);
  Px = vec2skew(P);
  R = T(1:3,1:3);
  
  X = [R' -R'*Px; zeros(3) R'];
  
% %   x = transformX(T)
% %   X = inv(x);

end