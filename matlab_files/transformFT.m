function X = transformFT(T,offset)

  T_ft_offset = [eye(3,3),offset;0 0 0 1];
  newT = T*T_ft_offset;
  newT(2,4) = -newT(2,4);
  X = transformXstar(newT);
  
end