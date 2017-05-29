function M = spatialInertia(m,I,c)

  Sc = vec2skew(c);
  M = [m*eye(3,3) -m*Sc; m*Sc I-m*Sc*Sc];
  
end