function M = spatialInertia(m,I,c)

  Sc = vec2skew(c);
  
  sc_check_direct = Sc*Sc;
  sc_check_exp = [-c(3)^2-c(2)^2 c(2)*c(1) c(3)*c(1);
                  c(2)*c(1) -c(3)^2-c(1)^2 c(2)*c(3);
                  c(3)*c(1) c(3)*c(2) -c(2)^2-c(1)^2];
              
  M = [m*eye(3,3) -m*Sc; m*Sc I-m*Sc*Sc];
  
end