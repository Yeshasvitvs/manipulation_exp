function skew=vec2skew(vec)

  skew(1,1) = 0;
  skew(1,2) = -vec(3);
  skew(1,3) = vec(2);
    
  skew(2,1) = vec(3);
  skew(2,2) = 0;
  skew(2,3) = -vec(1);
    
  skew(3,1) = -vec(2);
  skew(3,2) = vec(1);
  skew(3,3) = 0;
    
end