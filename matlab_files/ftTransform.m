function wrench = ftTransform(H,w)
  
  E = H(1:3,1:3);
  r = vec2skew(H(1:3,4)); 
  input_wrench = w';
  
  transform = [E,zeros(3,3); zeros(3,3),E] * [eye(3,3),zeros(3,3); r',eye(3,3)];
  wrench = inv(transform)' * input_wrench;
end