function R = rvec_to_R(rvec)

  theta = norm(rvec);
  rvec = rvec/theta;
  rvec_skew = [0, -rvec(1,3), rvec(1,2);
               rvec(1,3), 0, -rvec(1,1);
              -rvec(1,2), rvec(1,1), 0];
  
  R = eye(3,3) + rvec_skew*sin(theta) + (rvec_skew*rvec_skew)*(1-cos(theta));

end