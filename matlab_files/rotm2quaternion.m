function quat = rotm2quaternion(rotm)

rotm
    q4 = 0.5*(1 + rotm(1,1)+ rotm(2,2) + rotm(3,3))^0.5;
    q1 = (rotm(2,3) - rotm(3,2))/(4*q4);
    q2 = (rotm(3,1) - rotm(1,3))/(4*q4);
    q3 = (rotm(1,2) - rotm(2,1))/(4*q4);
    
    quat=[q1;q2;q3;q4];
  
end