function rotm = ang2rotm(ang) %%Not sure which rotation this angle values follow - it is XYZ
%%Check which rotation has to be used
s1 = sin(ang(1)); c1 = cos(ang(1)); %X
s2 = sin(ang(2)); c2 = cos(ang(2)); %Y
s3 = sin(ang(3)); c3 = cos(ang(3)); %Z



rz = [  c3,  s3,  0; ... 
       -s3,  c3,  0; ... 
         0,   0,  1]; 
ry = [  c2,  0,  s2; ... 
         0,  1,  0; ... 
       -s2,  0,  c2]; 
rx = [  1,   0,   0; ... 
        0,   c1, s1;... 
        0,  -s1, c1]; 
rotm = rx*ry*rz;






end