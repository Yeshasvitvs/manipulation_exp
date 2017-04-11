function rotm = quaternion2rotm(q)
    
    x = q(1);
    y = q(2);
    z = q(3);
    w = q(4);
    
    m11 = 1-2*(y^2)-2*(z^2);
    m12 = 2*x*y+2*w*z;
    m13 = 2*x*z-2*w*y;
    
    m21 = 2*x*y-2*w*z;
    m22 = 1-2*(x^2)-2*(z^2);
    m23 = 2*y*z+2*w*x;
    
    m31 = 2*x*z+2*w*y;
    m32 = 2*y*z-2*w*x;
    m33 = 1-2*(x^2)-2*(y^2);
    
    rotm = [m11, m12, m13;
            m21, m22, m23;
            m31, m32, m33];
end