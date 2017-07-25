% % l = 0.049;
% % b = 0.295;
% % h = 0.049;

% % l = 0.199;
% % b = 0.049;
% % h = 0.049;
% % m = 8;
% % 
% % ixx = (1/12)*m*(b^2 + h^2);
% % iyy = (1/12)*m*(l^2 + h^2);
% % izz = (1/12)*m*(l^2 + b^2);
% % 
% % sprintf('%.16f',ixx)
% % sprintf('%.16f',iyy)
% % sprintf('%.16f',izz)


r = 0.009;
h = 0.095;
m = 5;

ixx = (1/12)*m*(3*r*r + h*h);
iyy = (1/12)*m*(3*r*r + h*h);
izz = (1/2)*m*(r*r);
sprintf('%.16f',ixx)
sprintf('%.16f',iyy)
sprintf('%.16f',izz)

