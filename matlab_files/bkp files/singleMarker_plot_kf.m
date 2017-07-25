close all;
clear all;
clc;

camera_rot1 = -pi/2;
camera_rot2 = pi;

camera_height = 0.4;
R_z = [cos(camera_rot1) -sin(camera_rot1) 0;
       sin(camera_rot1)  cos(camera_rot1) 0;
       0                 0                1;];
R_y = [cos(camera_rot2), 0, sin(camera_rot2);
       0,               1,  0;
      -sin(camera_rot2), 0, cos(camera_rot2)];
              
world_R_camera = R_y*R_z;
world_P_camera = [0, 0, camera_height];
world_T_camera = [world_R_camera, world_P_camera'; 0, 0, 0, 1];

filename = '/home/yeshi/projects/aruco_markers/data/kf_zturn.txt';
data = importdata(filename);

camera_point = [0.9;0;0;1];

world_T_camera_point = world_T_camera*camera_point;

marker_point = [0;0;0;1];

figure(3)
grid on;
% view([90,90,90])
view(-61,38);
axis([-2 2 -2 2 -2 2]);
xL = xlim; 
yL = ylim;
zL = zlim;
line([0 1], [0 0], [0 0],'color','r','linewidth',1,'LineStyle','--') %x-axis 
line([0 0], [0 1], [0 0],'color','g','linewidth',1,'LineStyle','--') %y-axis
line([0 0], [0 0], [0 1],'color','b','linewidth',1,'LineStyle','--') %z-axis
xlabel('x');
ylabel('y');
zlabel('z');
hold on;

x(1) = world_P_camera(1,1);
y(1) = world_P_camera(1,2);
z(1) = world_P_camera(1,3);
ox(1) = x(1);
oy(1) = y(1);
oz(1) = z(1);
ux(1) = world_R_camera(1,1);
vx(1) = world_R_camera(2,1);
wx(1) = world_R_camera(3,1);
uy(1) = world_R_camera(1,2);
vy(1) = world_R_camera(2,2);
wy(1) = world_R_camera(3,2);
uz(1) = world_R_camera(1,3);
vz(1) = world_R_camera(2,3);
wz(1) = world_R_camera(3,3);

orgHandle = plot3(x, y, z, 'k.');
ShowArrowHeadStr = 'on';

quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandle = quiver3(ox, ox, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);


%Correct till the above part 

%Actual Measurements
lin = data(:,2:4);
ang = data(:,5:7);


%Get  Homogeneous transformation
for i = 1:1:size(ang,1)
%     axis_ang(i,:)=[ang(i,:),norm(ang(i,1:3))];
%     RotM(i).R = ang2rotm(ang(i,:));
    RotM(i).R = rvec_to_R(ang(i,:));
    camera_T_marker(i).H = [RotM(i).R,lin(i,:)'];
    camera_T_marker(i).H = [camera_T_marker(i).H; 0 0 0 1];
    marker_T_camera(i).H = [RotM(i).R.', -RotM(i).R.'*lin(i,:)';0 0 0 1];
    eye_check = marker_T_camera(i).H*camera_T_marker(i).H;
    %camera_T_marker(i).H = world_T_camera*inv(camera_T_marker(i).H);
    world_T_marker(i).H = world_T_camera*camera_T_marker(i).H;
end

world_T_marker_point = world_T_camera*camera_T_marker(1).H*marker_point;


for i = 1:1:size(ang,1)
    
   %camera_T_marker(i).H = marker_T_camera(i).H;
    R = world_T_marker(i).H(1:3,1:3);
    P = world_T_marker(i).H(1:3,4)';

    ShowArrowHeadStr = 'off';
    
    x(1) = P(1,1);
    y(1) = P(1,2);
    z(1) = P(1,3);
    ox(1) = x(1);
    oy(1) = y(1);
    oz(1) = z(1);
    ux(1) = R(1,1);
    vx(1) = R(2,1);
    wx(1) = R(3,1);
    uy(1) = R(1,2);
    vy(1) = R(2,2);
    wy(1) = R(3,2);
    uz(1) = R(1,3);
    vz(1) = R(2,3);
    wz(1) = R(3,3);

    orgHandle = plot3(x, y, z, 'k.');

    quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle = quiver3(ox, ox, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

    set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
    set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
    set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
    set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
%     pause(0.1)
end

figure(4)
grid on;
% view([90,90,90])
view(-61,38);
axis([-2 2 -2 2 -2 2]);
xL = xlim; 
yL = ylim;
zL = zlim;
line([0 1], [0 0], [0 0],'color','r','linewidth',1,'LineStyle','--') %x-axis 
line([0 0], [0 1], [0 0],'color','g','linewidth',1,'LineStyle','--') %y-axis
line([0 0], [0 0], [0 1],'color','b','linewidth',1,'LineStyle','--') %z-axis
xlabel('x');
ylabel('y');
zlabel('z');
hold on;

x(1) = world_P_camera(1,1);
y(1) = world_P_camera(1,2);
z(1) = world_P_camera(1,3);
ox(1) = x(1);
oy(1) = y(1);
oz(1) = z(1);
ux(1) = world_R_camera(1,1);
vx(1) = world_R_camera(2,1);
wx(1) = world_R_camera(3,1);
uy(1) = world_R_camera(1,2);
vy(1) = world_R_camera(2,2);
wy(1) = world_R_camera(3,2);
uz(1) = world_R_camera(1,3);
vz(1) = world_R_camera(2,3);
wz(1) = world_R_camera(3,3);

orgHandle = plot3(x, y, z, 'k.');
ShowArrowHeadStr = 'on';

quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandle = quiver3(ox, ox, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
%Kalman filter estimates
lin_kf = data(:,9:11);
ang_kf = data(:,12:14);


%Get  Homogeneous transformation
for i = 1:1:size(ang_kf,1)
%     axis_ang(i,:)=[ang(i,:),norm(ang(i,1:3))];
%     RotM(i).R = ang2rotm(ang(i,:));
    RotM_kf(i).R = rvec_to_R(ang_kf(i,:));
    camera_T_marker_kf(i).H = [RotM_kf(i).R,lin_kf(i,:)'];
    camera_T_marker_kf(i).H = [camera_T_marker_kf(i).H; 0 0 0 1];
    marker_T_camera_kf(i).H = [RotM_kf(i).R.', -RotM_kf(i).R.'*lin_kf(i,:)';0 0 0 1];
    eye_check = marker_T_camera_kf(i).H*camera_T_marker_kf(i).H;
    %camera_T_marker(i).H = world_T_camera*inv(camera_T_marker(i).H);
    world_T_marker_kf(i).H = world_T_camera*camera_T_marker_kf(i).H;
end


for i = 1:1:size(ang_kf,1)
    
   %camera_T_marker(i).H = marker_T_camera(i).H;
    R = world_T_marker_kf(i).H(1:3,1:3);
    P = world_T_marker_kf(i).H(1:3,4)';

    ShowArrowHeadStr = 'off';
    
    x(1) = P(1,1);
    y(1) = P(1,2);
    z(1) = P(1,3);
    ox(1) = x(1);
    oy(1) = y(1);
    oz(1) = z(1);
    ux(1) = R(1,1);
    vx(1) = R(2,1);
    wx(1) = R(3,1);
    uy(1) = R(1,2);
    vy(1) = R(2,2);
    wy(1) = R(3,2);
    uz(1) = R(1,3);
    vz(1) = R(2,3);
    wz(1) = R(3,3);

    orgHandle = plot3(x, y, z, 'k.');

    quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle = quiver3(ox, ox, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

    set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
    set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
    set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
    set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);
%     pause(0.1)
end
