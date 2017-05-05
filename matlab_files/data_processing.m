close all;
clear all;
clc;

camera_rot = pi/2;
camera_height = 1.2;
world_R_camera = [cos(camera_rot), 0, sin(camera_rot);
                  0,               1, 0;
                  -sin(camera_rot), 0, cos(camera_rot)];
world_P_camera = [0, 0, camera_height];
world_T_camera = [world_R_camera, world_P_camera'; 0, 0, 0, 1];

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/wall_fb_still.txt';
data = importdata(filename);

time = data(2:end,1);

right_pose = data(:,3:8);
left_pose = data(:,11:16);

left_wrench = data(:,17:22);
right_wrench = data(:,23:28);

%%Building Homogeneous transformation
lin1 = left_pose(:,1:3);
ang1 = left_pose(:,4:6);

for i = 1:1:size(ang1,1)
    %Get  Homogeneous transformation
    RotM1(i).R = ang2rotm(ang1(i,:));
    camera_T_1(i).H = [RotM1(i).R,lin1(i,:)'];
    camera_T_1(i).H = [camera_T_1(i).H; 0 0 0 1];
    world_T_1(i).H = world_T_camera*camera_T_1(i).H;
end


lin2 = right_pose(:,1:3);
ang2 = right_pose(:,4:6);

for i = 1:1:size(ang2,1)
    %Get  Homogeneous transformation
    RotM2(i).R = ang2rotm(ang2(i,:));
    camera_T_2(i).H = [RotM2(i).R,lin2(i,:)'];
    camera_T_2(i).H = [camera_T_2(i).H; 0 0 0 1];
    world_T_2(i).H = world_T_camera*camera_T_2(i).H;
end

%%Computing relative transformation 1_T_2
for i = 1:1:size(world_T_1,2)
    T_12(i).H = inv(world_T_1(i).H).*(world_T_2(i).H);
    RotM12(i).R = T_12(i).H(1:3,1:3);
    lin12(i,:) = (T_12(i).H(1:3,4))';
end

%%Computing axis angle vector for revolute joint
%%First three elements axis of rotation, last element angle of rotation -
%%This gives the properties of the revolute joint
%%TODO Check this with still data which should give constant axis angle
%%values
for i = 1:1:size(RotM12,2)
    axisAngleVector12(i,:) = (vrrotmat2vec(RotM12(i).R))';
    distance = norm(lin12(i,:)');
    axisDistanceVector12(i,:) = [lin12(1,:)/distance, distance];
end

% % figure(1)
% % title('Revolute Joint Variable');
% % hold on;
% % plot(axisAngleVector12(:,4));
% % 
% % figure(2)
% % title('Prismatic Joint Variable');
% % hold on;
% % plot(axisDistanceVector12(:,4))




%%In the data marker 4 is given first and then marker 5
%%Linear velocity
dlin1 = diff(lin1);
lin_v1=zeros(size(dlin1));
ang_v1=zeros(size(dlin1));

for i=1:1:size(dlin1,2)
    lin_v1(:,i) = dlin1(:,i)./time;
end

%%Angular velocity
for i = 1:1:size(RotM1,2)-1
    dRotM1(i).R = RotM1(i+1).R-RotM1(i).R;
    S_w1(i).m = (dRotM1(i).R).*(RotM1(1).R');
    ang_v1(i,:) = skew2vec(S_w1(i).m);
end

%%Right link
dlin2 = diff(lin2);
lin_v2 = zeros(size(dlin2));
ang_v2 = zeros(size(dlin2));

for i=1:1:size(dlin2,2)
    lin_v2(:,i) = dlin2(:,i)./time;
end

%%Angular velocity
for i = 1:1:size(RotM2,2)-1
    dRotM2(i).R = RotM2(i+1).R-RotM2(i).R;
    S_w2(i).m = (dRotM2(i).R).*(RotM2(1).R');
    ang_v2(i,:) = skew2vec(S_w2(i).m);
end