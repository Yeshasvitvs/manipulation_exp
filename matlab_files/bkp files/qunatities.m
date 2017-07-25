close all;
clear all;
clc;

%%Computing quantities with respect to gazebo world

%%Rigid Body Inertias
%Body1
m1 = 1; %Kgs
I1_c = eye(3,3); %Inertia at CoM
c1 = [0 -0.0125 0.55]; %CoM
c1x = vec2skew(c1); % Cx

I1 = [I1_c+m1*c1x*(c1x') m1*c1x;
      m1*(c1x') m1*eye(3,3)];

%Body2
m2 = 1; %Kgs
I2_c = eye(3,3); %Inertia at CoM
c2 = [-0.1305 0.0425 0.55]; %CoM Half way from center to second handle link
c2x = vec2skew(c2); % Cx

I2 = [I2_c+m2*c2x*(c2x') m2*c2x;
      m2*(c2x') m2*eye(3,3)];
  
camera_rot = pi/2;
camera_height = 1.2;
world_R_camera = [cos(camera_rot), 0, sin(camera_rot);
                  0,               1, 0;
                  -sin(camera_rot), 0, cos(camera_rot)];
world_P_camera = [0, 0, camera_height];
world_T_camera = [world_R_camera, world_P_camera'; 0, 0, 0, 1];

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/wall_fb_still_180.txt';
data = importdata(filename);

time = data(2:end,1);

%External Forces
left_wrench = data(:,17:22);
right_wrench = data(:,23:28);

total_wrench = left_wrench + right_wrench;

%%Building Homogeneous transformation
right_pose = data(:,3:8);
left_pose = data(:,11:16);

lin_left = left_pose(:,1:3);
ang_left = left_pose(:,4:6);

for i = 1:1:size(ang_left,1)
    %Get  Homogeneous transformation
    RotM_left(i).R = ang2rotm(ang_left(i,:));
    camera_T_left(i).H = [RotM_left(i).R,lin_left(i,:)'];
    camera_T_left(i).H = [camera_T_left(i).H; 0 0 0 1];
    world_T_left(i).H = world_T_camera*camera_T_left(i).H;
end


lin_right = right_pose(:,1:3);
ang_right = right_pose(:,4:6);

for i = 1:1:size(ang_right,1)
    %Get  Homogeneous transformation
    RotM_right(i).R = ang2rotm(ang_right(i,:));
    camera_T_right(i).H = [RotM_right(i).R,lin_right(i,:)'];
    camera_T_right(i).H = [camera_T_right(i).H; 0 0 0 1];
    world_T_right(i).H = world_T_camera*camera_T_right(i).H;
end

%%Computing relative transformation 1_T_2
for i = 1:1:size(world_T_left,2)
    T_12(i).H = inv(world_T_left(i).H).*(world_T_right(i).H);
    RotM12(i).R = T_12(i).H(1:3,1:3);
    lin12(i,:) = (T_12(i).H(1:3,4))';
end

%%Computing axis angle vector for revolute joint
%%First three elements axis of rotation, last element angle of rotation -
%%This gives the properties of the revolute joint
%%TODO Check this with still data which should give constant axis angle
%%The joint variable values are off - for 90 I measure 130, for distance
%%0.3m I measure 0.7m
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

%%Left Linear velocity
dlin1 = diff(lin_left);
lin_v1=zeros(size(dlin1));
ang_v1=zeros(size(dlin1));

for i=1:1:size(dlin1,2)
    lin_v1(:,i) = dlin1(:,i)./time;
end

%%Angular velocity
for i = 1:1:size(RotM_left,2)-1
    dRotM1(i).R = RotM_left(i+1).R-RotM_left(i).R;
    S_w1(i).m = (dRotM1(i).R).*(RotM_left(i).R');
    ang_v1(i,:) = skew2vec(S_w1(i).m);
end

%%Right Linear velocity
dlin2 = diff(lin_right);
lin_v2 = zeros(size(dlin2));
ang_v2 = zeros(size(dlin2));

for i=1:1:size(dlin2,2)
    lin_v2(:,i) = dlin2(:,i)./time;
end

%%Angular velocity
for i = 1:1:size(RotM_right,2)-1
    dRotM2(i).R = RotM_right(i+1).R-RotM_right(i).R;
    S_w2(i).m = (dRotM2(i).R).*(RotM_right(1).R');
    ang_v2(i,:) = skew2vec(S_w2(i).m);
end

%6D Vectors
V1_mes = ([ang_v1'; lin_v1'])';

V2_mes = ([ang_v2'; lin_v2'])' %This is measured