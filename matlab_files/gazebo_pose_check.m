close all;
clear all;
clc;

filename = '/home/yeshi/projects/manipulation_exp/data/motion_rp.txt';
pose_data = importdata(filename);
% pose_data = pose_data.data(1:1000);

left_pose = pose_data(:,1:7);
right_pose = pose_data(:,8:end);

lin_left = left_pose(:,1:3);
ang_left = left_pose(:,4:7);

for i = 1:1:size(ang_left,1)
    %Get  Homogeneous transformation
    RotM_left(i).R = quaternion2rotm(ang_left(i,:));
    world_T_left(i).H = [RotM_left(i).R,lin_left(i,:)'];
    world_T_left(i).H = [world_T_left(i).H; 0 0 0 1];
end


lin_right = right_pose(:,1:3);
ang_right = right_pose(:,4:7);

for i = 1:1:size(ang_right,1)
    %Get  Homogeneous transformation
    RotM_right(i).R = quaternion2rotm(ang_right(i,:));
    world_T_right(i).H = [RotM_right(i).R,lin_right(i,:)'];
    world_T_right(i).H = [world_T_right(i).H; 0 0 0 1];
end


%%Computing relative transformation 1_T_2
for i = 1:1:size(world_T_right,2)
    T_12(i).H = inv(world_T_left(i).H)*(world_T_right(i).H);
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
%     axisAngleVector12(i,:) = (vrrotmat2vec(RotM12(i).R))';
    euler(i,:) = rotm2euler(RotM12(i).R);
    distance = norm(lin12(i,:)');
    axisDistanceVector12(i,:) = [lin12(1,:)/distance, distance];
end

figure(1)
title('Revolute Joint Variable');
hold on;
plot(euler(:,1));
hold on;
plot(euler(:,2));
hold on;
plot(euler(:,3));
legend('\theta_{x}','\theta_{y}','\theta_{z}')

figure(2)
title('Prismatic Joint Variable');
hold on;
plot(axisDistanceVector12(:,4))