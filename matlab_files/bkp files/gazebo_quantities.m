close all;
clear all;
clc;

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/trajectories/paxis1.txt';
data = importdata(filename);

centerR=eye(3,3);
centerP=[0,0,0.55];
world_T_center=[centerR,centerP';0,0,0,1];

time = data(:,1); %%Time received in seconds
time = time - time(1,1); %%Corrected to zero

%%NOTE angles are in quaternions
left_pose = data(:,2:8);
right_pose = data(:,9:15);


left_wrench = data(:,16:21);
right_wrench = data(:,22:27);

dH_mes_local = left_wrench + right_wrench;
% % figure; plot(time,dH_mes_local); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Total External Wrench - Local Frame'); 

%%Computing quantities with respect to gazebo world

%%Rigid Body Inertias
%Body1
m1 = 1; %Kgs
I1_c = eye(3,3); %Inertia at CoM
% % c1 = [0 -0.0125 0.55]; %CoM
% % c1x = vec2skew(c1); % Cx
% % 
% % I1 = [m1*eye(3,3) m1*(c1x'); 
% %       m1*c1x I1_c+m1*c1x*(c1x');];

%Body2
m2 = 1; %Kgs
I2_c = eye(3,3); %Inertia at CoM
% % c2 = [-0.1305 0.0425 0.55]; %CoM Half way from center to second handle link
% % c2x = vec2skew(c2); % Cx
% % 
% % I2 = [m2*eye(3,3) m2*(c2x'); 
% %       m2*c2x I2_c+m2*c2x*(c2x');];

  
%%Pose Data  
lin_left = left_pose(:,1:3);
for i=1:1:size(lin_left,1)
    com_left(i,:) = lin_left(i,:)-[0,0,0.014];
end

ang_left = left_pose(:,4:7);

for i = 1:1:size(ang_left,1)
    %Get  Homogeneous transformation
    RotM_left(i).R = quaternion2rotm(ang_left(i,:));
    world_T_left(i).H = [RotM_left(i).R,lin_left(i,:)'];
    world_T_left(i).H = [world_T_left(i).H; 0 0 0 1];
    
    %%Left FT to world Homogeneous transformation
    left_ft_rot(i).R =  RotM_left(i).R;
    left_ft_pos(i).P = lin_left(i,:) + [0, 0.125, -0.028];
    world_T_left_ft(i).H = [left_ft_rot(i).R,left_ft_pos(i).P'; 0, 0, 0, 1];
    
    world_ft_left(i,:) = ftTransform(world_T_left_ft(i).H,left_wrench(i,:));
end


lin_right = right_pose(:,1:3);
ang_right = right_pose(:,4:7);

for i = 1:1:size(ang_right,1)
    %Get  Homogeneous transformation
    RotM_right(i).R = quaternion2rotm(ang_right(i,:));
    world_T_right(i).H = [RotM_right(i).R,lin_right(i,:)'];
    world_T_right(i).H = [world_T_right(i).H; 0 0 0 1];
    
    %%Right FT to world Homogeneous transformation
    right_ft_rot(i).R =  RotM_right(i).R;
    theta = (vrrotmat2vec(RotM_right(i).R))';
    if(theta(3) < 0)
        theta(4) = -theta(4);
    end
    right_ft_pos(i).P = lin_right(i,:) + [0.125*sin(theta(4)), 0.125*cos(theta(4)), -0.028];
    com_right(i,:) = lin_right(i,:)+[0.15*sin(theta(4)), 0.15*cos(theta(4)),0];
    world_T_right_ft(i).H = [right_ft_rot(i).R,right_ft_pos(i).P'; 0, 0, 0, 1];     
    world_ft_right(i,:) = ftTransform(world_T_right_ft(i).H,right_wrench(i,:));
end

%%Computing Body CoM
for i=1:1:size(com_left,1)
    body_com(i,:) = (m1*com_left(i,:)+m2*com_right(i,:))/(m1+m2);
    
    c1 = body_com(i,:)-com_left(i,:); %difference in CoM
    c1x = vec2skew(c1); % Cx

    I1(i).I = [m1*eye(3,3) m1*(c1x'); 
               m1*c1x I1_c+m1*c1x*(c1x');];
  
    c2 = body_com(i,:)-com_right(i,:); %difference in CoM
    c2x = vec2skew(c2); % Cx

    I2(i).I = [m2*eye(3,3) m2*(c2x'); 
      m1*c2x I2_c+m2*c2x*(c2x');];
end

%%Computing relative transformation 1_T_2
for i = 1:1:size(world_T_right,2)
    
    T_12(i).H = (world_T_left(i).H)\(world_T_right(i).H);

    RotM12(i).R = T_12(i).H(1:3,1:3);
    lin12(i,:) = (T_12(i).H(1:3,4));   
end

%%Computing axis angle vector for revolute joint
%%First three elements axis of rotation, last element angle of rotation -
%%This gives the properties of the revolute joint
%%TODO Check this with still data which should give constant axis angle
%%values
for i = 1:1:size(RotM12,2)
    axisAngleVector12(i,:) = (vrrotmat2vec(RotM12(i).R))';
%     euler(i,:) = rotm2euler(RotM12(i).R);
    
    distance = norm(lin12(i,:)');
    axisDistanceVector12(i,:) = [lin12(i,:)/distance, distance];
    
    jointDistance(i,:) = axisDistanceVector12(i,4); %%This is in meters
    jointAngle(i,:) = axisAngleVector12(i,4); %%This is in radians
     
    R_jointAxis(i,:) = [0,0,0,axisAngleVector12(i,1:3)]; %This is joint axis form angular part
    P_jointAxis(i,:) = [axisDistanceVector12(i,1:3),0,0,0];
    
end


dtime = diff(time);
%%Joint variable first derivative
dJointDistance = diff(jointDistance)./dtime;
dJointAngle = diff(jointAngle)./dtime;

ddtime = diff(time(2:end));
%%Joint variable second derivative
ddJointDistance = diff(dJointDistance)./ddtime;
ddJointAngle = diff(dJointAngle)./ddtime;

%%Left Linear velocity
lin_vleft(:,1) = diff(lin_left(:,1))./dtime;
lin_vleft(:,2) = diff(lin_left(:,2))./dtime;
lin_vleft(:,3) = diff(lin_left(:,3))./dtime;

%%Angular velocity
for i = 1:1:size(RotM_left,2)-1
    dRotM_left(i).R = RotM_left(i+1).R-RotM_left(i).R; %%Computing differences
    dRotM_left(i).R = dRotM_left(i).R/time(2,1); %%Component wise derivative
    S_w_left(i).m = (dRotM_left(i).R).*(RotM_left(i).R');
    ang_vleft(i,:) = skew2vec(S_w_left(i).m);
end

%%Left Linear Acceleration
lin_aleft(:,1) = diff(lin_vleft(:,1))./ddtime;
lin_aleft(:,2) = diff(lin_vleft(:,2))./ddtime;
lin_aleft(:,3) = diff(lin_vleft(:,3))./ddtime;

%%Left Angular Acceleration
ang_aleft(:,1) = diff(ang_vleft(:,1))./ddtime;
ang_aleft(:,2) = diff(ang_vleft(:,2))./ddtime;
ang_aleft(:,3) = diff(ang_vleft(:,3))./ddtime;

%6D Vectors
V_left_mes = ([lin_vleft'; ang_vleft'])';
A_left_mes = ([lin_aleft'; ang_aleft'])';

%%Right Linear velocity
lin_vright(:,1) = diff(lin_right(:,1))./dtime;
lin_vright(:,2) = diff(lin_right(:,2))./dtime;
lin_vright(:,3) = diff(lin_right(:,3))./dtime;
 
%%Angular velocity
for i = 1:1:size(RotM_right,2)-1
    dRotM_right(i).R = RotM_right(i+1).R-RotM_right(i).R; %%Computing differences
    dRotM_right(i).R = dRotM_right(i).R/time(2,1); %%Component wise derivative
    S_w_right(i).m = (dRotM_right(i).R).*(RotM_right(i).R');
    ang_vright(i,:) = skew2vec(S_w_right(i).m);
end

%%Right Linear Acceleration
lin_aright(:,1) = diff(lin_vright(:,1))./ddtime;
lin_aright(:,2) = diff(lin_vright(:,2))./ddtime;
lin_aright(:,3) = diff(lin_vright(:,3))./ddtime;

%%Right Angular Acceleration
ang_aright(:,1) = diff(ang_vright(:,1))./ddtime;
ang_aright(:,2) = diff(ang_vright(:,2))./ddtime;
ang_aright(:,3) = diff(ang_vright(:,3))./ddtime;

%6D Vectors
V_right_mes = ([lin_vright'; ang_vright'])'; %This is measured
A_right_mes = ([lin_aright'; ang_aright'])';

%%Computing velocities assuming a joint nature
for i=1:1:size(R_jointAxis,1)-1 
    R_V_right_computed(i,:) = V_left_mes(i,:) + R_jointAxis(i,:)*dJointAngle(i,1); %%Assuming Revolute joint
    P_V_right_computed(i,:) = V_left_mes(i,:) + P_jointAxis(i,:)*dJointDistance(i,1); %%Assuming Prismatic joint
    
    %%Computing s_dot = VxS - these values are near zeros
    R_dJointAxis(i,:) = crm(R_V_right_computed(i,:))*(R_jointAxis(i+1,:)'); %%Something fishy here
    P_dJointAxis(i,:) = crm(P_V_right_computed(i,:))*(P_jointAxis(i+1,:)');
end


%%Computing accelerations assuming a joint nature
for i=1:1:size(A_left_mes,1)
    R_A_right_computed(i,:) = A_left_mes(i,:) + R_dJointAxis(i+1,:)*dJointAngle(i,1) + R_jointAxis(i+1,:)*ddJointAngle(i,1);
    P_A_right_computed(i,:) = A_left_mes(i,:) + P_dJointAxis(i+1,:)*dJointDistance(i,1) + P_jointAxis(i+1,:)*ddJointDistance(i,1);
    
    R_dH(i,:) = I1(i+2).I*(A_left_mes(i,:)') + crf(V_left_mes(i,:))*I1(i+1).I*(V_left_mes(i,:)') + I2(i+2).I*(R_A_right_computed(i,:)') + crf(R_V_right_computed(i+1,:))*I2(i+1).I*(R_V_right_computed(i+1,:)');
    P_dH(i,:) = I1(i+2).I*(A_left_mes(i,:)') + crf(V_left_mes(i,:))*I1(i+1).I*(V_left_mes(i,:)') + I2(i+2).I*(P_A_right_computed(i,:)') + crf(P_V_right_computed(i+1,:))*I2(i+1).I*(P_V_right_computed(i+1,:)');
end

%%Adding padding zeros
dJointDistance = [0;dJointDistance];
dJointAngle = [0;dJointAngle];

ddJointDistance = [0;0;ddJointDistance];
ddJointAngle = [0;0;ddJointAngle];

V_right_mes = [0, 0, 0, 0, 0, 0; V_right_mes];
R_V_right_computed = [0, 0, 0, 0, 0, 0; R_V_right_computed];
P_V_right_computed = [0, 0, 0, 0, 0, 0; P_V_right_computed];

A_right_mes = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; A_right_mes];
R_A_right_computed = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; R_A_right_computed];
P_A_right_computed = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; P_A_right_computed];

R_dH = [R_dH; 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0;];
P_dH = [P_dH; 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0;];

%%Total wrench with respect to the world
dH_mes_abs = world_ft_left + world_ft_right;

%%Computing rate of change in momentum
R = R_dH - dH_mes_abs;
P = P_dH - dH_mes_abs;

%%Plotting Right CoM
figure;
subplot(2,1,1); plot(lin_right); legend('x','y','z');
subplot(2,1,2); plot(com_right); legend('x','y','z');

% % %%Plotting Joint Variables
% % figure; 
% % subplot(3,1,1); plot(time,jointAngle); title('Revolute Joint Angle');
% % subplot(3,1,2); plot(time,dJointAngle); title('Revolute Joint Vel');
% % subplot(3,1,3); plot(time,ddJointAngle); title('Revolute Joint Acc');
% % 
% % figure;
% % subplot(3,1,1); plot(time,jointDistance); title('Prismatic Joint Distance');
% % subplot(3,1,2); plot(time,dJointDistance); title('Prismatic Joint Vel');
% % subplot(3,1,3); plot(time,ddJointDistance); title('Prismatic Joint Acc');
% % 
% % figure;
% % subplot(2,1,1); plot(time,R_jointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Joint Axis');
% % subplot(2,1,2); plot(time,P_jointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Joint Axis');
% % 
% % R_dJointAxis = [0,0,0,0,0,0;R_dJointAxis];
% % P_dJointAxis = [0,0,0,0,0,0;P_dJointAxis];

% % figure;
% % subplot(2,1,1); plot(time,R_dJointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Axis Derivative');
% % subplot(2,1,2); plot(time,P_dJointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Axis Derivative')
% % 
% % %%Plotting Velocities
% % figure; 
% % subplot(3,1,1); plot(time,V_right_mes); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Vel_{right measured}');
% % subplot(3,1,2); plot(time,R_V_right_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Vel_{right computed}');
% % subplot(3,1,3); plot(time,P_V_right_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Vel_{right computed}');

% % % % %%Plotting Accelerations
% % figure;
% % subplot(3,1,1); plot(time,A_right_mes); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Acc_{right measured}');
% % subplot(3,1,2); plot(time,R_A_right_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Acc_{right computed}');
% % subplot(3,1,3); plot(time,P_A_right_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Acc_{right computed}');

% % %%Plotting External Wrench - Local Frames 
% % figure; 
% % subplot(3,1,1); plot(time,left_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Left External Wrench_{local}');
% % subplot(3,1,2); plot(time,right_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Right External Wrench_{local}');
% % subplot(3,1,3); plot(time,dH_mes_local); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Total External Wrench_{local}');

% % %Plotting External Wrench - wrt Inertial Frame
% % figure; 
% % subplot(3,1,1); plot(time,world_ft_left); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Left External Wrench_{world}');
% % subplot(3,1,2); plot(time,world_ft_right); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Right External Wrench_{world}');
% % subplot(3,1,3); plot(time,dH_mes_abs); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Total External Wrench_{world}');
% % 
% % %Plotting Joint Hypothesis RoM
% % figure; 
% % subplot(2,1,1); plot(time,R_dH); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Hypothesis - Rate of Change in Momentum');
% % subplot(2,1,2); plot(time,P_dH); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Hypothesis - Rate of Change in Momentum');
% % 
% % % % %%Plotting Joint Hypothesis
% % figure;
% % subplot(2,1,1); plot(time,R); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Hypothesis');
% % subplot(2,1,2); plot(time,P); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Hypothesis');
