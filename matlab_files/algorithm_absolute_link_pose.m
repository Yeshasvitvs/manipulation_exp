close all;
clear all;
clc;

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/axis/onlyp_str1.txt';
data = importdata(filename);
% data=data1(40:150,:);

time = data(:,1); %%Time received in seconds
time = time - time(1,1); %%Corrected to zero

%%NOTE angles are in quaternions
body1_pose = data(:,2:8);
body2_pose = data(:,9:15);

%%Interaction Wrench - Measured in local frame
left_wrench = data(:,16:21); %%FT at Body1
right_wrench = data(:,22:27); %%FT at Body2

measured_wrench_local = left_wrench + right_wrench;
% % figure; plot(time,measured_wrench_local); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Total External Wrench - Local Frame'); 

%%Inertial frame is gazebo world
%%Rigid Body Properties
%Body1
m1 = 1; %Kgs
I1_com = eye(3,3); %Inertia at CoM

%Body2
m2 = 1; %Kgs
I2_com = eye(3,3); %Inertia at CoM

  
%%Pose Data  
lin_body1 = body1_pose(:,1:3);
for i=1:1:size(lin_body1,1)
    com_body1(i,:) = lin_body1(i,:)+[0,0.15,0];
end

ang_body1 = body1_pose(:,4:7);

for i = 1:1:size(ang_body1,1)
    %Get  Homogeneous transformation
    RotM_body1(i).R = quaternion2rotm(ang_body1(i,:));
    world_T_body1(i).H = [RotM_body1(i).R,lin_body1(i,:)'];
    world_T_body1(i).H = [world_T_body1(i).H; 0 0 0 1];
    
    %%Left FT to world Homogeneous transformation
    left_ft_rot(i).R =  RotM_body1(i).R;
    left_ft_pos(i).P = lin_body1(i,:) + [0, 0.275, 0];
    world_T_left_ft(i).H = [left_ft_rot(i).R,left_ft_pos(i).P'; 0, 0, 0, 1];
    
    world_left_wrench(i,:) = ftTransform(world_T_left_ft(i).H,left_wrench(i,:));
end


lin_body2 = body2_pose(:,1:3);
ang_body2 = body2_pose(:,4:7);

for i = 1:1:size(ang_body2,1)
    %Get  Homogeneous transformation
    RotM_body2(i).R = quaternion2rotm(ang_body2(i,:));
    world_T_body2(i).H = [RotM_body2(i).R,lin_body2(i,:)'];
    world_T_body2(i).H = [world_T_body2(i).H; 0 0 0 1];
    
    %%Right FT to world Homogeneous transformation
    right_ft_rot(i).R =  RotM_body2(i).R;
    theta = (vrrotmat2vec(RotM_body2(i).R))';
    if(theta(3) < 0)
        theta(4) = -theta(4);
    end
    right_ft_pos(i).P = lin_body2(i,:) + [0.275*sin(theta(4)), 0.275*cos(theta(4)), 0];
    com_body2(i,:) = lin_body2(i,:)+[0.15*sin(theta(4)), 0.15*cos(theta(4)),0];
    world_T_right_ft(i).H = [right_ft_rot(i).R,right_ft_pos(i).P'; 0, 0, 0, 1];     
    
    world_right_wrench(i,:) = ftTransform(world_T_right_ft(i).H,right_wrench(i,:));
end

%%Computing Body CoM
for i=1:1:size(com_body1,1)
    com_body(i,:) = (m1*com_body1(i,:)+m2*com_body2(i,:))/(m1+m2);
    
    c1 = com_body(i,:)-com_body1(i,:); %difference in CoM
    c1x = vec2skew(c1); % Cx

    I1(i).I = [m1*eye(3,3) m1*(c1x'); 
               m1*c1x I1_com+m1*c1x*(c1x');];
  
    c2 = com_body(i,:)-com_body2(i,:); %difference in CoM
    c2x = vec2skew(c2); % Cx

    I2(i).I = [m2*eye(3,3) m2*(c2x'); 
               m2*c2x I2_com+m2*c2x*(c2x');];
end

%%Computing relative transformation 1_T_2
for i = 1:1:size(world_T_body2,2)
    T_12(i).H = inv(world_T_body1(i).H)*(world_T_body2(i).H);
    RotM12(i).R = T_12(i).H(1:3,1:3);
    lin12(i,:) = (T_12(i).H(1:3,4))';
end

%%Computing axis and joint variables
%%First three elements axis of translation or rotation, last element is
%%distance of translation or angle of rotation or 
%%This gives the properties of the prismatic joint or revolute joint 
for i = 1:1:size(RotM12,2)
    
    %%Prismatic Joint
    distance = norm(lin12(i,:)');
    axisDistanceVector12(i,:) = [lin12(i,:)/distance, distance];
    jointDistance(i,:) = axisDistanceVector12(i,4); %%This is in meters
    P_jointAxis(i,:) = [axisDistanceVector12(i,1:3),0,0,0];

    
    %%Revolute Joint
    axisAngleVector12(i,:) = (vrrotmat2vec(RotM12(i).R))';
%     euler(i,:) = rotm2euler(RotM12(i).R);
    jointAngle(i,:) = axisAngleVector12(i,4); %%This is in radians
    R_jointAxis(i,:) = [0,0,0,axisAngleVector12(i,1:3)];

end

% % %%Testing SGolay filtering
% % order = 7;
% % framelen = 71;
% % [sgFilter,G] = SgolayWrapper(order,framelen);
% % jointDistance_center = conv(jointDistance,sgFilter((framelen+1)/2,:),'valid');
% % jointDistance_begin = sgFilter(end:-1:(framelen+3)/2,:) * lin_body2(framelen:-1:1)';
% % jointDistance_end = sgFilter((framelen-1)/2:-1:1,:) * lin_body2(end:-1:end-(framelen-1))';
% % jointDistance = [jointDistance_begin; jointDistance_center; jointDistance_end];
% % % % figure;
% % % % a1=subplot(2,1,1); plot(lin_body2(:,2)); title('Prismatic Joint Distance Original');
% % % % a2=subplot(2,1,2); plot(pose); title('Prismatic Joint Distance Smoothed');
% % % % linkaxes([a1 a2],'xy');
% % 
% % % % [da,dda]=SgolayDerivation(order,framelen,lin_body2(:,2),0.01);
% % [da,dda]=SgolayDerivation(order,framelen,jointDistance,0.01);
% % % % figure;
% % % % pa1=subplot(3,1,1); plot(pose,'.'); title('Prismatic Joint Distance');
% % % % pa2=subplot(3,1,2); plot(da,'.'); title('Prismatic Joint Velocity');
% % % % pa3=subplot(3,1,3); plot(dda,'.'); title('Prismatic Joint Acceleration');
% % % % linkaxes([pa3 pa2 pa3],'xy');

dtime = diff(time);
%%Joint variable first derivative
dJointDistance = diff(jointDistance)./dtime;
dJointAngle = diff(jointAngle)./dtime;

%%Adding padding zeros
dJointDistance = [0;dJointDistance];
dJointAngle = [0;dJointAngle];

% % ddtime = diff(time(2:end));
%%Joint variable second derivative
ddJointDistance = diff(dJointDistance)./dtime;
ddJointAngle = diff(dJointAngle)./dtime;

%%Adding padding zeros
ddJointDistance = [0;ddJointDistance];
ddJointAngle = [0;ddJointAngle];

%%Body1 Linear velocity
lin_vel_body1(:,1) = diff(lin_body1(:,1))./dtime;
lin_vel_body1(:,2) = diff(lin_body1(:,2))./dtime;
lin_vel_body1(:,3) = diff(lin_body1(:,3))./dtime;


%%Body1 Angular velocity
for i = 1:1:size(RotM_body1,2)-1
    dRotM_body1(i).R = RotM_body1(i+1).R-RotM_body1(i).R; %%Computing differences
    dRotM_body1(i).R = dRotM_body1(i).R/time(i+1,1); %%Component wise derivative
    s_w_body1(i).m = (dRotM_body1(i).R).*(RotM_body1(i).R');
    ang_vel_body1(i,:) = skew2vec(s_w_body1(i).m);
end

%%Adding padding zeros
lin_vel_body1 = [0,0,0;lin_vel_body1];
ang_vel_body1 = [0,0,0;ang_vel_body1];


%%Body1 Linear Acceleration
lin_acc_body1(:,1) = diff(lin_vel_body1(:,1))./dtime;
lin_acc_body1(:,2) = diff(lin_vel_body1(:,2))./dtime;
lin_acc_body1(:,3) = diff(lin_vel_body1(:,3))./dtime;


%%Body1 Angular Acceleration
ang_acc_body1(:,1) = diff(ang_vel_body1(:,1))./dtime;
ang_acc_body1(:,2) = diff(ang_vel_body1(:,2))./dtime;
ang_acc_body1(:,3) = diff(ang_vel_body1(:,3))./dtime;

%%Adding padding zeros
lin_acc_body1 = [0,0,0;lin_acc_body1];
ang_acc_body1 = [0,0,0;ang_acc_body1];

%6D Vectors
measured_vel_body1 = ([lin_vel_body1'; ang_vel_body1'])';
measured_acc_body1 = ([lin_acc_body1'; ang_acc_body1'])';

%%Body2 Linear velocity
lin_vel_body2(:,1) = diff(lin_body2(:,1))./dtime;
lin_vel_body2(:,2) = diff(lin_body2(:,2))./dtime;
lin_vel_body2(:,3) = diff(lin_body2(:,3))./dtime;
 
%%Body2 Angular velocity
for i = 1:1:size(RotM_body2,2)-1
    dRotM_body2(i).R = RotM_body2(i+1).R-RotM_body2(i).R; %%Computing differences
    dRotM_body2(i).R = dRotM_body2(i).R/time(i+1,1); %%Component wise derivative
    s_w_body2(i).m = (dRotM_body2(i).R).*(RotM_body2(i).R');
    ang_vel_body2(i,:) = skew2vec(s_w_body2(i).m);
end

%%Adding padding zeros
lin_vel_body2 = [0,0,0;lin_vel_body2];
ang_vel_body2 = [0,0,0;ang_vel_body2];

%%Body2 Linear Acceleration
lin_acc_body2(:,1) = diff(lin_vel_body2(:,1))./dtime;
lin_acc_body2(:,2) = diff(lin_vel_body2(:,2))./dtime;
lin_acc_body2(:,3) = diff(lin_vel_body2(:,3))./dtime;

%%Body2 Angular Acceleration
ang_acc_body2(:,1) = diff(ang_vel_body2(:,1))./dtime;
ang_acc_body2(:,2) = diff(ang_vel_body2(:,2))./dtime;
ang_acc_body2(:,3) = diff(ang_vel_body2(:,3))./dtime;

%%Adding padding zeros
lin_acc_body2 = [0,0,0;lin_acc_body2];
ang_acc_body2 = [0,0,0;ang_acc_body2];

%6D Vectors
measured_vel_body2 = ([lin_vel_body2'; ang_vel_body2'])'; %This is measured
measured_acc_body2 = ([lin_acc_body2'; ang_acc_body2'])';

%%Computing velocities assuming a joint nature
for i=1:1:size(R_jointAxis,1)-1 
    R_vel_body2_computed(i,:) = measured_vel_body1(i,:) + R_jointAxis(i,:)*dJointAngle(i,1); %%Assuming Revolute joint
    P_vel_body2_computed(i,:) = measured_vel_body1(i,:) + P_jointAxis(i,:)*dJointDistance(i,1); %%Assuming Prismatic joint
    
    %%Computing s_dot = VxS - these values are near zeros
    R_dJointAxis(i,:) = crm(R_vel_body2_computed(i,:))*(R_jointAxis(i+1,:)'); %%Something fishy here
    P_dJointAxis(i,:) = crm(P_vel_body2_computed(i,:))*(P_jointAxis(i+1,:)');
end

%%Adding padding zeros
R_vel_body2_computed = [0,0,0,0,0,0;R_vel_body2_computed];
P_vel_body2_computed = [0,0,0,0,0,0;P_vel_body2_computed];

R_dJointAxis = [0,0,0,0,0,0;R_dJointAxis];
P_dJointAxis = [0,0,0,0,0,0;P_dJointAxis];

%%Computing accelerations assuming a joint nature
for i=1:1:size(measured_acc_body1,1)
    R_acc_body2_computed(i,:) = measured_acc_body1(i,:) + R_dJointAxis(i,:)*dJointAngle(i,1) + R_jointAxis(i,:)*ddJointAngle(i,1);
    P_acc_body2_computed(i,:) = measured_acc_body1(i,:) + P_dJointAxis(i,:)*dJointDistance(i,1) + P_jointAxis(i,:)*ddJointDistance(i,1);
end

for i=1:1:size(R_acc_body2_computed,1)
    R_momentum_change(i,:) = I1(i).I*(measured_acc_body1(i,:)') + crf(measured_vel_body1(i,:))*I1(i).I*(measured_vel_body1(i,:)') + I2(i).I*(R_acc_body2_computed(i,:)') + crf(R_vel_body2_computed(i,:))*I2(i).I*(R_vel_body2_computed(i,:)');
    P_momentum_change(i,:) = I1(i).I*(measured_acc_body1(i,:)') + crf(measured_vel_body1(i,:))*I1(i).I*(measured_vel_body1(i,:)') + I2(i).I*(P_acc_body2_computed(i,:)') + crf(P_vel_body2_computed(i,:))*I2(i).I*(P_vel_body2_computed(i,:)');
end

%%Adding padding zeros
% % dJointDistance = [0;dJointDistance];
% % dJointAngle = [0;dJointAngle];
% % 
% % measured_vel_body2 = [0, 0, 0, 0, 0, 0; measured_vel_body2];
% % R_vel_body2_computed = [0, 0, 0, 0, 0, 0; R_vel_body2_computed];
% % P_vel_body2_computed = [0, 0, 0, 0, 0, 0; P_vel_body2_computed];
% % 
% % measured_acc_body2 =   [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; measured_acc_body2];
% % R_acc_body2_computed = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; R_acc_body2_computed];
% % P_acc_body2_computed = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0; P_acc_body2_computed];
% % 
% % R_momentum_change = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0;R_momentum_change];
% % P_momentum_change = [0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0;P_momentum_change];

%%Total wrench with respect to the world
measured_wrench_abs = world_left_wrench + world_right_wrench;

%%Computing rate of change in momentum
R = measured_wrench_abs - R_momentum_change;
P = measured_wrench_abs - P_momentum_change;

% % %%Plotting Link and Body CoM
% % figure;
% % com_plt1 = subplot(3,1,1); plot(com_body1); legend('x','y','z');title('Left Link CoM');
% % com_plt2 = subplot(3,1,2); plot(com_body2); legend('x','y','z');title('Right Link CoM');
% % com_plt3 = subplot(3,1,3); plot(com_body); legend('x','y','z');title('Body CoM');
% % % % linkaxes([com_plt1 com_plt2 com_plt3],'xy');

%%Plotting Joint Variables
figure; 
Rjoint_plt1 = subplot(3,1,1); plot(time,jointAngle); title('Revolute Joint Angle');
Rjoint_plt2 = subplot(3,1,2); plot(time,dJointAngle); title('Revolute Joint Vel');
Rjoint_plt3 = subplot(3,1,3); plot(time,ddJointAngle); title('Revolute Joint Acc');
% % linkaxes([Rjoint_plt1 Rjoint_plt2 Rjoint_plt3],'xy')

figure;
Pjoint_plt1 = subplot(3,1,1); plot(time,jointDistance); title('Prismatic Joint Distance');
Pjoint_plt2 = subplot(3,1,2); plot(time,dJointDistance); title('Prismatic Joint Vel');
Pjoint_plt3 = subplot(3,1,3); plot(time,ddJointDistance); title('Prismatic Joint Acc');
% % linkaxes([Pjoint_plt1 Pjoint_plt2 Pjoint_plt3],'xy')

figure;
R_axis_plt = subplot(2,1,1); plot(time,R_jointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Joint Axis');
P_axis_plt = subplot(2,1,2); plot(time,P_jointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Joint Axis');
% % linkaxes([R_axis_plt P_axis_plt],'xy')

% % R_dJointAxis = [0,0,0,0,0,0;R_dJointAxis];
% % P_dJointAxis = [0,0,0,0,0,0;P_dJointAxis];
% % 
% % figure;
% % dR_plt = subplot(2,1,1); plot(time,R_dJointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Axis Derivative');
% % dP_plt = subplot(2,1,2); plot(time,P_dJointAxis); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Axis Derivative')
% % linkaxes([dR_plt dP_plt],'xy')

% % %%Plotting Velocities
% % figure;
% % V_plt1 = subplot(3,1,1); plot(time,measured_vel_body2); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Body2 velocity measured');
% % V_plt2 = subplot(3,1,2); plot(time,R_vel_body2_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute - Body2 velocity computed');
% % V_plt3 = subplot(3,1,3); plot(time,P_vel_body2_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic - Body2 velocity computed');
% % linkaxes([V_plt1 V_plt2 V_plt3],'xy')
% % 
% % % % %%Plotting Accelerations
% % figure;
% % A_plt1 = subplot(3,1,1); plot(time,measured_acc_body2); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Body2 acceleration measured');
% % A_plt2 = subplot(3,1,2); plot(time,R_acc_body2_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute - Body2 acceleration computed');
% % A_plt3 = subplot(3,1,3); plot(time,P_acc_body2_computed); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic - Body2 acceleration computed');
% % linkaxes([A_plt1 A_plt2 A_plt3],'xy')

% % %%Plotting External Wrench - Local Frames 
% % figure; 
% % subplot(3,1,1); plot(time,left_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Left Wrench_{local}');
% % subplot(3,1,2); plot(time,right_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Right Wrench_{local}');
% % subplot(3,1,3); plot(time,measured_wrench_local); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Total Wrench_{local}');

% % %%Plotting External Wrench - wrt Inertial Frame
% % figure; 
% % w_plt1 = subplot(3,1,1); plot(time,world_left_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Left Wrench_{world}');
% % w_plt2 = subplot(3,1,2); plot(time,world_right_wrench); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Right Wrench_{world}');
% % w_plt3 = subplot(3,1,3); plot(time,measured_wrench_abs); legend('e_{ox}','e_{oy}','e_{oz}','e_x','e_y','e_z'); title('Total Wrench_{world}');
% % linkaxes([w_plt1 w_plt2 w_plt3],'xy')
% % 
% % %%Plotting Joint Hypothesis RoM
% % figure; 
% % RH_plt = subplot(2,1,1); plot(time,R_momentum_change); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Hypothesis - Rate of Change in Momentum');
% % PH_plt = subplot(2,1,2); plot(time,P_momentum_change); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Hypothesis - Rate of Change in Momentum');
% % % % linkaxes([RH_plt PH_plt],'xy')

%%Plotting Joint Hypothesis
figure;
R_plt1 = subplot(2,1,1); plot(time,R); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Revolute Hypothesis');
P_plt2 = subplot(2,1,2); plot(time,P); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Prismatic Hypothesis');
% % plt3 = subplot(3,1,3); plot(time,R-P); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}'); title('Difference in Hypothesis');
% linkaxes([R_plt1 P_plt2],'xy')