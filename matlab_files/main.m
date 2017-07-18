close all;
clear all;
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/guirdata5.txt';
data = importdata(filename);

% % data = [];
% % 
% % %%Adding Noise
% % %%Pose Values - Angular part is quaternion
% % p1 = data1(:,2:8);
% % p2 = data1(:,9:15);
% % 
% % %%Noise and Filtering
% % K = 3;  % Order of polynomial fit
% % F = 1031;  % Window length
% % HalfWin  = ((F+1)/2) -1;
% % SNR = 1000;  %Signal-Noise ratio
% % STD = 0.01; %Noise Standard Deviation
% % 
% % data(:,2:8)  = poseNoiseSmoothing(SNR, STD, K, F, p1);
% % data(:,9:15)  = poseNoiseSmoothing(SNR, STD, K, F, p2);
% % 
% % data(:,1) = data1(HalfWin+1:end-HalfWin-1,1); %%Time received in seconds
% % 
% % %%Interaction Wrench - Measured in local frame
% % data(:,16:21) = data1(HalfWin+1:end-HalfWin-1,16:21); %%FT at Body1
% % data(:,22:27) = data1(HalfWin+1:end-HalfWin-1,22:27); %%FT at Body2

%%Revolute Model - Rigid Body Properties
m1 =4.5; %%Kgs
I_c1 = [0.001801   0         0;
        0          0.01575   0;
        0          0         0.01575]; %%Inertia at CoM - taken from  SDF
com1 = [0.1; 0; 0];

m2 = 2; %%Kgs
I_c2 = [0.0008   0         0;
        0        0.007   0;
        0        0         0.007]; %%Inertia at CoM - taken from  SDF
com2 = [0.15; 0; 0];

algorithmRevolute(data,m1,m2,com1,com2,I_c1,I_c2,g);


% % %%Prismatic Model - Rigid Body Properties
% % m1 = 4.5; %%Kgs
% % I_c1 = [0.001133   0         0;
% %         0         0.01508   0;
% %         0         0         0.01575]; %%Inertia at CoM - taken from  SDF
% % com1 = [0.1; 0; 0.0125];
% % 
% % m2 = 2; %%Kgs
% % I_c2 = [0.0005035   0         0;
% %         0         0.0067035   0;
% %         0         0         0.007]; %%Inertia at CoM - taken from  SDF
% % com2 = [0.1; 0; -0.0125];
% % 
% % algorithmPrismatic(data,m1,m2,com1,com2,I_c1,I_c2,g);
