close all;
clear all; 
clc; 

g = [0;0;-9.8;0;0;0]; %%Gravity

filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/onetestrdata3.txt';
data = importdata(filename);

check_size = 50;
m1 =4.5*ones(check_size,1);
m2 = 2*ones(check_size,1);

%%These are for revolute model
com1 = repmat([0.1;0;0], 1, check_size);
com2 = repmat([0.15;0;0], 1, check_size);

I1 = [0.001801   0         0;
      0          0.01575   0;
      0          0         0.01575]; %%Inertia at CoM - taken from  SDF
I_c1 = [];

I2 = [0.0008   0         0;
      0        0.007     0;
      0        0         0.007]; %%Inertia at CoM - taken from  SDF
I_c2 = [];

% % %%These are for prismatic model
% % com1 = repmat([0.1; 0; 0.0125], 1, check_size);
% % com2 = repmat([0.1; 0; -0.0125], 1, check_size);
% % 
% % I1 = [0.001133   0         0;
% %       0          0.01508   0;
% %       0          0         0.01575]; %%Inertia at CoM - taken from  SDF
% % I_c1 = [];
% % 
% % I2 = [0.0005035   0           0;
% %       0           0.0067035   0;
% %       0           0           0.007]; %%Inertia at CoM - taken from  SDF
% % I_c2 = [];

for step=1:1:check_size
     
    %%Adding Noise to Inertial Parameters
    noise = 0.2;
    a = -noise; b = noise;
    m1(step) = m1(step) + m1(step)*(a + (b-a).*rand(1,1));
 
    com1(:,step) = com1(:,step) + [com1(1,step)*(a + (b-a).*rand(1,1)); 
                                   com1(2,step)*(a + (b-a).*rand(1,1));
                                   com1(3,step)*(a + (b-a).*rand(1,1));];
    
    [U1,S1,V1] = svd(I1);
    S1(1,1) = S1(1,1) + S1(1,1)*(a + (b-a).*rand(1,1));
    S1(2,2) = S1(2,2) + S1(2,2)*(a + (b-a).*rand(1,1));
    S1(3,3) = S1(3,3) + S1(3,3)*(a + (b-a).*rand(1,1));
    I_c1(step).I = U1*S1*V1';
    
    m2(step) = m2(step) + m2(step)*(a + (b-a).*rand(1,1));
    com2(:,step) = com2(:,step) + [com2(1,step)*(a + (b-a).*rand(1,1)); 
                                   com2(2,step)*(a + (b-a).*rand(1,1));
                                   com2(3,step)*(a + (b-a).*rand(1,1));];
    
    [U2,S2,V2] = svd(I2);
    S2(1,1) = S2(1,1) + S2(1,1)*(a + (b-a).*rand(1,1));
    S2(2,2) = S2(2,2) + S2(2,2)*(a + (b-a).*rand(1,1));
    S2(3,3) = S2(3,3) + S2(3,3)*(a + (b-a).*rand(1,1));
    I_c2(step).I = U2*S2*V2';
    
    
    [Phyp(step,:) Rhyp(step,:)] = algorithmRevolute(data,m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I,g);
%     [Phyp(step,:) Rhyp(step,:)] = algorithmPrismatic(data,m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I,g);
   
end



 