close all;
clear all;
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'revolute';

mat_data_directory_path = strcat(char(pwd),'/FINAL_DATA_MAT/force_control/');
cd ../
data_directory_path = strcat(char(pwd),'/manipulation/data/FINAL_DATA/force_control/');

if(strcmp(model,'prismatic'))
    mat_data_directory_name = strcat(mat_data_directory_path,'mchange_pdata/');
    data_directory_name = strcat(data_directory_path,'pdata/');
else
    mat_data_directory_name = strcat(mat_data_directory_path,'mchange_rdata/');
    data_directory_name = strcat(data_directory_path,'rdata/');
end

mat_data_directory = dir([mat_data_directory_name]);

data_directory = dir([data_directory_name,'*.txt']);
num_files = length(data_directory(not([data_directory.isdir])));

mchange_step = 0.05;
mchange_value = 0.5;
mchange = [-mchange_value:mchange_step:mchange_value];
mchange_run_size = size(mchange,2);

mass1 = 4.5;
mass2 = 2;

for n=1:1:num_files
    
    data_file_name = strcat(data_directory_name,data_directory(n).name);
    delimiterIn = ' ';
    data1 = importdata(data_file_name,delimiterIn);
    
    half_data_size = round(size(data1,1)/2);
    data = data1(1:half_data_size,:);

   if(strcmp(model,'prismatic'))

       %%These are for prismatic model
       com1 = [0.1; 0; 0.0125];
       com2 = [0.1; 0; -0.0125];

       I1 = [0.001133   0         0;
             0          0.01508   0;
             0          0         0.01575]; %%Inertia at CoM - taken from  SDF
       I_c1 = [];

       I2 = [0.0005035   0           0;
             0           0.0067035   0;
             0           0           0.007]; %%Inertia at CoM - taken from  SDF
       I_c2 = [];
       
   else
           
       %%These are for revolute model
       com1 = [0.1;0;0];
       com2 = [0.15;0;0];

       I1 = [0.001801   0         0;
           0          0.01575   0;
           0          0         0.01575]; %%Inertia at CoM - taken from  SDF
       I_c1 = [];

       I2 = [0.0008   0         0;
           0        0.007     0;
           0        0         0.007]; %%Inertia at CoM - taken from  SDF
       I_c2 = [];

   end
    
   for j = 1:1:mchange_run_size
       
       %%Adding Noise to Inertial Parameters
       change = mchange(j);

       m1add = mass1*change;
       m1 = mass1 +m1add;

       [U1,S1,V1] = svd(I1);
       S1(1,1) = S1(1,1) + S1(1,1)*change;
       S1(2,2) = S1(2,2) + S1(2,2)*change;
       S1(3,3) = S1(3,3) + S1(3,3)*change;
       I_c1 = U1*S1*V1';

       m2add = mass2*change;
       m2 = mass2 + m2add;

       [U2,S2,V2] = svd(I2);
       S2(1,1) = S2(1,1) + S2(1,1)*change;
       S2(2,2) = S2(2,2) + S2(2,2)*change;
       S2(3,3) = S2(3,3) + S2(3,3)*change;
       I_c2 = U2*S2*V2';

       [hypdiff phyp rhyp] = jointEstimationAlgorithm(model,data,m1,m2,com1,com2,I_c1,I_c2,g);
       
       dummy = strsplit(data_file_name,'/');
       fname = strsplit(char(dummy(end)),'.');
       mat_file_name = strcat(char(fname(1)),'mchange',num2str(j),'.mat');
       cd(mat_data_directory_name);
       save(mat_file_name);
       pause(1);
   end 
   
end

cd ../../../