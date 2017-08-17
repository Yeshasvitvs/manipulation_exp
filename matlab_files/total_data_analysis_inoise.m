close all;
clear all;
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'prismatic';

mat_data_directory_path = strcat(char(pwd),'/FINAL_DATA_MAT/force_control/');
cd ../
data_directory_path = strcat(char(pwd),'/manipulation/data/FINAL_DATA/force_control/');

if(strcmp(model,'prismatic'))
    mat_data_directory_name = strcat(mat_data_directory_path,'inoise_pdata/');
    data_directory_name = strcat(data_directory_path,'pdata/');
else
    mat_data_directory_name = strcat(mat_data_directory_path,'inoise_rdata/');
    data_directory_name = strcat(data_directory_path,'rdata/');
end

mat_data_directory = dir([mat_data_directory_name]);

data_directory = dir([data_directory_name,'*.txt']);
num_files = length(data_directory(not([data_directory.isdir])));

inoise_check_count = 30;

inoise = [0, 0.1, 0.2, 0.3, 0.4, 0.5];
inoise_run_size = size(inoise,2);

for n=1:1:num_files
    
    data_file_name = strcat(data_directory_name,data_directory(n).name);
    delimiterIn = ' ';
    data = importdata(data_file_name,delimiterIn);
    
   for j = 1:1:inoise_run_size
       
       m1 =4.5*ones(inoise_check_count,1);
       m2 = 2*ones(inoise_check_count,1);
       
       if(strcmp(model,'prismatic'))
          
           %%These are for prismatic model
           com1 = repmat([0.1; 0; 0.0125], 1, inoise_check_count);
           com2 = repmat([0.1; 0; -0.0125], 1, inoise_check_count);

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
           com1 = repmat([0.1;0;0], 1, inoise_check_count);
           com2 = repmat([0.15;0;0], 1, inoise_check_count);
           
           I1 = [0.001801   0         0;
               0          0.01575   0;
               0          0         0.01575]; %%Inertia at CoM - taken from  SDF
           I_c1 = [];
           
           I2 = [0.0008   0         0;
               0        0.007     0;
               0        0         0.007]; %%Inertia at CoM - taken from  SDF
           I_c2 = [];
           
       end
    
       for step=1:1:inoise_check_count
        
           %%Adding Noise to Inertial Parameters
           noise = inoise(j);
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
     
           if(strcmp(model,'prismatic'))
               [hypdiff(step) phyp(step) rhyp(step)] = algorithmPrismatic(data,m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I,g);
           else
               [hypdiff(step) phyp(step) rhyp(step)] = algorithmRevolute(data,m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I,g);
           end
           
           dummy = strsplit(data_file_name,'/');
           fname = strsplit(char(dummy(end)),'.');
           mat_file_name = strcat(char(fname(1)),'inoise',num2str(j),'.mat');
           cd(mat_data_directory_name);
           save(mat_file_name);
           pause(1);
           
       end
       
   end 
   
end

cd ../../../