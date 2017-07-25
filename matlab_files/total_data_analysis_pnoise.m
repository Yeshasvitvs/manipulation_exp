close all;
clear all;
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'revolute';

if(strcmp(model,'prismatic'))
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/force_control/pnoise_pdata/';
    data_directory_name = '/home/yeshi/projects/manipulation_exp/manipulation/data/FINAL_DATA/force_control/pdata/';
else
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/force_control/pnoise_rdata/';
    data_directory_name = '/home/yeshi/projects/manipulation_exp/manipulation/data/FINAL_DATA/force_control/rdata/';
end

mat_data_directory = dir([mat_data_directory_name]);
data_directory = dir([data_directory_name,'*.txt']);
num_files = length(data_directory(not([data_directory.isdir])));

for n=1:1:num_files
    
    data_file_name = strcat(data_directory_name,data_directory(n).name);
    delimiterIn = ' ';
    data2 = importdata(data_file_name,delimiterIn);
    
    POS_STD = [0.01,0.02,0.03,0.04,0.05]; %Position - Meters
    ORI_STD = [1,2,3,4,5].*(pi/180); %Orientation - Radians
    pnoise_run_size = size(POS_STD,2);
    
    half_data_size = round(size(data2,1)/2);
    data1 = data2(1:half_data_size,:);
    
    %%Noise and Filtering
    K = 3;  % Order of polynomial fit
    F = 121;  % Window length
    HalfWin  = ((F+1)/2) -1;
    
    %%Pose Values - Angular part is quaternion
    p1 = data1(:,2:8);
    p2 = data1(:,9:15);

   for j = 1:1:pnoise_run_size
       
       pnoise_check_count = 100;
    
       for i=1:1:pnoise_check_count
        
           data = [];
           
           %%Adding Noise
           data(:,2:8) = poseNoiseSmoothing(POS_STD(j), ORI_STD(j), K, F, p1);
           data(:,9:15) = poseNoiseSmoothing(POS_STD(j), ORI_STD(j), K, F, p2);
        
           data(:,1) = data1(HalfWin+1:end-HalfWin-1,1); %%Time received in seconds
        
           %%Interaction Wrench - Measured in local frame
           data(:,16:21) = data1(HalfWin+1:end-HalfWin-1,16:21); %%FT at Body1
           data(:,22:27) = data1(HalfWin+1:end-HalfWin-1,22:27); %%FT at Body2
        
           if(strcmp(model,'prismatic'))
               
               %%Prismatic Model - Rigid Body Properties
               m1 = 4.5; %%Kgs
               I_c1 = [0.001133   0         0;
                   0         0.01508   0;
                   0         0         0.01575]; %%Inertia at CoM - taken from  SDF
               com1 = [0.1; 0; 0.0125];
               
               m2 = 2; %%Kgs
               I_c2 = [0.0005035   0         0;
                   0         0.0067035   0;
                   0         0         0.007]; %%Inertia at CoM - taken from  SDF
               com2 = [0.1; 0; -0.0125];
               
               [hypdiff(i) value(i)] = algorithmPrismatic(data,m1,m2,com1,com2,I_c1,I_c2,g);

           else
               
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
               
               [hypdiff(i) value(i)] = algorithmRevolute(data,m1,m2,com1,com2,I_c1,I_c2,g);
               
           end
           
       end
       
       dummy = strsplit(data_file_name,'/');
       fname = strsplit(char(dummy(end)),'.');
       if(strcmp(model,'prismatic'))
           mat_file_name = strcat(char(fname(1)),'pnoise',num2str(j),'.mat');
       else
           mat_file_name = strcat(char(fname(1)),'rnoise',num2str(j),'.mat');
       end
       cd(mat_data_directory_name)
       save(mat_file_name)
   end
    
end