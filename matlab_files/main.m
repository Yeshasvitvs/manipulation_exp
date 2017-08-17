close all;
clear all;
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'revolulte';
add_noise = false;
noise_count = 1;

cd ../
path =  strcat(char(pwd),'/manipulation/data/');

if(strcmp(model,'prismatic'))
    filename = strcat(path,'fonetestpdata5.txt');
else
    filename = strcat(path,'fonetestrdata5.txt');
end

data1 = importdata(filename);
% data1 = data2(1:120,:);

if(add_noise)
    
    %%Noise and Filtering
    K = 3;  % Order of polynomial fit
    F = 121;  % Window length
    HalfWin  = ((F+1)/2) -1;

    %%Gaussian Noise Standard Deviation
    POS_STD = 0.05; %Position - Meters
    ORI_STD = 5*(pi/180); %Orientation - Radians

    %%Pose Values - Angular part is quaternion
    p1 = data1(:,2:8);
    p2 = data1(:,9:15);
    
    noise_check_count = noise_count;
    
else
    
    data = data1;
    noise_check_count = 1;
    
end

    
for i=1:1:noise_check_count
    
    if(add_noise)
        
        data = [];

        %%Adding Noise
        data(:,2:8)  = poseNoiseSmoothing(POS_STD, ORI_STD, K, F, p1);
        data(:,9:15)  = poseNoiseSmoothing(POS_STD, ORI_STD, K, F, p2);

        data(:,1) = data1(HalfWin+1:end-HalfWin-1,1); %%Time received in seconds

        %%Interaction Wrench - Measured in local frame
        data(:,16:21) = data1(HalfWin+1:end-HalfWin-1,16:21); %%FT at Body1
        data(:,22:27) = data1(HalfWin+1:end-HalfWin-1,22:27); %%FT at Body2
        
    end
    

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
        
    end
    
    [hypdiff(i) phyp(i) rhyp(i)] = jointEstimationAlgorithm(model,data,m1,m2,com1,com2,I_c1,I_c2,g);
    
end

if(add_noise)
    
    hypdiff;
    
    s = sign (hypdiff);
    ipositif = sum (s (:) == 1);
    inegatif = sum (s (:) == - 1);
    
    if(strcmp(model,'prismatic'))
        phypdiffErr = (ipositif/noise_check_count)*100
    else
        rhypdiffErr = (inegatif/noise_check_count)*100
    end
    
else

    hypdiff
    
end

cd ./matlab_files/

% % figure; plot(P); hold on; plot(R); legend('P','R')