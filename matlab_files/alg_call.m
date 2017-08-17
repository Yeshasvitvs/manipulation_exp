close all;
clear all; 
clc;

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'prismatic';

cd ../
path =  strcat(char(pwd),'/manipulation/data/');

if(strcmp(model,'prismatic'))
    
    filename = strcat(path,'fonetestpdata1.txt');
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
    
    filename = strcat(path,'fonetestrdata1.txt');
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
% % STD = 0.000000001; %Noise Standard Deviation
% % 
% % data(:,2:8)  = poseNoiseSmoothing(SNR, STD, K, F, p1);
% % data(:,9:15)  = poseNoiseSmoothing(SNR, STD, K, F, p2);
% % 
% % data(:,1) = data1(HalfWin+1:end-HalfWin-1,1); %%Time received in seconds
% % 
% % %%Interaction Wrench - Measured in local frame
% % data(:,16:21) = data1(HalfWin+1:end-HalfWin-1,16:21); %%FT at Body1
% % data(:,22:27) = data1(HalfWin+1:end-HalfWin-1,22:27); %%FT at Body2

for step=10:50:size(data,1)
    
    %%Empty Initialization of Hypothesis Vectors
    Phyp = [];
    Rhyp = [];
    
    step_size = step; %%Indicates the number of samples of motion data considered for computing the hypothesis
    max_index = ceil(size(data,1)/step_size);
    
    if(max_index >= 2)
        index=1;
        for i=1:1:max_index-1 %%Given a max index, the number of motion blocks to analyse
%             [Phyp(i,:) Rhyp(i,:)] = algorithmPrismatic(data(index:index+step_size,:),m1,m2,com1,com2,I_c1,I_c2,g);
            [Phyp(i,:) Rhyp(i,:)] = algorithmRevolute(data(index:index+step_size,:),m1,m2,com1,com2,I_c1,I_c2,g);
            index=index+step_size;
        end
        
        %%Plot - Hypothesis
        clf;
        figure(1);
        if(size(Phyp,1) <= 1)
            plot(Phyp,'*b'); hold on;
            plot(Rhyp,'*r'); hold on;
        else
            plot(Phyp); hold on;
            plot(Rhyp); hold on;
        end
        
        title('Joint Nature Hypothesis');
        legend('Prismatic','Revolute');
    
        %%Text box to print step size
        textBox = uicontrol('style','text');
        stepStr = sprintf('Step Size : %d', step_size);
        set(textBox,'String',stepStr,...
            'Position',[235 2 120 20],...
            'HorizontalAlignment','left',...
            'FontSize',10);
        hold off;
    
        %%Plot - Hypothesis Difference
        figure(2);
        hyp_diff = Phyp - Rhyp;
        if(size(Phyp,1) <= 1)
            plot(hyp_diff,'*k');
        else
            plot(hyp_diff);
        end
        title('Hypothesis Difference(P-R)');
    
        max_diff = max(Phyp-Rhyp);
        min_diff = min(Phyp-Rhyp);
        if( min_diff < 0)
            display('**********NEGATIVE CHECK*********') %%Should never occur for revolute joint
            sprintf('Index : %d',step)
        end
        if( max_diff > 0)
            display('**********POSITIVE CHECK*********') %%Should never occur for prismatic joint
            sprintf('Index : %d',step)
        end
    
        pause(0.3);
        
    else
        break;
    end
end

cd ./matlab_files