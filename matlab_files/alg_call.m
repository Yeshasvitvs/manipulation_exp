close all;
clear all; 
clc; 

%%This code works for this particular prismatic motion data set
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/onetestpdata1.txt';
data = importdata(filename);
% % % data = [];
% % % 
% % % %%Adding Noise
% % % %%Pose Values - Angular part is quaternion
% % % p1 = data1(:,2:8);
% % % p2 = data1(:,9:15);
% % % 
% % % %%Noise and Filtering
% % % K = 3;  % Order of polynomial fit
% % % F = 1031;  % Window length
% % % HalfWin  = ((F+1)/2) -1;
% % % SNR = 1000;  %Signal-Noise ratio
% % % STD = 0.000000001; %Noise Standard Deviation
% % % 
% % % data(:,2:8)  = poseNoiseSmoothing(SNR, STD, K, F, p1);
% % % data(:,9:15)  = poseNoiseSmoothing(SNR, STD, K, F, p2);
% % % 
% % % data(:,1) = data1(HalfWin+1:end-HalfWin-1,1); %%Time received in seconds
% % % 
% % % %%Interaction Wrench - Measured in local frame
% % % data(:,16:21) = data1(HalfWin+1:end-HalfWin-1,16:21); %%FT at Body1
% % % data(:,22:27) = data1(HalfWin+1:end-HalfWin-1,22:27); %%FT at Body2

for step=10:10:20000
    
    %%Empty Initialization of Hypothesis Vectors
    Phyp = [];
    Rhyp = [];
    
    step_size = step; %%Indicates the number of samples of motion data considered for computing the hypothesis
    max_index = ceil(size(data,1)/step_size);
    
    if(max_index >= 2)
        index=1;
        for i=1:1:max_index-1 %%Given a max index, the number of motion blocks to analyse
            [Phyp(i,:) Rhyp(i,:)] = algorithmPri(data(index:index+step_size,:));
%             [Phyp(i,:) Rhyp(i,:)] = algorithmRev(data(index:index+step_size,:));
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