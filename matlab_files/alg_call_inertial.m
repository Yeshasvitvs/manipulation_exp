close all;
clear all; 
clc; 

%%This code works for this particular prismatic motion data set
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/onetestpdata1.txt';
data = importdata(filename);

check_size = 10;
m1 =4.5*ones(check_size,1);
m2 = 2*ones(check_size,1);

%%These are for revolute model
% % com1 = repmat([0.1;0;0], 1, check_size);
% % com2 = repmat([0.15;0;0], 1, check_size);
% % 
% % I1 = [0.001801   0         0;
% %       0          0.01575   0;
% %       0          0         0.01575]; %%Inertia at CoM - taken from  SDF
% % I_c1 = [];
% % 
% % I2 = [0.0008   0         0;
% %       0        0.007     0;
% %       0        0         0.007]; %%Inertia at CoM - taken from  SDF
% % I_c2 = [];

%%These are for prismatic model
com1 = repmat([0.1; 0; 0.0125], 1, check_size);
com2 = repmat([0.1; 0; -0.0125], 1, check_size);

I1 = [0.001133   0         0;
      0          0.01508   0;
      0          0         0.01575]; %%Inertia at CoM - taken from  SDF
I_c1 = [];

I2 = [0.0005035   0           0;
      0           0.0067035   0;
      0           0           0.007]; %%Inertia at CoM - taken from  SDF
I_c2 = [];

for step=1:1:check_size
    
    %%Empty Initialization of Hypothesis Vectors
    Phyp = [];
    Rhyp = [];
     
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
    
    step_size = 550; %%Indicates the number of samples of motion data considered for computing the hypothesis
    max_index = ceil(size(data,1)/step_size);
    
    if(max_index >= 2)
        index=1;
        for i=1:1:max_index-1 %%Given a max index, the number of motion blocks to analyse
            [Phyp(i,:) Rhyp(i,:)] = algorithmPriInertial(data(index:index+step_size,:),m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I);
%             [Phyp(i,:) Rhyp(i,:)] = algorithmRevInertial(data(index:index+step_size,:),m1(step),m2(step),com1(:,step),com2(:,step),I_c1(step).I,I_c2(step).I);
            index=index+step_size;
        end
       
% %        %%Plot - Hypothesis
% %         clf;
% %         figure(1);
% %         if(size(Phyp,1) <= 1)
% %             plot(Phyp,'*b'); hold on;
% %             plot(Rhyp,'*r'); hold on;
% %         else
% %             plot(Phyp); hold on;
% %             plot(Rhyp); hold on;
% %         end
% %         
% %         title('Joint Nature Hypothesis');
% %         legend('Prismatic','Revolute');
% %     
% %         %%Text box to print step size
% %         textBox = uicontrol('style','text');
% %         stepStr = sprintf('Step Size : %d', step_size);
% %         set(textBox,'String',stepStr,...
% %             'Position',[235 2 120 20],...
% %             'HorizontalAlignment','left',...
% %             'FontSize',10);
% %         hold off;
    
        %%Plot - Hypothesis Difference
        figure(2);
        hyp_diff = Phyp - Rhyp;
        if(size(Phyp,1) <= 1)
            plot(hyp_diff,'*k'); hold on;
        else
            plot(hyp_diff); hold on;
        end
        title('Hypothesis Difference(P-R)');
    
        max_diff = max(Phyp-Rhyp);
        min_diff = min(Phyp-Rhyp);
        if( min_diff < 0)
            display('**********NEGATIVE CHECK*********') %%Should never occur for revolute joint
            sprintf('Check Index : %d',step)
        end
        if( max_diff > 0)
            display('**********POSITIVE CHECK*********') %%Should never occur for prismatic joint
            sprintf('Check Index : %d',step)
        end
        
        pause(0.3);
    
    else
        break;
    end
    
   
    

end




 %%Adding noise using add_awgn_noise function
% %     snr = 1;

% %     m1_check = true;
% %     while(m1_check)
% %         m1 = 4.5;
% %         m1 = add_awgn_noise(m1,snr);
% %         if(m1 <= 0)
% %             m1_check = true;
% %         else
% %             m1_check = false;
% %         end
% %     end
% %     
% %     com1 = [0.1; 0; 0];
% %     com1 = add_awgn_noise(com1,snr);    
% %     
% %     I_c1_check = true;
% %     while(I_c1_check)
% %         I_c1 = [0.001801   0         0;
% %                 0          0.01575   0;
% %                 0          0         0.01575]; %%Inertia at CoM - taken from  SDF
% %         [U1,S1,V1] = svd(I_c1);
% %         S1(1,1) = add_awgn_noise(S1(1,1),snr);
% %         S1(2,2) = add_awgn_noise(S1(2,2),snr);
% %         S1(3,3) = add_awgn_noise(S1(3,3),snr);
% %         I_c1 = U1*S1*V1';
% %         if(min(I_c1(:)) < 0 || I_c1(1,1) < 0.000000001 || I_c1(2,2) < 0.000000001 || I_c1(3,3) < 0.000000001)
% %             I_c1_check = true;
% %         else
% %             I_c1_check = false;
% %         end
% %     end
% % 
% %     m2_check = true;
% %     while(m2_check)
% %         m2 = 2;
% %         m2 = add_awgn_noise(m2,snr);
% %         if(m2 <= 0)
% %             m2_check = true;
% %         else
% %             m2_check = false;
% %         end
% %     end
% %     
% %     com2 = [0.15; 0; 0];
% %     com2 = add_awgn_noise(com2,snr);
% %     
% %     I_c2 = [0.0008   0         0;
% %             0        0.007     0;
% %             0        0         0.007]; %%Inertia at CoM - taken from  SDF
% %     [U2,S2,V2] = svd(I_c2);
% %     I_c2_check = true;
% %     while(I_c2_check)
% %         S2(1,1) = add_awgn_noise(S2(1,1),snr);
% %         S2(2,2) = add_awgn_noise(S2(2,2),snr);
% %         S2(3,3) = add_awgn_noise(S2(3,3),snr);
% %         I_c2 = U2*S2*V2';
% %         if(min(I_c2(:)) < 0 || I_c2(1,1) < 0.000000001 || I_c2(2,2) < 0.000000001 || I_c2(3,3) < 0.000000001)
% %             I_c2_check = true;
% %         else
% %             I_c2_check = false;
% %         end
% %     end
% %     I_c2;
% %     