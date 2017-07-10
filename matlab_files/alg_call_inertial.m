close all;
clear all; 
clc; 

%%This code works for this particular prismatic motion data set
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/dgnewrmotion2.txt';
data = importdata(filename);

for step=1:1:50
    
    %%Empty Initialization of Hypothesis Vectors
    Phyp = [];
    Rhyp = [];
    
    %%Adding Noise to Inertial Parameters
    noise = 0.2;
    a = -noise; b = noise;
    m1 = 4.5;
    m1 = m1 + m1*(a + (b-a).*rand(1,1));
    com1 = [0.1; 0; 0];
    com1 = com1 + [com1(1)*(a + (b-a).*rand(1,1)); 
                   com1(2)*(a + (b-a).*rand(1,1));
                   com1(3)*(a + (b-a).*rand(1,1));];
    I_c1 = [0.001801   0         0;
            0          0.01575   0;
            0          0         0.01575]; %%Inertia at CoM - taken from  SDF
    [U1,S1,V1] = svd(I_c1);
    S1(1,1) = S1(1,1) + S1(1,1)*(a + (b-a).*rand(1,1));
    S1(2,2) = S1(2,2) + S1(2,2)*(a + (b-a).*rand(1,1));
    S1(3,3) = S1(3,3) + S1(3,3)*(a + (b-a).*rand(1,1));
    I_c1 = U1*S1*V1';
    
    m2 = 2;
    m2 = m2 + m2*(a + (b-a).*rand(1,1));
    com2 = [0.15; 0; 0];
    com2 = com2 + [com2(1)*(a + (b-a).*rand(1,1)); 
                   com2(2)*(a + (b-a).*rand(1,1));
                   com2(3)*(a + (b-a).*rand(1,1));];
    I_c2 = [0.0008   0         0;
            0        0.007     0;
            0        0         0.007]; %%Inertia at CoM - taken from  SDF
    [U2,S2,V2] = svd(I_c2);
    S2(1,1) = S2(1,1) + S2(1,1)*(a + (b-a).*rand(1,1));
    S2(2,2) = S2(2,2) + S2(2,2)*(a + (b-a).*rand(1,1));
    S2(3,3) = S2(3,3) + S2(3,3)*(a + (b-a).*rand(1,1));
    I_c2 = U2*S2*V2';
    
    
    index=1;
    step_size = 2000; %%Indicates the number of samples of motion data considered for computing the hypothesis
    max_index = ceil(size(data,1)/step_size);
    for i=1:1:max_index
        [Phyp(index,:) Rhyp(index,:)] = algorithmRevInertial(data(i:i+step_size,:),m1,m2,com1,com2,I_c1,I_c2);
        index=index+1;
    end
    
    %%Plot - Hypothesis
%     clf;
% %     figure(1);
% %     plot(Phyp); hold on;
% %     plot(Rhyp); hold on;
% %     title('Joint Nature Hypothesis');
% %     legend('Prismatic','Revolute');
% %     
% %     %%Text box to print step size
% %     textBox = uicontrol('style','text');
% %     stepStr = sprintf('Step Size : %d', step_size);
% %     set(textBox,'String',stepStr,...
% %         'Position',[235 2 120 20],...
% %         'HorizontalAlignment','left',...
% %         'FontSize',10);
% %     hold off;
    
    %%Plot - Hypothesis Difference
    figure(2);
    plot(Phyp-Rhyp); hold on;
    title('Hypothesis Difference(P-R)');
    
    pause(0.1);
end