close all;
clear all; 
clc; 

%%This code works for this particular prismatic motion data set
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/newrmotion10.txt';
data = importdata(filename);

for step=100:50:5000
    
    %%Empty Initialization of Hypothesis Vectors
    Phyp = [];
    Rhyp = [];
    
    index=1;
    step_size = step; %%Indicates the number of samples of motion data considered for computing the hypothesis
    max_index = ceil(size(data,1)/step_size);
    for i=1:1:max_index
        %%[Phyp(index,:) Rhyp(index,:)] = algorithmPri(data(i:i+pri_step,:));
        [Phyp(index,:) Rhyp(index,:)] = algorithmRev(data(i:i+step_size,:));
        index=index+1;
    end
    
    %%Plot - Hypothesis
    clf;
    figure(1);
    plot(Phyp); hold on;
    plot(Rhyp); hold on;
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
    plot(Phyp-Rhyp);
    title('Hypothesis Difference(P-R)');
    
    pause(0.1);
end