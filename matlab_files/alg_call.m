close all;
clear all; 
clc; 

%%This code works for this particular prismatic motion data set
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/WORKING_primotion1.txt';
data = importdata(filename);

index=1;
pri_step = 1500;
max_index = ceil(size(data,1)/pri_step);
for i=1:1:max_index
    [Phyp(index,:) Rhyp(index,:)] = algorithmPri(data(i:i+pri_step,:));
    index=index+1;
end


% % filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/rmotion3.txt';
% % data = importdata(filename);
% % 
% % index=1;
% % rev_step = 2000;
% % max_index = ceil(size(data,1)/rev_step);
% % for i=1:1:max_index
% %     [Phyp(index,:) Rhyp(index,:)] = algorithmRev(data(i:i+rev_step,:));
% %     index=index+1;
% % end

figure;
Phandle = subplot(2,1,1); plot(Phyp); title('Prismatic Hypothesis'); xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
Rhandle = subplot(2,1,2); plot(Rhyp); title('Revolute Hypothesis'); xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')
% linkaxes([Phandle Rhandle],'xy');

figure;
plot(Phyp-Rhyp); title('Hypothesis Difference(P-R)'); xlabel('sec'); legend('d_x','d_y','d_z','d_{ox}','d_{oy}','d_{oz}')