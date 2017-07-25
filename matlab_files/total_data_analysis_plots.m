close all;
clear all;
clc; 

g = [0;0;-9.8;0;0;0]; %%Gravity

model = 'revolute';

if(strcmp(model,'prismatic'))
    data_directory_name = '/home/yeshi/projects/manipulation_exp/manipulation/data/FINAL_DATA/force_control/pdata/';
else
    data_directory_name = '/home/yeshi/projects/manipulation_exp/manipulation/data/FINAL_DATA/force_control/rdata/';
end

data_directory = dir([data_directory_name,'*.txt']);
num_files = length(data_directory(not([data_directory.isdir])));

for n = 1:1:num_files
    
    data_file_name = strcat(data_directory_name,data_directory(n).name);
    delimiterIn = ' ';
    
    data = importdata(data_file_name,delimiterIn);
    
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

        [hypdiff(n) phyp(n) rhyp(n)] = algorithmPrismatic(data,m1,m2,com1,com2,I_c1,I_c2,g);
        
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

        [hypdiff(n) phyp(n) rhyp(n)] = algorithmRevolute(data,m1,m2,com1,com2,I_c1,I_c2,g);
        
    end
    
end

figure(1);
xlim([ 0 6]); hold on;
plot(1,phyp(1),'+r','LineWidth',1.5); hold on; plot(1,rhyp(1),'xb','LineWidth',1.5);
plot(2,phyp(2),'+r','LineWidth',1.5); hold on; plot(2,rhyp(2),'xb','LineWidth',1.5);
plot(3,phyp(3),'+r','LineWidth',1.5); hold on; plot(3,rhyp(3),'xb','LineWidth',1.5);
plot(4,phyp(4),'+r','LineWidth',1.5); hold on; plot(4,rhyp(4),'xb','LineWidth',1.5);
plot(5,phyp(5),'+r','LineWidth',1.5); hold on; plot(5,rhyp(5),'xb','LineWidth',1.5);
ylabel('Hypothesis Error Value');
legend('Prismatic','Revolute','Location','northwest')
legend boxoff;

if(strcmp(model,'prismatic'))
    set(gca,'XTickLabel',{'', '0.125 N','0.15 N','0.175 N','0.2 N','0.225 N',' '}); hold on;
    print('phypdiff.png','-dpng','-r300');
else
    set(gca,'XTickLabel',{'', '0.51 N','0.52 N','0.53 N','0.54 N','0.55 N',' '}); hold on;
    print('rhypdiff.png','-dpng','-r300');
end