close all;
clear all;
clc;

model = 'revolute';

if(strcmp(model,'prismatic'))
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/mchange_pdata/';
else
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/mchange_rdata/';
end

mat_directory = dir([mat_data_directory_name,'*.mat']);
num_files = length(mat_directory(not([mat_directory.isdir])));

for n=1:1:num_files

    mat_file_name = strcat(mat_data_directory_name,mat_directory(n).name);
    data = importdata(mat_file_name);
    
    v(n) = data.value;
    hd(n) = data.hypdiff;

end

%%Reshaping into matrices
value = reshape(v,data.mchange_run_size,num_files/data.mchange_run_size);
hypdiff = reshape(hd,data.mchange_run_size,num_files/data.mchange_run_size);

figure(1);
xindex = [-data.mchange_value:data.mchange_step:data.mchange_value]; hold on;
plot(xindex.*100,value(:,1),'-b', 'LineWidth',1); hold on;
plot(xindex.*100,value(:,2),'-r', 'LineWidth',1); hold on;
plot(xindex.*100,value(:,3),'-g', 'LineWidth',1); hold on;
plot(xindex.*100,value(:,4),'-m', 'LineWidth',1); hold on;
plot(xindex.*100,value(:,5),'-k', 'LineWidth',1); hold on;
ylabel('Hypothesis  Difference'); xlabel('Percentage Change');

if(strcmp(model,'prismatic'))
    
    legend('0.007 m/s','0.014 m/s', '0.022 m/s', '0.030 m/s','0.037 m/s','Location','southeast');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1)
    % %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/mchange_pvalue_err.png','-dpng','-r300');
else
    legend('1 deg/s','2 deg/s', '3 deg/s', '4 deg/s','5 deg/s','Location','northeast');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1.2)
    % %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/mchange_rvalue_err.png','-dpng','-r300');
end


figure(2);
xindex = [-data.mchange_value:data.mchange_step:data.mchange_value]; hold on;
plot(xindex.*100,hypdiff(:,1),'-b', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,2),'-r', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,3),'-g', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,4),'-m', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,5),'-k', 'LineWidth',1); hold on;
ylabel('Hypothesis Difference'); xlabel('Percentage Change');
    
if(strcmp(model,'prismatic'))
    
    legend('0.007 m/s','0.014 m/s', '0.022 m/s', '0.030 m/s','0.037 m/s','Location','southeast');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1)
    % %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/mchange_phypdiff_err.png','-dpng','-r300');

else
   
    legend('1 deg/s','2 deg/s', '3 deg/s', '4 deg/s','5 deg/s','Location','northeast');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1)
    % %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/mchange_rhypdiff_err.png','-dpng','-r300');

end
    