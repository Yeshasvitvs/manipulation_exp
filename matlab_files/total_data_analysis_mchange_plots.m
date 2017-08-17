close all;
clear all;
clc;

model = 'revolute';

mat_data_directory_path = strcat(char(pwd),'/FINAL_DATA_MAT/force_control/');
if(strcmp(model,'prismatic'))
    mat_data_directory_name = strcat(mat_data_directory_path,'mchange_pdata/');
else
    mat_data_directory_name = strcat(mat_data_directory_path,'mchange_rdata/');
end

mat_directory = dir([mat_data_directory_name,'*.mat']);
num_files = length(mat_directory(not([mat_directory.isdir])));

% Engine
sorted_list = [mat_directory(:).datenum].'; 
[sorted_list,sorted_list] = sort(sorted_list);
sorted_list = {mat_directory(sorted_list).name}; % Cell array of names in order by datenum.

for n=1:1:num_files

    mat_file_name = strcat(mat_data_directory_name,char(sorted_list(n)));
    data = importdata(mat_file_name);

    hd(n) = data.hypdiff;
    
    Phyp(n) = data.phyp;
    Rhyp(n) = data.rhyp;
    
end

%%Reshaping into matrices
hypdiff = reshape(hd,data.mchange_run_size,num_files/data.mchange_run_size);
phyp = reshape(Phyp,data.mchange_run_size,num_files/data.mchange_run_size);
rhyp = reshape(Rhyp,data.mchange_run_size,num_files/data.mchange_run_size);

figure(1);
xindex = [-data.mchange_value:data.mchange_step:data.mchange_value]; hold on;
plot(xindex.*100,hypdiff(:,1),'-*b', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,2),'-+r', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,3),'-sg', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,4),'-^m', 'LineWidth',1); hold on;
plot(xindex.*100,hypdiff(:,5),'-ok', 'LineWidth',1); hold on;
ylabel('Hypothesis Difference'); xlabel('Percentage Change');
    
if(strcmp(model,'prismatic'))
    
    l = legend('\begin{tabular}{p{0.5cm}r}0.125&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.15&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.175&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.2&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.122&N\end{tabular}',...
               'Location','southwest');
    set(l,'interpreter','latex');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1);
% %     set(gcf,'Position',get(0,'Screensize'));
    print('figures/mchange_phypdiff_err.png','-dpng','-r300');

else
   
    l = legend('\begin{tabular}{p{0.5cm}r}0.51&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.52&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.53&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.54&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.55&N\end{tabular}',...
               'Location','southwest');
    set(l,'interpreter','latex');
    legend('0.51 N','0.52 N', '0.53 N', '0.54 N','0.55 N','Location','northwest');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1);
% %     set(gcf,'Position',get(0,'Screensize'));
    print('figures/mchange_rhypdiff_err.png','-dpng','-r300');

end

figure(2);
xindex = [1:1:size(phyp,1)]; xlim([0 size(phyp,1)]); hold on;
plot(xindex,phyp(:,1)./size(phyp,1),'-*b', 'LineWidth',1.5); hold on;
plot(xindex,rhyp(:,1)./size(phyp,1),'-xr', 'LineWidth',1.5); hold on;
ylabel('Hypothesis Error Value'); xlabel('Percentage Change');

% % figure(3);
% % xindex = [1:1:size(phyp,1)]; xlim([0 size(phyp,1)]); hold on;
% % plot(xindex,phyp(:,2)./size(phyp,1),'-*b', 'LineWidth',1.5); hold on;
% % plot(xindex,rhyp(:,2)./size(phyp,1),'-xr', 'LineWidth',1.5); hold on;
% % ylabel('Hypothesis Error Value'); xlabel('Percentage Change');
% % 
% % figure(4);
% % xindex = [1:1:size(phyp,1)]; xlim([0 size(phyp,1)]); hold on;
% % plot(xindex,phyp(:,3)./size(phyp,1),'-*b', 'LineWidth',1.5); hold on;
% % plot(xindex,rhyp(:,3)./size(phyp,1),'-xr', 'LineWidth',1.5); hold on;
% % ylabel('Hypothesis Error Value'); xlabel('Percentage Change');
% % 
% % figure(5);
% % xindex = [1:1:size(phyp,1)]; xlim([0 size(phyp,1)]); hold on;
% % plot(xindex,phyp(:,4)./size(phyp,1),'-*b', 'LineWidth',1.5); hold on;
% % plot(xindex,rhyp(:,4)./size(phyp,1),'-xr', 'LineWidth',1.5); hold on;
% % ylabel('Hypothesis Error Value'); xlabel('Percentage Change');
% % 
% % figure(6);
% % xindex = [1:1:size(phyp,1)]; xlim([0 size(phyp,1)]); hold on;
% % plot(xindex,phyp(:,5)./size(phyp,1),'-*b', 'LineWidth',1.5); hold on;
% % plot(xindex,rhyp(:,5)./size(phyp,1),'-xr', 'LineWidth',1.5); hold on;
% % ylabel('Hypothesis Error Value'); xlabel('Percentage Change');
