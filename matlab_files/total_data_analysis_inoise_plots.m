close all;
clear all;
clc;

model = 'prismatic';

if(strcmp(model,'prismatic'))
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/force_control/inoise_pdata/';
else
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/force_control/inoise_rdata/';
end

mat_directory = dir([mat_data_directory_name,'*.mat']);
num_files = length(mat_directory(not([mat_directory.isdir])));

sorted_list = [mat_directory(:).datenum].'; 
[sorted_list,sorted_list] = sort(sorted_list);
sorted_list = {mat_directory(sorted_list).name}; % Cell array of names in order by datenum.


for n=1:1:num_files

    mat_file_name = strcat(mat_data_directory_name,char(sorted_list(n)));
    data = importdata(mat_file_name);

    s = sign (data.hypdiff);
    ipositif = sum (s (:) == 1);
    inegatif = sum (s (:) == - 1);
    
    if(strcmp(model,'prismatic'))
        hd_err(n) = (ipositif/data.inoise_check_count)*100;
    else
        hd_err(n) = (inegatif/data.inoise_check_count)*100;
    end
    
% %     Phyp(n) = data.phyp;
% %     Rhpy(n) = data.rhyp;

end

%%Reshaping into matrices
hypdiff_err = reshape(hd_err,data.inoise_run_size,num_files/data.inoise_run_size);
% % phyp = reshape(Phyp,data.inoise_run_size,num_files/data.inoise_run_size);
% % rhyp = reshape(Rhyp,data.inoise_run_size,num_files/data.inoise_run_size);

figure(1);
xindex = [0:1:size(hypdiff_err,2)]; xlim([0 size(hypdiff_err,1)]); hold on;
plot(xindex,hypdiff_err(:,1),'-*b', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,2),'-xr', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,3),'-og', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,4),'-+m', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,5),'-^k', 'LineWidth',1.5); hold on;
ylabel('Err%'); xlabel('Percentage Change');

if(strcmp(model,'prismatic'))
    
    l = legend('\begin{tabular}{p{0.5cm}r}0.125&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.15&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.175&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.2&N\end{tabular}',...
               '\begin{tabular}{p{0.5cm}r}0.22&N\end{tabular}',...
               'Location','southeast');
    set(l,'interpreter','latex');
    legend boxoff;
    set(gca,'FontSize',10.5,'LineWidth',1);
% %     set(gcf,'Position',get(0,'Screensize'));
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/inoise_phypdiff_err.png','-dpng','-r300');

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
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/inoise_rhypdiff_err.png','-dpng','-r300');

end

