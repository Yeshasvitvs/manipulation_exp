close all;
clear all;
clc;

model = 'prismatic';

mat_data_directory_path = strcat(char(pwd),'/FINAL_DATA_MAT/force_control/');
if(strcmp(model,'prismatic'))
    mat_data_directory_name = strcat(mat_data_directory_path,'pnoise_pdata/');
else
    mat_data_directory_name = strcat(mat_data_directory_path,'pnoise_rdata/');
end

mat_directory = dir([mat_data_directory_name,'*.mat']);
num_files = length(mat_directory(not([mat_directory.isdir])));

for n=1:1:num_files

    mat_file_name = strcat(mat_data_directory_name,mat_directory(n).name);
    data = importdata(mat_file_name);
   
    s = sign (data.hypdiff);
    ipositif = sum (s (:) == 1);
    inegatif = sum (s (:) == - 1);
    
    if(strcmp(model,'prismatic'))
        hd_err(n) = (ipositif/data.pnoise_check_count)*100;
    else
        hd_err(n) = (inegatif/data.pnoise_check_count)*100;
    end
end

%%Reshaping into matrices
hypdiff_err = reshape(hd_err,data.pnoise_run_size,num_files/data.pnoise_run_size);

figure(1);
xindex = [1:1:size(hypdiff_err,1)]; xlim([0 size(hypdiff_err,1)+1]); hold on;
plot(xindex,hypdiff_err(:,1),'-*b', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,2),'-xr', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,3),'-og', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,4),'-+m', 'LineWidth',1.5); hold on;
plot(xindex,hypdiff_err(:,5),'-^k', 'LineWidth',1.5); hold on;
ylabel('Err%'); xlabel('Standard Deviation');

if(strcmp(model,'prismatic'))
    
    legend('0.007 m/s','0.014 m/s', '0.022 m/s', '0.030 m/s','0.037 m/s','Location','northeast');
    legend boxoff;
    set(gca,'XTickLabel',{'', '(\sigma_{p_1}, \sigma_{o_1})',...
                              '(\sigma_{p_2}, \sigma_{o_2})',...
                              '(\sigma_{p_3}, \sigma_{o_3}',...
                              '(\sigma_{p_4}, \sigma_{o_4})',...
                              '(\sigma_{p_5}, \sigma_{o_5})',...
                              ' '}); hold on;
    set(gca,'FontSize',10.5,'LineWidth',1.2)
% %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('figures/pnoise_phypdiff_err.png','-dpng','-r300');
    
else
    
    legend('5 deg/s','10 deg/s', '15 deg/s', '20 deg/s','25 deg/s','Location','northwest');
    legend boxoff;
    set(gca,'XTickLabel',{'', '(\sigma_{p_1}, \sigma_{o_1})',...
                              '(\sigma_{p_2}, \sigma_{o_2})',...
                              '(\sigma_{p_3}, \sigma_{o_3}',...
                              '(\sigma_{p_4}, \sigma_{o_4})',...
                              '(\sigma_{p_5}, \sigma_{o_5})',...
                              ' '}); hold on;
    set(gca,'FontSize',10.5,'LineWidth',1.2)
% %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('figures/pnoise_rhypdiff_err.png','-dpng','-r300');

end
