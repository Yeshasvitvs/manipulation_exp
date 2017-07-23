close all;
clear all;
clc;

model = 'revolute';

if(strcmp(model,'prismatic'))
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/pnoise_pdata/';
else
    mat_data_directory_name = '/home/yeshi/projects/manipulation_exp/matlab_files/FINAL_DATA_MAT/pnoise_rdata/';
end

mat_directory = dir([mat_data_directory_name,'*.mat']);
num_files = length(mat_directory(not([mat_directory.isdir])));

for n=1:1:num_files

    mat_file_name = strcat(mat_data_directory_name,mat_directory(n).name);
    data = importdata(mat_file_name);

    s1 = sign (data.value);
    ipositif1 = sum (s1 (:) == 1);
    inegatif1 = sum (s1 (:) == - 1);
    pv_err(n) = (ipositif1/data.pnoise_check_count)*100;
    rv_err(n) = (inegatif1/data.pnoise_check_count)*100;

    s2 = sign (data.hypdiff);
    ipositif2 = sum (s2 (:) == 1);
    inegatif2 = sum (s2 (:) == - 1);
    phd_err(n) = (ipositif2/data.pnoise_check_count)*100;
    rhd_err(n) = (inegatif2/data.pnoise_check_count)*100;

end

%%Reshaping into matrices
pvalue_err = reshape(pv_err,data.pnoise_run_size,num_files/data.pnoise_run_size);
rvalue_err = reshape(rv_err,data.pnoise_run_size,num_files/data.pnoise_run_size);

phypdiff_err = reshape(phd_err,data.pnoise_run_size,num_files/data.pnoise_run_size);
rhypdiff_err = reshape(rhd_err,data.pnoise_run_size,num_files/data.pnoise_run_size);

if(strcmp(model,'prismatic'))
    
    %%Prismatic Plots
    figure(1);
    xindex = [1:1:size(pvalue_err,2)]; xlim([0 size(pvalue_err,1)+1]); hold on;
    plot(xindex,pvalue_err(:,1),'-*b', 'LineWidth',1.5); hold on;
    plot(xindex,pvalue_err(:,2),'-xr', 'LineWidth',1.5); hold on;
    plot(xindex,pvalue_err(:,3),'-og', 'LineWidth',1.5); hold on;
    plot(xindex,pvalue_err(:,4),'-+m', 'LineWidth',1.5); hold on;
    plot(xindex,pvalue_err(:,5),'-^k', 'LineWidth',1.5); hold on;
    ylabel('Err%'); xlabel('Standard Deviation');
    legend('0.007 m/s','0.014 m/s', '0.022 m/s', '0.030 m/s','0.037 m/s','Location','southeast');
    legend boxoff;
    set(gca,'XTickLabel',{'', '(\sigma_{p_1}, \sigma_{o_1})',...
                              '(\sigma_{p_2}, \sigma_{o_2})',...
                              '(\sigma_{p_3}, \sigma_{o_3}',...
                              '(\sigma_{p_4}, \sigma_{o_4})',...
                              '(\sigma_{p_5}, \sigma_{o_5})',...
                              ' '}); hold on;
    set(gca,'FontSize',10.5,'LineWidth',1.2)
% %     set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/pnoise_pvalue_err.png','-dpng','-r300');
    
    figure(2);
    xindex = [1:1:size(pvalue_err,2)]; xlim([0 size(pvalue_err,1)+1]); hold on;
    plot(xindex,phypdiff_err(:,1),'-*b', 'LineWidth',1.5); hold on;
    plot(xindex,phypdiff_err(:,2),'-xr', 'LineWidth',1.5); hold on;
    plot(xindex,phypdiff_err(:,3),'-og', 'LineWidth',1.5); hold on;
    plot(xindex,phypdiff_err(:,4),'-+m', 'LineWidth',1.5); hold on;
    plot(xindex,phypdiff_err(:,5),'-^k', 'LineWidth',1.5); hold on;
    ylabel('Err%'); xlabel('Standard Deviation');
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
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/pnoise_phypdiff_err.png','-dpng','-r300');
    
else
    
    %%Revolute Plots
    figure(1);
    xindex = [1:1:size(pvalue_err,2)]; xlim([0 size(pvalue_err,1)+1]); hold on;
    plot(xindex,rvalue_err(:,1),'-*b', 'LineWidth',1.5); hold on;
    plot(xindex,rvalue_err(:,2),'-xr', 'LineWidth',1.5); hold on;
    plot(xindex,rvalue_err(:,3),'-og', 'LineWidth',1.5); hold on;
    plot(xindex,rvalue_err(:,4),'-+m', 'LineWidth',1.5); hold on;
    plot(xindex,rvalue_err(:,5),'-^k', 'LineWidth',1.5); hold on;
    ylabel('Err%'); xlabel('Standard Deviation');
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
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/pnoise_rvalue_err.png','-dpng','-r300');
    
    
    figure(2);
    xindex = [1:1:size(pvalue_err,2)]; xlim([0 size(pvalue_err,1)+1]); hold on;
    plot(xindex,rhypdiff_err(:,1),'-*b', 'LineWidth',1.5); hold on;
    plot(xindex,rhypdiff_err(:,2),'-xr', 'LineWidth',1.5); hold on;
    plot(xindex,rhypdiff_err(:,3),'-og', 'LineWidth',1.5); hold on;
    plot(xindex,rhypdiff_err(:,4),'-+m', 'LineWidth',1.5); hold on;
    plot(xindex,rhypdiff_err(:,5),'-^k', 'LineWidth',1.5); hold on;
    ylabel('Err%'); xlabel('Standard Deviation');
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
    print('/home/yeshi/projects/manipulation_exp/matlab_files/figures/pnoise_rhypdiff_err.png','-dpng','-r300');

end

