%%Prismatic Motion
figure;
xlim([0 0.05]); ylim([-25 0]); hold on;
plot(0.007,-4.7974,'*r','LineWidth',1.5); hold on;
plot(0.014,-9.5361,'+g','LineWidth',1.5); hold on;
plot(0.022,-15.4107,'xb','LineWidth',1.5); hold on;
plot(0.03,-19.054,'om','LineWidth',1.5); hold on;
plot(0.037,-20.8733,'^k','LineWidth',1.5); hold on;
set(gca,'XTick',[0 0.007 0.014 0.022 0.03 0.037 0.043 0.05] )
xlabel('meters/second'); ylabel('Hypothesis Difference');
legend('0.007 m/s','0.014 m/s', '0.022 m/s', '0.030 m/s','0.037 m/s')
legend boxoff;
print('wonoise_pdiff.png','-dpng','-r300');


% % %%Revolute Motion
% % figure;
% % xlim([0 30]); ylim([0 80]); hold on;
% % plot(5,19.5847,'*r','LineWidth',1.5); hold on;
% % plot(10,36.7964,'+g','LineWidth',1.5); hold on;
% % plot(15,51.3222,'xb','LineWidth',1.5); hold on;
% % plot(20,62.4513,'om','LineWidth',1.5); hold on;
% % plot(25,74.6761,'^k','LineWidth',1.5); hold on;
% % xlabel('degrees/second'); ylabel('Hypothesis Difference');
% % legend('5 deg/s','10 deg/s', '15 deg/s', '20 deg/s','25 deg/s', 'Location','northwest');
% % legend boxoff;
% % print('wonoise_rdiff.png','-dpng','-r300');


