function  P = poseNoiseSmoothing(pos_std, ori_std, k, f, pose)
  
  pose_noisy_pos(:,1:3) = pose(:,1:3) + pos_std*randn(size(pose,1),3);
 
  %%Quaternions to Euler Angles
  for i = 1:1:size(pose,1)
       pose_ang(i,:) = quaternion2euler(pose(i,4:7));
  end

  pose_noisy_ori(:,1:3) = pose_ang(:,1:3) + ori_std*randn(size(pose,1),3);

  %%SG Filter Smoothing of Noisy Signal
  K = k;                 % Order of polynomial fit
  F = f;                 % Window length
  [b,g] = SgolayWrapper(K,F);
  HalfWin  = ((F+1)/2) -1;
  data_size = size(pose,1);
  for n = (F+1)/2:data_size-(F+1)/2,
     
      % Zero-th derivative (smoothing only)
      pose_corr_pos(n,1) =   dot(g(:,1), pose_noisy_pos(n - HalfWin: n + HalfWin,1));
      pose_corr_pos(n,2) =   dot(g(:,1), pose_noisy_pos(n - HalfWin: n + HalfWin,2));
      pose_corr_pos(n,3) =   dot(g(:,1), pose_noisy_pos(n - HalfWin: n + HalfWin,3));
    
      pose_corr_ori(n,1) =   dot(g(:,1), pose_noisy_ori(n - HalfWin: n + HalfWin,1));
      pose_corr_ori(n,2) =   dot(g(:,1), pose_noisy_ori(n - HalfWin: n + HalfWin,2));
      pose_corr_ori(n,3) =   dot(g(:,1), pose_noisy_ori(n - HalfWin: n + HalfWin,3));
    
  end

  %%Euler Angles to Quaternions
  for i = 1:1:size(pose_corr_ori,1)
       quat(i,:) = euler2quaternion(pose_corr_ori(i,:));
  end
  
  %%Pruning HalfWin data at the begining and the end
  P(:,1:3) = pose_corr_pos(HalfWin+1:end,:);
  P(:,4:7) = quat(HalfWin+1:end,:);
  
% %   %%This is without HalfWin data pruning
% %   %%Euler Angles to Quaternions
% %   for i = 1:1:size(pose_noisy_ori,1)
% %        quat(i,:) = euler2quaternion(pose_noisy_ori(i,:));
% %   end
% %   
% %   P(:,1:3) = pose_noisy_pos;
% %   P(:,4:7) = quat;
  
  
% %   %%Plotting Smoothed Signal vs Original
% %   figure(1); 
% %   plot(pose_corr_pos(:,1)); hold on;
% %   plot(pose(:,1));
% % 
% %   figure(2); 
% %   plot(pose_corr_pos(:,2)); hold on;
% %   plot(pose(:,2));
% % 
% %   figure(3); 
% %   plot(pose_corr_pos(:,3)); hold on;
% %   plot(pose(:,3));
% % 
% %   figure(4); 
% %   plot(pose_corr_ori(:,1)); hold on;
% %   plot(pose_ang(:,1));
% % 
% %   figure(5);
% %   plot(pose_corr_ori(:,2)); hold on;
% %   plot(pose_ang(:,2));
% % 
% %   figure(6); 
% %   plot(pose_corr_ori(:,3)); hold on;
% %   plot(pose_ang(:,3));

% %   %%This is to plot prismatic noise - onetestpdata5.txt
% %   figure(7);
% %   h1 = subplot(3,1,1);plot(pose(:,1)); xlabel('sec'); ylabel('meters'); hold on;
% %   text(475, -0.265, '(a)');
% %   h2 = subplot(3,1,2);plot(pose_noisy_pos(:,1)); xlabel('sec'); ylabel('meters'); hold on;
% %   text(475, -0.265, '(b)');
% %   h3 = subplot(3,1,3); plot(pose_corr_pos(:,1)); xlabel('sec'); ylabel('meters'); ylim([-0.05 0.3]); hold on;
% %   text(475, -0.265, '(c)');
% %   linkaxes([h3,h1,h2],'xy');
% %   print('pnoise.png','-dpng','-r300');

% %   %%This is to plot revolute noise 
% %   %%Joint Angle radians to angles
% %   rad2angconv = (180/pi);
% %   figure(8);
% %   h1 = subplot(3,1,1);plot(rad2angconv.*pose_ang(:,3)); xlabel('sec'); ylabel('degrees');
% % %   text(765, -0, '(a)');
% %   h2 = subplot(3,1,2);plot(rad2angconv.*pose_noisy_ori(:,3)); xlabel('sec'); ylabel('degrees'); hold on;
% % %   text(765, -0.122, '(b)');
% %   h3 = subplot(3,1,3); plot(rad2angconv.*pose_corr_ori(:,3)); xlabel('sec'); ylabel('degrees'); ylim([10 140]); hold on;
% % %   text(765, -0.122, '(c)');
% %   linkaxes([h3,h1,h2],'xy');
% %   print('rnoise.png','-dpng','-r300');

end