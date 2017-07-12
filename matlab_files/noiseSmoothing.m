function  P = noiseSmoothing(snr, k, f, pose)

  %%Adding Additive White Gaussian Noise
  pos_snr = snr;
  pose_noisy_pos(:,1) = add_awgn_noise(pose(:,1),pos_snr);
  pose_noisy_pos(:,2) = add_awgn_noise(pose(:,2),pos_snr);
  pose_noisy_pos(:,3) = add_awgn_noise(pose(:,3),pos_snr);

  %%Quaternions to Euler Angles
  for i = 1:1:size(pose,1)
       pose_ang(i,:) = quaternion2euler(pose(i,4:7));
  end

  ori_snr = snr;
  pose_noisy_ori(:,1) = add_awgn_noise(pose_ang(:,1),ori_snr);
  pose_noisy_ori(:,2) = add_awgn_noise(pose_ang(:,2),ori_snr);
  pose_noisy_ori(:,3) = add_awgn_noise(pose_ang(:,3),ori_snr);

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
  
  %%Pruning HalfWin data at the begining
  P(:,1:3) = pose_corr_pos(HalfWin+1:end,:);
  P(:,4:7) = quat(HalfWin+1:end,:);
  
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



end