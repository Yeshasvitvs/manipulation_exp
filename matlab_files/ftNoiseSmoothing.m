function  w = ftNoiseSmoothing(snr, k, f, wrench)

  %%Adding Additive White Gaussian Noise
  wrench_noisy(:,1) = add_awgn_noise(wrench(:,1),snr);
  wrench_noisy(:,2) = add_awgn_noise(wrench(:,2),snr);
  wrench_noisy(:,3) = add_awgn_noise(wrench(:,3),snr);
  wrench_noisy(:,4) = add_awgn_noise(wrench(:,4),snr);
  wrench_noisy(:,5) = add_awgn_noise(wrench(:,5),snr);
  wrench_noisy(:,6) = add_awgn_noise(wrench(:,6),snr);

  %%SG Filter Smoothing of Noisy Signal
  K = k;                 % Order of polynomial fit
  F = f;                 % Window length
  [b,g] = SgolayWrapper(K,F);
  HalfWin  = ((F+1)/2) -1;
  data_size = size(wrench,1);
  for n = (F+1)/2:data_size-(F+1)/2,
      % Zero-th derivative (smoothing only)
      wrench_corr(n,1) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,1));
      wrench_corr(n,2) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,2));
      wrench_corr(n,3) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,3));
      wrench_corr(n,4) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,4));
      wrench_corr(n,5) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,5));
      wrench_corr(n,6) =   dot(g(:,1), wrench_noisy(n - HalfWin: n + HalfWin,6));
      
  end
  
  %%Pruning HalfWin data at the begining
  w = wrench_corr(HalfWin+1:end,:);

  
% %   %%Plotting Smoothed Signal vs Original
% %   figure(1); 
% %   plot(wrench_corr(:,1)); hold on;
% %   plot(wrench(:,1));
% % 
% %   figure(2); 
% %   plot(wrench_corr(:,2)); hold on;
% %   plot(wrench(:,2));
% % 
% %   figure(3); 
% %   plot(wrench_corr(:,3)); hold on;
% %   plot(wrench(:,3));
% % 
% %   figure(4); 
% %   plot(wrench_corr(:,1)); hold on;
% %   plot(wrench(:,1));
% % 
% %   figure(5);
% %   plot(wrench_corr(:,2)); hold on;
% %   plot(wrench(:,2));
% % 
% %   figure(6); 
% %   plot(wrench_corr(:,3)); hold on;
% %   plot(wrench(:,3));


end