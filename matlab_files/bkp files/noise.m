% % %%With Communications System Toolbox
% % t = (0:0.1:10)';
% % x = sin(t);
% % 
% % y = awgn(x,10,'measured');
% % plot(t,[x y])
% % legend('Original Signal','Signal with AWGN')

% % noise_signal = 0.01*randn(1000,6);
% % plot(noise_signal)

% plot(sqrt(0.1)*randn(100,1)+1)

SNR_dB = 5; %Signal to noise ratio in dB
t = 0:.1:10;%time base 
x = sin(t); % Create sawtooth signal.

%Method 1: using custom function ’add_awgn_noise
rng('default');%set the random generator seed to default (for comparison only)
%If the seed above is not set, the results may vary from run-run 
y_custom = add_awgn_noise(x,SNR_dB); %our custom function

% % %Method 2:Using in-Built Function (needs comm toolbox)
% % rng('default');%set the random generator seed to default (for comparison only) 
% % y_inbuilt = awgn(x,SNR_dB,'measured'); % Add white Gaussian noise.

%Plotting results
figure; %%subplot(1,2,1); 
plot(t,x,'b',t,y_custom,'r') % Plot both signals. 
legend('signal','signal with noise'); 
xlabe l(’timebase’);
ylabe l(’y_{custom}’); 
title('custom add\_awgn\_noise function')

% % subplot(1,2,2); plot(t,x,'b',t,y_inbuilt,'r') % Plot both signals. 
% % legend('signal','signal with noise');
% % xlabe l(’timebase’);
% % ylabe l(’y_{inbuilt}’);
% % title('Inbuilt awgn function')

%check for visual linearity between custom function and AWGN inbuilt function 
figur e; 
plot(y_inbuilt,y_custom); %check for linearity between custom function and AWGN inbuilt function
title('output of custom function Vs in-built awgn f');
xlabe l(’y_{inbuilt}’);ylabe l(’y_{custom}’);

