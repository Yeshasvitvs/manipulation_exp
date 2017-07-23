close all;
clear all;
clc; 

% % filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/onetestpdata5.txt';
filename = '/home/yeshi/projects/manipulation_exp/manipulation/data/onetestrdata2.txt';
data = importdata(filename);

%%Noise and Filtering
K = 3;  % Order of polynomial fit
F = 121;  % Window length
HalfWin  = ((F+1)/2) -1;
SNR = 1000;  %Signal-Noise ratio

%%Gaussian Noise Standard Deviation
POS_STD = 0.05; %Position - Meters
ORI_STD = 5*(pi/180); %Orientation - Radians

p2 = data(:,9:15);
poseNoiseSmoothing(SNR, POS_STD, ORI_STD, K, F, p2);