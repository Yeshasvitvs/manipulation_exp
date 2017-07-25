close all;
clear all;
clc;

% pos_vel - Program to compute velocity given x(t)
help pos_vel; % Print header
%@ Enter displacement x(t)
fprintf('Enter x(t) as an equation; for example, ');
fprintf(' 5*t + 8*t^2 + 4*t^3 - 0.25*t^4 ');
xFunction = input(': ','s');
% Read input as a text string
%@ Initialize variables
tMax = 10.0; % Maximum value for time
NPoints = 100; % Number of points to plot
%DeltaT = 1.e-4; % Delta t for estimating derivatives
fprintf('Enter delta t for estimating derivatives; for example, ');
fprintf(' 1.e-4 ');
DeltaT = input('Enter DeltaT in seconds: ');

if(DeltaT<=0.0) 
    DeltaT=1.e-4;
end

%@ Loop over points to evaluate x and v

for iPoint=1:NPoints
   %@ Evaluate x(t) and x(t+DeltaT)
   t = (iPoint-1)/(NPoints-1)*tMax % Time t
   tPlot(iPoint) = t; % Record time for plotting
   temp = t; x_t = eval(xFunction); % Compute x(t)
   t = temp - DeltaT; % Time t-DeltaT
   x_tMinus = eval(xFunction); % Compute x(t-DeltaT)
   t = temp + DeltaT; % Time t+DeltaT
   x_tPlus = eval(xFunction); % Compute x(t+DeltaT)

    %@ Estimate velocity as (x(t+DeltaT) - x(t))/DeltaT
   v = (x_tPlus - x_t)/DeltaT;

    %@ Estimate acceleration as (x(t+DeltaT) - x(t))/DeltaT
   a = (x_tPlus - 2*x_t+x_tMinus)/DeltaT^2;

   %@ Record displacement and velocity for plotting
   xPlot(iPoint) = x_t; % Record x(t)
   vPlot(iPoint) = v; %    Record velocity
   aPlot(iPoint) = a; % Record acceleration
end

%@ Plot displacement, velocity, and acceleration versus time
clf; % Clear graphics figure window
figure(gcf); % Bring figure window forward
subplot(1,3,1); plot(tPlot,xPlot); xlabel('Time (s)'); ylabel('Displacement (m)');
subplot(1,3,2); plot(tPlot,vPlot); xlabel('Time (s)'); ylabel('Velocity (m/s)');
subplot(1,3,3); plot(tPlot,aPlot); xlabel('Time (s)'); ylabel('Acceleration (m/s)'); 