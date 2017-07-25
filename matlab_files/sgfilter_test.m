close all;
clear all;
clc;

K = 4;                 % Order of polynomial fit
F = 21;                % Window length
[b,g] = SgolayWrapper(K,F);   % Calculate S-G coefficients

dx = .2;
xLim = 200;
x = 0:dx:xLim-1;

y = 5*sin(0.4*pi*x)+randn(size(x));  % Sinusoid with noise

HalfWin  = ((F+1)/2) -1;
for n = (F+1)/2:996-(F+1)/2,
  % Zero-th derivative (smoothing only)
  SG0(n) =   dot(g(:,1), y(n - HalfWin: n + HalfWin));

  % 1st differential
  SG1(n) =   dot(g(:,2), y(n - HalfWin: n + HalfWin));

  % 2nd differential
  SG2(n) = 2*dot(g(:,3)', y(n - HalfWin: n + HalfWin))';
end

SG1 = SG1/dx;         % Turn differential into derivative
SG2 = SG2/(dx*dx);    % and into 2nd derivative

% Scale the "diff" results
DiffD1 = (diff(y(1:length(SG0)+1)))/ dx;    
DiffD2 = (diff(diff(y(1:length(SG0)+2)))) / (dx*dx);

subplot(3,1,1);
plot([y(1:length(SG0))', SG0'])
legend('Noisy Sinusoid','S-G Smoothed sinusoid')

subplot(3, 1, 2);
plot([DiffD1',SG1'])
legend('Diff-generated 1st-derivative', 'S-G Smoothed 1st-derivative')

subplot(3, 1, 3);
plot([DiffD2',SG2'])
legend('Diff-generated 2nd-derivative', 'S-G Smoothed 2nd-derivative')
