%% Model-Free Adaptive Control (MFAC) Example
%  Demonstrates CFDL-MFAC on a first-order nonlinear plant.
%
%  See also: ddc.mfac.MFACController

clear; clc; close all;
fprintf('=== MFAC Example ===\n\n');

%% Plant: y(k) = 0.8*y(k-1) + u(k-1) + 0.1*sin(y(k-1))  (nonlinear)
a1 = -0.8;
b1 = 1.0;

ctrl = ddc.mfac.MFACController('PhiInit', 1);

Tsim = 200;
r    = ones(1, Tsim);
y    = 0;
yLog = zeros(1, Tsim);
uLog = zeros(1, Tsim);

for k = 1:Tsim
    u = ctrl.step(y, r(k));
    y = -a1*y + b1*u + 0.1*sin(y) + 0.005*randn;
    yLog(k) = y;
    uLog(k) = u;
end

fprintf('Final y = %.4f  (target = 1.0)\n', yLog(end));

figure('Name', 'MFAC Step Response');
subplot(2,1,1);
plot((1:Tsim)*0.1, yLog, 'b-'); hold on;
plot((1:Tsim)*0.1, r, 'r--');
ylabel('y(k)'); title('MFAC Closed-Loop'); legend('y','r'); grid on;
subplot(2,1,2);
stairs((1:Tsim)*0.1, uLog, 'Color', [0 0.5 0]);
ylabel('u(k)'); xlabel('Time (s)'); grid on;
