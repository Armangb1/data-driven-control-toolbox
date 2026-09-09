%% Direct / Indirect Self-Tuning Regulator (STR) Example
%  Compares direct and indirect pole-placement STR on a first-order plant.
%
%  See also: ddc.str.DirectSTRController, ddc.str.IndirectSTRController

clear; clc; close all;
fprintf('=== Self-Tuning Regulator Example ===\n\n');

a1 = -0.8; b1 = 1.0;
Tsim = 200; r = ones(1,Tsim);

% Direct STR
dctrl = ddc.str.DirectSTRController('Ac1', -0.5);
yd = 0; ydLog = zeros(1,Tsim);
for k = 1:Tsim
    [u, ~] = dctrl.step(yd, r(k));
    yd = -a1*yd + b1*u + 0.005*randn;
    ydLog(k) = yd;
end

% Indirect STR (general ARX pole placement)
%   plant: A y(k) = B u(k)  =>  y(k) = 0.8*y(k-1) + 1.0*u(k)
%   desired CL pole at z = 0.5 (Am = [1 -0.5]), unity DC gain (Bm = [0.5])
ictrl = ddc.str.IndirectSTRController( ...
    'Na', 1, 'Nb', 1, ...
    'Am', [1 -0.5], 'Bm', [0.5]);
yi = 0; yiLog = zeros(1,Tsim);
for k = 1:Tsim
    [u, ~] = ictrl.step(yi, r(k));
    yi = -a1*yi + b1*u + 0.005*randn;
    yiLog(k) = yi;
end

fprintf('Direct   final y = %.4f\n', ydLog(end));
fprintf('Indirect final y = %.4f\n', yiLog(end));

figure('Name','STR Comparison');
plot(ydLog,'b-','DisplayName','Direct STR'); hold on;
plot(yiLog,'r-','DisplayName','Indirect STR');
plot(r,'k--','DisplayName','Reference');
ylabel('y(k)'); xlabel('Step'); legend; title('STR Comparison'); grid on;
