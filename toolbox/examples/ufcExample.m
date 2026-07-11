%% Unfalsified Adaptive Switching Control (UFC) Example
%  Demonstrates the P-bank masked subsystem on a first-order plant.
%
%  See also: ddc.ufc.UnfalsifiedSwitchingController,
%            ddc.ufc.CandidateControllerBank

clear; clc; close all;
fprintf('=== Unfalsified Switching Control Example ===\n\n');

% All gains keep the CL pole (0.8-0.5*K) inside the unit circle:
%   K=0.2 -> pole 0.7, K=0.5 -> pole 0.55, K=1 -> pole 0.3
Gains = [0.2; 0.5; 1];
bank  = ddc.ufc.CandidateControllerBank('Gains', Gains);
sw    = ddc.ufc.UnfalsifiedSwitchingController('Gains', Gains, ...
        'MinDwellSteps', 2, 'ForgettingFactor', 0.9, 'ResetCostOnSwitch', true);

a1 = -0.8; b1 = 1.0;
Tsim = 100; y = 0;
yLog = zeros(1,Tsim); idxLog = zeros(1,Tsim);

for k = 1:Tsim
    uCand = bank.step(1, y);
    [uSel, idx] = sw.step(uCand);
    y = -a1*y + b1*uSel + 0.005*randn;
    yLog(k) = y; idxLog(k) = idx;
end

fprintf('Final y = %.4f, active gain = %.2f\n', yLog(end), Gains(idxLog(end)));

figure('Name','UFC Step Response');
subplot(2,1,1); plot(yLog,'b-'); hold on; plot(ones(1,Tsim),'r--');
ylabel('y'); title('Unfalsified Switching'); grid on;
subplot(2,1,2); stairs(idxLog,'k-');
ylabel('Active index'); xlabel('Step'); grid on;
