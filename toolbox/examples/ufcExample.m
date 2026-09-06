%% Unfalsified Adaptive Switching Control (UFC) Example
%  Demonstrates the candidate-controller bank + switching supervisor on
%  a first-order plant, using continuous-time PI candidate controllers
%  that are auto-discretized internally.
%
%  See also: ddc.ufc.UnfalsifiedSwitchingController,
%            ddc.ufc.CandidateControllerBank

clear; clc; close all;
fprintf('=== Unfalsified Switching Control Example ===\n\n');

Ts = 0.1;

% Continuous-time PI candidates:  C(s) = Kp + Ki/s
% These are auto-discretized to zoh internally.
C1 = tf([0.2, 0.05], [1, 0]);   % Kp=0.2, Ki=0.05
C2 = tf([0.5, 0.1],  [1, 0]);   % Kp=0.5, Ki=0.1
C3 = tf([1.0, 0.2],  [1, 0]);   % Kp=1.0, Ki=0.2
Controllers = [C1, C2, C3];

bank = ddc.ufc.CandidateControllerBank('Controllers', Controllers, ...
       'SampleTime', Ts);
sw   = ddc.ufc.UnfalsifiedSwitchingController('Controllers', Controllers, ...
       'SampleTime', Ts, 'ForgettingFactor', 0.9);

a1 = -0.8; b1 = 1.0;
Tsim = 200; y = 0;
yLog = zeros(1,Tsim); idxLog = zeros(1,Tsim);

for k = 1:Tsim
    uCand = bank.step(1, y);
    [uSel, idx] = sw.step(uCand, y);
    y = -a1*y + b1*uSel + 0.005*randn;
    yLog(k) = y; idxLog(k) = idx;
end

fprintf('Final y = %.4f, active candidate = %d\n', yLog(end), idxLog(end));

figure('Name','UFC Step Response');
subplot(2,1,1); plot(yLog,'b-'); hold on; plot(ones(1,Tsim),'r--');
ylabel('y'); title('Unfalsified Switching (continuous PI, auto-discretized)'); grid on;
subplot(2,1,2); stairs(idxLog,'k-');
ylabel('Active index'); xlabel('Step'); grid on;
