%% SPSA Online Tuning Example
%  Tunes a scalar gain online using the SPSA optimizer.
%
%  See also: ddc.spsa.SPSAOptimizer

clear; clc; close all;
fprintf('=== SPSA Online Tuning Example ===\n\n');

target = 2.5;
opt = ddc.spsa.SPSAOptimizer('NumParameters', 1, ...
    'InitialTheta', 5, 'ATuning', 0.5, 'CTuning', 0.5);

thetaLog = zeros(1,400);
paramToApply = 5;
lossVal = (paramToApply - target)^2;

for k = 1:400
    [paramToApply, thetaHat] = opt.step(lossVal);
    lossVal = (paramToApply - target)^2;
    thetaLog(k) = thetaHat;
end

fprintf('Estimated theta = %.4f  (target = %.1f)\n', thetaHat, target);

figure('Name','SPSA Convergence');
plot(thetaLog,'b-','LineWidth',1.2); hold on;
yline(target,'r--','LineWidth',1.2);
ylabel('\theta'); xlabel('Iteration'); title('SPSA Parameter Convergence'); grid on;
legend('Estimate','True value');
