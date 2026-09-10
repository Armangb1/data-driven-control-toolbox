%% Data-Enabled Predictive Control (DeePC) -- End-to-End Example
%  Demonstrates the complete DeePC workflow: offline data collection,
%  offline Hankel construction / persistency-of-excitation check, and
%  online receding-horizon control against a simple discrete plant.
%
%  See also: ddc.deepc.DeePCController, ddc.deepc.deepcDesign,
%            ddc.common.checkPersistencyExcitation

clear; clc; close all;
fprintf('=== DeePC End-to-End Example ===\n\n');

%% 1. Offline data collection (open-loop)
%    Plant:  y(k) = 0.8*y(k-1) + u(k-1) + 0.01*noise
Ts = 0.1;
a1 = -0.8;
b1 = 1.0;

Toffline = 200;
rng(42);
uOffline = 2*(rand(1, Toffline) - 0.5);   % PRBS-like excitation
yOffline = zeros(1, Toffline);
for k = 2:Toffline
    yOffline(k) = -a1*yOffline(k-1) + b1*uOffline(k-1) + 0.01*randn;
end

fprintf('Offline data: T = %d samples collected.\n', Toffline);

%% 2. Persistency-of-excitation check
Tini = 20;  % past window length (must be large enough for g to be well-constrained)
N    = 10;  % prediction horizon
L    = Tini + N;  % required Hankel depth

[isPE, r, rReq] = ddc.common.checkPersistencyExcitation(uOffline, L);
if isPE
    peStatus = 'OK';
else
    peStatus = 'FAIL';
end
fprintf('PE check (order %d): rank = %d, required = %d -> %s\n', ...
    L, r, rReq, peStatus);

%% 3. Build offline design data
design = ddc.deepc.deepcDesign(uOffline, yOffline, Tini, N, 'AssumedOrder', 2);
fprintf('Offline design: Up=[%dx%d] Uf=[%dx%d] Yp=[%dx%d] Yf=[%dx%d]\n', ...
    size(design.Up,1), size(design.Up,2), ...
    size(design.Uf,1), size(design.Uf,2), ...
    size(design.Yp,1), size(design.Yp,2), ...
    size(design.Yf,1), size(design.Yf,2));

%% 4. Online closed-loop simulation
ctrl = ddc.deepc.DeePCController( ...
    'DataU', uOffline, 'DataY', yOffline, ...
    'Tini', Tini, 'N', N, ...
    'Q', 10, 'R', 1, 'LambdaG', 1, 'LambdaY', 1e4);

% Collect initial open-loop data for the DeePC initial-condition window.
% Without this, zero history forces the QP constraint Up*g=0,
% preventing the controller from moving the plant.
rng(1);
y0 = 0;
yHist = zeros(1, Tini);
for k = 1:Tini
    uInit = 2*(rand - 0.5);   % short PRBS burst
    y0 = -a1*y0 + b1*uInit;
    yHist(k) = y0;
    ctrl.step(y0, ones(1, N)); % seed the controller's internal history
end

Tsim   = 80;
rSim   = ones(1, Tsim);
yLog   = zeros(1, Tsim);
uLog   = zeros(1, Tsim);

y = yHist(end);
for k = 1:Tsim
    uApply = ctrl.step(y, rSim(k)*ones(1, N));
    y = -a1*y + b1*uApply + 0.005*randn;

    yLog(k) = y;
    uLog(k) = uApply;
end

%% 5. Results
fprintf('\nClosed-loop performance:\n');
fprintf('  Final y = %.4f  (target = 1.0)\n', yLog(end));
fprintf('  Max |u| = %.4f\n', max(abs(uLog)));
fprintf('  IAE     = %.4f\n', sum(abs(rSim - yLog)));

figure('Name', 'DeePC Step Response');
subplot(2,1,1);
plot((1:Tsim)*Ts, yLog, 'b-', 'LineWidth', 1.5); hold on;
plot((1:Tsim)*Ts, rSim, 'r--', 'LineWidth', 1);
ylabel('y(k)'); title('DeePC Closed-Loop Response');
legend('y', 'r', 'Location', 'southeast');
grid on;
subplot(2,1,2);
stairs((1:Tsim)*Ts, uLog, 'Color', [0 0.5 0], 'LineWidth', 1);
ylabel('u(k)'); xlabel('Time (s)'); title('Control Input');
grid on;
