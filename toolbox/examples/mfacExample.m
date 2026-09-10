%% Model-Free Adaptive Control (MFAC) Example
%  Demonstrates CFDL, PFDL, and FFDL variants of MFAC on the non-linear
%  plant used in the K.N.Toosi course material on Model-Free Adaptive
%  Control (Hou & Jin).
%
%  Plant:
%      x1(k) = x2(k-1)
%      x2(k) = -(1/64)*((1.7*sin(k*pi/10))*x2(k-1) + (exp(-k)+3)*x2(k-1)^3)
%              + (1/128)*u(k-1)
%      y(k)  = x1(k)
%  Setpoint:
%      yd(k) = 0.4036*sin(k*pi/180) + 0.12*cos(k*pi/50)
%
%  The three MFAC variants are just different pseudo-orders (Ly, Lu):
%      CFDL    : Ly=0, Lu=1   (scalar PPD)
%      PFDL L=3: Ly=0, Lu=3   (vector PPD over u)
%      FFDL    : Ly=2, Lu=2   (full H vector with y and u)
%
%  See also: ddc.mfac.MFACController

clear; clc; close all;
fprintf('=== MFAC Example (CFDL / PFDL / FFDL) ===\n\n');

%% Simulation setup
Tsim = 300;
k = (1:Tsim)';
yd = 0.4036*sin(k*pi/180) + 0.12*cos(k*pi/50);

%% Controller presets (parameters from the K.N.Toosi MFAC problem set)
presets = {
    'CFDL',  0, 1, 0.01,                   0.3,                1e-6;
    'PFDL',  0, 3, [0.01 0.01 0.01],      [0.3 0.3 0.3],       1e-6;
    'FFDL',  2, 2, 0.01*ones(4,1),         0.3*ones(4,1),      2e-6;
    };

yLogAll = cell(1, 3);
uLogAll = cell(1, 3);
names   = cell(1, 3);

for p = 1:3
    name = presets{p,1};
    Ly   = presets{p,2};
    Lu   = presets{p,3};
    Phi0 = presets{p,4};
    Rho  = presets{p,5};
    Lambda = presets{p,6};

    ctrl = ddc.mfac.MFACController('Ly', Ly, 'Lu', Lu, ...
        'PhiInit', Phi0, 'Rho', Rho, ...
        'Eta', 0.1, 'Mu', 0.5, 'Lambda', Lambda, 'Epsilon', 1e-5);

    x2 = 0; y = 0;
    yLog = zeros(1, Tsim); uLog = zeros(1, Tsim);
    for t = 1:Tsim
        u = ctrl.step(y, yd(t));
        x1n = x2;
        x2n = -(1/64)*((1.7*sin(t*pi/10))*x2 + (exp(-t)+3)*x2^3) + (1/128)*u;
        y = x1n; x2 = x2n;
        yLog(t) = y; uLog(t) = u;
    end
    yLogAll{p} = yLog; uLogAll{p} = uLog; names{p} = name;

    err = max(abs(yLog(end-99:end) - yd(end-99:end)'));
    fprintf('%-5s  max|y-y_d| over last 100 samples: %.4f\n', name, err);
end

%% Plots
figure('Name', 'MFAC: CFDL / PFDL / FFDL tracking');
subplot(2,1,1);
plot(yd, 'k--', 'LineWidth', 1.2); hold on;
c = lines(3);
for p = 1:3
    plot(yLogAll{p}, 'Color', c(p,:));
end
ylabel('y(k)'); title('Closed-loop tracking of y_d');
legend(['y_d', names], 'Location', 'best'); grid on;

subplot(2,1,2);
for p = 1:3
    stairs(uLogAll{p}, 'Color', c(p,:)); hold on;
end
ylabel('u(k)'); xlabel('Sample k'); grid on;
legend(names, 'Location', 'best');
