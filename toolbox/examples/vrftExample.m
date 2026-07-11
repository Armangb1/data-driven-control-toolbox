%% Virtual Reference Feedback Tuning (VRFT) Example
%  Designs a discrete PI controller offline for a first-order plant.
%
%  See also: ddc.vrft.vrftDesign, ddc.vrft.prefilterDesign

clear; clc; close all;
fprintf('=== VRFT Example ===\n\n');

Ts = 0.1;
plant = tf(0.5, [1 -0.8], Ts);          % 0.5 / (z - 0.8)
Md   = tf(0.3, [1 -0.7], Ts);          % desired CL: 0.3 / (z - 0.7)
z    = tf('z', Ts);
basis = { tf(1,1,Ts), tf(Ts,[1 -1],Ts) };  % {1,  Ts/(z-1)}  -- PI basis

%% Collect open-loop data
T = 1000;
rng(7);
u = 2*(rand(T,1) > 0.5) - 1;
t = (0:T-1)'*Ts;
y = lsim(plant, u, t);

%% VRFT design
[theta, info] = ddc.vrft.vrftDesign(u, y, Md, basis);

C = theta(1) + theta(2)*Ts/(z-1);
fprintf('Designed PI controller:  C(z) = %.4f + %.4f * Ts/(z-1)\n', theta(1), theta(2));
fprintf('Ideal matching:          C(z) = 0.6000 + 1.2000 * Ts/(z-1)\n');

%% Verify closed-loop poles
CL = feedback(C*plant, 1);
fprintf('Closed-loop pole: %.4f  (target Md pole: 0.7)\n', pole(CL));

%% Step response comparison
figure('Name','VRFT Result');
subplot(2,1,1);
[ycl, tcl] = step(CL, 5);
plot(tcl, ycl, 'b-', 'LineWidth', 1.5); hold on;
[yMd, tMd] = step(Md, 5);
plot(tMd, yMd, 'r--', 'LineWidth', 1.2);
ylabel('y'); title('Step Response: VRFT-designed CL vs Desired Md');
legend('VRFT CL', 'Md', 'Location', 'southeast'); grid on;
subplot(2,1,2);
bode(C); title('Designed Controller C(z)');
