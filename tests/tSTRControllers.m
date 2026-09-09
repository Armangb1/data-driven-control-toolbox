classdef tSTRControllers < matlab.unittest.TestCase
    %TSTRCONTROLLERS Tests for ddc.str.DirectSTRController / IndirectSTRController.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testDirectSTRTracksAndIdentifies(testCase)
            ctrl = ddc.str.DirectSTRController('Ac1', -0.5);
            y = 0;
            for k = 1:100
                [u, theta] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.02);
            testCase.verifyEqual(theta(1), 0.5, 'AbsTol', 0.02); % beta0 = b1
            testCase.verifyEqual(theta(2), 0.3, 'AbsTol', 0.02); % beta1 = b1*s0 = Ac1-a1
        end

        function testIndirectSTRTracksAndIdentifies(testCase)
            % Regression: first-order ARX plant with the original test's
            % same-sample convention y(k) = 0.8*y(k-1) + 0.5*u(k).
            % RLS model (controller index): y(k) = theta1*y(k-1) + theta2*u(k-1),
            % so theta = [0.8; 0.5], i.e. A = [1 -0.8], B = [0.5].
            % Desired CL pole at z = 0.5 (Am = [1 -0.5]),
            % unity DC gain (Bm = [0.5], Bm(1)/Am(1) = 0.5/0.5 = 1).
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', 1, 'Nb', 1, ...
                'Am', [1 -0.5], 'Bm', [0.5]);
            y = 0;
            for k = 1:100
                [u, theta] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.02);
            testCase.verifyEqual(theta(1), 0.8, 'AbsTol', 0.02);  % -a1 = 0.8
            testCase.verifyEqual(theta(2), 0.5, 'AbsTol', 0.02);  % b0
        end

        function testNoDeadlockFromZeroInitialGains(testCase)
            % Regression test: initial S0=0,T0=0 previously caused u==0
            % forever (certainty-equivalence deadlock).
            ctrl = ddc.str.DirectSTRController();
            u1 = ctrl.step(0, 1);
            testCase.verifyNotEqual(u1, 0);
        end

        function testSecondOrderARXPlant(testCase)
            % Second-order ARX (same-sample convention):
            %   y(k) = 1.1*y(k-1) - 0.3*y(k-2) + u(k) + 0.5*u(k-1)
            % => A = [1 -1.1 0.3], B = [1 0.5]
            % True theta = [1.1; -0.3; 1.0; 0.5]
            Na = 2; Nb = 2;

            Am = [1 -0.9 0.2];   % desired CL poles at 0.4, 0.5
            Bm = [0.15 0.15];    % deg 1; Bm(1)/Am(1) = 0.3/0.3 = 1 (unity DC gain)

            ctrl = ddc.str.IndirectSTRController( ...
                'Na', Na, 'Nb', Nb, ...
                'Am', Am, 'Bm', Bm);

            y = 0; ym1 = 0; um1 = 0;
            theta = [];
            for k = 1:500
                [u, theta] = ctrl.step(y, 1);
                ynew = 1.1*y - 0.3*ym1 + 1.0*u + 0.5*um1;
                um1 = u; ym1 = y;
                y = ynew;
            end

            testCase.verifyEqual(y, 1, 'AbsTol', 0.1);

            testCase.verifyLength(theta, Na + Nb);
            testCase.verifyEqual(theta(1), 1.1, 'AbsTol', 0.1);  % -a1
            testCase.verifyEqual(theta(2), -0.3, 'AbsTol', 0.1); % -a2
            testCase.verifyEqual(theta(3), 1.0, 'AbsTol', 0.1);  % b0
            testCase.verifyEqual(theta(4), 0.5, 'AbsTol', 0.1);  % b1

            R = ctrl.getR();
            S = ctrl.getS();
            testCase.verifyGreaterThan(length(R), 1);
            testCase.verifyGreaterThan(length(S), 1);
        end

        function testDifferentAOrders(testCase)
            % Mixed orders Na=2, Nb=1 (same-sample convention):
            %   y(k) = 1.2*y(k-1) - 0.4*y(k-2) + u(k)
            % => A = [1 -1.2 0.4], B = [1.0] (scalar)
            % True theta = [1.2; -0.4; 1.0]
            Na = 2; Nb = 1;

            Am = [1 -0.9 0.2];   % CL poles at 0.4, 0.5
            Bm = [0.3];          % scalar Bm: Bm(1)/Am(1) = 0.3/0.3 = 1

            ctrl = ddc.str.IndirectSTRController( ...
                'Na', Na, 'Nb', Nb, ...
                'Am', Am, 'Bm', Bm);

            y = 0; ym1 = 0;
            theta = [];
            for k = 1:500
                [u, theta] = ctrl.step(y, 1);
                ynew = 1.2*y - 0.4*ym1 + 1.0*u;
                ym1 = y;
                y = ynew;
            end

            testCase.verifyEqual(y, 1, 'AbsTol', 0.1);

            testCase.verifyLength(theta, Na + Nb);
            testCase.verifyEqual(theta(1), 1.2, 'AbsTol', 0.1);
            testCase.verifyEqual(theta(2), -0.4, 'AbsTol', 0.1);
            testCase.verifyEqual(theta(3), 1.0, 'AbsTol', 0.1);
        end

        function testOnlineAdaptation(testCase)
            % Plant parameters change mid-simulation; verify controller adapts.
            Na = 1; Nb = 1;
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', Na, 'Nb', Nb, ...
                'Am', [1 -0.5], 'Bm', [0.5], ...
                'ForgettingFactor', 0.92);

            % Phase 1: plant y(k) = 0.8*y(k-1) + 0.5*u(k), constant reference.
            y = 0;
            theta1 = []; R1 = []; S1 = [];
            for k = 1:300
                [u, theta1] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            R1 = ctrl.getR();
            S1 = ctrl.getS();

            % Phase 2: plant changes to y(k) = 0.5*y(k-1) + 0.8*u(k), with a
            % slowly-varying reference to keep the RLS regressor exciting.
            T2 = 600;
            r = 1 + 0.4*sin(0.05*(1:T2));
            yHist = zeros(1, T2);
            for k = 1:T2
                [u, theta2] = ctrl.step(y, r(k));
                y = 0.5*y + 0.8*u;
                yHist(k) = y;
            end

            R2 = ctrl.getR();
            S2 = ctrl.getS();

            % RLS should track the new plant parameters.
            testCase.verifyGreaterThan(abs(theta2(1) - theta1(1)), 0.05);
            testCase.verifyGreaterThan(abs(theta2(2) - theta1(2)), 0.05);

            % Controller polynomials should have changed.
            testCase.verifyGreaterThan(norm(R2 - R1) + norm(S2 - S1), 1e-6);

            % Output should track the (near-1) reference mean in steady state.
            testCase.verifyEqual(mean(yHist(end-199:end)), 1, 'AbsTol', 0.1);
        end

        function testReset(testCase)
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', 1, 'Nb', 1, ...
                'Am', [1 -0.5], 'Bm', [0.5]);

            % Run for a while to let RLS adapt.
            y = 0;
            thetaBefore = [];
            for k = 1:50
                [u, thetaBefore] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end

            % Verify controller has adapted (S should no longer be zero;
            % for this first-order plant R converges to [1] by design).
            testCase.verifyTrue(any(thetaBefore ~= 0));
            testCase.verifyNotEqual(ctrl.getS(), [0], ...
                'Controller should have adapted away from initial state');

            reset(ctrl);

            % Verify all state is reset.
            testCase.verifyEqual(ctrl.getR(), [1]);
            testCase.verifyEqual(ctrl.getS(), [0]);
            testCase.verifyEqual(ctrl.getT(), [1]);

            % After reset, subsequent execution is deterministic.
            u1 = ctrl.step(0, 1);
            u2 = ctrl.step(0, 1);
            testCase.verifyEqual(u1, 1);  % initial controller: u = r
            testCase.verifyEqual(u2, 1);
        end

        function testPoorInitialEstimate(testCase)
            % Startup does not produce NaN, Inf, or indexing errors.
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', 1, 'Nb', 1, ...
                'Am', [1 -0.5], 'Bm', [0.5]);

            for k = 1:20
                [u, theta] = ctrl.step(0, 1);
                testCase.verifyTrue(isfinite(u));
                testCase.verifyTrue(all(isfinite(theta)));
            end

            R = ctrl.getR();
            S = ctrl.getS();
            T = ctrl.getT();
            testCase.verifyTrue(all(isfinite(R)));
            testCase.verifyTrue(all(isfinite(S)));
            testCase.verifyTrue(all(isfinite(T)));
        end

        function testRSTPolynomialsCorrectDimension(testCase)
            % Verify R, S, T have the expected degree for a 2nd-order plant.
            Na = 2; Nb = 2;
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', Na, 'Nb', Nb, ...
                'Am', [1 -0.9 0.2], 'Bm', [0.15 0.15]);

            y = 0; ym1 = 0; um1 = 0;
            for k = 1:400
                [u, ~] = ctrl.step(y, 1);
                ynew = 1.1*y - 0.3*ym1 + 1.0*u + 0.5*um1;
                um1 = u; ym1 = y; y = ynew;
            end

            R = ctrl.getR();
            S = ctrl.getS();
            T = ctrl.getT();

            % For deg(A) = 2, deg(R) = deg(S) = deg(T) = 1, so length = 2.
            testCase.verifyEqual(length(R), 2);
            testCase.verifyEqual(length(S), 2);
            testCase.verifyEqual(length(T), 2);
            testCase.verifyTrue(abs(R(1)) > 1e-6, 'R leading coeff must be nonzero');
        end

        function testOutputInterface(testCase)
            % Verify outputs: [u, theta], correct sizes and types.
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', 2, 'Nb', 1, ...
                'Am', [1 -0.9 0.2], 'Bm', [0.3]);

            [u, theta] = ctrl.step(0, 1);
            testCase.verifyTrue(isfinite(u));
            testCase.verifySize(theta, [3 1]);  % Na + Nb = 3
        end

        function testRLSEstimatesCorrectPolynomials(testCase)
            % Verify that A_hat and B_hat converge to the true plant.
            Na = 1; Nb = 1;
            ctrl = ddc.str.IndirectSTRController( ...
                'Na', Na, 'Nb', Nb, ...
                'Am', [1 -0.5], 'Bm', [0.5]);

            % Run plant y = 0.8*y + 0.5*u to let RLS converge.
            y = 0;
            for k = 1:200
                [u, ~] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end

            Ahat = ctrl.getEstimatedA();
            Bhat = ctrl.getEstimatedB();

            % A = [1 -0.8], B = [0.5]
            testCase.verifyEqual(Ahat, [1 -0.8], 'AbsTol', 0.02);
            testCase.verifyEqual(Bhat, [0.5], 'AbsTol', 0.02);
        end
    end
end
