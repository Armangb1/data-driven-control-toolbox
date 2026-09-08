classdef tMultimodelSwitching < matlab.unittest.TestCase
    %TMULTIMODELSWITCHING Tests for ddc.ufc.MultimodelSwitchingController
    %   (multi-model unfalsified adaptive supervisory switching control).
    %
    %   Verifies the corrected pairwise (C_i, M_i) cost formulation:
    %   a candidate whose controller/model pair matches the true plant
    %   reproduces the real signal pair z = [y; u], so its cost V_i
    %   settles near zero and the supervisor switches to it.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testSelectsMatchingPairStaticGains(testCase)
            % Matching pair (C_3, M_3) is placed last so the supervisor
            % must switch away from the initially-active candidate.
            Ts = 1;
            Mtrue = tf(0.5, [1 -0.8], Ts);
            controllers = [tf(0.5,1,Ts), tf(2,1,Ts), tf(1,1,Ts)];
            models = [tf(0.3,[1 -0.5],Ts), tf(0.9,[1 -0.95],Ts), Mtrue];

            bank = ddc.ufc.CandidateControllerBank('Controllers', controllers, 'SampleTime', Ts);
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', controllers, 'Models', models, ...
                'SampleTime', Ts, 'ForgettingFactor', 0.95, ...
                'HysteresisMargin', 1e-3);

            y = 0; idx = 1;
            for k = 1:300
                uCand = bank.step(1, y);
                [u, idx, costs] = mm.step(uCand, y);
                y = 0.8*y + 0.5*u;
            end

            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(costs(3), 0, 'AbsTol', 1e-10);
            testCase.verifyGreaterThan(costs(1), costs(3));
            testCase.verifyGreaterThan(costs(2), costs(3));
            testCase.verifyEqual(numel(costs), 3);
        end

        function testMatchingPairCostStaysZeroWhenFirst(testCase)
            % Matching pair (C_1, M_1) is first/initially active; its cost
            % must remain exactly zero while the plant is driven.
            Ts = 1;
            Mtrue = tf(0.5, [1 -0.8], Ts);
            controllers = [tf(1,1,Ts), tf(2,1,Ts), tf(0.5,1,Ts)];
            models = [Mtrue, tf(0.3,[1 -0.5],Ts), tf(0.9,[1 -0.95],Ts)];

            bank = ddc.ufc.CandidateControllerBank('Controllers', controllers, 'SampleTime', Ts);
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', controllers, 'Models', models, ...
                'ForgettingFactor', 0.95);

            y = 0; idx = 1; costs = [1;1;1];
            for k = 1:300
                uCand = bank.step(1, y);
                [u, idx, costs] = mm.step(uCand, y);
                y = 0.8*y + 0.5*u;
                testCase.verifyEqual(costs(1), 0, 'AbsTol', 1e-12);
            end
            testCase.verifyEqual(idx, 1);
            testCase.verifyTrue(isfinite(u));
        end

        function testDynamicPiControllersSelectMatchingPair(testCase)
            % Dynamic (velocity-form PI) controllers still discriminate:
            % the correct (C, M) pair has near-zero cost.
            Ts = 1;
            Mtrue = tf(0.5, [1 -0.8], Ts);
            piC = @(Kp, Ki) tf([Kp + Ki*Ts, -Kp], [1, -1], Ts);
            controllers = [piC(0.2, 0.05), piC(2, 1), piC(0.8, 0.2)];
            models = [tf(0.1,[1 -0.5],Ts), tf(0.9,[1 -0.95],Ts), Mtrue];

            bank = ddc.ufc.CandidateControllerBank('Controllers', controllers, 'SampleTime', Ts);
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', controllers, 'Models', models, ...
                'SampleTime', Ts, 'ForgettingFactor', 0.95);

            y = 0; idx = 1;
            for k = 1:300
                uCand = bank.step(1, y);
                [u, idx, costs] = mm.step(uCand, y);
                y = 0.8*y + 0.5*u;
            end

            testCase.verifyEqual(idx, 3);
            testCase.verifyEqual(costs(3), 0, 'AbsTol', 1e-6);
            testCase.verifyGreaterThan(costs(1), costs(3));
            testCase.verifyGreaterThan(costs(2), costs(3));
            testCase.verifyTrue(isfinite(u));
        end

        function testContinuousAndDiscreteControllersEquivalent(testCase)
            % Discretization handling: a continuous controller auto-
            % discretized by c2d must behave exactly like the explicitly
            % discrete controller it discretizes to.
            Ts = 0.1;
            Mtrue_d = c2d(tf(5, [1 2]), Ts, 'zoh');
            C_cont = [tf(0.2); tf(0.5)];
            C_disc = c2d(C_cont, Ts, 'zoh');
            models = [Mtrue_d, tf(0.1, [1 -0.99], Ts)];

            bankA = ddc.ufc.CandidateControllerBank('Controllers', C_cont, 'SampleTime', Ts);
            bankB = ddc.ufc.CandidateControllerBank('Controllers', C_disc);
            mmA = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', C_cont, 'Models', models, 'SampleTime', Ts);
            mmB = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', C_disc, 'Models', models, 'SampleTime', Ts);

            uA = zeros(1, 5); uB = zeros(1, 5);
            yA = 0; yB = 0;
            for k = 1:5
                uCandA = bankA.step(0.5, yA);
                uCandB = bankB.step(0.5, yB);
                [uA(k), ~, ~] = mmA.step(uCandA, yA);
                [uB(k), ~, ~] = mmB.step(uCandB, yB);
                yA = 0.453173*uA(k) + 0.818731*yA;
                yB = 0.453173*uB(k) + 0.818731*yB;
            end
            testCase.verifyEqual(uA, uB, 'AbsTol', 1e-10);
        end

        function testNumericGainsAccepted(testCase)
            % Controllers/Models as plain numeric vectors wrap to tf and
            % the block runs, produces finite outputs and valid indices.
            bank = ddc.ufc.CandidateControllerBank('Controllers', [0.5; 1; 2]);
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', [0.5; 1; 2], 'Models', [0.5; 1; 2], ...
                'ForgettingFactor', 0.9);
            y = 0;
            for k = 1:100
                uCand = bank.step(1, y);
                [u, idx, costs] = mm.step(uCand, y);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyTrue(isfinite(u));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
            testCase.verifyEqual(numel(costs), 3);
        end

        function testPairSizeMismatchErrors(testCase)
            % Controllers and Models must describe the same number of
            % candidates.
            C = [tf(1, 1, -1); tf(2, 1, -1)];
            M = tf(0.5, [1 -0.8], -1);
            testCase.verifyError(@() ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', C, 'Models', M), ...
                'ddc:ufc:MultimodelSwitchingController:PairSizeMismatch');
        end

        function testResetRestoresZeroCosts(testCase)
            % After accounting for transients, reset() clears the cost
            % accumulators so a fresh first step has zero cost.
            Ts = 1;
            Mtrue = tf(0.5, [1 -0.8], Ts);
            bank = ddc.ufc.CandidateControllerBank('Controllers', tf(1, 1, Ts), 'SampleTime', Ts);
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', tf(1, 1, Ts), 'Models', Mtrue, ...
                'ForgettingFactor', 0.9);

            y = 0;
            for k = 1:20
                uCand = bank.step(1, y);
                [~, ~, ~] = mm.step(uCand, y);
                y = 0.8*y + 0.5*1; % plant driven by previous u
            end

            mm.reset();
            bank.reset();
            uCand = bank.step(1, 0);
            [~, ~, costs] = mm.step(uCand, 0);
            testCase.verifyEqual(costs, 0, 'AbsTol', 1e-12);
        end
    end
end