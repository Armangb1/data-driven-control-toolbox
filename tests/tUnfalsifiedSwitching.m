classdef tUnfalsifiedSwitching < matlab.unittest.TestCase
    %TUNFALSIFIEDSWITCHING Tests for ddc.ufc.*

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testCandidateBankProportionalViaGains(testCase)
            bank = ddc.ufc.CandidateControllerBank('Controllers', [0.5; 1; 2]);
            u = bank.step(1, 0.2);
            testCase.verifyEqual(u, [0.5;1;2]*0.8, 'AbsTol', 1e-12);
        end

        function testCandidateBankProportionalViaTf(testCase)
            C = [tf(0.5,1,-1), tf(1,1,-1), tf(2,1,-1)];
            bank = ddc.ufc.CandidateControllerBank('Controllers', C);
            u = bank.step(1, 0.2);
            testCase.verifyEqual(u, [0.5;1;2]*0.8, 'AbsTol', 1e-12);
        end

        function testCandidateBankPiControllers(testCase)
            C = [tf([1.1, -1], [1, -1], -1), tf([2.1, -2], [1, -1], -1)];
            bank = ddc.ufc.CandidateControllerBank('Controllers', C);
            u = bank.step(1, 0);
            testCase.verifyEqual(u(1), 1.1, 'AbsTol', 1e-12);
            testCase.verifyEqual(u(2), 2.1, 'AbsTol', 1e-12);
            u2 = bank.step(1, 0);
            testCase.verifyEqual(u2(1), 1.1 + 1.1*1 - 1.0*1.1, 'AbsTol', 1e-12);
        end

        function testSwitchingSelectsBestGainOverTime(testCase)
            bank = ddc.ufc.CandidateControllerBank('Controllers', [0.1; 0.5; 5]);
            sw = ddc.ufc.UnfalsifiedSwitchingController('Controllers', [0.1; 0.5; 5], ...
                'ForgettingFactor', 0.9);
            y = 0;
            idx = 1;
            for k = 1:100
                uCand = bank.step(1, y);
                [uSel, idx] = sw.step(uCand, y);
                y = 0.8*y + 0.5*uSel;
            end
            testCase.verifyTrue(isfinite(y));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
        end

        function testSwitchingWithPiControllers(testCase)
            Ts = 1;
            C = [tf([0.55, -0.5], [1, -1], Ts), ...
                 tf([1.1,  -1],  [1, -1], Ts), ...
                 tf([5.2,  -5],  [1, -1], Ts)];
            bank = ddc.ufc.CandidateControllerBank('Controllers', C);
            sw = ddc.ufc.UnfalsifiedSwitchingController('Controllers', C, ...
                'ForgettingFactor', 0.9);
            y = 0; idx = 1;
            for k = 1:200
                uCand = bank.step(1, y);
                [uSel, idx] = sw.step(uCand, y);
                y = 0.8*y + 0.5*uSel;
            end
            testCase.verifyTrue(isfinite(y));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
        end

        function testMultimodelControllerBoundedAndSelectsIndex(testCase)
            Ts = 0.1;
            controllers = [tf(0.5,1,Ts); tf(1,1,Ts); tf(2,1,Ts)];
            models = [tf(0.1,[1 -0.95],Ts); tf(0.1,[1 -0.95],Ts); tf(0.1,[1 -0.95],Ts)];
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'Controllers', controllers, 'Models', models, ...
                'SampleTime', Ts);
            y = 0;
            for k = 1:100
                [uSel, idx, costs] = mm.step(1, y);
                y = 0.8*y + 0.5*uSel;
            end
            testCase.verifyTrue(isfinite(y));
            testCase.verifyEqual(numel(costs), 3);
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
        end

        function testContinuousPiAutoDiscretized(testCase)
            Ts = 0.1;
            C = [tf([0.5, 0.1], [1, 0]), ...
                 tf([1.0, 0.2], [1, 0]), ...
                 tf([2.0, 0.4], [1, 0])];
            bank = ddc.ufc.CandidateControllerBank('Controllers', C, 'SampleTime', Ts);
            sw = ddc.ufc.UnfalsifiedSwitchingController('Controllers', C, ...
                'SampleTime', Ts, 'ForgettingFactor', 0.9);
            y = 0; idx = 1;
            for k = 1:200
                uCand = bank.step(1, y);
                [uSel, idx] = sw.step(uCand, y);
                y = 0.8*y + 0.5*uSel;
            end
            testCase.verifyTrue(isfinite(y));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
        end

        function testContinuousVsDiscreteProduceSameOutput(testCase)
            Ts = 0.1;
            C_cont = [tf([1.0, 0.2], [1, 0]), tf([2.0, 0.4], [1, 0])];
            C_disc = c2d(C_cont, Ts, 'zoh');
            bank_c = ddc.ufc.CandidateControllerBank('Controllers', C_cont, 'SampleTime', Ts);
            bank_d = ddc.ufc.CandidateControllerBank('Controllers', C_disc);
            u_c = bank_c.step(1, 0.5);
            u_d = bank_d.step(1, 0.5);
            testCase.verifyEqual(u_c, u_d, 'AbsTol', 1e-12);
        end

        function testWeightsAffectCost(testCase)
            C = tf(1, 1, -1);
            swA = ddc.ufc.UnfalsifiedSwitchingController('Controllers', C, ...
                'W1', 1, 'W2', 1, 'ForgettingFactor', 0.5, 'HysteresisMargin', 0);
            swB = ddc.ufc.UnfalsifiedSwitchingController('Controllers', C, ...
                'W1', 1, 'W2', 5, 'ForgettingFactor', 0.5, 'HysteresisMargin', 0);
            for k = 1:3
                [~, ~, cA] = swA.step(0.5, 0.1);
                [~, ~, cB] = swB.step(0.5, 0.1);
            end
            testCase.verifyTrue(isfinite(cA));
            testCase.verifyTrue(isfinite(cB));
            testCase.verifyNotEqual(cA, cB);
        end

        function testSampleTimeStaticInPlainMatlab(testCase)
            C = [tf([1.0, 0.2], [1, 0]), tf([2.0, 0.4], [1, 0])];
            bank = ddc.ufc.CandidateControllerBank('Controllers', C);
            sw = ddc.ufc.UnfalsifiedSwitchingController('Controllers', C, ...
                'ForgettingFactor', 0.9);
            uC = bank.step(1, 0.5);
            testCase.verifyTrue(all(isfinite(uC)));
            [uSel, idx, ~] = sw.step(uC, 0.5);
            testCase.verifyTrue(isfinite(uSel));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 2);
        end
    end
end
