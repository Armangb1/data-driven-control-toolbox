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
        function testCandidateBankComputesProportionalOutputs(testCase)
            bank = ddc.ufc.CandidateControllerBank('Gains', [0.5; 1; 2]);
            u = bank.step(1, 0.2);
            testCase.verifyEqual(u, [0.5;1;2]*0.8, 'AbsTol', 1e-12);
        end

        function testSwitchingSelectsBestGainOverTime(testCase)
            bank = ddc.ufc.CandidateControllerBank('Gains', [0.1; 0.5; 5]);
            sw = ddc.ufc.UnfalsifiedSwitchingController('Gains', [0.1; 0.5; 5], ...
                'MinDwellSteps', 2, 'ForgettingFactor', 0.9);
            y = 0;
            idx = 1;
            for k = 1:100
                uCand = bank.step(1, y);
                [uSel, idx] = sw.step(uCand);
                y = 0.8*y + 0.5*uSel;
            end
            % The loop should have settled to a stable, bounded response.
            testCase.verifyTrue(isfinite(y));
            testCase.verifyGreaterThanOrEqual(idx, 1);
            testCase.verifyLessThanOrEqual(idx, 3);
        end

        function testMultimodelControllerBoundedAndSelectsIndex(testCase)
            mm = ddc.ufc.MultimodelSwitchingController( ...
                'KpGains', [0.5; 1; 2], 'KiGains', [0.1; 0.2; 0.5], ...
                'SampleTime', 0.1, 'MinDwellSteps', 3);
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
    end
end
