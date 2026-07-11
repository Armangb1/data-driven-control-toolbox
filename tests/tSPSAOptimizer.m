classdef tSPSAOptimizer < matlab.unittest.TestCase
    %TSPSAOPTIMIZER Tests for ddc.spsa.SPSAOptimizer.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testConvergesToScalarMinimum(testCase)
            rng(3);
            opt = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', 0.5);
            loss = @(th) (th-2).^2;
            paramToApply = 5;
            lossVal = loss(paramToApply);
            for k = 1:400
                [paramToApply, thetaHat] = opt.step(lossVal);
                lossVal = loss(paramToApply);
            end
            testCase.verifyEqual(thetaHat, 2, 'AbsTol', 0.05);
        end

        function testConvergesForVectorParameter(testCase)
            rng(4);
            target = [1; -2];
            opt = ddc.spsa.SPSAOptimizer('NumParameters', 2, 'InitialTheta', [0;0], ...
                'ATuning', 0.5, 'CTuning', 0.3);
            loss = @(th) sum((th-target).^2);
            paramToApply = [0;0];
            lossVal = loss(paramToApply);
            for k = 1:600
                [paramToApply, thetaHat] = opt.step(lossVal);
                lossVal = loss(paramToApply);
            end
            testCase.verifyEqual(thetaHat, target, 'AbsTol', 0.15);
        end
    end
end
