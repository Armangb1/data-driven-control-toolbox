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

        function testDeferredConvergesToScalarMinimum(testCase)
            rng(3);
            opt = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', 0.5, 'ParameterUpdateMode', 'deferred');
            loss = @(th) (th-2).^2;
            paramToApply = 5;
            lossVal = loss(paramToApply);
            for k = 1:300
                [paramToApply, thetaHat] = opt.step(lossVal);
                lossVal = loss(paramToApply);
            end
            testCase.verifyEqual(thetaHat, 2, 'AbsTol', 0.05);
        end

        function testDeferredTwoPhaseCycle(testCase)
            rng(10);
            opt = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', 0.5, 'ParameterUpdateMode', 'deferred');
            loss = @(th) (th-2).^2;

            % Call 1 — Phase 1 (first iteration, no stored J+, gradient skipped)
            [p1, th1] = opt.step(loss(5));
            testCase.verifyEqual(th1, 5, 'AbsTol', 1e-12, ...
                'Theta must not change on the first Phase 1 call');
            testCase.verifyEqual(p1, 5 + 0.5);  % delta=+1 with this rng

            % Call 2 — Phase 2: store J+, emit theta-ck*delta. No update yet.
            [p2, th2] = opt.step(loss(p1));
            testCase.verifyEqual(th2, 5, 'AbsTol', 1e-12, ...
                'Theta must not change during Phase 2');
            testCase.verifyEqual(p2, 5 - 0.5);

            % Call 3 — Phase 1 (second iteration): gradient computed,
            % theta updated, new perturbation emitted around UPDATED theta
            [p3, th3] = opt.step(loss(p2));
            testCase.verifyLessThan(th3, 5, ...
                'Theta should have moved toward the minimum at 2');
            testCase.verifyNotEqual(p3, th3, ...
                'ParamToApply must differ from theta (a perturbation was applied)');
        end
    end
end
