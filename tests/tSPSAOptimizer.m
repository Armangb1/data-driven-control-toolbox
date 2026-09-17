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

        function testGradientClippingLimitsUpdateStep(testCase)
            loss = @(th) 100*(th-2).^2;

            rng(1);
            optRaw = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', 0.5);
            pRaw = 5; lRaw = loss(pRaw);
            [pRaw, thRaw] = optRaw.step(lRaw);
            lRaw = loss(pRaw);
            [pRaw, thRaw] = optRaw.step(lRaw);
            lRaw = loss(pRaw);
            [~, thRaw] = optRaw.step(lRaw);

            rng(1);
            optClip = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', 0.5, ...
                'GradientClipLower', -0.1, 'GradientClipUpper', 0.1);
            pClip = 5; lClip = loss(pClip);
            [pClip, thClip] = optClip.step(lClip);
            lClip = loss(pClip);
            [pClip, thClip] = optClip.step(lClip);
            lClip = loss(pClip);
            [~, thClip] = optClip.step(lClip);

            testCase.verifyGreaterThan(abs(thRaw - 5), 0.5, ...
                'Unclipped gradient on a steep loss should produce a large step');
            testCase.verifyLessThan(abs(thClip - 5), 0.05, ...
                'Clipped gradient must keep the parameter update small');
            testCase.verifyLessThan(abs(thClip - 5), abs(thRaw - 5), ...
                'Gradient clipping must reduce the magnitude of the parameter update');
        end

        function testClippingNoopWithInfiniteBounds(testCase)
            loss = @(th) sum((th - [2;3]).^2);

            rng(7);
            optRaw = ddc.spsa.SPSAOptimizer('NumParameters', 2, 'InitialTheta', [1;-1], ...
                'ATuning', 0.5, 'CTuning', 0.3, 'ACommon', 5);
            pRaw = [1;-1]; lRaw = loss(pRaw);
            for k = 1:40
                [pRaw, thRaw] = optRaw.step(lRaw);
                lRaw = loss(pRaw);
            end

            rng(7);
            optExpl = ddc.spsa.SPSAOptimizer('NumParameters', 2, 'InitialTheta', [1;-1], ...
                'ATuning', 0.5, 'CTuning', 0.3, 'ACommon', 5, ...
                'GradientClipLower', -inf, 'GradientClipUpper', inf);
            pEn = [1;-1]; lEn = loss(pEn);
            for k = 1:40
                [pEn, thEn] = optExpl.step(lEn);
                lEn = loss(pEn);
            end
            testCase.verifyEqual(thEn, thRaw, 'AbsTol', 1e-12, ...
                'Infinite clip bounds must leave the update unchanged');
        end

        function testInvalidClipBoundsThrow(testCase)
            opt = ddc.spsa.SPSAOptimizer('NumParameters', 1, ...
                'GradientClipLower', 1, 'GradientClipUpper', 0);
            testCase.verifyError(@() opt.step(0), 'ddc:spsa:GradientClipBounds');
        end

        function testExternalResetRestartsIterationCounter(testCase)
            CTuning = 0.5;
            loss = @(th) (th-2).^2;

            % No-reset reference: complete one full iteration (phases 1-2-3),
            % so K_ = 1, then emit the next phase-1 perturbation.
            rng(7);
            optNr = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', CTuning, 'ACommon', 5);
            p = 5; l = loss(p);
            [p, ~] = optNr.step(l); l = loss(p);
            [p, ~] = optNr.step(l); l = loss(p);
            [p, ~] = optNr.step(l); l = loss(p);
            thNr = p;
            [pNr, ~] = optNr.step(l);
            pertNr = abs(pNr - thNr);

            % Reset-enabled run with reset asserted on the fourth call.
            rng(7);
            optR = ddc.spsa.SPSAOptimizer('NumParameters', 1, 'InitialTheta', 5, ...
                'ATuning', 0.5, 'CTuning', CTuning, 'ACommon', 5, 'ExternalReset', true);
            p = 5; l = loss(p);
            [p, ~] = optR.step(l, false); l = loss(p);
            [p, ~] = optR.step(l, false); l = loss(p);
            [p, ~] = optR.step(l, false); l = loss(p);
            thR = p;
            [pR, ~] = optR.step(l, true);
            pertR = abs(pR - thR);

            testCase.verifyEqual(pertR, CTuning, 'AbsTol', 1e-9, ...
                'After a reset, K_ must be 0 so the perturbation c_k equals CTuning');
            testCase.verifyGreaterThan(pertR, pertNr, ...
                'Resetting K_ must restore a larger perturbation than without reset');
        end
    end
end
