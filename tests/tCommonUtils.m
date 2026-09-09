classdef tCommonUtils < matlab.unittest.TestCase
    %TCOMMONUTILS Tests for ddc.common.*

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testHankelBuilderSize(testCase)
            data = 1:10;
            H = ddc.common.HankelBuilder(data, 3);
            testCase.verifySize(H, [3, 8]);
            testCase.verifyEqual(H(:,1), [1;2;3]);
            testCase.verifyEqual(H(:,end), [8;9;10]);
        end

        function testHankelBuilderMultiChannel(testCase)
            data = [1:6; 11:16];
            H = ddc.common.HankelBuilder(data, 2);
            testCase.verifySize(H, [4, 5]);
            testCase.verifyEqual(H(:,1), [1;11;2;12]);
        end

        function testPersistencyExcitationRandom(testCase)
            rng(1);
            u = randn(1, 200);
            [isPE, r, rReq] = ddc.common.checkPersistencyExcitation(u, 5);
            testCase.verifyTrue(isPE);
            testCase.verifyEqual(r, rReq);
        end

        function testPersistencyExcitationConstant(testCase)
            u = ones(1, 50); % constant signal: not persistently exciting for order>1
            [isPE, r] = ddc.common.checkPersistencyExcitation(u, 3);
            testCase.verifyFalse(isPE);
            testCase.verifyEqual(r, 1);
        end

        function testDataBufferFillsAndSlides(testCase)
            buf = ddc.common.DataBuffer('WindowLength', 4, 'SignalWidth', 1);
            for k = 1:3
                [isFull, ~] = buf.step(k);
            end
            testCase.verifyFalse(isFull);
            [isFull, data] = buf.step(4);
            testCase.verifyTrue(isFull);
            testCase.verifyEqual(data, [1 2 3 4]);
            [~, data2] = buf.step(5);
            testCase.verifyEqual(data2, [2 3 4 5]);
        end

        function testRLSEstimatorConverges(testCase)
            rng(2);
            est = ddc.common.RLSEstimator('NumParameters', 2, 'ForgettingFactor', 1);
            trueTheta = [3; -2];
            theta = zeros(2,1);
            for k = 1:300
                phi = randn(2,1);
                y = trueTheta.'*phi;
                theta = est.step(phi, y);
            end
            testCase.verifyEqual(theta, trueTheta, 'AbsTol', 1e-3);
        end

        function testRLSEstimatorExternalResetEquivalent(testCase)
            % With the reset input held false, the 3-input form must be
            % identical to the default 2-input RLS.
            rng(7);
            trueTheta = [3; -2];
            estDef = ddc.common.RLSEstimator('NumParameters', 2);
            estExt = ddc.common.RLSEstimator('NumParameters', 2, 'ExternalReset', true);
            for k = 1:20
                phi = randn(2, 1);
                y = trueTheta.' * phi;
                [tDef, eDef] = estDef.step(phi, y);
                [tExt, eExt] = estExt.step(phi, y, false);
                testCase.verifyEqual(tExt, tDef, 'AbsTol', 1e-12);
                testCase.verifyEqual(eExt, eDef, 'AbsTol', 1e-12);
            end
        end

        function testRLSEstimatorExternalResetReadapts(testCase)
            % A true reset re-initializes the covariance (P -> C*I) so RLS
            % re-adapts quickly to a changed plant, while leaving theta
            % finite. With P converged to ~0 and no reset, theta barely moves.
            rng(11);
            theta0 = [3; -2];
            theta1 = [2; 5];

            estR = ddc.common.RLSEstimator('NumParameters', 2, 'ExternalReset', true);
            estN = ddc.common.RLSEstimator('NumParameters', 2, 'ExternalReset', true);
            for k = 1:300
                phi = randn(2, 1);
                y = theta0.' * phi;
                estR.step(phi, y, false);
                estN.step(phi, y, false);
            end

            phiNew = randn(2, 1);
            yNew = theta1.' * phiNew;
            [thetaR, ~] = estR.step(phiNew, yNew, true);   % covariance reset
            [thetaN, ~] = estN.step(phiNew, yNew, false);  % P ~ 0, no reset

            % Without reset, P is nearly converged: theta barely moves.
            testCase.verifyEqual(thetaN, theta0, 'AbsTol', 0.1);
            % With reset, P is re-initialized: theta re-adapts strongly
            % toward the new plant.
            testCase.verifyGreaterThan(norm(thetaR - theta0), 1);
            testCase.verifyLessThan(norm(thetaR - theta1), norm(thetaN - theta1));
        end

        function testExcitationSignalGeneratorPRBSBounded(testCase)
            gen = ddc.common.ExcitationSignalGenerator('SignalType', 'prbs', 'Amplitude', 2);
            vals = zeros(1, 20);
            for k = 1:20
                vals(k) = gen.step();
            end
            testCase.verifyTrue(all(abs(vals) == 2));
        end

        function testExcitationSignalGeneratorRandomBounded(testCase)
            gen = ddc.common.ExcitationSignalGenerator('SignalType', 'random', 'Amplitude', 1.5);
            vals = zeros(1, 50);
            for k = 1:50
                vals(k) = gen.step();
            end
            testCase.verifyTrue(all(abs(vals) <= 1.5));
        end
    end
end
