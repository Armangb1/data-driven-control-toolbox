classdef MDPPDesignTest < matlab.unittest.TestCase
% MDPPDesignTest  Unit tests for the mdpp package (minimum-degree pole placement).
%
%   Covers: B factorization, Diophantine solve, controller construction,
%   compatibility checks, causality, complex-conjugate pairing, damping-ratio
%   rejection, and auto-augmentation of the reference model.

    properties (Constant)
        Tol = 1e-9;
    end

    methods (TestClassSetup)
        function addMdppPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Static)
        function s = polyAdd(p, q)
        % polyAdd  Add two descending-power polynomials with leading-zero padding
        % so that coefficients align by highest power. (MATLAB's default vector
        % addition pads on the right, which is WRONG for this convention.)
            p = ddc.str.mdpp.trimLeadingZeros(p(:).');
            q = ddc.str.mdpp.trimLeadingZeros(q(:).');
            n = max(length(p), length(q));
            s = [zeros(1, n - length(p)), p] + [zeros(1, n - length(q)), q];
            s = ddc.str.mdpp.trimLeadingZeros(s);
        end

        function ok = hasControlToolbox()
            ok = ~isempty(which('tf')) && ~isempty(which('zpk')) && ~isempty(which('ss'));
        end
    end

    % =====================================================================
    % Test 1: Clean textbook case — all zeros minimum-phase (full cancellation)
    % =====================================================================
    methods (Test)
        function testAllZerosCancelable(testCase)
            % Plant: A = q^2 - 1.5q + 0.7, B = q + 0.5 (root at -0.5, stable)
            A  = [1 -1.5 0.7];
            B  = [1 0.5];
            Am = [1 -1.1 0.3];   % target roots 0.5 and 0.6
            Bm = [1 0.5];        % matches B (all cancelable)
            Ao = [1];            % deg(Ao) = deg(A) - deg(B+) - 1 = 0

            result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, 'Tol', testCase.Tol);

            % All zeros cancelable: Bplus reproduces B (monic), Bminus is scalar.
            testCase.verifyEqual(result.Bplus, [1 0.5], 'AbsTol', testCase.Tol);
            testCase.verifyTrue(isscalar(result.Bminus));

            % Diophantine residual must be tiny.
            residual = norm(testCase.polyAdd(conv(A, result.Rprime), ...
                conv(result.Bminus, result.S)) - conv(Ao, Am), Inf);
            testCase.verifyLessThan(residual, testCase.Tol);

            % Closed-loop identity: A*R + B*S == Bplus * (Ao * Am).
            clhs = testCase.polyAdd(conv(A, result.R), conv(B, result.S));
            crhs = conv(result.Bplus, conv(Ao, Am));
            testCase.verifyEqual(clhs, crhs / crhs(1) * clhs(1), 'AbsTol', 1e-6);

            % IsCausal and low residual.
            testCase.verifyTrue(result.IsCausal);
            testCase.verifyLessThan(residual, 1e-12);
        end

        % =================================================================
        % Test 2: Non-minimum-phase root — Bm not containing Bminus throws
        % =================================================================
        function testNMPWithIncompatibleBm(testCase)
            % B = q - 1.5 has a zero at 1.5 (outside the unit disk).
            A  = [1 -1.1 0.3];
            B  = [1 -1.5];
            Am = [1 -0.7 0.12];
            Bm = [1 0.5];        % does NOT contain the zero at 1.5
            Ao = [1 0.2];        % deg(Ao) = deg(A) - deg(B+) - 1 = 1

            testCase.verifyError(@() ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao), ...
                'MDPP:IncompatibleReferenceModel');
        end

        % =================================================================
        % Test 2b: NMP root lands in Bminus (compatible Bm succeeds)
        % =================================================================
        function testNMProotLandsInBminus(testCase)
            A  = [1 -1.1 0.3];
            B  = [1 -1.5];
            Am = [1 -0.7 0.12];
            Bm = [1 -1.5];       % contains the NMP zero
            Ao = [1 0.2];

            result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, 'Tol', testCase.Tol);

            % The NMP zero must never be cancelled: it lives in Bminus.
            testCase.verifyEqual(length(result.Bplus), 1, ...
                'Bplus must be scalar (no cancelable zeros).');
            testCase.verifyTrue(any(abs(roots(result.Bminus) - 1.5) < testCase.Tol), ...
                'Bminus must contain the zero at 1.5.');
            testCase.verifyTrue(result.IsCausal);

            % Closed-loop identity still holds.
            clhs = testCase.polyAdd(conv(A, result.R), conv(B, result.S));
            crhs = conv(result.Bplus, conv(Ao, Am));
            testCase.verifyEqual(clhs, crhs / crhs(1) * clhs(1), 'AbsTol', 1e-6);
        end

        % =================================================================
        % Test 3: NMP case with AutoAugmentBm — verify augmentation
        % =================================================================
        function testNMPAutoAugmentBm(testCase)
            A  = [1 -1.1 0.3];
            B  = [1 -1.5];
            Am = [1 -0.7 0.12];
            Bm = [1 0.5];        % does NOT contain root 1.5
            Ao = [1 0.2];

            result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, ...
                'AutoAugmentBm', true, 'Tol', testCase.Tol);

            % AchievedBm must contain the NMP root.
            achRoots = roots(result.AchievedBm);
            testCase.verifyTrue(any(abs(achRoots - 1.5) < testCase.Tol), ...
                'AchievedBm must contain the NMP root 1.5.');

            % Warnings must explain the substitution.
            testCase.verifyNotEmpty(result.Warnings);

            % Controller must be causal.
            testCase.verifyTrue(result.IsCausal);
            nR = length(result.R) - 1;
            nS = length(result.S) - 1;
            nT = length(result.T) - 1;
            testCase.verifyEqual(nR, nS);
            testCase.verifyEqual(nR, nT);
        end

        % =================================================================
        % Test 4: A and Bminus share a root — NotCoprime error
        % =================================================================
        function testNotCoprime(testCase)
            % A = q^2 - 2.5q + 1.5 has a root at 1.5, shared with B.
            A  = [1 -2.5 1.5];
            B  = [1 -1.5];
            Am = [1 -0.7 0.12];
            Bm = [1 -1.5];       % compatible, so failure is purely coprimeness
            Ao = [1 0.2];

            testCase.verifyError(@() ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao), ...
                'MDPP:NotCoprime');
        end

        % =================================================================
        % Test 5: Incorrect Ao degree — IncompatibleReferenceModel error
        % =================================================================
        function testIncorrectAoDegree(testCase)
            A  = [1 -1.5 0.7];
            B  = [1 0.5];
            Am = [1 -1.1 0.3];
            Bm = [1 0.5];
            Ao = [1 0.5 0.1];    % deg 2, but expected deg 0

            testCase.verifyError(@() ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao), ...
                'MDPP:IncompatibleReferenceModel');
        end

        % =================================================================
        % Test 6: Explicit causality degree verification on passing cases
        % =================================================================
        function testCausalityDegrees(testCase)
            A  = [1 -1.5 0.7];
            B  = [1 0.5];
            Am = [1 -1.1 0.3];
            Bm = [1 0.5];
            Ao = [1];

            result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, 'Tol', testCase.Tol);

            nA = length(A) - 1;
            nR = length(result.R) - 1;
            nS = length(result.S) - 1;
            nT = length(result.T) - 1;

            testCase.verifyEqual(nR, nA - 1, 'R degree mismatch');
            testCase.verifyEqual(nS, nA - 1, 'S degree mismatch');
            testCase.verifyEqual(nT, nA - 1, 'T degree mismatch');
            testCase.verifyEqual(nR, nS, 'R and S degrees must match');
            testCase.verifyEqual(nR, nT, 'R and T degrees must match');
        end

        % =================================================================
        % Test 7: Complex-conjugate zeros — real coefficients in factorization
        % =================================================================
        function testComplexConjugatePairing(testCase)
            % B = (q + 0.3 - 0.4i)(q + 0.3 + 0.4i) = q^2 + 0.6q + 0.25
            A  = [1 -1.2 0.41 -0.03];
            B  = [1 0.6 0.25];
            Am = [1 -1.2 0.41 -0.042];
            Bm = [1 0.6 0.25];
            Ao = [1];

            result = ddc.str.mdpp.mdpp_design(A, B, Am, Bm, Ao, 'Tol', testCase.Tol);

            % Factorization must reconstruct real coefficient vectors.
            testCase.verifyEqual(imag(result.Bplus), zeros(size(result.Bplus)), ...
                'AbsTol', testCase.Tol, ...
                'Bplus must have real coefficients after complex-conjugate pairing.');
            testCase.verifyEqual(imag(result.Bminus), zeros(size(result.Bminus)), ...
                'AbsTol', testCase.Tol, ...
                'Bminus must have real coefficients after complex-conjugate pairing.');

            % Reconstruction identity.
            testCase.verifyEqual(conv(result.Bplus, result.Bminus), B, ...
                'AbsTol', testCase.Tol);

            % Causality holds.
            nR = length(result.R) - 1;
            nS = length(result.S) - 1;
            nT = length(result.T) - 1;
            testCase.verifyEqual(nR, nS);
            testCase.verifyEqual(nR, nT);
        end

        % =================================================================
        % Transfer-function object inputs (tf/zpk/ss) — identical results to
        % the polynomial API. Requires the Control System Toolbox; skipped
        % otherwise so the suite stays runnable in base MATLAB.
        % =================================================================
        function testTransferFunctionObjectInputs(testCase)
            if ~testCase.hasControlToolbox()
                testCase.assumeFail(...
                    'Control System Toolbox not installed; tf/zpk/ss tests skipped.');
            end

            P   = tf([1 0.5], [1 -1.5 0.7], 1);    % discrete plant model
            Am  = tf(1, [1 -1.1 0.3], 1);          % tracking denominator
            Bm  = tf([1 0.5], 1, 1);               % reference numerator
            Ao  = tf(1, 1, 1);                     % observer (deg 0)

            reference = ddc.str.mdpp.mdpp_design([1 -1.5 0.7], [1 0.5], ...
                [1 -1.1 0.3], [1 0.5], [1], 'Tol', testCase.Tol);

            % A as tf object with B omitted ([]) -> B derived from numerator.
            resTF = ddc.str.mdpp.mdpp_design(P, [], Am, Bm, Ao, 'Tol', testCase.Tol);
            testCase.verifyEqual(resTF.R, reference.R, 'AbsTol', testCase.Tol, ...
                'tf-plant design must match the polynomial design.');
            testCase.verifyEqual(resTF.S, reference.S, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(resTF.T, reference.T, 'AbsTol', testCase.Tol);
            testCase.verifyTrue(resTF.IsCausal);

            % zpk variant.
            resZPK = ddc.str.mdpp.mdpp_design(zpk(P), [], Am, Bm, Ao, 'Tol', testCase.Tol);
            testCase.verifyEqual(resZPK.R, reference.R, 'AbsTol', 1e-6, ...
                'zpk-plant design must match the polynomial design.');

            % ss variant.
            resSS = ddc.str.mdpp.mdpp_design(ss(P), [], Am, Bm, Ao, 'Tol', testCase.Tol);
            testCase.verifyEqual(resSS.R, reference.R, 'AbsTol', 1e-6, ...
                'ss-plant design must match the polynomial design.');

            % Explicit B alongside an object A must also be honoured.
            resMixed = ddc.str.mdpp.mdpp_design(P, [1 0.5], Am, Bm, Ao, 'Tol', testCase.Tol);
            testCase.verifyEqual(resMixed.R, reference.R, 'AbsTol', testCase.Tol);
        end

        % =================================================================
        % B omitted with a plain polynomial A must be rejected.
        % =================================================================
        function testBemptyRequiresModel(testCase)
            testCase.verifyError(@() ddc.str.mdpp.mdpp_design([1 -1.5 0.7], [], ...
                [1 -1.1 0.3], [1 0.5], [1]), 'MDPP:InvalidInput');
        end

        % =================================================================
        % Extra: DampingMin rejects stable but poorly-damped zeros
        % =================================================================
        function testDampingMinRejectsPoorlyDampedZero(testCase)
            % B = q^2 + 0.25 has roots at +/- 0.5i, |root| = 0.5 < 1 (stable
            % zero), but the mapped damping ratio is only ~0.40.
            B = [1 0 0.25];

            % Without DampingMin the zero is cancelable.
            [bpOk, bmOk] = ddc.str.mdpp.factorB(B, 1, [], 1e-9);
            testCase.verifyEqual(bpOk, [1 0 0.25], 'AbsTol', 1e-9);
            testCase.verifyEqual(bmOk, 1, 'AbsTol', 1e-9);

            % With DampingMin >= 0.6 the zero must be classified non-cancelable.
            [bpRej, bmRej] = ddc.str.mdpp.factorB(B, 1, 0.6, 1e-9);
            testCase.verifyEqual(bpRej, 1, 'AbsTol', 1e-9);
            testCase.verifyEqual(bmRej, [1 0 0.25], 'AbsTol', 1e-9);
        end
    end
end