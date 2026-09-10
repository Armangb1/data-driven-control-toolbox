classdef tMFACController < matlab.unittest.TestCase
    %TMFACCONTROLLER Tests for ddc.mfac.MFACController (CFDL / PFDL / FFDL).

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testCfdRegressionMatchesGolden(testCase)
            % CFDL (Ly=0, Lu=1) must reproduce the pre-change implementation
            % exactly on the original test fixture (bit-identical to FP tol).
            ctrl = ddc.mfac.MFACController('PhiInit', 1, 'Eta', 1, 'Mu', 1, ...
                'Rho', 1, 'Lambda', 1, 'Epsilon', 1e-5);
            uGolden = [0.5, 0.87292817679558, 1.05400977731697, ...
                1.03593864372082, 0.862695901712187, 0.609762132880106, ...
                0.357507272090357, 0.167053542338504, 0.0716596584194105, ...
                0.0759990677871532];
            phiGolden = [1, 0.9, 0.916641097615714, 0.957620693722847, ...
                0.951694585500083, 0.896721814156902, 0.842064737850488, ...
                0.821011216980452, 0.8279540435755, 0.839678865105638];
            y = 0;
            for k = 1:numel(uGolden)
                [u, phi] = ctrl.step(y, 1);
                testCase.verifyEqual(u, uGolden(k), 'AbsTol', 1e-9, ...
                    'RelTol', 1e-11);
                testCase.verifyEqual(phi, phiGolden(k), 'AbsTol', 1e-9, ...
                    'RelTol', 1e-11);
                y = 0.8*y + 0.5*u;
            end
        end

        function testTracksConstantSetpoint(testCase)
            ctrl = ddc.mfac.MFACController('PhiInit', 1);
            y = 0;
            for k = 1:200
                u = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.02);
        end

        function testResetRestoresInitialState(testCase)
            ctrl = ddc.mfac.MFACController('PhiInit', 1);
            y = 0;
            for k = 1:50
                u = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            reset(ctrl);
            [u2, phi2] = ctrl.step(0, 1);
            testCase.verifyEqual(phi2, 1);
            testCase.verifyGreaterThan(u2, 0);
        end

        function testPfdlMatchesReference(testCase)
            % PFDL: Ly=0, Lu=3. Compare against an independent, explicitly
            % written implementation of the PFDL projection-algorithm law
            % on a synthetic trajectory (u-delta sum term active).
            Ly = 0; Lu = 3;
            params.PhiInit = [0.5; 0.4; 0.3];
            params.Rho     = [0.8; 0.6; 0.4];
            params.Eta = 0.9; params.Mu = 0.7; params.Lambda = 0.2;
            params.Epsilon = 1e-4;

            [yT, rT] = syntheticTrajectory(60);

            ctrl = ddc.mfac.MFACController('Ly', Ly, 'Lu', Lu, ...
                'PhiInit', params.PhiInit, 'Rho', params.Rho, ...
                'Eta', params.Eta, 'Mu', params.Mu, ...
                'Lambda', params.Lambda, 'Epsilon', params.Epsilon);
            uCls = zeros(1, numel(yT));
            for k = 1:numel(yT)
                uCls(k) = ctrl.step(yT(k), rT(k));
            end

            uRef = mfacReference(yT, rT, Ly, Lu, params.PhiInit, params.Rho, ...
                params.Eta, params.Mu, params.Lambda, params.Epsilon, Ly+1);
            testCase.verifyEqual(uCls, uRef, 'AbsTol', 1e-9, 'RelTol', 1e-11);
        end

        function testFfdlMatchesReference(testCase)
            % FFDL: Ly=2, Lu=2. Both delta-y and delta-u sums are active.
            Ly = 2; Lu = 2;
            params.PhiInit = [0.5; 0.5; 0.5; 0.5];
            params.Rho     = [0.7; 0.5; 0.8; 0.3];
            params.Eta = 0.9; params.Mu = 0.7; params.Lambda = 0.2;
            params.Epsilon = 1e-4;

            [yT, rT] = syntheticTrajectory(60);

            ctrl = ddc.mfac.MFACController('Ly', Ly, 'Lu', Lu, ...
                'PhiInit', params.PhiInit, 'Rho', params.Rho, ...
                'Eta', params.Eta, 'Mu', params.Mu, ...
                'Lambda', params.Lambda, 'Epsilon', params.Epsilon);
            uCls = zeros(1, numel(yT));
            for k = 1:numel(yT)
                uCls(k) = ctrl.step(yT(k), rT(k));
            end

            uRef = mfacReference(yT, rT, Ly, Lu, params.PhiInit, params.Rho, ...
                params.Eta, params.Mu, params.Lambda, params.Epsilon, Ly+1);
            testCase.verifyEqual(uCls, uRef, 'AbsTol', 1e-9, 'RelTol', 1e-11);
        end

        function testFfdlTracksAndStaysBounded(testCase)
            % FFDL controller drives the PDF plant toward a moving setpoint.
            Tsim = 300;
            t = (1:Tsim)';
            yd = 0.4036*sin(t*pi/180) + 0.12*cos(t*pi/50);
            ctrl = ddc.mfac.MFACController('Ly', 2, 'Lu', 2, ...
                'PhiInit', 0.01*ones(4,1), 'Eta', 0.1, 'Mu', 0.5, ...
                'Rho', 0.3*ones(4,1), 'Lambda', 2e-6, 'Epsilon', 1e-5);
            [yLog, uLog] = runPdfPlant(ctrl, yd);
            testCase.verifyTrue(all(isfinite(yLog)) && all(isfinite(uLog)));
            % Tracking error over the last 100 samples should be small.
            err = abs(yLog(end-99:end) - yd(end-99:end)');
            testCase.verifyLessThan(max(err), 0.05);
        end

        function testFfdlResetUsesLyPlusOne(testCase)
            % The reset sign-check must inspect phi(Ly+1), NOT phi(1).
            % Feed a trajectory that flips phi(1) while phi(3) stays put;
            % a wrong "check element 1" logic would spuriously reset.
            Ly = 2; Lu = 1;
            PhiInit = [0.5; 0.5; 0.5];
            params.Eta = 5; params.Mu = 1e-6;
            params.Rho = [0.3; 0.3; 0.3];
            params.Lambda = 1; params.Epsilon = 1e-5;

            yT = [0, 0, 6, -5, 7, -6, 0.1, 2];
            rT = ones(size(yT));

            ctrl = ddc.mfac.MFACController('Ly', Ly, 'Lu', Lu, ...
                'PhiInit', PhiInit, 'Rho', params.Rho, ...
                'Eta', params.Eta, 'Mu', params.Mu, ...
                'Lambda', params.Lambda, 'Epsilon', params.Epsilon);
            uCls = zeros(1, numel(yT));
            for k = 1:numel(yT)
                uCls(k) = ctrl.step(yT(k), rT(k));
            end

            % Correct logic (element Ly+1) must match the class exactly...
            uOk = mfacReference(yT, rT, Ly, Lu, PhiInit, params.Rho, ...
                params.Eta, params.Mu, params.Lambda, params.Epsilon, Ly+1);
            testCase.verifyEqual(uCls, uOk, 'AbsTol', 1e-9, 'RelTol', 1e-11);

            % ...and the wrong logic (element 1) must produce a different
            % trajectory on at least one sample.
            uBad = mfacReference(yT, rT, Ly, Lu, PhiInit, params.Rho, ...
                params.Eta, params.Mu, params.Lambda, params.Epsilon, 1);
            testCase.verifyTrue(any(abs(uCls - uBad) > 1e-8), ...
                'scenario did not exercise the reset-index difference');
        end

        function testLyZeroLuZeroErrors(testCase)
            % Lu is validated positive at the property level, so a (0,0)
            % configuration is rejected before any step is ever possible.
            testCase.verifyError(@() ddc.mfac.MFACController('Ly', 0, 'Lu', 0), ...
                'MATLAB:validators:mustBePositive');
        end

        function testBadPhiInitLengthErrors(testCase)
            ctrl = ddc.mfac.MFACController('Ly', 2, 'Lu', 2, ...
                'PhiInit', [0.1 0.1 0.1], 'Rho', 0.3*ones(4,1));
            testCase.verifyError(@() ctrl.step(0, 1), ...
                'ddc:mfac:MFACController:BadPhiInit');
        end

        function testBadRhoLengthErrors(testCase)
            ctrl = ddc.mfac.MFACController('Ly', 2, 'Lu', 2, ...
                'PhiInit', 0.01*ones(4,1), 'Rho', [0.3 0.3]);
            testCase.verifyError(@() ctrl.step(0, 1), ...
                'ddc:mfac:MFACController:BadRho');
        end
    end
end

function [yT, rT] = syntheticTrajectory(N)
    k = (1:N);
    yT = 0.6*sin(0.25*k) + 0.2*cos(0.09*k.^0.7) + 0.05*k.*sin(0.02*k.^2);
    rT = 0.4*sin(0.2*k) + 0.3*cos(0.15*k);
end

function [yLog, uLog] = runPdfPlant(ctrl, yd)
    Tsim = numel(yd);
    x2 = 0; y = 0;
    yLog = zeros(1, Tsim); uLog = zeros(1, Tsim);
    for k = 1:Tsim
        u = ctrl.step(y, yd(k));
        x1n = x2;
        x2n = -(1/64)*((1.7*sin(k*pi/10))*x2 + (exp(-k)+3)*x2^3) + (1/128)*u;
        y = x1n; x2 = x2n;
        yLog(k) = y; uLog(k) = u;
    end
end

function uSeq = mfacReference(yIn, rIn, Ly, Lu, PhiInit, Rho, Eta, Mu, Lambda, Epsilon, resetIdx)
    % Independent reference: explicit FFDL/PFDL/CFDL recursion written from
    % the projection algorithm + control law, using growing histories.
    N = numel(rIn);
    n = Ly + Lu;
    PhiInit = PhiInit(:);
    Rho = Rho(:);

    yHist = zeros(1, max(Ly, 1) + 2);
    uHist = zeros(1, Lu + 2);
    HPrev = zeros(n, 1);
    phi = PhiInit;
    PrevU = 0;
    uSeq = zeros(1, N);

    for k = 1:N
        H = zeros(n, 1);
        if Ly > 0
            H(1:Ly) = yHist(1:Ly);
        end
        H(Ly+1:end) = uHist(1:Lu);
        dH = H - HPrev;

        yHist = [yIn(k), yHist(1:end-1)];
        dy = yHist(1) - yHist(2);

        dHn2 = dH' * dH;
        phiN = phi + (Eta/(Mu + dHn2)) * dH * (dy - phi' * dH);

        if sqrt(phiN'*phiN) <= Epsilon || dHn2 <= Epsilon^2 || ...
                sign(phiN(resetIdx)) ~= sign(PhiInit(resetIdx))
            phiN = PhiInit;
        end

        phi1 = phiN(Ly+1);
        denom = Lambda + phi1^2;
        du = Rho(Ly+1)*phi1*(rIn(k) - yHist(1))/denom;
        for i = 1:Ly
            du = du - phi1*Rho(i)*phiN(i)*(yHist(i) - yHist(i+1))/denom;
        end
        for i = (Ly+2):(Ly+Lu)
            j = i - Ly - 1;
            du = du - phi1*Rho(i)*phiN(i)*(uHist(j) - uHist(j+1))/denom;
        end
        u = PrevU + du;

        uHist = [u, uHist(1:end-1)];
        phi = phiN; PrevU = u; HPrev = H;
        uSeq(k) = u;
    end
end
