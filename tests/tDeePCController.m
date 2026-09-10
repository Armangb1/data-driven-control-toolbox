classdef tDeePCController < matlab.unittest.TestCase
    %TDEEPCCONTROLLER Tests for ddc.deepc.DeePCController / deepcDesign.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testDeepcDesignShapes(testCase)
            rng(1);
            T = 60; Tini = 3; N = 5;
            uD = 2*(rand(1,T)-0.5);
            yD = zeros(1,T);
            for k = 1:T-1
                yD(k+1) = 0.8*yD(k) + uD(k);
            end
            design = ddc.deepc.deepcDesign(uD, yD, Tini, N);
            nCols = T - (Tini+N) + 1;
            testCase.verifySize(design.Up, [Tini, nCols]);
            testCase.verifySize(design.Uf, [N, nCols]);
            testCase.verifySize(design.Yp, [Tini, nCols]);
            testCase.verifySize(design.Yf, [N, nCols]);
            testCase.verifyTrue(design.isPE);
        end

        function testDeepcControllerTracksSetpoint(testCase)
            rng(1);
            T = 80; Tini = 3; N = 6;
            uD = 2*(rand(1,T)-0.5);
            yD = zeros(1,T);
            for k = 1:T-1
                yD(k+1) = 0.8*yD(k) + uD(k);
            end

ctrl = ddc.deepc.DeePCController('DataU', uD, 'DataY', yD, ...
                'Tini', Tini, 'N', N, ...
                'Q', 1, 'R', 0.001, 'LambdaG', 0.1, 'LambdaY', 1e5);

            % Closed-loop simulation against the same first-order plant.
            % Controller maintains its own input/output history buffers
            % internally via LastU_ and the UBuf_/YBuf_ mechanism.
            y = 0;
            rFuture = ones(1, N);
            for k = 1:40
                uApply = ctrl.step(y, rFuture);
                y = 0.8*y + uApply;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.05);
        end
    end
end
