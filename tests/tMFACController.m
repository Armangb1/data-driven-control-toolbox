classdef tMFACController < matlab.unittest.TestCase
    %TMFACCONTROLLER Tests for ddc.mfac.MFACController.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testTracksConstantSetpoint(testCase)
            ctrl = ddc.mfac.MFACController('PhiInit', 1);
            y = 0;
            for k = 1:200
                r = 1;
                u = ctrl.step(y, r);
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
    end
end
