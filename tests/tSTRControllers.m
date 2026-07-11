classdef tSTRControllers < matlab.unittest.TestCase
    %TSTRCONTROLLERS Tests for ddc.str.DirectSTRController / IndirectSTRController.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testDirectSTRTracksAndIdentifies(testCase)
            ctrl = ddc.str.DirectSTRController('Ac1', -0.5);
            y = 0;
            for k = 1:100
                [u, theta] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.02);
            testCase.verifyEqual(theta(1), 0.5, 'AbsTol', 0.02); % beta0 = b1
            testCase.verifyEqual(theta(2), 0.3, 'AbsTol', 0.02); % beta1 = b1*s0 = Ac1-a1
        end

        function testIndirectSTRTracksAndIdentifies(testCase)
            ctrl = ddc.str.IndirectSTRController('Ac1', -0.5);
            y = 0;
            for k = 1:100
                [u, theta] = ctrl.step(y, 1);
                y = 0.8*y + 0.5*u;
            end
            testCase.verifyEqual(y, 1, 'AbsTol', 0.02);
            testCase.verifyEqual(theta(1), 0.8, 'AbsTol', 0.02);  % -a1
            testCase.verifyEqual(theta(2), 0.5, 'AbsTol', 0.02);  % b1
        end

        function testNoDeadlockFromZeroInitialGains(testCase)
            % Regression test: initial S0=0,T0=0 previously caused u==0
            % forever (certainty-equivalence deadlock).
            ctrl = ddc.str.DirectSTRController();
            u1 = ctrl.step(0, 1);
            testCase.verifyNotEqual(u1, 0);
        end
    end
end
