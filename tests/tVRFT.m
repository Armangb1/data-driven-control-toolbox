classdef tVRFT < matlab.unittest.TestCase
    %TVRFT Tests for ddc.vrft.vrftDesign / prefilterDesign.

    methods (TestClassSetup)
        function addToolboxPath(testCase)
            here = fileparts(mfilename('fullpath'));
            toolboxPath = fullfile(here, '..', 'toolbox');
            addpath(toolboxPath);
            testCase.addTeardown(@() rmpath(toolboxPath));
        end
    end

    methods (Test)
        function testRecoversIdealControllerNoiselessCase(testCase)
            rng(2);
            Ts = 0.1;
            plant = tf(0.5, [1 -0.8], Ts);
            Md = tf(0.3, [1 -0.7], Ts);

            T = 500;
            u = 2*(rand(T,1) > 0.5) - 1;
            t = (0:T-1)'*Ts;
            y = lsim(plant, u, t);

            basis = { tf(1,1,Ts), tf(Ts,[1 -1],Ts) }; % {1, Ts/(z-1)}

            [theta, info] = ddc.vrft.vrftDesign(u, y, Md, basis);

            % Ideal matching controller for this plant/Md pair:
            % C_ideal(z) = 0.6 - 0.48/(z-1) analytically -> theta = [0.6; 1.2]
            testCase.verifyEqual(theta, [0.6; 1.2], 'AbsTol', 1e-3);
            testCase.verifyEqual(info.relativeDegree, 1);
        end

        function testPrefilterDesignRequiresDiscreteModel(testCase)
            Md = tf(1, [1 1]); % continuous-time
            testCase.verifyError(@() ddc.vrft.prefilterDesign(Md), ...
                'ddc:vrft:prefilterDesign:ContinuousModel');
        end
    end
end
